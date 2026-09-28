"""Proof that synchronized contact observation is deterministic and non-invasive."""

from __future__ import annotations

import mujoco
import numpy as np
import pytest

from v6_mujoco.model import PROJECT_ROOT, geom_id
from v6_mujoco.fpmfc.audit_contact import CONTACTS, _npz_is_real
from v6_mujoco.fpmfc.contact_model import default_contact_model_spec
from v6_mujoco.fpmfc.contact_observation import ContactObserver, assert_observation_safe


def _model_observer():
    spec = default_contact_model_spec()
    model = spec.compile_model()
    model.geom_pos[geom_id(model, "workspace_obstacle_0")] = [10.0, 10.0, 10.0]
    return model, ContactObserver(model, tool_face_recess_m=0.0002)


def _initialized(model, trace):
    data = mujoco.MjData(model)
    data.qpos[:] = trace["initial_qpos"]
    data.qvel[:] = trace["initial_qvel"]
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)
    return data


@pytest.mark.parametrize("variant", tuple(CONTACTS))
def test_observer_leaves_full_replay_sequence_unchanged(variant):
    path = PROJECT_ROOT / CONTACTS[variant] / "trace.npz"
    if not _npz_is_real(path):
        pytest.skip("archived LFS trace is not hydrated")
    model, observer = _model_observer()
    with np.load(path, allow_pickle=False) as trace:
        control = _initialized(model, trace)
        observed = _initialized(model, trace)
        for torque in trace["torque"]:
            control.ctrl[:] = torque
            observed.ctrl[:] = torque
            mujoco.mj_step(model, control)
            mujoco.mj_step(model, observed)
            before = {name: np.asarray(getattr(observed, name)).copy() for name in (
                "qpos", "qvel", "ctrl", "qfrc_applied", "xfrc_applied", "qacc_warmstart", "mocap_pos", "mocap_quat", "eq_active",
            )}
            before_time = float(observed.time)
            snapshot = observer.observe(observed)
            assert snapshot.time_s == before_time
            for name, value in before.items():
                np.testing.assert_array_equal(getattr(observed, name), value)
            assert observed.time == before_time
            np.testing.assert_allclose(observed.qpos, control.qpos, atol=1e-8, rtol=0)
            np.testing.assert_allclose(observed.qvel, control.qvel, atol=1e-8, rtol=0)
            np.testing.assert_array_equal(observed.ctrl, control.ctrl)


def test_same_snapshot_recomputes_same_derived_fields():
    model, observer = _model_observer()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    first = observer.observe(data)
    second = observer.observe(data)
    assert first.time_s == second.time_s
    np.testing.assert_array_equal(first.qpos, second.qpos)
    np.testing.assert_array_equal(first.qvel, second.qvel)
    np.testing.assert_array_equal(first.flange_position_world_m, second.flange_position_world_m)
    np.testing.assert_array_equal(first.grasp_linear_velocity_world_m_s, second.grasp_linear_velocity_world_m_s)
    np.testing.assert_array_equal(first.contact_force_world_n, second.contact_force_world_n)
    assert first.contact_count == second.contact_count
    assert first.penetration_m == second.penetration_m


def test_tool_face_and_relative_twist_use_one_world_frame_and_reference_point():
    model, observer = _model_observer()
    data = mujoco.MjData(model)
    data.qvel[:6] = [0.01, -0.02, 0.03, 0.2, -0.1, 0.3]
    observed = observer.observe(data)
    displacement = observed.tool_face_position_world_m - observed.flange_position_world_m
    np.testing.assert_allclose(
        observed.tool_face_linear_velocity_world_m_s,
        observed.flange_linear_velocity_world_m_s
        + np.cross(observed.flange_angular_velocity_world_rad_s, displacement),
        atol=1e-14, rtol=0,
    )
    np.testing.assert_allclose(
        observed.relative_linear_velocity_world_m_s,
        observed.flange_linear_velocity_world_m_s-observed.grasp_linear_velocity_world_m_s,
        atol=1e-14, rtol=0,
    )
    np.testing.assert_allclose(
        observed.relative_angular_velocity_world_rad_s,
        observed.flange_angular_velocity_world_rad_s-observed.grasp_angular_velocity_world_rad_s,
        atol=1e-14, rtol=0,
    )


def test_recomputation_refuses_active_callback():
    model, observer = _model_observer()
    previous = mujoco.get_mjcb_control()
    try:
        mujoco.set_mjcb_control(lambda _model, _data: None)
        with pytest.raises(RuntimeError, match="callbacks"):
            assert_observation_safe(model)
        with pytest.raises(RuntimeError, match="callbacks"):
            observer.observe(mujoco.MjData(model))
    finally:
        mujoco.set_mjcb_control(previous)

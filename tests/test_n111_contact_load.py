"""N111 contact load algebra, causal input, and safety limits."""

import copy

import mujoco
import numpy as np
import pytest

from v6_mujoco.fpmfc.contact_model import default_contact_model_spec
from v6_mujoco.model import PROJECT_ROOT, geom_id
from v6_mujoco.fpmfc.n111_contact_servo import (
    ContactServo, RecordingObserver, bounded_compensation, reduced_contact_load,
    transport_wrench,
)
from v6_mujoco.run import servo_torque


def test_known_load_sign_and_protection():
    raw = np.array([0.0, 9.8])
    reduced = np.array([2.0, -3.0])
    requested, protected, torque = bounded_compensation(raw, reduced, 1.0, np.array([10.0, 10.0]))
    np.testing.assert_array_equal(requested, [2.0, -3.0])
    np.testing.assert_array_equal(protected, [1.0, -1.0])
    np.testing.assert_array_equal(torque, [-1.0, 10.0])
    np.testing.assert_array_equal(bounded_compensation(raw, reduced, 0.0, np.ones(2)*10)[2],
                                  np.clip(raw, -10, 10))


def test_wrench_transport_power_and_schur_elimination():
    force = np.array([0.0, 2.0, 0.0])
    old = np.array([1.0, 0.0, 0.0])
    new = np.zeros(3)
    np.testing.assert_array_equal(transport_wrench(force, np.zeros(3), old, new), [0.0, 0.0, 2.0])
    rng = np.random.default_rng(2111)
    a = rng.standard_normal((5, 5))
    mass = a.T @ a + np.eye(5)
    jp, jr = rng.standard_normal((3, 5)), rng.standard_normal((3, 5))
    torque = rng.standard_normal(3)
    base, joints = np.arange(2), np.arange(2, 5)
    full, reduced = reduced_contact_load(mass, jp, jr, force, torque, base, joints)
    velocity = rng.standard_normal(5)
    np.testing.assert_allclose(full @ velocity, force @ (jp @ velocity) + torque @ (jr @ velocity), atol=1e-13)
    acceleration = np.linalg.solve(mass, full)
    red_mass = mass[np.ix_(joints, joints)] - mass[np.ix_(joints, base)] @ np.linalg.solve(
        mass[np.ix_(base, base)], mass[np.ix_(base, joints)])
    np.testing.assert_allclose(np.linalg.solve(red_mass, reduced), acceleration[joints], atol=1e-13)
    np.testing.assert_array_equal(reduced_contact_load(mass, jp, jr, np.zeros(3), np.zeros(3), base, joints)[1], np.zeros(3))


def test_nonfinite_input_is_rejected():
    with pytest.raises(ValueError, match="nonfinite"):
        bounded_compensation(np.array([np.nan]), np.zeros(1), 0.5, np.ones(1))
    with pytest.raises(ValueError, match="nonfinite"):
        reduced_contact_load(np.eye(2), np.zeros((3, 2)), np.zeros((3, 2)),
                             np.array([np.inf, 0, 0]), np.zeros(3), np.array([0]), np.array([1]))


def test_gamma_zero_original_servo_and_observation_isolation():
    spec = default_contact_model_spec()
    model = spec.compile_model()
    data = mujoco.MjData(model)
    spec.reset_home(model, data)
    before = copy.copy(data)
    observer = RecordingObserver(model, tool_face_recess_m=0.02)
    observation = observer.observe(data)
    np.testing.assert_array_equal(data.qpos, before.qpos)
    np.testing.assert_array_equal(data.qvel, before.qvel)
    np.testing.assert_array_equal(data.ctrl, before.ctrl)
    assert observation.forward_data is not data
    qpos_ids, dof_ids = spec.joint_addresses(model)
    base = spec.base_slices(model)[1]
    q = np.asarray(data.qpos[qpos_ids]).copy()
    dq = np.zeros(7)
    ddq = np.zeros(7)
    original_data = copy.copy(data)
    original_mass = np.zeros((model.nv, model.nv))
    expected = servo_torque(model, original_data, dof_ids, base, qpos_ids, q, dq, ddq,
                            spec.torque_limits_nm, 34.0, 70.0, original_mass)[0]
    candidate = ContactServo(0.0, observer=observer)
    actual = candidate(model, data, dof_ids, base, qpos_ids, q, dq, ddq,
                       spec.torque_limits_nm, 34.0, 70.0, np.zeros_like(original_mass))[0]
    np.testing.assert_array_equal(actual, expected)
    np.testing.assert_array_equal(data.qpos, before.qpos)
    np.testing.assert_array_equal(data.qvel, before.qvel)
    assert len(candidate.rows) == 1 and observer.calls == 1
    data.time += 0.002
    with pytest.raises(RuntimeError, match="causal"):
        candidate(model, data, dof_ids, base, qpos_ids, q, dq, ddq,
                  spec.torque_limits_nm, 34.0, 70.0, np.zeros_like(original_mass))


def test_archived_d10_contact_states_are_finite_and_causal():
    spec = default_contact_model_spec()
    model = spec.compile_model()
    model.geom_pos[geom_id(model, "workspace_obstacle_0")] = [10.0, 10.0, 10.0]
    observer = RecordingObserver(model, tool_face_recess_m=0.02)
    servo = ContactServo(0.5, observer=observer)
    qpos_ids, dof_ids = spec.joint_addresses(model)
    base = spec.base_slices(model)[1]
    path = PROJECT_ROOT / "output/fpmfc/n110/force_regulation/D10/trace.npz"
    with np.load(path, allow_pickle=False) as trace:
        for i in (0, 100, 450):
            data = mujoco.MjData(model)
            data.time = float(trace["feedback_time_s"][i])
            data.qpos[:] = trace["feedback_qpos"][i]
            data.qvel[:] = trace["feedback_qvel"][i]
            data.ctrl[:] = trace["feedback_ctrl_used_for_forward"][i]
            data.qacc_warmstart[:] = (trace["initial_qacc_warmstart"] if i == 0
                                       else trace["qacc_warmstart_used_for_forward"][i-1])
            data.mocap_pos[:] = (trace["initial_mocap_pos"] if i == 0 else trace["mocap_pos"][i-1])
            data.mocap_quat[:] = (trace["initial_mocap_quat"] if i == 0 else trace["mocap_quat"][i-1])
            observer.observe(data)
            before_qpos, before_qvel = data.qpos.copy(), data.qvel.copy()
            torque, _ = servo(model, data, dof_ids, base, qpos_ids,
                              trace["reference_q"][i], trace["reference_dq"][i],
                              trace["reference_ddq"][i], spec.torque_limits_nm,
                              34.0, 70.0, np.zeros((model.nv, model.nv)))
            assert np.all(np.isfinite(torque))
            np.testing.assert_array_equal(data.qpos, before_qpos)
            np.testing.assert_array_equal(data.qvel, before_qvel)
    assert len(servo.rows) == 3 and observer.calls == 3

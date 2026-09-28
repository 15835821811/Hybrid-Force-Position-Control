"""Contact-only affine mapping against the full MuJoCo site Jacobian."""

from __future__ import annotations

import copy

import mujoco
import numpy as np
import pytest

from v6_mujoco.collision import build_collision_pairs
from v6_mujoco.model import PROJECT_ROOT, geom_id, site_id
from v6_mujoco.fpmfc.audit_contact import CONTACTS, _npz_is_real
from v6_mujoco.fpmfc.config import load_fpmfc_config
from v6_mujoco.fpmfc.contact_affine_controller import AffineContactFPMFCHQP
from v6_mujoco.fpmfc.contact_config import load_contact_config
from v6_mujoco.fpmfc.contact_model import default_contact_model_spec
from v6_mujoco.fpmfc.controller import FPMFCHQP
from v6_mujoco.fpmfc.run_capture import _constraint_config, _controller_config, controller_variant_config
from v6_mujoco.fpmfc.shape import ArmShapeKinematics


def _fixture():
    path = PROJECT_ROOT / CONTACTS["admittance"] / "trace.npz"
    if not _npz_is_real(path):
        pytest.skip("N103 archived LFS trace is not hydrated")
    config = load_contact_config()
    pre = load_fpmfc_config(PROJECT_ROOT / config["precontact_config"])
    spec = default_contact_model_spec()
    model = spec.compile_model()
    model.geom_pos[geom_id(model, "workspace_obstacle_0")] = [10.0, 10.0, 10.0]
    data = mujoco.MjData(model)
    spec.reset_home(model, data)
    shape_config = pre["shape"]
    shape = ArmShapeKinematics(
        spec, model, data,
        shoulder_joint=shape_config["shoulder_joint"],
        elbow_joint=shape_config["elbow_joint"],
        wrist_joint=shape_config["wrist_joint"],
        singularity_margin=float(shape_config["singularity_margin"]),
    )
    with np.load(path, allow_pickle=False) as trace:
        data.qpos[:] = trace["qpos"][200]
        data.qvel[:] = trace["qvel"][200]
    mujoco.mj_forward(model, data)
    controller_config = controller_variant_config(pre, "full")
    kwargs = dict(
        controller_config=_controller_config(controller_config),
        constraint_config=_constraint_config(controller_config),
    )
    pairs = build_collision_pairs(model)
    legacy = FPMFCHQP(spec, model, pairs, shape, **kwargs)
    affine = AffineContactFPMFCHQP(spec, model, pairs, shape, **kwargs)
    flange = site_id(model, "flange_site")
    target = dict(
        target_position=np.asarray(data.site_xpos[flange]).copy(),
        target_velocity=np.array([0.01, 0.0, -0.005]),
        target_rotation=np.asarray(data.site_xmat[flange]).reshape(3, 3).copy(),
        target_angular_velocity=np.array([0.0, 0.01, 0.0]),
        target_arm_angle_rad=float(shape.sample(data).angle_rad),
        target_arm_angle_velocity_rad_s=0.0,
    )
    return spec, model, data, legacy, affine, target


def test_zero_base_bias_affine_degenerates_to_homogeneous():
    spec, model, data, legacy, affine, target = _fixture()
    mapping, _ = legacy.reaction_velocity_map(data)
    _, base_slice = spec.base_slices(model)
    _, arm_ids = spec.joint_addresses(model)
    data.qvel[base_slice] = mapping[base_slice, :] @ data.qvel[arm_ids]
    mujoco.mj_forward(model, data)
    old = legacy.solve_fpmfc(copy.copy(data), **target)
    new = affine.solve_fpmfc(copy.copy(data), **target)
    assert np.linalg.norm(affine.last_bias) < 1e-14
    np.testing.assert_allclose(new.joint_velocity, old.joint_velocity, atol=1e-8, rtol=0)
    np.testing.assert_allclose(affine.last_debiased_task_rhs, affine.last_physical_task_command, atol=1e-13)


def test_nonzero_base_bias_reconstructs_physical_site_velocity_and_base_objective():
    spec, model, data, _legacy, affine, target = _fixture()
    result = affine.solve_fpmfc(data, **target)
    assert np.linalg.norm(affine.last_bias) > 1e-4
    mapping, _ = affine.reaction_velocity_map(data)
    qvel_command = mapping @ result.joint_velocity
    qvel_command[affine.base_dof_slice] += affine.last_bias
    linear = np.zeros((3, model.nv))
    angular = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, linear, angular, affine.flange_site)
    physical_twist = np.r_[linear @ qvel_command, angular @ qvel_command]
    np.testing.assert_allclose(physical_twist, affine.last_predicted_task_velocity, atol=1e-12)
    np.testing.assert_allclose(result.base_twist_residual, affine.last_base_map @ result.joint_velocity + affine.last_bias, atol=1e-12)
    assert np.linalg.norm(affine.last_physical_task_command[:3]) <= affine.fpmfc_config.linear_speed_limit_m_s + 1e-12
    assert np.linalg.norm(affine.last_physical_task_command[3:]) <= affine.fpmfc_config.angular_speed_limit_rad_s + 1e-12
    assert result.hierarchy_linear_degradation_m_s <= affine.fpmfc_config.level1_position_tolerance_m_s + 1e-5
    assert result.hierarchy_angular_degradation_rad_s <= affine.fpmfc_config.level1_angular_tolerance_rad_s + 1e-5
    assert result.minimum_constraint_slack >= -affine.config.feasibility_tolerance - 1e-12


def test_existing_collision_pairs_exclude_intentional_contact_interface():
    _spec, model, _data, legacy, _affine, _target = _fixture()
    pad = geom_id(model, "gripper_contact_pad")
    plate = geom_id(model, "target_contact_plate")
    assert all(pad not in (pair.geom_a, pair.geom_b) and plate not in (pair.geom_a, pair.geom_b) for pair in legacy.pairs)

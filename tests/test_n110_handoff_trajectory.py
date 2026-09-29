"""Independent boundary and finite-difference checks for contact-only paths."""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from v6_mujoco.fpmfc.handoff_trajectory import ContactHandoffTrajectory, contact_flange_boundary
from v6_mujoco.fpmfc.target import target_from_config
from v6_mujoco.fpmfc.trajectory import PoseShapeTrajectory
from v6_mujoco.fpmfc.config import load_fpmfc_config
from v6_mujoco.model import PROJECT_ROOT


def _paths():
    config = load_fpmfc_config(PROJECT_ROOT / "configs/fpmfc_paper_planning45_effective.yaml")
    target = target_from_config(config)
    boundary = contact_flange_boundary(target, 8.0, tool_face_recess_m=0.0002)
    p0 = np.array([0.3, -0.2, 0.5])
    r0 = Rotation.from_euler("xyz", [0.1, -0.2, 0.3]).as_matrix()
    common = dict(
        initial_position_world_m=p0,
        final_position_world_m=boundary.position_world_m,
        initial_rotation_world=r0,
        final_rotation_world=boundary.rotation_world,
        initial_arm_angle_rad=-0.5,
        final_arm_angle_rad=-1.492877399060744,
        duration_s=8.0,
    )
    zero = ContactHandoffTrajectory(
        **common, terminal_linear_velocity_world_m_s=np.zeros(3),
        terminal_linear_acceleration_world_m_s2=np.zeros(3),
        terminal_angular_velocity_world_rad_s=np.zeros(3),
        terminal_angular_acceleration_world_rad_s2=np.zeros(3),
    )
    moving = ContactHandoffTrajectory(
        **common,
        terminal_linear_velocity_world_m_s=boundary.linear_velocity_world_m_s,
        terminal_linear_acceleration_world_m_s2=boundary.linear_acceleration_world_m_s2,
        terminal_angular_velocity_world_rad_s=boundary.angular_velocity_world_rad_s,
        terminal_angular_acceleration_world_rad_s2=boundary.angular_acceleration_world_rad_s2,
    )
    return target, boundary, zero, moving


def test_zero_mode_equals_legacy_quintic_when_boundaries_match():
    _target, _boundary, zero, _moving = _paths()
    legacy = PoseShapeTrajectory(
        zero.initial_position_world_m, zero.final_position_world_m,
        zero.initial_rotation_world, zero.final_rotation_world,
        zero.initial_arm_angle_rad, zero.final_arm_angle_rad, zero.duration_s,
    )
    for t in (0.0, 0.1, 3.7, 7.9, 8.0):
        a, b = zero.sample(t), legacy.sample(t)
        for field in (
            "position_world_m", "linear_velocity_world_m_s", "linear_acceleration_world_m_s2",
            "rotation_world", "angular_velocity_world_rad_s", "angular_acceleration_world_rad_s2",
        ):
            np.testing.assert_allclose(getattr(a, field), getattr(b, field), atol=2e-15, rtol=0)
        assert abs(a.arm_angle_rad-b.arm_angle_rad) < 1e-14


def test_terminal_contact_reference_and_tool_face_twist():
    target, boundary, zero, moving = _paths()
    sample = target.sample(8.0)
    offset = boundary.contact_normal_world*boundary.tool_face_recess_m
    np.testing.assert_allclose(boundary.position_world_m-offset, sample.grasp_position_world_m, atol=1e-14)
    np.testing.assert_allclose(
        boundary.linear_velocity_world_m_s,
        sample.grasp_linear_velocity_world_m_s+np.cross(sample.grasp_angular_velocity_world_rad_s, offset),
        atol=1e-14,
    )
    for path in (zero, moving):
        endpoint = path.sample(8.0)
        np.testing.assert_allclose(endpoint.position_world_m, boundary.position_world_m, atol=1e-14)
        np.testing.assert_allclose(endpoint.rotation_world, boundary.rotation_world, atol=1e-14)
    endpoint = moving.sample(8.0)
    np.testing.assert_allclose(endpoint.linear_velocity_world_m_s, boundary.linear_velocity_world_m_s, atol=1e-14)
    np.testing.assert_allclose(endpoint.linear_acceleration_world_m_s2, boundary.linear_acceleration_world_m_s2, atol=1e-14)
    np.testing.assert_allclose(endpoint.angular_velocity_world_rad_s, boundary.angular_velocity_world_rad_s, atol=1e-14)
    np.testing.assert_allclose(endpoint.angular_acceleration_world_rad_s2, boundary.angular_acceleration_world_rad_s2, atol=1e-14)
    assert np.linalg.norm(zero.sample(8.0).linear_velocity_world_m_s) == 0.0
    assert np.linalg.norm(zero.sample(8.0).angular_velocity_world_rad_s) == 0.0


def test_so3_velocity_and_acceleration_match_independent_finite_differences():
    _target, _boundary, _zero, moving = _paths()
    h = 1e-4
    for t in (0.1, 1.0, 4.0, 7.5, 7.999):
        before, center, after = (moving.sample(x) for x in (t-h, t, t+h))
        linear_fd = (after.position_world_m-before.position_world_m)/(2*h)
        linear_acc_fd = (after.linear_velocity_world_m_s-before.linear_velocity_world_m_s)/(2*h)
        rotation_fd = Rotation.from_matrix(after.rotation_world @ before.rotation_world.T).as_rotvec()/(2*h)
        angular_acc_fd = (after.angular_velocity_world_rad_s-before.angular_velocity_world_rad_s)/(2*h)
        np.testing.assert_allclose(center.linear_velocity_world_m_s, linear_fd, atol=2e-8, rtol=0)
        np.testing.assert_allclose(center.linear_acceleration_world_m_s2, linear_acc_fd, atol=2e-8, rtol=0)
        np.testing.assert_allclose(center.angular_velocity_world_rad_s, rotation_fd, atol=2e-8, rtol=0)
        np.testing.assert_allclose(center.angular_acceleration_world_rad_s2, angular_acc_fd, atol=2e-8, rtol=0)


def test_trajectory_refuses_hidden_post_handoff_hold():
    _target, _boundary, _zero, moving = _paths()
    with pytest.raises(ValueError, match="only through T"):
        moving.sample(8.001)

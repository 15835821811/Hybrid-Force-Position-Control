"""Contact-only C2 pose/shape trajectory with a physical terminal twist.

The rotation is a zero-end-rate geodesic multiplied by a scalar rotation
about the terminal body angular-velocity axis. This gives exact spatial
angular velocity and acceleration without equating a rotvec derivative to a
physical angular velocity. Sampling beyond the handoff is deliberately
rejected: the moving physical target owns the contact-stage reference.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from .target import PrescribedTumblingTarget
from .trajectory import PoseShapeSample, quintic_time_scaling


def _vector(value: np.ndarray, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite three-vector")
    return result.copy()


def _rotation(value: np.ndarray, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (3, 3) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite 3x3 matrix")
    if np.linalg.det(result) < 0.999999 or np.linalg.norm(result.T @ result-np.eye(3)) > 1e-8:
        raise ValueError(f"{name} must be a proper rotation")
    return result.copy()


def _hermite_zero_start(
    initial: np.ndarray | float,
    final: np.ndarray | float,
    terminal_velocity: np.ndarray | float,
    terminal_acceleration: np.ndarray | float,
    time_s: float,
    duration_s: float,
) -> tuple[np.ndarray | float, np.ndarray | float, np.ndarray | float]:
    """Quintic with zero initial velocity/acceleration and specified final jet."""
    u = time_s / duration_s
    delta = final-initial
    velocity_scaled = terminal_velocity*duration_s
    acceleration_scaled = terminal_acceleration*duration_s**2
    c3 = 10*delta-4*velocity_scaled+0.5*acceleration_scaled
    c4 = -15*delta+7*velocity_scaled-acceleration_scaled
    c5 = 6*delta-3*velocity_scaled+0.5*acceleration_scaled
    position = initial+c3*u**3+c4*u**4+c5*u**5
    velocity = (3*c3*u**2+4*c4*u**3+5*c5*u**4)/duration_s
    acceleration = (6*c3*u+12*c4*u**2+20*c5*u**3)/duration_s**2
    return position, velocity, acceleration


@dataclass(frozen=True)
class FlangeBoundary:
    """World-frame contact-stage flange reference at absolute target time T."""

    target_absolute_time_s: float
    position_world_m: np.ndarray
    rotation_world: np.ndarray
    linear_velocity_world_m_s: np.ndarray
    linear_acceleration_world_m_s2: np.ndarray
    angular_velocity_world_rad_s: np.ndarray
    angular_acceleration_world_rad_s2: np.ndarray
    target_grasp_position_world_m: np.ndarray
    target_grasp_linear_velocity_world_m_s: np.ndarray
    contact_normal_world: np.ndarray
    tool_face_recess_m: float


def contact_flange_boundary(
    target: PrescribedTumblingTarget,
    absolute_time_s: float,
    *,
    tool_face_recess_m: float,
) -> FlangeBoundary:
    """Use the actual pad-face recess exactly once in the flange reference."""
    sample = target.sample(absolute_time_s)
    recess = float(tool_face_recess_m)
    if not np.isfinite(recess) or recess < 0:
        raise ValueError("tool-face recess must be finite and nonnegative")
    normal = sample.grasp_rotation_world[:, 2]
    offset = normal*recess
    omega = sample.grasp_angular_velocity_world_rad_s
    center_to_grasp = sample.grasp_position_world_m-sample.center_position_world_m
    # PrescribedTumblingTarget has constant center velocity and a principal-
    # axis constant body spin, hence world angular acceleration is zero.
    grasp_acceleration = np.cross(omega, np.cross(omega, center_to_grasp))
    return FlangeBoundary(
        target_absolute_time_s=float(absolute_time_s),
        position_world_m=sample.grasp_position_world_m+offset,
        rotation_world=sample.grasp_rotation_world.copy(),
        linear_velocity_world_m_s=sample.grasp_linear_velocity_world_m_s+np.cross(omega, offset),
        linear_acceleration_world_m_s2=grasp_acceleration+np.cross(omega, np.cross(omega, offset)),
        angular_velocity_world_rad_s=omega.copy(),
        angular_acceleration_world_rad_s2=np.zeros(3),
        target_grasp_position_world_m=sample.grasp_position_world_m.copy(),
        target_grasp_linear_velocity_world_m_s=sample.grasp_linear_velocity_world_m_s.copy(),
        contact_normal_world=normal.copy(),
        tool_face_recess_m=recess,
    )


@dataclass(frozen=True)
class ContactHandoffTrajectory:
    initial_position_world_m: np.ndarray
    final_position_world_m: np.ndarray
    initial_rotation_world: np.ndarray
    final_rotation_world: np.ndarray
    initial_arm_angle_rad: float
    final_arm_angle_rad: float
    duration_s: float
    terminal_linear_velocity_world_m_s: np.ndarray
    terminal_linear_acceleration_world_m_s2: np.ndarray
    terminal_angular_velocity_world_rad_s: np.ndarray
    terminal_angular_acceleration_world_rad_s2: np.ndarray

    def __post_init__(self) -> None:
        for name in (
            "initial_position_world_m", "final_position_world_m",
            "terminal_linear_velocity_world_m_s", "terminal_linear_acceleration_world_m_s2",
            "terminal_angular_velocity_world_rad_s", "terminal_angular_acceleration_world_rad_s2",
        ):
            object.__setattr__(self, name, _vector(getattr(self, name), name))
        for name in ("initial_rotation_world", "final_rotation_world"):
            object.__setattr__(self, name, _rotation(getattr(self, name), name))
        if not np.isfinite(self.duration_s) or self.duration_s <= 0:
            raise ValueError("duration must be finite and positive")
        if not np.isfinite(self.initial_arm_angle_rad) or not np.isfinite(self.final_arm_angle_rad):
            raise ValueError("arm angles must be finite")
        angular_velocity = self.terminal_angular_velocity_world_rad_s
        angular_acceleration = self.terminal_angular_acceleration_world_rad_s2
        speed = float(np.linalg.norm(angular_velocity))
        if speed < 1e-14 and np.linalg.norm(angular_acceleration) > 1e-14:
            raise ValueError("nonzero angular acceleration with zero angular velocity is unsupported")
        if speed >= 1e-14 and np.linalg.norm(np.cross(angular_velocity, angular_acceleration)) > 1e-10:
            raise ValueError("terminal angular acceleration must share the spin axis")

    def sample(self, time_s: float) -> PoseShapeSample:
        raw_time = float(time_s)
        if not np.isfinite(raw_time) or raw_time < -1e-9 or raw_time > self.duration_s+1e-9:
            raise ValueError("handoff trajectory is defined only through T; contact target takes over afterward")
        t = float(np.clip(raw_time, 0.0, self.duration_s))
        position, linear_velocity, linear_acceleration = _hermite_zero_start(
            self.initial_position_world_m, self.final_position_world_m,
            self.terminal_linear_velocity_world_m_s,
            self.terminal_linear_acceleration_world_m_s2, t, self.duration_s,
        )
        scalar, scalar_velocity, scalar_acceleration = quintic_time_scaling(t, self.duration_s)
        relative_rotation = Rotation.from_matrix(
            self.initial_rotation_world.T @ self.final_rotation_world
        ).as_rotvec()
        base_rotation = self.initial_rotation_world @ Rotation.from_rotvec(
            relative_rotation*scalar
        ).as_matrix()
        base_axis_world = self.initial_rotation_world @ relative_rotation
        base_omega = base_axis_world*scalar_velocity
        base_alpha = base_axis_world*scalar_acceleration
        terminal_omega = self.terminal_angular_velocity_world_rad_s
        speed = float(np.linalg.norm(terminal_omega))
        if speed < 1e-14:
            axis_body = np.zeros(3)
            theta = theta_velocity = theta_acceleration = 0.0
        else:
            axis_body = self.final_rotation_world.T @ terminal_omega/speed
            terminal_alpha = float(axis_body @ (self.final_rotation_world.T @ self.terminal_angular_acceleration_world_rad_s2))
            theta, theta_velocity, theta_acceleration = _hermite_zero_start(
                0.0, 0.0, speed, terminal_alpha, t, self.duration_s,
            )
        rotation = base_rotation @ Rotation.from_rotvec(axis_body*theta).as_matrix()
        moving_axis_world = base_rotation @ axis_body
        angular_velocity = base_omega+moving_axis_world*theta_velocity
        angular_acceleration = (
            base_alpha+np.cross(base_omega, moving_axis_world*theta_velocity)
            +moving_axis_world*theta_acceleration
        )
        arm_delta = self.final_arm_angle_rad-self.initial_arm_angle_rad
        return PoseShapeSample(
            time_s=raw_time,
            position_world_m=position,
            linear_velocity_world_m_s=linear_velocity,
            linear_acceleration_world_m_s2=linear_acceleration,
            rotation_world=rotation,
            angular_velocity_world_rad_s=angular_velocity,
            angular_acceleration_world_rad_s2=angular_acceleration,
            arm_angle_rad=float(self.initial_arm_angle_rad+scalar*arm_delta),
            arm_angle_velocity_rad_s=float(scalar_velocity*arm_delta),
            arm_angle_acceleration_rad_s2=float(scalar_acceleration*arm_delta),
        )

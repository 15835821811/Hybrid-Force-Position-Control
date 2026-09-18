"""C2 quintic SE(3) and arm-shape trajectories for capture approach."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation


def quintic_time_scaling(time_s: float, duration_s: float) -> tuple[float, float, float]:
    """Return position, velocity, and acceleration time scalars."""

    duration = float(duration_s)
    if not np.isfinite(duration) or duration <= 0.0:
        raise ValueError("duration must be finite and positive")
    time_value = float(time_s)
    if not np.isfinite(time_value):
        raise ValueError("time must be finite")
    if time_value <= 0.0:
        return 0.0, 0.0, 0.0
    if time_value >= duration:
        return 1.0, 0.0, 0.0
    tau = time_value / duration
    scalar = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
    scalar_velocity = (30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4) / duration
    scalar_acceleration = (60.0 * tau - 180.0 * tau**2 + 120.0 * tau**3) / duration**2
    return scalar, scalar_velocity, scalar_acceleration


@dataclass(frozen=True)
class PoseShapeSample:
    time_s: float
    position_world_m: np.ndarray
    linear_velocity_world_m_s: np.ndarray
    linear_acceleration_world_m_s2: np.ndarray
    rotation_world: np.ndarray
    angular_velocity_world_rad_s: np.ndarray
    angular_acceleration_world_rad_s2: np.ndarray
    arm_angle_rad: float
    arm_angle_velocity_rad_s: float
    arm_angle_acceleration_rad_s2: float


@dataclass(frozen=True)
class PoseShapeTrajectory:
    initial_position_world_m: np.ndarray
    final_position_world_m: np.ndarray
    initial_rotation_world: np.ndarray
    final_rotation_world: np.ndarray
    initial_arm_angle_rad: float
    final_arm_angle_rad: float
    duration_s: float

    def __post_init__(self) -> None:
        for name in ("initial_position_world_m", "final_position_world_m"):
            vector = np.asarray(getattr(self, name), dtype=np.float64)
            if vector.shape != (3,) or not np.all(np.isfinite(vector)):
                raise ValueError(f"{name} must be a finite three-vector")
            object.__setattr__(self, name, vector)
        for name in ("initial_rotation_world", "final_rotation_world"):
            matrix = np.asarray(getattr(self, name), dtype=np.float64)
            if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
                raise ValueError(f"{name} must be a finite 3x3 matrix")
            if np.linalg.det(matrix) < 0.999999 or np.linalg.norm(matrix.T @ matrix - np.eye(3)) > 1e-8:
                raise ValueError(f"{name} must be a proper rotation matrix")
            object.__setattr__(self, name, matrix)
        if not np.isfinite(self.duration_s) or self.duration_s <= 0.0:
            raise ValueError("duration must be finite and positive")
        if not np.isfinite(self.initial_arm_angle_rad) or not np.isfinite(self.final_arm_angle_rad):
            raise ValueError("arm angles must be finite")

    def sample(self, time_s: float) -> PoseShapeSample:
        scalar, scalar_velocity, scalar_acceleration = quintic_time_scaling(time_s, self.duration_s)
        position_delta = self.final_position_world_m - self.initial_position_world_m
        relative_rotvec_initial = Rotation.from_matrix(
            self.initial_rotation_world.T @ self.final_rotation_world
        ).as_rotvec()
        rotation = self.initial_rotation_world @ Rotation.from_rotvec(
            scalar * relative_rotvec_initial
        ).as_matrix()
        rotation_axis_world = self.initial_rotation_world @ relative_rotvec_initial
        arm_delta = self.final_arm_angle_rad - self.initial_arm_angle_rad
        return PoseShapeSample(
            time_s=float(time_s),
            position_world_m=self.initial_position_world_m + scalar * position_delta,
            linear_velocity_world_m_s=scalar_velocity * position_delta,
            linear_acceleration_world_m_s2=scalar_acceleration * position_delta,
            rotation_world=rotation,
            angular_velocity_world_rad_s=scalar_velocity * rotation_axis_world,
            angular_acceleration_world_rad_s2=scalar_acceleration * rotation_axis_world,
            arm_angle_rad=float(self.initial_arm_angle_rad + scalar * arm_delta),
            arm_angle_velocity_rad_s=float(scalar_velocity * arm_delta),
            arm_angle_acceleration_rad_s2=float(scalar_acceleration * arm_delta),
        )

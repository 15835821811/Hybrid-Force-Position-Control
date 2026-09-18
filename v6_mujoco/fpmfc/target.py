"""Prescribed tumbling-target kinematics used by the paper-faithful A stage."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation


TARGET_BODY_NAME = "tumbling_target"
TARGET_GEOM_NAME = "tumbling_target_geom"


def _vector3(value: Any, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must be a finite three-vector")
    return array


@dataclass(frozen=True)
class TargetSample:
    """Target-center and grasp-frame state at one instant."""

    time_s: float
    center_position_world_m: np.ndarray
    center_rotation_world: np.ndarray
    center_linear_velocity_world_m_s: np.ndarray
    angular_velocity_world_rad_s: np.ndarray
    grasp_position_world_m: np.ndarray
    grasp_rotation_world: np.ndarray
    grasp_linear_velocity_world_m_s: np.ndarray
    grasp_angular_velocity_world_rad_s: np.ndarray


@dataclass(frozen=True)
class PrescribedTumblingTarget:
    """Rigid target with constant center velocity and principal-axis body spin.

    The A-stage reproduction uses a kinematic target because the source paper's
    reported simulation stops before physical contact.  Constant body angular
    velocity is torque-free when it is aligned with a principal inertia axis,
    which is the configured 5 deg/s z-axis case.
    """

    geometry: str
    side_length_m: float
    initial_center_position_world_m: np.ndarray
    center_linear_velocity_world_m_s: np.ndarray
    initial_rotation_world: np.ndarray
    angular_velocity_body_rad_s: np.ndarray
    grasp_offset_body_m: np.ndarray
    grasp_rotation_body: np.ndarray

    def __post_init__(self) -> None:
        if self.geometry != "cube":
            raise ValueError("the prescribed target geometry must be 'cube'")
        side_length = float(self.side_length_m)
        if not np.isfinite(side_length) or side_length <= 0.0:
            raise ValueError("side_length_m must be finite and positive")
        object.__setattr__(self, "side_length_m", side_length)
        vector_fields = (
            "initial_center_position_world_m",
            "center_linear_velocity_world_m_s",
            "angular_velocity_body_rad_s",
            "grasp_offset_body_m",
        )
        for name in vector_fields:
            object.__setattr__(self, name, _vector3(getattr(self, name), name))
        for name in ("initial_rotation_world", "grasp_rotation_body"):
            matrix = np.asarray(getattr(self, name), dtype=np.float64)
            if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
                raise ValueError(f"{name} must be a finite 3x3 matrix")
            if np.linalg.det(matrix) < 0.999999 or np.linalg.norm(matrix.T @ matrix - np.eye(3)) > 1e-8:
                raise ValueError(f"{name} must be a proper rotation matrix")
            object.__setattr__(self, name, matrix)

    def sample(self, time_s: float) -> TargetSample:
        time_value = float(time_s)
        if not np.isfinite(time_value):
            raise ValueError("time must be finite")
        center_position = (
            self.initial_center_position_world_m
            + time_value * self.center_linear_velocity_world_m_s
        )
        body_increment = Rotation.from_rotvec(self.angular_velocity_body_rad_s * time_value).as_matrix()
        rotation_world = self.initial_rotation_world @ body_increment
        angular_velocity_world = rotation_world @ self.angular_velocity_body_rad_s
        grasp_offset_world = rotation_world @ self.grasp_offset_body_m
        grasp_position = center_position + grasp_offset_world
        grasp_linear_velocity = (
            self.center_linear_velocity_world_m_s
            + np.cross(angular_velocity_world, grasp_offset_world)
        )
        return TargetSample(
            time_s=time_value,
            center_position_world_m=center_position.copy(),
            center_rotation_world=rotation_world,
            center_linear_velocity_world_m_s=self.center_linear_velocity_world_m_s.copy(),
            angular_velocity_world_rad_s=angular_velocity_world,
            grasp_position_world_m=grasp_position,
            grasp_rotation_world=rotation_world @ self.grasp_rotation_body,
            grasp_linear_velocity_world_m_s=grasp_linear_velocity,
            grasp_angular_velocity_world_rad_s=angular_velocity_world.copy(),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "mode": "prescribed_principal_axis_tumbling",
            "geometry": self.geometry,
            "side_length_m": self.side_length_m,
            "initial_center_position_world_m": self.initial_center_position_world_m.tolist(),
            "center_linear_velocity_world_m_s": self.center_linear_velocity_world_m_s.tolist(),
            "initial_rotation_world": self.initial_rotation_world.tolist(),
            "angular_velocity_body_rad_s": self.angular_velocity_body_rad_s.tolist(),
            "grasp_offset_body_m": self.grasp_offset_body_m.tolist(),
            "grasp_rotation_body": self.grasp_rotation_body.tolist(),
        }


def target_from_config(config: Mapping[str, Any]) -> PrescribedTumblingTarget:
    target = config["target"]
    return PrescribedTumblingTarget(
        geometry=str(target["geometry"]),
        side_length_m=float(target["side_length_m"]),
        initial_center_position_world_m=np.asarray(target["center_position_world_m"], dtype=np.float64),
        center_linear_velocity_world_m_s=np.asarray(
            target["center_linear_velocity_world_m_s"], dtype=np.float64
        ),
        initial_rotation_world=Rotation.from_euler(
            "xyz", target["initial_rpy_xyz_deg"], degrees=True
        ).as_matrix(),
        angular_velocity_body_rad_s=np.deg2rad(
            np.asarray(target["angular_velocity_body_deg_s"], dtype=np.float64)
        ),
        grasp_offset_body_m=np.asarray(target["grasp_offset_body_m"], dtype=np.float64),
        grasp_rotation_body=Rotation.from_euler(
            "xyz", target["grasp_frame_rpy_xyz_deg"], degrees=True
        ).as_matrix(),
    )


def sync_mujoco_target(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    target: PrescribedTumblingTarget,
    sample: TargetSample,
) -> int:
    """Apply configured cube size and sampled center pose to the MuJoCo mocap body."""

    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, TARGET_BODY_NAME)
    geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, TARGET_GEOM_NAME)
    if body < 0 or geom < 0:
        raise ValueError("MuJoCo tumbling-target body or geometry is missing")
    mocap_id = int(model.body_mocapid[body])
    if mocap_id < 0:
        raise ValueError("MuJoCo tumbling-target body must be a mocap body")
    model.geom_size[geom, :3] = 0.5 * target.side_length_m
    data.mocap_pos[mocap_id] = sample.center_position_world_m
    mujoco.mju_mat2Quat(
        data.mocap_quat[mocap_id], sample.center_rotation_world.reshape(-1)
    )
    return mocap_id

"""Free-floating reaction and generalized kinematics from MuJoCo mass matrices."""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from ..model import FlexivModelSpec, site_id


@dataclass(frozen=True)
class ReactionMapResult:
    base_from_joint_velocity: np.ndarray
    full_velocity_from_joint_velocity: np.ndarray
    momentum_residual_norm: float


class FreeFloatingKinematics:
    """Paper equations (10)-(14) evaluated from the engine mass matrix."""

    def __init__(self, spec: FlexivModelSpec, model: mujoco.MjModel) -> None:
        self.spec = spec
        self.model = model
        self.qpos_ids, self.dof_ids = spec.joint_addresses(model)
        _, self.base_dof_slice = spec.base_slices(model)
        self._mass = np.zeros((model.nv, model.nv), dtype=np.float64)
        self._jacobian_position = np.zeros((3, model.nv), dtype=np.float64)
        self._jacobian_rotation = np.zeros((3, model.nv), dtype=np.float64)

    def reaction_map(self, data: mujoco.MjData) -> ReactionMapResult:
        mujoco.mj_forward(self.model, data)
        mujoco.mj_fullM(self.model, self._mass, data.qM)
        base = np.arange(self.base_dof_slice.start, self.base_dof_slice.stop)
        mass_bb = self._mass[np.ix_(base, base)]
        mass_bq = self._mass[np.ix_(base, self.dof_ids)]
        base_map = -np.linalg.solve(mass_bb, mass_bq)
        full_map = np.zeros((self.model.nv, len(self.dof_ids)), dtype=np.float64)
        full_map[base, :] = base_map
        full_map[self.dof_ids, :] = np.eye(len(self.dof_ids))
        residual = float(np.linalg.norm(mass_bb @ base_map + mass_bq))
        return ReactionMapResult(base_map, full_map, residual)

    def generalized_site_jacobians(
        self, data: mujoco.MjData, site: int | str = "flange_site"
    ) -> tuple[np.ndarray, np.ndarray, ReactionMapResult]:
        site_index = site_id(self.model, site) if isinstance(site, str) else int(site)
        reaction = self.reaction_map(data)
        self._jacobian_position.fill(0.0)
        self._jacobian_rotation.fill(0.0)
        mujoco.mj_jacSite(
            self.model,
            data,
            self._jacobian_position,
            self._jacobian_rotation,
            site_index,
        )
        return (
            self._jacobian_position @ reaction.full_velocity_from_joint_velocity,
            self._jacobian_rotation @ reaction.full_velocity_from_joint_velocity,
            reaction,
        )

    def base_twist(self, data: mujoco.MjData, joint_velocity: np.ndarray) -> np.ndarray:
        joint_velocity_value = np.asarray(joint_velocity, dtype=np.float64)
        if joint_velocity_value.shape != (len(self.dof_ids),):
            raise ValueError("joint velocity has the wrong shape")
        return self.reaction_map(data).base_from_joint_velocity @ joint_velocity_value


def rotation_delta_world(initial_rotation: np.ndarray, final_rotation: np.ndarray) -> np.ndarray:
    """World-frame rotation vector taking initial orientation to final orientation."""

    initial = np.asarray(initial_rotation, dtype=np.float64).reshape(3, 3)
    final = np.asarray(final_rotation, dtype=np.float64).reshape(3, 3)
    return Rotation.from_matrix(final @ initial.T).as_rotvec()


def rotation_distance_rad(reference_rotation: np.ndarray, current_rotation: np.ndarray) -> float:
    reference = np.asarray(reference_rotation, dtype=np.float64).reshape(3, 3)
    current = np.asarray(current_rotation, dtype=np.float64).reshape(3, 3)
    return float(Rotation.from_matrix(reference.T @ current).magnitude())

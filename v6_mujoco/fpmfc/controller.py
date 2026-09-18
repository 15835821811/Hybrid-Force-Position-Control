"""Strict two-level velocity HQP for the paper's position-shape hierarchy."""

from __future__ import annotations

import time
from dataclasses import dataclass

import mujoco
import numpy as np

from ..collision import CollisionPair
from ..hierarchical_qp import (
    HierarchicalVelocityQP,
    QPConfig,
    rotation_error_vector_world,
    smooth_saturate_norm,
)
from ..model import FlexivModelSpec
from .shape import ArmShapeKinematics, _wrap_to_pi


def _clip_norm(value: np.ndarray, limit: float) -> np.ndarray:
    norm = float(np.linalg.norm(value))
    return value if norm <= limit else value * (limit / max(norm, 1e-12))


@dataclass(frozen=True)
class FPMFCControllerConfig:
    position_gain: float = 5.0
    orientation_gain: float = 8.0
    shape_gain: float = 2.0
    position_weight: float = 500.0
    orientation_weight: float = 4000.0
    shape_weight: float = 1.0
    base_reaction_weight: float = 1.0
    smoothness_weight: float = 1e-3
    primary_regularization: float = 1e-7
    level1_position_tolerance_m_s: float = 1e-4
    level1_angular_tolerance_rad_s: float = 1e-4
    linear_speed_limit_m_s: float = 0.24
    angular_speed_limit_rad_s: float = 0.45
    angular_saturation_transition_ratio: float = 0.90

    def validate(self) -> None:
        positive = (
            self.position_gain,
            self.orientation_gain,
            self.shape_gain,
            self.position_weight,
            self.orientation_weight,
            self.smoothness_weight,
            self.primary_regularization,
            self.level1_position_tolerance_m_s,
            self.level1_angular_tolerance_rad_s,
            self.linear_speed_limit_m_s,
            self.angular_speed_limit_rad_s,
        )
        if any(value <= 0.0 or not np.isfinite(value) for value in positive):
            raise ValueError("positive FPMFC controller parameters must be finite")
        if self.shape_weight < 0.0 or self.base_reaction_weight < 0.0:
            raise ValueError("secondary objective weights cannot be negative")
        if not 0.0 < self.angular_saturation_transition_ratio < 1.0:
            raise ValueError("angular saturation transition ratio must be in (0, 1)")


@dataclass(frozen=True)
class FPMFCResult:
    joint_velocity: np.ndarray
    success: bool
    primary_feasible: bool
    secondary_feasible: bool
    primary_status: str
    secondary_status: str
    primary_iterations: int
    secondary_iterations: int
    full_latency_s: float
    position_error_m: float
    orientation_error_rad: float
    primary_linear_velocity_residual_m_s: float
    primary_angular_velocity_residual_rad_s: float
    hierarchy_linear_degradation_m_s: float
    hierarchy_angular_degradation_rad_s: float
    arm_angle_rad: float
    arm_angle_error_rad: float
    arm_angle_velocity_residual_rad_s: float
    base_twist_residual: np.ndarray
    base_twist_residual_norm: float
    momentum_map_residual_norm: float
    minimum_queried_clearance_m: float
    active_clearance_constraints: int
    minimum_constraint_slack: float
    primary_minimum_constraint_slack: float


class FPMFCHQP(HierarchicalVelocityQP):
    """Lock the end-effector optimum before optimizing shape and base reaction."""

    def __init__(
        self,
        spec: FlexivModelSpec,
        model: mujoco.MjModel,
        pairs: tuple[CollisionPair, ...],
        shape: ArmShapeKinematics,
        *,
        controller_config: FPMFCControllerConfig = FPMFCControllerConfig(),
        constraint_config: QPConfig = QPConfig(),
    ) -> None:
        controller_config.validate()
        super().__init__(spec, model, pairs, constraint_config)
        self.fpmfc_config = controller_config
        self.shape = shape

    def _solve_scalar_nullspace_secondary(
        self,
        primary: np.ndarray,
        task_jacobian: np.ndarray,
        hessian: np.ndarray,
        linear: np.ndarray,
        constraint_matrix: np.ndarray,
        constraint_lower: np.ndarray,
        constraint_upper: np.ndarray,
    ) -> tuple[np.ndarray, bool, str, int]:
        """Solve the one redundant Flexiv coordinate exactly in closed form."""

        _u, singular_values, vh = np.linalg.svd(task_jacobian, full_matrices=True)
        rank_tolerance = max(task_jacobian.shape) * max(float(singular_values[0]), 1.0) * 1e-10
        rank = int(np.sum(singular_values > rank_tolerance))
        if task_jacobian.shape[1] - rank != 1:
            return primary.copy(), False, "nullity_not_one", 0
        null_direction = vh[-1]
        lower_scalar = -np.inf
        upper_scalar = np.inf
        feasibility = float(self.config.feasibility_tolerance)
        for row, lower, upper in zip(
            constraint_matrix, constraint_lower, constraint_upper
        ):
            slope = float(row @ null_direction)
            offset = float(row @ primary)
            if abs(slope) <= 1e-12:
                if offset < lower - feasibility or offset > upper + feasibility:
                    return primary.copy(), False, "primary_constraint_infeasible", 0
                continue
            row_lower = (lower - feasibility - offset) / slope
            row_upper = (upper + feasibility - offset) / slope
            if row_lower > row_upper:
                row_lower, row_upper = row_upper, row_lower
            lower_scalar = max(lower_scalar, row_lower)
            upper_scalar = min(upper_scalar, row_upper)
        if lower_scalar > upper_scalar:
            return primary.copy(), False, "empty_nullspace_interval", 0
        curvature = float(null_direction @ hessian @ null_direction)
        if curvature <= 1e-14:
            scalar = 0.0
        else:
            scalar = -float(null_direction @ (hessian @ primary + linear)) / curvature
        scalar = float(np.clip(scalar, lower_scalar, upper_scalar))
        candidate = primary + scalar * null_direction
        slack = np.minimum(
            constraint_matrix @ candidate - constraint_lower,
            constraint_upper - constraint_matrix @ candidate,
        )
        feasible = bool(float(np.min(slack)) >= -feasibility - 1e-12)
        return candidate, feasible, "solved_nullspace" if feasible else "nullspace_infeasible", 1

    def solve_fpmfc(
        self,
        data: mujoco.MjData,
        *,
        target_position: np.ndarray,
        target_velocity: np.ndarray,
        target_rotation: np.ndarray,
        target_angular_velocity: np.ndarray,
        target_arm_angle_rad: float,
        target_arm_angle_velocity_rad_s: float,
        desired_base_twist: np.ndarray | None = None,
    ) -> FPMFCResult:
        started = time.perf_counter()
        mujoco.mj_forward(self.model, data)
        cfg = self.fpmfc_config
        mapping, momentum_map_residual = self.reaction_velocity_map(data)
        jacobian_position, jacobian_rotation = self.task_jacobians(data, mapping)
        base_map = mapping[self.base_dof_slice, :]

        position = np.asarray(data.site_xpos[self.flange_site]).copy()
        rotation = np.asarray(data.site_xmat[self.flange_site]).reshape(3, 3).copy()
        position_error = np.asarray(target_position, dtype=np.float64) - position
        orientation_error = rotation_error_vector_world(target_rotation, rotation)
        velocity_command = _clip_norm(
            np.asarray(target_velocity, dtype=np.float64) + cfg.position_gain * position_error,
            cfg.linear_speed_limit_m_s,
        )
        angular_command = smooth_saturate_norm(
            np.asarray(target_angular_velocity, dtype=np.float64)
            + cfg.orientation_gain * orientation_error,
            cfg.angular_speed_limit_rad_s,
            cfg.angular_saturation_transition_ratio,
        )
        task_jacobian = np.vstack((jacobian_position, jacobian_rotation))
        task_command = np.concatenate((velocity_command, angular_command))

        q = np.asarray(data.qpos[self.qpos_ids])
        clearance_matrix, clearance_lower, minimum_clearance = self._clearance_constraints(
            data, mapping
        )
        joint_lower, joint_upper = self._bounds(q)
        primary_matrix = np.vstack((clearance_matrix, np.eye(7)))
        primary_lower = np.concatenate((clearance_lower, joint_lower))
        primary_upper = np.concatenate((np.full(len(clearance_lower), np.inf), joint_upper))
        initial = np.clip(self.previous_velocity, joint_lower, joint_upper)

        primary_hessian = (
            cfg.position_weight * jacobian_position.T @ jacobian_position
            + cfg.orientation_weight * jacobian_rotation.T @ jacobian_rotation
            + cfg.primary_regularization * np.eye(7)
        )
        primary_linear = -(
            cfg.position_weight * jacobian_position.T @ velocity_command
            + cfg.orientation_weight * jacobian_rotation.T @ angular_command
        )
        primary_hessian = 0.5 * (primary_hessian + primary_hessian.T)
        primary, primary_success, primary_status, primary_iterations = self._solve_admm(
            primary_hessian,
            primary_linear,
            primary_matrix,
            primary_lower,
            primary_upper,
            initial,
        )
        primary_achieved = task_jacobian @ primary
        primary_slack = np.minimum(
            primary_matrix @ primary - primary_lower,
            primary_upper - primary_matrix @ primary,
        )
        primary_minimum_slack = float(np.min(primary_slack))

        shape_sample = self.shape.sample(data)
        if shape_sample.singular:
            primary_success = False
        shape_jacobian = self.shape.jacobian(data)
        shape_error = _wrap_to_pi(float(target_arm_angle_rad) - shape_sample.angle_rad)
        shape_command = float(target_arm_angle_velocity_rad_s) + cfg.shape_gain * shape_error
        base_command = (
            np.zeros(6, dtype=np.float64)
            if desired_base_twist is None
            else np.asarray(desired_base_twist, dtype=np.float64)
        )
        if base_command.shape != (6,):
            raise ValueError("desired base twist must have shape (6,)")

        secondary_hessian = (
            cfg.shape_weight * np.outer(shape_jacobian, shape_jacobian)
            + cfg.base_reaction_weight * base_map.T @ base_map
            + cfg.smoothness_weight * np.eye(7)
        )
        secondary_linear = -(
            cfg.shape_weight * shape_jacobian * shape_command
            + cfg.base_reaction_weight * base_map.T @ base_command
            + cfg.smoothness_weight * self.previous_velocity
        )
        lock_tolerance = np.asarray(
            [cfg.level1_position_tolerance_m_s] * 3
            + [cfg.level1_angular_tolerance_rad_s] * 3,
            dtype=np.float64,
        )
        secondary_matrix = np.vstack((primary_matrix, task_jacobian))
        secondary_lower = np.concatenate((primary_lower, primary_achieved - lock_tolerance))
        secondary_upper = np.concatenate((primary_upper, primary_achieved + lock_tolerance))
        secondary, secondary_success, secondary_status, secondary_iterations = (
            self._solve_scalar_nullspace_secondary(
                primary,
                task_jacobian,
                secondary_hessian,
                secondary_linear,
                primary_matrix,
                primary_lower,
                primary_upper,
            )
        )
        if secondary_status == "nullity_not_one":
            secondary, secondary_success, secondary_status, secondary_iterations = self._solve_admm(
                secondary_hessian,
                secondary_linear,
                secondary_matrix,
                secondary_lower,
                secondary_upper,
                primary,
            )
        candidate = secondary if secondary_success else primary
        success = bool(primary_success and secondary_success and not shape_sample.singular)
        achieved = task_jacobian @ candidate
        base_residual = base_map @ candidate - base_command
        all_slack = np.minimum(
            secondary_matrix @ candidate - secondary_lower,
            secondary_upper - secondary_matrix @ candidate,
        )
        minimum_slack = float(np.min(all_slack))
        success = bool(
            success
            and minimum_slack >= -self.config.feasibility_tolerance - 1e-12
        )
        if not primary_success:
            candidate = np.clip(self.previous_velocity, joint_lower, joint_upper)
            achieved = task_jacobian @ candidate
            base_residual = base_map @ candidate - base_command
        candidate = np.clip(candidate, joint_lower, joint_upper)
        achieved = task_jacobian @ candidate
        base_residual = base_map @ candidate - base_command
        self._commit_velocity(candidate)
        return FPMFCResult(
            joint_velocity=candidate,
            success=success,
            primary_feasible=primary_success,
            secondary_feasible=secondary_success,
            primary_status=primary_status,
            secondary_status=secondary_status,
            primary_iterations=primary_iterations,
            secondary_iterations=secondary_iterations,
            full_latency_s=time.perf_counter() - started,
            position_error_m=float(np.linalg.norm(position_error)),
            orientation_error_rad=float(np.linalg.norm(orientation_error)),
            primary_linear_velocity_residual_m_s=float(
                np.linalg.norm(primary_achieved[:3] - task_command[:3])
            ),
            primary_angular_velocity_residual_rad_s=float(
                np.linalg.norm(primary_achieved[3:] - task_command[3:])
            ),
            hierarchy_linear_degradation_m_s=float(
                np.linalg.norm(achieved[:3] - primary_achieved[:3])
            ),
            hierarchy_angular_degradation_rad_s=float(
                np.linalg.norm(achieved[3:] - primary_achieved[3:])
            ),
            arm_angle_rad=shape_sample.angle_rad,
            arm_angle_error_rad=shape_error,
            arm_angle_velocity_residual_rad_s=float(shape_jacobian @ candidate - shape_command),
            base_twist_residual=base_residual,
            base_twist_residual_norm=float(np.linalg.norm(base_residual)),
            momentum_map_residual_norm=momentum_map_residual,
            minimum_queried_clearance_m=float(minimum_clearance),
            active_clearance_constraints=len(clearance_lower),
            minimum_constraint_slack=minimum_slack,
            primary_minimum_constraint_slack=primary_minimum_slack,
        )

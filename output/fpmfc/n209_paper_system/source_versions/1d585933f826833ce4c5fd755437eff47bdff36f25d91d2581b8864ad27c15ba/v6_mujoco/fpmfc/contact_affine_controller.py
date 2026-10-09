"""Opt-in affine velocity HQP for contact, leaving the frozen controller intact."""

from __future__ import annotations

import copy
import time

import mujoco
import numpy as np

from ..hierarchical_qp import rotation_error_vector_world, smooth_saturate_norm
from .controller import FPMFCHQP, FPMFCResult, _clip_norm
from .shape import _wrap_to_pi


class AffineContactFPMFCHQP(FPMFCHQP):
    """Hold one measured base-velocity bias fixed throughout each task solve."""

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.last_bias = np.zeros(6)
        self.last_physical_task_command = np.zeros(6)
        self.last_debiased_task_rhs = np.zeros(6)
        self.last_predicted_task_velocity = np.zeros(6)
        self.last_primary_physical_residual = np.zeros(6)
        self.last_clearance_drift = np.zeros(0)
        self.last_task_jacobian = np.zeros((6, 7))
        self.last_base_map = np.zeros((6, 7))

    def _distance_bias_drift(self, data: mujoco.MjData, pair, base_bias: np.ndarray) -> float:
        """Directional distance rate for base bias and any moving target pair."""
        full_bias = np.zeros(self.model.nv)
        full_bias[self.base_dof_slice] = base_bias
        target_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "target_free_joint")
        if target_joint >= 0:
            target_dof = int(self.model.jnt_dofadr[target_joint])
            target_body = int(self.model.jnt_bodyid[target_joint])
            pair_bodies = {int(self.model.geom_bodyid[pair.geom_a]), int(self.model.geom_bodyid[pair.geom_b])}
            if target_body in pair_bodies:
                full_bias[target_dof:target_dof + 6] = data.qvel[target_dof:target_dof + 6]
        if not np.any(full_bias):
            return 0.0
        epsilon = 1e-5
        probe = copy.copy(data)
        distance = float(mujoco.mj_geomDistance(
            self.model, data, pair.geom_a, pair.geom_b,
            self.config.clearance_query_max_m, None,
        ))
        mujoco.mj_integratePos(self.model, probe.qpos, full_bias, epsilon)
        mujoco.mj_forward(self.model, probe)
        shifted = float(mujoco.mj_geomDistance(
            self.model, probe, pair.geom_a, pair.geom_b,
            self.config.clearance_query_max_m, None,
        ))
        return (shifted-distance)/epsilon

    def _affine_clearance_constraints(self, data: mujoco.MjData, mapping: np.ndarray,
                                      bias: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
        rows: list[np.ndarray] = []
        lower: list[float] = []
        drift_values: list[float] = []
        minimum = float("inf")
        for pair in self.pairs:
            distance, gradient = self.clearance_gradient_for_pair(data, pair, mapping)
            minimum = min(minimum, distance)
            if distance > self.config.clearance_activation_m:
                continue
            drift = self._distance_bias_drift(data, pair, bias)
            required = -self.config.clearance_barrier_gain * (distance-self.config.clearance_safe_m) - drift
            if np.linalg.norm(gradient) > 1e-10 or required > 0.0:
                rows.append(gradient)
                lower.append(required)
                drift_values.append(drift)
        self.last_clearance_drift = np.asarray(drift_values)
        return (np.vstack(rows) if rows else np.zeros((0, 7)), np.asarray(lower), minimum)

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
        task_jacobian = np.vstack((jacobian_position, jacobian_rotation))
        base_map = mapping[self.base_dof_slice, :]
        base_bias = np.asarray(data.qvel[self.base_dof_slice]) - base_map @ np.asarray(data.qvel[self.dof_ids])
        site_linear = np.zeros((3, self.model.nv))
        site_angular = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, data, site_linear, site_angular, self.flange_site)
        task_bias = np.r_[site_linear[:, self.base_dof_slice] @ base_bias,
                          site_angular[:, self.base_dof_slice] @ base_bias]

        position = np.asarray(data.site_xpos[self.flange_site]).copy()
        rotation = np.asarray(data.site_xmat[self.flange_site]).reshape(3, 3).copy()
        position_error = np.asarray(target_position, dtype=np.float64) - position
        orientation_error = rotation_error_vector_world(target_rotation, rotation)
        # Clip the requested PHYSICAL velocity before subtracting model bias.
        velocity_command = _clip_norm(
            np.asarray(target_velocity, dtype=np.float64) + cfg.position_gain * position_error,
            cfg.linear_speed_limit_m_s,
        )
        angular_command = smooth_saturate_norm(
            np.asarray(target_angular_velocity, dtype=np.float64) + cfg.orientation_gain * orientation_error,
            cfg.angular_speed_limit_rad_s,
            cfg.angular_saturation_transition_ratio,
        )
        physical_command = np.r_[velocity_command, angular_command]
        task_rhs = physical_command - task_bias

        q = np.asarray(data.qpos[self.qpos_ids])
        clearance_matrix, clearance_lower, minimum_clearance = self._affine_clearance_constraints(data, mapping, base_bias)
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
            cfg.position_weight * jacobian_position.T @ task_rhs[:3]
            + cfg.orientation_weight * jacobian_rotation.T @ task_rhs[3:]
        )
        primary_hessian = 0.5 * (primary_hessian + primary_hessian.T)
        primary, primary_success, primary_status, primary_iterations = self._solve_admm(
            primary_hessian, primary_linear, primary_matrix, primary_lower,
            primary_upper, initial,
        )
        primary_achieved = task_jacobian @ primary
        primary_slack = np.minimum(
            primary_matrix @ primary-primary_lower,
            primary_upper-primary_matrix @ primary,
        )
        primary_minimum_slack = float(np.min(primary_slack))

        shape_sample = self.shape.sample(data)
        if shape_sample.singular:
            primary_success = False
        shape_jacobian = self.shape.jacobian(data)
        shape_error = _wrap_to_pi(float(target_arm_angle_rad)-shape_sample.angle_rad)
        shape_command = float(target_arm_angle_velocity_rad_s)+cfg.shape_gain*shape_error
        base_command = np.zeros(6) if desired_base_twist is None else np.asarray(desired_base_twist, dtype=np.float64)
        if base_command.shape != (6,):
            raise ValueError("desired base twist must have shape (6,)")
        base_rhs = base_command-base_bias
        secondary_hessian = (
            cfg.shape_weight * np.outer(shape_jacobian, shape_jacobian)
            + cfg.base_reaction_weight * base_map.T @ base_map
            + cfg.smoothness_weight * np.eye(7)
        )
        secondary_linear = -(
            cfg.shape_weight * shape_jacobian * shape_command
            + cfg.base_reaction_weight * base_map.T @ base_rhs
            + cfg.smoothness_weight * self.previous_velocity
        )
        lock_tolerance = np.asarray(
            [cfg.level1_position_tolerance_m_s]*3 + [cfg.level1_angular_tolerance_rad_s]*3,
            dtype=np.float64,
        )
        secondary_matrix = np.vstack((primary_matrix, task_jacobian))
        secondary_lower = np.concatenate((primary_lower, primary_achieved-lock_tolerance))
        secondary_upper = np.concatenate((primary_upper, primary_achieved+lock_tolerance))
        secondary, secondary_success, secondary_status, secondary_iterations = self._solve_scalar_nullspace_secondary(
            primary, task_jacobian, secondary_hessian, secondary_linear,
            primary_matrix, primary_lower, primary_upper,
        )
        if secondary_status == "nullity_not_one":
            secondary, secondary_success, secondary_status, secondary_iterations = self._solve_admm(
                secondary_hessian, secondary_linear, secondary_matrix,
                secondary_lower, secondary_upper, primary,
            )
        candidate = secondary if secondary_success else primary
        success = bool(primary_success and secondary_success and not shape_sample.singular)
        achieved = task_jacobian @ candidate
        base_residual = base_map @ candidate + base_bias - base_command
        all_slack = np.minimum(
            secondary_matrix @ candidate-secondary_lower,
            secondary_upper-secondary_matrix @ candidate,
        )
        minimum_slack = float(np.min(all_slack))
        success = bool(success and minimum_slack >= -self.config.feasibility_tolerance-1e-12)
        if not primary_success:
            candidate = np.clip(self.previous_velocity, joint_lower, joint_upper)
        candidate = np.clip(candidate, joint_lower, joint_upper)
        achieved = task_jacobian @ candidate
        base_residual = base_map @ candidate + base_bias - base_command
        self._commit_velocity(candidate)

        self.last_bias = base_bias.copy()
        self.last_physical_task_command = physical_command.copy()
        self.last_debiased_task_rhs = task_rhs.copy()
        self.last_predicted_task_velocity = achieved + task_bias
        self.last_primary_physical_residual = primary_achieved + task_bias - physical_command
        self.last_task_jacobian = task_jacobian.copy()
        self.last_base_map = base_map.copy()
        return FPMFCResult(
            joint_velocity=candidate,
            success=success, primary_feasible=primary_success,
            secondary_feasible=secondary_success,
            primary_status=primary_status, secondary_status=secondary_status,
            primary_iterations=primary_iterations, secondary_iterations=secondary_iterations,
            full_latency_s=time.perf_counter()-started,
            position_error_m=float(np.linalg.norm(position_error)),
            orientation_error_rad=float(np.linalg.norm(orientation_error)),
            primary_linear_velocity_residual_m_s=float(np.linalg.norm(self.last_primary_physical_residual[:3])),
            primary_angular_velocity_residual_rad_s=float(np.linalg.norm(self.last_primary_physical_residual[3:])),
            hierarchy_linear_degradation_m_s=float(np.linalg.norm(achieved[:3]-primary_achieved[:3])),
            hierarchy_angular_degradation_rad_s=float(np.linalg.norm(achieved[3:]-primary_achieved[3:])),
            arm_angle_rad=shape_sample.angle_rad, arm_angle_error_rad=shape_error,
            arm_angle_velocity_residual_rad_s=float(shape_jacobian @ candidate-shape_command),
            base_twist_residual=base_residual,
            base_twist_residual_norm=float(np.linalg.norm(base_residual)),
            momentum_map_residual_norm=momentum_map_residual,
            minimum_queried_clearance_m=float(minimum_clearance),
            active_clearance_constraints=len(clearance_lower),
            minimum_constraint_slack=minimum_slack,
            primary_minimum_constraint_slack=primary_minimum_slack,
        )

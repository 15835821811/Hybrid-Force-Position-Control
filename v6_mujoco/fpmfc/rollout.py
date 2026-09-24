"""Fast 50 Hz zero-momentum rollout used inside capture-time/shape optimization."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Mapping

import mujoco
import numpy as np

from ..collision import build_collision_pairs
from ..hierarchical_qp import QPConfig
from ..model import FlexivModelSpec, body_id, default_model_spec, geom_id, site_id
from .controller import FPMFCControllerConfig, FPMFCHQP
from .dynamics import FreeFloatingKinematics, rotation_distance_rad
from .shape import ArmShapeKinematics
from .target import PrescribedTumblingTarget, sync_mujoco_target, target_from_config
from .trajectory import PoseShapeTrajectory


@dataclass(frozen=True)
class PrecontactRolloutResult:
    capture_time_s: float
    terminal_arm_angle_rad: float
    objective: float
    base_cost: float
    alignment_cost: float
    constraint_penalty: float
    feasible: bool
    controller_failure_count: int
    task_tick_count: int
    maximum_base_angular_velocity_rad_s: float
    rms_base_angular_velocity_rad_s: float
    maximum_base_linear_velocity_m_s: float
    maximum_base_orientation_drift_rad: float
    terminal_position_error_m: float
    terminal_orientation_error_rad: float
    terminal_arm_angle_error_rad: float
    terminal_linear_velocity_m_s: float
    terminal_angular_velocity_rad_s: float
    alignment_angle_rad: float
    impulse_torque_proxy_m2_s: float
    minimum_clearance_m: float
    maximum_hierarchy_linear_degradation_m_s: float
    maximum_hierarchy_angular_degradation_rad_s: float
    maximum_momentum_map_residual: float
    trace: dict[str, np.ndarray] | None = None

    def to_dict(self, *, include_trace: bool = False) -> dict[str, Any]:
        result = asdict(self)
        trace = result.pop("trace")
        if result["minimum_clearance_m"] > 1e6:
            result["minimum_clearance_m"] = None
        if include_trace and trace is not None:
            result["trace"] = {key: np.asarray(value).tolist() for key, value in trace.items()}
        return result


def _controller_config(config: Mapping[str, Any]) -> FPMFCControllerConfig:
    values = config["controller"]
    return FPMFCControllerConfig(
        position_gain=float(values["position_gain"]),
        orientation_gain=float(values["orientation_gain"]),
        shape_gain=float(values["shape_gain"]),
        shape_weight=float(values["shape_weight"]),
        base_reaction_weight=float(values["base_reaction_weight"]),
        angular_saturation_transition_ratio=float(
            values["angular_saturation_transition_ratio"]
        ),
        level1_position_tolerance_m_s=float(values["level1_position_tolerance_m_s"]),
        level1_angular_tolerance_rad_s=float(values["level1_angular_tolerance_rad_s"]),
    )


def _constraint_config(config: Mapping[str, Any]) -> QPConfig:
    values = config["controller"]
    return QPConfig(
        qp_tolerance=float(values["qp_tolerance"]),
        qp_max_iterations=int(values["qp_max_iterations"]),
        feasibility_tolerance=float(values["feasibility_tolerance"]),
        joint_jerk_limit_rad_s3=float(values["joint_jerk_limit_rad_s3"]),
    )


class PrecontactRolloutEvaluator:
    """Evaluate one capture-time and terminal-arm-angle candidate."""

    def __init__(
        self,
        config: Mapping[str, Any],
        *,
        spec: FlexivModelSpec | None = None,
        target: PrescribedTumblingTarget | None = None,
    ) -> None:
        self.config = config
        self.spec = default_model_spec() if spec is None else spec
        self.model = self.spec.compile_model()
        obstacle = geom_id(self.model, "workspace_obstacle_0")
        self.model.geom_pos[obstacle] = [10.0, 10.0, 10.0]
        self.data = mujoco.MjData(self.model)
        self.spec.reset_home(self.model, self.data)
        self.target = target_from_config(config) if target is None else target
        sync_mujoco_target(self.model, self.data, self.target, self.target.sample(0.0))
        mujoco.mj_forward(self.model, self.data)
        shape_values = config["shape"]
        self.shape = ArmShapeKinematics(
            self.spec,
            self.model,
            self.data,
            shoulder_joint=shape_values["shoulder_joint"],
            elbow_joint=shape_values["elbow_joint"],
            wrist_joint=shape_values["wrist_joint"],
            singularity_margin=float(shape_values["singularity_margin"]),
        )
        self.kinematics = FreeFloatingKinematics(self.spec, self.model)
        self.controller = FPMFCHQP(
            self.spec,
            self.model,
            build_collision_pairs(self.model),
            self.shape,
            controller_config=_controller_config(config),
            constraint_config=_constraint_config(config),
        )
        self.flange = site_id(self.model, "flange_site")
        self.base_body = body_id(self.model, "base_link_0")
        self.initial_qpos = np.asarray(self.data.qpos).copy()

    def reset(self) -> None:
        self.spec.reset_home(self.model, self.data)
        sync_mujoco_target(self.model, self.data, self.target, self.target.sample(0.0))
        mujoco.mj_forward(self.model, self.data)
        self.controller.reset()

    def evaluate(
        self,
        capture_time_s: float,
        terminal_arm_angle_rad: float,
        *,
        keep_trace: bool = False,
    ) -> PrecontactRolloutResult:
        capture_time = float(capture_time_s)
        terminal_arm_angle = float(terminal_arm_angle_rad)
        time_bounds = self.config["trajectory"]["capture_time_bounds_s"]
        if not float(time_bounds[0]) <= capture_time <= float(time_bounds[1]):
            raise ValueError("capture time lies outside the configured search bounds")
        arm_bounds = self.config["optimization"]["terminal_arm_angle_bounds_rad"]
        if not float(arm_bounds[0]) <= terminal_arm_angle <= float(arm_bounds[1]):
            raise ValueError("terminal arm angle lies outside the configured search bounds")

        self.reset()
        initial_position = np.asarray(self.data.site_xpos[self.flange]).copy()
        initial_rotation = np.asarray(self.data.site_xmat[self.flange]).reshape(3, 3).copy()
        initial_arm_angle = self.shape.sample(self.data).angle_rad
        initial_base_rotation = np.asarray(self.data.xmat[self.base_body]).reshape(3, 3).copy()
        final_target = self.target.sample(capture_time)
        trajectory = PoseShapeTrajectory(
            initial_position_world_m=initial_position,
            final_position_world_m=final_target.grasp_position_world_m,
            initial_rotation_world=initial_rotation,
            final_rotation_world=final_target.grasp_rotation_world,
            initial_arm_angle_rad=initial_arm_angle,
            final_arm_angle_rad=terminal_arm_angle,
            duration_s=capture_time,
        )

        period = float(self.spec.task_period_s)
        times = np.linspace(0.0, capture_time, int(np.ceil(capture_time / period)) + 1)
        failure_count = 0
        maximum_base_angular = 0.0
        squared_base_angular_sum = 0.0
        maximum_base_linear = 0.0
        maximum_base_orientation = 0.0
        minimum_clearance = float("inf")
        maximum_hierarchy_linear = 0.0
        maximum_hierarchy_angular = 0.0
        maximum_momentum_residual = 0.0
        last_result = None
        log: dict[str, list[Any]] = {
            key: []
            for key in (
                "time",
                "qpos",
                "joint_velocity",
                "flange_position",
                "flange_rotation",
                "desired_position",
                "desired_rotation",
                "base_twist",
                "base_orientation_drift_rad",
                "arm_angle_rad",
                "arm_angle_desired_rad",
                "position_error_m",
                "orientation_error_rad",
                "minimum_clearance_m",
                "controller_success",
                "primary_status",
                "secondary_status",
                "minimum_constraint_slack",
                "primary_minimum_constraint_slack",
                "primary_feasible",
                "secondary_feasible",
            )
        }

        for index, time_value in enumerate(times):
            target_sample = self.target.sample(float(time_value))
            sync_mujoco_target(self.model, self.data, self.target, target_sample)
            mujoco.mj_forward(self.model, self.data)
            reference = trajectory.sample(float(time_value))
            result = self.controller.solve_fpmfc(
                self.data,
                target_position=reference.position_world_m,
                target_velocity=reference.linear_velocity_world_m_s,
                target_rotation=reference.rotation_world,
                target_angular_velocity=reference.angular_velocity_world_rad_s,
                target_arm_angle_rad=reference.arm_angle_rad,
                target_arm_angle_velocity_rad_s=reference.arm_angle_velocity_rad_s,
            )
            last_result = result
            if not result.success:
                failure_count += 1
            reaction = self.kinematics.reaction_map(self.data)
            base_twist = reaction.base_from_joint_velocity @ result.joint_velocity
            base_angular = float(np.linalg.norm(base_twist[3:]))
            base_linear = float(np.linalg.norm(base_twist[:3]))
            maximum_base_angular = max(maximum_base_angular, base_angular)
            squared_base_angular_sum += base_angular**2
            maximum_base_linear = max(maximum_base_linear, base_linear)
            current_base_rotation = np.asarray(self.data.xmat[self.base_body]).reshape(3, 3)
            orientation_drift = rotation_distance_rad(initial_base_rotation, current_base_rotation)
            maximum_base_orientation = max(maximum_base_orientation, orientation_drift)
            minimum_clearance = min(minimum_clearance, result.minimum_queried_clearance_m)
            maximum_hierarchy_linear = max(
                maximum_hierarchy_linear, result.hierarchy_linear_degradation_m_s
            )
            maximum_hierarchy_angular = max(
                maximum_hierarchy_angular, result.hierarchy_angular_degradation_rad_s
            )
            maximum_momentum_residual = max(
                maximum_momentum_residual, result.momentum_map_residual_norm
            )

            if keep_trace:
                log["time"].append(float(time_value))
                log["qpos"].append(np.asarray(self.data.qpos).copy())
                log["joint_velocity"].append(result.joint_velocity.copy())
                log["flange_position"].append(
                    np.asarray(self.data.site_xpos[self.flange]).copy()
                )
                log["flange_rotation"].append(
                    np.asarray(self.data.site_xmat[self.flange]).reshape(3, 3).copy()
                )
                log["desired_position"].append(reference.position_world_m.copy())
                log["desired_rotation"].append(reference.rotation_world.copy())
                log["base_twist"].append(base_twist.copy())
                log["base_orientation_drift_rad"].append(orientation_drift)
                log["arm_angle_rad"].append(result.arm_angle_rad)
                log["arm_angle_desired_rad"].append(reference.arm_angle_rad)
                log["position_error_m"].append(result.position_error_m)
                log["orientation_error_rad"].append(result.orientation_error_rad)
                log["minimum_clearance_m"].append(result.minimum_queried_clearance_m)
                log["controller_success"].append(result.success)
                log["primary_status"].append(result.primary_status)
                log["secondary_status"].append(result.secondary_status)
                log["minimum_constraint_slack"].append(result.minimum_constraint_slack)
                log["primary_minimum_constraint_slack"].append(
                    result.primary_minimum_constraint_slack
                )
                log["primary_feasible"].append(result.primary_feasible)
                log["secondary_feasible"].append(result.secondary_feasible)

            if index + 1 < len(times):
                delta = float(times[index + 1] - time_value)
                full_velocity = reaction.full_velocity_from_joint_velocity @ result.joint_velocity
                self.data.qvel[:] = full_velocity
                mujoco.mj_integratePos(self.model, self.data.qpos, full_velocity, delta)
                mujoco.mj_forward(self.model, self.data)

        if last_result is None:
            raise RuntimeError("rollout produced no controller samples")
        terminal_position = np.asarray(self.data.site_xpos[self.flange]).copy()
        terminal_rotation = np.asarray(self.data.site_xmat[self.flange]).reshape(3, 3).copy()
        terminal_position_error = float(
            np.linalg.norm(final_target.grasp_position_world_m - terminal_position)
        )
        terminal_orientation_error = rotation_distance_rad(
            final_target.grasp_rotation_world, terminal_rotation
        )
        terminal_arm_error = abs(
            float(
                (terminal_arm_angle - self.shape.sample(self.data).angle_rad + np.pi)
                % (2.0 * np.pi)
                - np.pi
            )
        )
        terminal_reaction = self.kinematics.reaction_map(self.data)
        terminal_base_twist = (
            terminal_reaction.base_from_joint_velocity @ last_result.joint_velocity
        )
        terminal_position_jacobian, terminal_rotation_jacobian, _ = (
            self.kinematics.generalized_site_jacobians(self.data)
        )
        terminal_linear_velocity = float(
            np.linalg.norm(terminal_position_jacobian @ last_result.joint_velocity)
        )
        terminal_angular_velocity = float(
            np.linalg.norm(terminal_rotation_jacobian @ last_result.joint_velocity)
        )

        system_com = np.asarray(self.data.subtree_com[self.base_body]).copy()
        center_to_grasp = final_target.grasp_position_world_m - system_com
        relative_velocity = final_target.grasp_linear_velocity_world_m_s
        direction_denominator = float(
            np.linalg.norm(center_to_grasp) * np.linalg.norm(relative_velocity)
        )
        if direction_denominator <= 1e-12:
            alignment_angle = np.pi
            impulse_proxy = float("inf")
        else:
            cosine = float(
                np.clip(center_to_grasp @ relative_velocity / direction_denominator, -1.0, 1.0)
            )
            alignment_angle = float(np.arccos(cosine))
            impulse_proxy = float(np.linalg.norm(np.cross(center_to_grasp, relative_velocity)))

        optimization = self.config["optimization"]
        acceptance = self.config["acceptance"]
        base_reference = float(optimization["base_angular_velocity_reference_rad_s"])
        angle_reference = float(optimization["alignment_angle_reference_rad"])
        weight_base, weight_alignment = map(float, optimization["objective_weights"])
        base_cost = (maximum_base_angular / base_reference) ** 2
        alignment_cost = (alignment_angle / angle_reference) ** 2
        clearance_required = float(self.config["controller"]["minimum_clearance_m"])
        position_limit = float(acceptance["terminal_position_error_m"])
        orientation_limit = float(acceptance["terminal_orientation_error_rad"])
        shape_limit = float(optimization["planning_terminal_arm_angle_error_rad"])
        linear_velocity_limit = float(acceptance["terminal_linear_velocity_m_s"])
        angular_velocity_limit = float(acceptance["terminal_angular_velocity_rad_s"])
        position_excess = max(0.0, terminal_position_error - position_limit) / position_limit
        orientation_excess = max(0.0, terminal_orientation_error - orientation_limit) / orientation_limit
        shape_excess = max(0.0, terminal_arm_error - shape_limit) / shape_limit
        clearance_excess = max(0.0, clearance_required - minimum_clearance) / clearance_required
        terminal_velocity_excess = max(0.0, terminal_linear_velocity - linear_velocity_limit) / linear_velocity_limit
        terminal_angular_excess = max(0.0, terminal_angular_velocity - angular_velocity_limit) / angular_velocity_limit
        constraint_penalty = 1e3 * (
            failure_count / len(times)
            + position_excess**2
            + orientation_excess**2
            + shape_excess**2
            + clearance_excess**2
            + terminal_velocity_excess**2
            + terminal_angular_excess**2
        )
        objective = weight_base * base_cost + weight_alignment * alignment_cost + constraint_penalty
        feasible = bool(
            1.0 - failure_count / len(times) >= float(acceptance["minimum_task_success_rate"])
            and position_excess == 0.0
            and orientation_excess == 0.0
            and shape_excess == 0.0
            and clearance_excess == 0.0
            and terminal_velocity_excess == 0.0
            and terminal_angular_excess == 0.0
        )
        trace = {key: np.asarray(value) for key, value in log.items()} if keep_trace else None
        return PrecontactRolloutResult(
            capture_time_s=capture_time,
            terminal_arm_angle_rad=terminal_arm_angle,
            objective=float(objective),
            base_cost=float(base_cost),
            alignment_cost=float(alignment_cost),
            constraint_penalty=float(constraint_penalty),
            feasible=feasible,
            controller_failure_count=failure_count,
            task_tick_count=len(times),
            maximum_base_angular_velocity_rad_s=maximum_base_angular,
            rms_base_angular_velocity_rad_s=float(
                np.sqrt(squared_base_angular_sum / len(times))
            ),
            maximum_base_linear_velocity_m_s=maximum_base_linear,
            maximum_base_orientation_drift_rad=maximum_base_orientation,
            terminal_position_error_m=terminal_position_error,
            terminal_orientation_error_rad=terminal_orientation_error,
            terminal_arm_angle_error_rad=terminal_arm_error,
            terminal_linear_velocity_m_s=terminal_linear_velocity,
            terminal_angular_velocity_rad_s=terminal_angular_velocity,
            alignment_angle_rad=alignment_angle,
            impulse_torque_proxy_m2_s=impulse_proxy,
            minimum_clearance_m=minimum_clearance,
            maximum_hierarchy_linear_degradation_m_s=maximum_hierarchy_linear,
            maximum_hierarchy_angular_degradation_rad_s=maximum_hierarchy_angular,
            maximum_momentum_map_residual=maximum_momentum_residual,
            trace=trace,
        )

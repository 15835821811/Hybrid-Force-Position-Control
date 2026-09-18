"""Run one paper-scope pre-contact capture candidate in full MuJoCo dynamics.

The optimizer uses a fast 50 Hz zero-momentum kinematic rollout.  This module
replays a selected ``(capture time, terminal arm angle)`` pair through the
500 Hz torque-controlled free-floating plant without writing qpos/qvel after
initialization.  It is the mandatory fidelity check before reporting a paper
comparison.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping

import mujoco
import numpy as np

from ..collision import build_collision_pairs, minimum_signed_distance
from ..hierarchical_qp import QPConfig
from ..model import PROJECT_ROOT, body_id, default_model_spec, geom_id, site_id
from ..run import robot_momentum, servo_torque
from .config import DEFAULT_CONFIG_PATH, load_fpmfc_config, validate_fpmfc_config
from .controller import FPMFCControllerConfig, FPMFCHQP
from .dynamics import rotation_distance_rad
from .shape import ArmShapeKinematics
from .provenance import implementation_identity
from .target import sync_mujoco_target, target_from_config
from .trajectory import PoseShapeTrajectory


@dataclass(frozen=True)
class DynamicRunConfig:
    """Parameters belonging to the 500 Hz implementation, not to the paper."""

    reference_tracking_band_rad: float = 0.012
    servo_natural_frequency_rad_s: float = 34.0
    servo_acceleration_limit_rad_s2: float = 70.0


CONTROLLER_VARIANTS = ("full", "no-shape", "no-base-reaction")


def controller_variant_config(
    config: Mapping[str, Any], variant: str
) -> dict[str, Any]:
    """Return a validated config with exactly one level-2 objective disabled."""

    if variant not in CONTROLLER_VARIANTS:
        raise ValueError(
            f"controller variant must be one of {CONTROLLER_VARIANTS}, got {variant!r}"
        )
    effective = copy.deepcopy(dict(config))
    if variant == "no-shape":
        effective["controller"]["shape_weight"] = 0.0
    elif variant == "no-base-reaction":
        effective["controller"]["base_reaction_weight"] = 0.0
    validate_fpmfc_config(effective)
    return effective


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


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _effective_config_sha256(config: Mapping[str, Any]) -> str:
    encoded = json.dumps(
        config, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _matrix_to_mujoco_quaternion(rotation: np.ndarray) -> np.ndarray:
    quaternion = np.zeros(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quaternion, np.asarray(rotation, dtype=np.float64).reshape(9))
    return quaternion


def _safe_float(value: float) -> float | None:
    value = float(value)
    return value if math.isfinite(value) and abs(value) <= 1e6 else None


def _json_default(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


def run_precontact_candidate(
    config: Mapping[str, Any],
    *,
    capture_time_s: float,
    terminal_arm_angle_rad: float,
    output_dir: Path,
    run_config: DynamicRunConfig = DynamicRunConfig(),
    controller_variant: str = "full",
) -> dict[str, Any]:
    """Execute and persist one torque-level paper-scope capture approach."""

    config = controller_variant_config(config, controller_variant)

    requested_capture_time = float(capture_time_s)
    terminal_arm_angle = float(terminal_arm_angle_rad)
    time_bounds = list(map(float, config["trajectory"]["capture_time_bounds_s"]))
    arm_bounds = list(map(float, config["optimization"]["terminal_arm_angle_bounds_rad"]))
    if not time_bounds[0] <= requested_capture_time <= time_bounds[1]:
        raise ValueError("capture time lies outside the configured search bounds")
    if not arm_bounds[0] <= terminal_arm_angle <= arm_bounds[1]:
        raise ValueError("terminal arm angle lies outside the configured search bounds")

    spec = default_model_spec()
    physics_steps = int(round(requested_capture_time / spec.timestep_s))
    capture_time = physics_steps * spec.timestep_s
    if physics_steps < 1:
        raise ValueError("capture time is shorter than one physics step")
    model = spec.compile_model()
    # The paper's simulated scene has no workspace obstacle and stops before
    # contact.  Keep self-distance queries, move only the migration demo object.
    obstacle = geom_id(model, "workspace_obstacle_0")
    model.geom_pos[obstacle] = [10.0, 10.0, 10.0]
    model.geom_contype[:] = 0
    model.geom_conaffinity[:] = 0
    data = mujoco.MjData(model)
    spec.reset_home(model, data)

    shape_values = config["shape"]
    shape = ArmShapeKinematics(
        spec,
        model,
        data,
        shoulder_joint=shape_values["shoulder_joint"],
        elbow_joint=shape_values["elbow_joint"],
        wrist_joint=shape_values["wrist_joint"],
        singularity_margin=float(shape_values["singularity_margin"]),
    )
    pairs = build_collision_pairs(model)
    controller = FPMFCHQP(
        spec,
        model,
        pairs,
        shape,
        controller_config=_controller_config(config),
        constraint_config=_constraint_config(config),
    )
    target = target_from_config(config)
    sync_mujoco_target(model, data, target, target.sample(0.0))
    mujoco.mj_forward(model, data)
    flange = site_id(model, "flange_site")
    base_body = body_id(model, "base_link_0")
    qpos_ids, dof_ids = spec.joint_addresses(model)
    base_qpos_slice, base_dof_slice = spec.base_slices(model)

    initial_position = np.asarray(data.site_xpos[flange]).copy()
    initial_rotation = np.asarray(data.site_xmat[flange]).reshape(3, 3).copy()
    initial_shape = shape.sample(data).angle_rad
    final_target = target.sample(capture_time)
    trajectory = PoseShapeTrajectory(
        initial_position_world_m=initial_position,
        final_position_world_m=final_target.grasp_position_world_m,
        initial_rotation_world=initial_rotation,
        final_rotation_world=final_target.grasp_rotation_world,
        initial_arm_angle_rad=initial_shape,
        final_arm_angle_rad=terminal_arm_angle,
        duration_s=capture_time,
    )

    task_stride = int(round(spec.task_period_s / spec.timestep_s))
    initial_qpos = np.asarray(data.qpos).copy()
    initial_qvel = np.asarray(data.qvel).copy()
    initial_momentum = robot_momentum(model, data, base_body)
    reference_q = np.asarray(data.qpos[qpos_ids]).copy()
    command_velocity = np.zeros(7, dtype=np.float64)
    segment_start_velocity = command_velocity.copy()
    segment_step = 0
    full_mass = np.zeros((model.nv, model.nv), dtype=np.float64)
    saturation_samples = 0

    physics_log: dict[str, list[Any]] = {
        key: []
        for key in (
            "time",
            "qpos",
            "qvel",
            "torque",
            "reference_q",
            "reference_dq",
            "reference_ddq",
            "flange_position",
            "flange_rotation",
            "desired_position",
            "desired_rotation",
            "target_center_position",
            "target_center_rotation",
            "target_grasp_position",
            "target_grasp_rotation",
            "desired_arm_angle_rad",
            "arm_angle_rad",
            "position_error_m",
            "orientation_error_rad",
            "base_qpos",
            "base_twist",
            "momentum",
            "minimum_clearance_m",
        )
    }
    task_log: dict[str, list[Any]] = {
        key: []
        for key in (
            "task_time",
            "task_success",
            "task_primary_feasible",
            "task_secondary_feasible",
            "task_primary_status",
            "task_secondary_status",
            "task_latency_s",
            "task_minimum_constraint_slack",
            "task_momentum_map_residual",
            "task_hierarchy_linear_degradation_m_s",
            "task_hierarchy_angular_degradation_rad_s",
            "task_joint_velocity_rad_s",
            "task_joint_acceleration_rad_s2",
            "task_joint_velocity_lower_rad_s",
            "task_joint_velocity_upper_rad_s",
            "task_velocity_bound_conflict",
        )
    }

    started = time.perf_counter()
    for step in range(physics_steps):
        current_time = float(data.time)
        reference = trajectory.sample(current_time)
        target_sample = target.sample(current_time)
        sync_mujoco_target(model, data, target, target_sample)
        data.mocap_pos[0] = target_sample.grasp_position_world_m
        data.mocap_quat[0] = _matrix_to_mujoco_quaternion(
            target_sample.grasp_rotation_world
        )

        if step % task_stride == 0:
            result = controller.solve_fpmfc(
                data,
                target_position=reference.position_world_m,
                target_velocity=reference.linear_velocity_world_m_s,
                target_rotation=reference.rotation_world,
                target_angular_velocity=reference.angular_velocity_world_rad_s,
                target_arm_angle_rad=reference.arm_angle_rad,
                target_arm_angle_velocity_rad_s=reference.arm_angle_velocity_rad_s,
            )
            segment_start_velocity = command_velocity.copy()
            command_velocity = result.joint_velocity.copy()
            segment_step = 0
            task_log["task_time"].append(current_time)
            task_log["task_success"].append(result.success)
            task_log["task_primary_feasible"].append(result.primary_feasible)
            task_log["task_secondary_feasible"].append(result.secondary_feasible)
            task_log["task_primary_status"].append(result.primary_status)
            task_log["task_secondary_status"].append(result.secondary_status)
            task_log["task_latency_s"].append(result.full_latency_s)
            task_log["task_minimum_constraint_slack"].append(
                result.minimum_constraint_slack
            )
            task_log["task_momentum_map_residual"].append(
                result.momentum_map_residual_norm
            )
            task_log["task_hierarchy_linear_degradation_m_s"].append(
                result.hierarchy_linear_degradation_m_s
            )
            task_log["task_hierarchy_angular_degradation_rad_s"].append(
                result.hierarchy_angular_degradation_rad_s
            )
            task_log["task_joint_velocity_rad_s"].append(command_velocity.copy())
            task_log["task_joint_acceleration_rad_s2"].append(
                controller.previous_acceleration.copy()
            )
            task_log["task_joint_velocity_lower_rad_s"].append(
                controller.last_velocity_lower.copy()
            )
            task_log["task_joint_velocity_upper_rad_s"].append(
                controller.last_velocity_upper.copy()
            )
            task_log["task_velocity_bound_conflict"].append(
                controller.last_bound_conflicts.copy()
            )

        interpolation = float(segment_step + 1) / task_stride
        reference_dq = segment_start_velocity + interpolation * (
            command_velocity - segment_start_velocity
        )
        feedforward_ddq = (
            command_velocity - segment_start_velocity
        ) / spec.task_period_s
        reference_q = np.clip(
            reference_q + reference_dq * spec.timestep_s,
            controller.joint_lower,
            controller.joint_upper,
        )
        measured_q = np.asarray(data.qpos[qpos_ids])
        reference_q = np.clip(
            reference_q,
            measured_q - run_config.reference_tracking_band_rad,
            measured_q + run_config.reference_tracking_band_rad,
        )
        torque, _desired_acceleration = servo_torque(
            model,
            data,
            dof_ids,
            base_dof_slice,
            qpos_ids,
            reference_q,
            reference_dq,
            feedforward_ddq,
            spec.torque_limits_nm,
            run_config.servo_natural_frequency_rad_s,
            run_config.servo_acceleration_limit_rad_s2,
            full_mass,
        )
        saturation_samples += int(
            np.count_nonzero(np.abs(torque) >= spec.torque_limits_nm - 1e-9)
        )
        data.ctrl[:] = torque
        mujoco.mj_step(model, data)
        segment_step += 1

        now = float(data.time)
        reference_now = trajectory.sample(now)
        target_now = target.sample(now)
        flange_position = np.asarray(data.site_xpos[flange]).copy()
        flange_rotation = np.asarray(data.site_xmat[flange]).reshape(3, 3).copy()
        clearance, _pair_name = minimum_signed_distance(model, data, pairs)
        current_shape = shape.sample(data).angle_rad
        physics_log["time"].append(now)
        physics_log["qpos"].append(np.asarray(data.qpos).copy())
        physics_log["qvel"].append(np.asarray(data.qvel).copy())
        physics_log["torque"].append(torque.copy())
        physics_log["reference_q"].append(reference_q.copy())
        physics_log["reference_dq"].append(reference_dq.copy())
        physics_log["reference_ddq"].append(feedforward_ddq.copy())
        physics_log["flange_position"].append(flange_position)
        physics_log["flange_rotation"].append(flange_rotation)
        physics_log["desired_position"].append(reference_now.position_world_m.copy())
        physics_log["desired_rotation"].append(reference_now.rotation_world.copy())
        physics_log["target_center_position"].append(
            target_now.center_position_world_m.copy()
        )
        physics_log["target_center_rotation"].append(
            target_now.center_rotation_world.copy()
        )
        physics_log["target_grasp_position"].append(
            target_now.grasp_position_world_m.copy()
        )
        physics_log["target_grasp_rotation"].append(
            target_now.grasp_rotation_world.copy()
        )
        physics_log["desired_arm_angle_rad"].append(reference_now.arm_angle_rad)
        physics_log["arm_angle_rad"].append(current_shape)
        physics_log["position_error_m"].append(
            float(np.linalg.norm(reference_now.position_world_m - flange_position))
        )
        physics_log["orientation_error_rad"].append(
            rotation_distance_rad(reference_now.rotation_world, flange_rotation)
        )
        physics_log["base_qpos"].append(np.asarray(data.qpos[base_qpos_slice]).copy())
        physics_log["base_twist"].append(np.asarray(data.qvel[base_dof_slice]).copy())
        physics_log["momentum"].append(robot_momentum(model, data, base_body))
        physics_log["minimum_clearance_m"].append(clearance)

    elapsed = time.perf_counter() - started
    arrays = {
        key: np.asarray(value)
        for key, value in {**physics_log, **task_log}.items()
    }
    arrays["initial_qpos"] = initial_qpos
    arrays["initial_qvel"] = initial_qvel
    arrays["initial_momentum"] = initial_momentum
    arrays["capture_time_s"] = np.asarray(capture_time)
    arrays["requested_capture_time_s"] = np.asarray(requested_capture_time)
    arrays["terminal_arm_angle_rad"] = np.asarray(terminal_arm_angle)
    output = Path(output_dir).resolve()
    output.mkdir(parents=True, exist_ok=True)
    trace_path = output / "trace.npz"
    np.savez_compressed(trace_path, **arrays)

    final_position_error = float(arrays["position_error_m"][-1])
    final_orientation_error = float(arrays["orientation_error_rad"][-1])
    final_shape_error = abs(
        float(
            (terminal_arm_angle - float(arrays["arm_angle_rad"][-1]) + np.pi)
            % (2.0 * np.pi)
            - np.pi
        )
    )
    base_twist = np.asarray(arrays["base_twist"])
    base_angular_speed = np.linalg.norm(base_twist[:, 3:], axis=1)
    base_linear_speed = np.linalg.norm(base_twist[:, :3], axis=1)
    initial_base_quaternion = initial_qpos[3:7]
    base_quaternions = np.asarray(arrays["base_qpos"])[:, 3:7]
    quaternion_dots = np.abs(base_quaternions @ initial_base_quaternion)
    base_orientation_drift = 2.0 * np.arccos(np.clip(quaternion_dots, -1.0, 1.0))
    momentum_delta = np.asarray(arrays["momentum"]) - initial_momentum[None, :]
    torque_values = np.asarray(arrays["torque"])
    task_acceleration = np.asarray(arrays["task_joint_acceleration_rad_s2"])
    if len(task_acceleration) > 1:
        task_jerk = np.diff(task_acceleration, axis=0) / spec.task_period_s
    else:
        task_jerk = np.zeros((0, 7), dtype=np.float64)

    terminal_jacobian_position = np.zeros((3, model.nv), dtype=np.float64)
    terminal_jacobian_rotation = np.zeros((3, model.nv), dtype=np.float64)
    mujoco.mj_jacSite(
        model,
        data,
        terminal_jacobian_position,
        terminal_jacobian_rotation,
        flange,
    )
    terminal_linear_speed = float(
        np.linalg.norm(terminal_jacobian_position @ np.asarray(data.qvel))
    )
    terminal_angular_speed = float(
        np.linalg.norm(terminal_jacobian_rotation @ np.asarray(data.qvel))
    )
    clearance_minimum = float(np.min(arrays["minimum_clearance_m"]))
    task_success_rate = float(np.mean(arrays["task_success"]))
    momentum_delta_maximum = float(np.max(np.linalg.norm(momentum_delta, axis=1)))
    torque_saturation_fraction = saturation_samples / (7.0 * physics_steps)
    acceptance_values = config["acceptance"]
    position_limit = float(acceptance_values["terminal_position_error_m"])
    orientation_limit = float(acceptance_values["terminal_orientation_error_rad"])
    shape_limit = float(acceptance_values["terminal_arm_angle_error_rad"])
    linear_velocity_limit = float(acceptance_values["terminal_linear_velocity_m_s"])
    angular_velocity_limit = float(acceptance_values["terminal_angular_velocity_rad_s"])
    planning_clearance = float(config["controller"]["minimum_clearance_m"])
    acceptance_clearance = float(
        acceptance_values.get("minimum_clearance_m", planning_clearance)
    )
    acceptance = {
        "task_success_rate": task_success_rate
        >= float(acceptance_values["minimum_task_success_rate"]),
        "terminal_position_error": final_position_error <= position_limit,
        "terminal_orientation_error": final_orientation_error <= orientation_limit,
        "terminal_shape_error": final_shape_error <= shape_limit,
        "minimum_clearance": clearance_minimum >= acceptance_clearance,
        "terminal_linear_speed": terminal_linear_speed <= linear_velocity_limit,
        "terminal_angular_speed": terminal_angular_speed <= angular_velocity_limit,
        "torque_saturation_fraction": torque_saturation_fraction
        <= float(acceptance_values["maximum_torque_saturation_fraction"]),
        "momentum_delta": momentum_delta_maximum
        <= float(acceptance_values["maximum_momentum_delta"]),
        "command_joint_jerk": bool(
            not task_jerk.size
            or float(np.max(np.abs(task_jerk)))
            <= float(config["controller"]["joint_jerk_limit_rad_s3"]) + 1e-8
        ),
        "velocity_bound_compatibility": bool(
            not np.any(arrays["task_velocity_bound_conflict"])
        ),
    }
    acceptance["passed"] = bool(all(acceptance.values()))

    system_com = np.asarray(data.subtree_com[base_body]).copy()
    center_to_grasp = final_target.grasp_position_world_m - system_com
    relative_velocity = final_target.grasp_linear_velocity_world_m_s
    alignment_denominator = float(
        np.linalg.norm(center_to_grasp) * np.linalg.norm(relative_velocity)
    )
    if alignment_denominator <= 1e-12:
        alignment_angle = np.pi
        impulse_torque_proxy = float("inf")
    else:
        alignment_angle = float(
            np.arccos(
                np.clip(
                    center_to_grasp @ relative_velocity / alignment_denominator,
                    -1.0,
                    1.0,
                )
            )
        )
        impulse_torque_proxy = float(
            np.linalg.norm(np.cross(center_to_grasp, relative_velocity))
        )
    optimizer_values = config["optimization"]
    base_reference = float(optimizer_values["base_angular_velocity_reference_rad_s"])
    angle_reference = float(optimizer_values["alignment_angle_reference_rad"])
    weight_base, weight_alignment = map(
        float, optimizer_values["objective_weights"]
    )
    base_cost = (float(np.max(base_angular_speed)) / base_reference) ** 2
    alignment_cost = (alignment_angle / angle_reference) ** 2
    position_excess = max(0.0, final_position_error - position_limit) / position_limit
    orientation_excess = max(
        0.0, final_orientation_error - orientation_limit
    ) / orientation_limit
    shape_excess = max(0.0, final_shape_error - shape_limit) / shape_limit
    clearance_excess = max(
        0.0,
        planning_clearance - clearance_minimum,
    ) / planning_clearance
    terminal_velocity_excess = max(
        0.0, terminal_linear_speed - linear_velocity_limit
    ) / linear_velocity_limit
    terminal_angular_excess = max(
        0.0, terminal_angular_speed - angular_velocity_limit
    ) / angular_velocity_limit
    torque_saturation_excess = max(
        0.0,
        torque_saturation_fraction
        - float(acceptance_values["maximum_torque_saturation_fraction"]),
    ) / float(acceptance_values["maximum_torque_saturation_fraction"])
    momentum_excess = max(
        0.0,
        momentum_delta_maximum - float(acceptance_values["maximum_momentum_delta"]),
    ) / float(acceptance_values["maximum_momentum_delta"])
    constraint_penalty = 1e3 * (
        (1.0 - task_success_rate)
        + position_excess**2
        + orientation_excess**2
        + shape_excess**2
        + clearance_excess**2
        + terminal_velocity_excess**2
        + terminal_angular_excess**2
        + torque_saturation_excess**2
        + momentum_excess**2
    )
    dynamic_objective = (
        weight_base * base_cost
        + weight_alignment * alignment_cost
        + constraint_penalty
    )

    metrics = {
        "material_passport": {
            "origin_skill": "experiment-agent",
            "origin_mode": "run",
            "origin_date": datetime.now(timezone.utc).isoformat(),
            "verification_status": "UNVERIFIED",
            "version_label": "exp_result_v1",
        },
        "experiment_id": config["experiment_id"],
        "effective_config_sha256": _effective_config_sha256(config),
        "effective_config": config,
        "implementation_identity": implementation_identity(),
        "scope": "paper_precontact_zero_desired_force",
        "controller_variant": controller_variant,
        "clearance_thresholds_m": {
            "planning": planning_clearance,
            "acceptance": acceptance_clearance,
        },
        "candidate": {
            "requested_capture_time_s": requested_capture_time,
            "executed_capture_time_s": capture_time,
            "capture_time_quantization_error_s": capture_time - requested_capture_time,
            "terminal_arm_angle_rad": terminal_arm_angle,
        },
        "dynamic_run_config": asdict(run_config),
        "model_identity": spec.identity(),
        "target": target.to_dict(),
        "elapsed_wall_s": elapsed,
        "physics_steps": physics_steps,
        "task_ticks": int(len(arrays["task_time"])),
        "metrics": {
            "dynamic_objective": dynamic_objective,
            "base_cost": base_cost,
            "alignment_cost": alignment_cost,
            "constraint_penalty": constraint_penalty,
            "alignment_angle_rad": alignment_angle,
            "impulse_torque_proxy_m2_s": _safe_float(impulse_torque_proxy),
            "task_success_rate": task_success_rate,
            "tracking_position_rms_m": float(
                np.sqrt(np.mean(np.asarray(arrays["position_error_m"]) ** 2))
            ),
            "tracking_position_max_m": float(np.max(arrays["position_error_m"])),
            "tracking_orientation_max_rad": float(
                np.max(arrays["orientation_error_rad"])
            ),
            "terminal_position_error_m": final_position_error,
            "terminal_orientation_error_rad": final_orientation_error,
            "terminal_arm_angle_error_rad": final_shape_error,
            "terminal_linear_speed_m_s": terminal_linear_speed,
            "terminal_angular_speed_rad_s": terminal_angular_speed,
            "maximum_base_angular_velocity_rad_s": float(
                np.max(base_angular_speed)
            ),
            "rms_base_angular_velocity_rad_s": float(
                np.sqrt(np.mean(base_angular_speed**2))
            ),
            "maximum_base_linear_velocity_m_s": float(np.max(base_linear_speed)),
            "maximum_base_orientation_drift_rad": float(
                np.max(base_orientation_drift)
            ),
            "minimum_clearance_m": _safe_float(clearance_minimum),
            "maximum_momentum_delta": momentum_delta_maximum,
            "maximum_torque_nm": np.max(np.abs(torque_values), axis=0),
            "maximum_command_joint_acceleration_rad_s2": float(
                np.max(np.abs(task_acceleration))
            ),
            "maximum_command_joint_jerk_rad_s3": float(
                np.max(np.abs(task_jerk)) if task_jerk.size else 0.0
            ),
            "velocity_bound_conflict_count": int(
                np.count_nonzero(arrays["task_velocity_bound_conflict"])
            ),
            "torque_saturation_fraction": torque_saturation_fraction,
            "task_latency_p99_ms": float(
                1000.0 * np.percentile(arrays["task_latency_s"], 99.0)
            ),
            "maximum_hierarchy_linear_degradation_m_s": float(
                np.max(arrays["task_hierarchy_linear_degradation_m_s"])
            ),
            "maximum_hierarchy_angular_degradation_rad_s": float(
                np.max(arrays["task_hierarchy_angular_degradation_rad_s"])
            ),
            "maximum_momentum_map_residual": float(
                np.max(arrays["task_momentum_map_residual"])
            ),
            "qpos_write_count_after_initialization": 0,
            "qvel_write_count_after_initialization": 0,
        },
        "acceptance": acceptance,
        "trace": {
            "path": str(trace_path),
            "sha256": _sha256(trace_path),
        },
    }
    metrics_path = output / "metrics.json"
    metrics_path.write_text(
        json.dumps(metrics, indent=2, ensure_ascii=False, default=_json_default) + "\n",
        encoding="utf-8",
    )
    return metrics


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--capture-time", type=float, required=True)
    parser.add_argument("--arm-angle", type=float, required=True)
    parser.add_argument(
        "--controller-variant",
        choices=CONTROLLER_VARIANTS,
        default="full",
        help="Disable one level-2 objective for a controller ablation.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=PROJECT_ROOT / "output" / "fpmfc" / "precontact" / "manual_candidate",
    )
    args = parser.parse_args()
    config = load_fpmfc_config(args.config)
    result = run_precontact_candidate(
        config,
        capture_time_s=args.capture_time,
        terminal_arm_angle_rad=args.arm_angle,
        output_dir=args.output_dir,
        controller_variant=args.controller_variant,
    )
    print(
        json.dumps(
            {
                "acceptance": result["acceptance"],
                "metrics": result["metrics"],
                "output_dir": str(Path(args.output_dir).resolve()),
            },
            indent=2,
            ensure_ascii=False,
            default=_json_default,
        )
    )


if __name__ == "__main__":
    main()

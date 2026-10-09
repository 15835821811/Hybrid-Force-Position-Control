"""Run exactly one N110C C0/C1 torque-level precontact trajectory."""

from __future__ import annotations

import argparse
import copy
import json
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from ..collision import build_collision_pairs, minimum_signed_distance
from ..model import PROJECT_ROOT, body_id, default_model_spec, geom_id, site_id
from ..run import robot_momentum, servo_torque
from .config import load_fpmfc_config
from .controller import FPMFCHQP
from .dynamics import rotation_distance_rad
from .handoff_contract import DEFAULT_OUTPUT_ROOT, load_manifest, repo_path, sha256
from .handoff_trajectory import ContactHandoffTrajectory
from .run_capture import DynamicRunConfig, _constraint_config, _controller_config, _json_default
from .shape import ArmShapeKinematics
from .target import sync_mujoco_target, target_from_config


def _site_twist(model: mujoco.MjModel, data: mujoco.MjData, site: int) -> tuple[np.ndarray, np.ndarray]:
    linear = np.zeros((3, model.nv))
    angular = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, linear, angular, site)
    return linear @ data.qvel, angular @ data.qvel


def _trajectory(manifest: dict[str, Any], condition: str) -> ContactHandoffTrajectory:
    if condition not in ("C0", "C1"):
        raise ValueError("condition must be C0 or C1")
    initial = manifest["initial_reference"]
    final = manifest["terminal_reference"]
    return ContactHandoffTrajectory(
        initial_position_world_m=initial["flange_position_world_m"],
        final_position_world_m=final["flange_position_world_m"],
        initial_rotation_world=initial["flange_rotation_world"],
        final_rotation_world=final["flange_rotation_world"],
        initial_arm_angle_rad=initial["arm_angle_rad"],
        final_arm_angle_rad=manifest["source_candidate"]["terminal_arm_angle_rad"],
        duration_s=manifest["source_candidate"]["capture_time_s"],
        terminal_linear_velocity_world_m_s=final[f"{condition}_terminal_linear_velocity_world_m_s"],
        terminal_linear_acceleration_world_m_s2=final[f"{condition}_terminal_linear_acceleration_world_m_s2"],
        terminal_angular_velocity_world_rad_s=final[f"{condition}_terminal_angular_velocity_world_rad_s"],
        terminal_angular_acceleration_world_rad_s2=final[f"{condition}_terminal_angular_acceleration_world_rad_s2"],
    )


def run_precontact(condition: str, *, manifest_path: Path | str = DEFAULT_OUTPUT_ROOT/"pairing_manifest.json",
                   output_dir: Path | str | None = None) -> dict[str, Any]:
    manifest_path = Path(manifest_path).resolve()
    manifest = load_manifest(manifest_path)
    trajectory = _trajectory(manifest, condition)
    output = Path(output_dir or manifest_path.parent/"precontact"/condition).resolve()
    if (output/"trace.npz").exists() or (output/"metrics.json").exists():
        raise FileExistsError(f"precontact result already exists: {output}")
    output.mkdir(parents=True, exist_ok=True)
    precontact = load_fpmfc_config(repo_path(manifest["precontact_config_path"]))
    spec = default_model_spec()
    dt, task_dt = spec.timestep_s, spec.task_period_s
    steps = int(round(trajectory.duration_s/dt))
    if abs(steps*dt-trajectory.duration_s) > 1e-10:
        raise ValueError("archived capture time is not on the physics grid")
    task_stride = int(round(task_dt/dt))
    model = spec.compile_model()
    model.geom_pos[geom_id(model, "workspace_obstacle_0")] = [10.0, 10.0, 10.0]
    model.geom_contype[:] = 0
    model.geom_conaffinity[:] = 0
    data = mujoco.MjData(model)
    spec.reset_home(model, data)
    shape_cfg = precontact["shape"]
    shape = ArmShapeKinematics(
        spec, model, data,
        shoulder_joint=shape_cfg["shoulder_joint"],
        elbow_joint=shape_cfg["elbow_joint"],
        wrist_joint=shape_cfg["wrist_joint"],
        singularity_margin=float(shape_cfg["singularity_margin"]),
    )
    pairs = build_collision_pairs(model)
    controller = FPMFCHQP(
        spec, model, pairs, shape,
        controller_config=_controller_config(precontact),
        constraint_config=_constraint_config(precontact),
    )
    target = target_from_config(precontact)
    sync_mujoco_target(model, data, target, target.sample(0.0))
    mujoco.mj_forward(model, data)
    flange = site_id(model, "flange_site")
    base_body = body_id(model, "base_link_0")
    qpos_ids, dof_ids = spec.joint_addresses(model)
    _base_qpos, base_dof = spec.base_slices(model)
    np.testing.assert_array_equal(data.qpos, manifest["initial_reference"]["initial_qpos"])
    np.testing.assert_array_equal(data.qvel, manifest["initial_reference"]["initial_qvel"])
    initial_qpos = np.asarray(data.qpos).copy()
    initial_qvel = np.asarray(data.qvel).copy()
    initial_momentum = robot_momentum(model, data, base_body)
    initial_ctrl = np.asarray(data.ctrl).copy()
    initial_qacc_warmstart = np.asarray(data.qacc_warmstart).copy()
    run_cfg = DynamicRunConfig()
    reference_q = np.asarray(data.qpos[qpos_ids]).copy()
    command_velocity = np.zeros(7)
    segment_start = command_velocity.copy()
    segment_step = 0
    full_mass = np.zeros((model.nv, model.nv))
    saturation_count = 0
    records: dict[str, list[Any]] = {}

    def record(name: str, value: Any) -> None:
        records.setdefault(name, []).append(np.asarray(value).copy())

    started = time.perf_counter()
    for step in range(steps):
        current_time = float(data.time)
        reference = trajectory.sample(current_time)
        target_now = target.sample(current_time)
        sync_mujoco_target(model, data, target, target_now)
        pre_mocap_pos = np.asarray(data.mocap_pos).copy()
        pre_mocap_quat = np.asarray(data.mocap_quat).copy()
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
            segment_start = command_velocity.copy()
            command_velocity = result.joint_velocity.copy()
            segment_step = 0
            record("task_time", current_time)
            record("task_tick_physics_step", step)
            record("task_success", result.success)
            record("task_primary_feasible", result.primary_feasible)
            record("task_secondary_feasible", result.secondary_feasible)
            record("task_minimum_constraint_slack", result.minimum_constraint_slack)
            record("task_joint_velocity_rad_s", command_velocity)
            record("task_joint_acceleration_rad_s2", controller.previous_acceleration)
            record("task_velocity_bound_conflict", controller.last_bound_conflicts)
            record("task_joint_velocity_lower_rad_s", controller.last_velocity_lower)
            record("task_joint_velocity_upper_rad_s", controller.last_velocity_upper)
        interpolation = (segment_step+1)/task_stride
        reference_dq = segment_start+interpolation*(command_velocity-segment_start)
        reference_ddq = (command_velocity-segment_start)/task_dt
        reference_q = np.clip(reference_q+reference_dq*dt, controller.joint_lower, controller.joint_upper)
        measured_q = np.asarray(data.qpos[qpos_ids])
        reference_q = np.clip(
            reference_q,
            measured_q-run_cfg.reference_tracking_band_rad,
            measured_q+run_cfg.reference_tracking_band_rad,
        )
        torque, _ = servo_torque(
            model, data, dof_ids, base_dof, qpos_ids,
            reference_q, reference_dq, reference_ddq,
            spec.torque_limits_nm, run_cfg.servo_natural_frequency_rad_s,
            run_cfg.servo_acceleration_limit_rad_s2, full_mass,
        )
        saturation_count += int(np.count_nonzero(np.abs(torque) >= spec.torque_limits_nm-1e-9))
        data.ctrl[:] = torque
        mujoco.mj_step(model, data)
        segment_step += 1

        # Target mocap belongs to absolute time, while the robot state is the
        # actual torque-integrated endpoint. Synchronize only a copied MjData.
        now = float(data.time)
        post = copy.copy(data)
        target_sample = target.sample(now)
        sync_mujoco_target(model, post, target, target_sample)
        mujoco.mj_forward(model, post)
        ref_now = trajectory.sample(now)
        flange_position = np.asarray(post.site_xpos[flange]).copy()
        flange_rotation = np.asarray(post.site_xmat[flange]).reshape(3, 3).copy()
        flange_linear, flange_angular = _site_twist(model, post, flange)
        recess = float(manifest["terminal_reference"]["tool_face_recess_m"])
        tool_face_position = flange_position-flange_rotation[:, 2]*recess
        tool_face_linear = flange_linear+np.cross(flange_angular, tool_face_position-flange_position)
        clearance, _ = minimum_signed_distance(model, post, pairs)
        record("time", now)
        record("qpos", post.qpos)
        record("qvel", post.qvel)
        record("torque", torque)
        record("mocap_pos_pre_step", pre_mocap_pos)
        record("mocap_quat_pre_step", pre_mocap_quat)
        record("reference_q", reference_q)
        record("reference_dq", reference_dq)
        record("reference_ddq", reference_ddq)
        record("flange_position", flange_position)
        record("flange_rotation", flange_rotation)
        record("flange_linear_velocity_world_m_s", flange_linear)
        record("flange_angular_velocity_world_rad_s", flange_angular)
        record("tool_face_position_world_m", tool_face_position)
        record("tool_face_linear_velocity_world_m_s", tool_face_linear)
        record("target_center_position", target_sample.center_position_world_m)
        record("target_center_rotation", target_sample.center_rotation_world)
        record("target_grasp_position", target_sample.grasp_position_world_m)
        record("target_grasp_rotation", target_sample.grasp_rotation_world)
        record("target_grasp_linear_velocity_world_m_s", target_sample.grasp_linear_velocity_world_m_s)
        record("target_grasp_angular_velocity_world_rad_s", target_sample.grasp_angular_velocity_world_rad_s)
        record("desired_position", ref_now.position_world_m)
        record("desired_rotation", ref_now.rotation_world)
        record("desired_linear_velocity_world_m_s", ref_now.linear_velocity_world_m_s)
        record("desired_linear_acceleration_world_m_s2", ref_now.linear_acceleration_world_m_s2)
        record("desired_angular_velocity_world_rad_s", ref_now.angular_velocity_world_rad_s)
        record("desired_angular_acceleration_world_rad_s2", ref_now.angular_acceleration_world_rad_s2)
        record("desired_arm_angle_rad", ref_now.arm_angle_rad)
        record("arm_angle_rad", shape.sample(post).angle_rad)
        record("position_error_m", np.linalg.norm(ref_now.position_world_m-flange_position))
        record("orientation_error_rad", rotation_distance_rad(ref_now.rotation_world, flange_rotation))
        record("flange_reference_linear_velocity_error_m_s", np.linalg.norm(flange_linear-ref_now.linear_velocity_world_m_s))
        record("flange_reference_angular_velocity_error_rad_s", np.linalg.norm(flange_angular-ref_now.angular_velocity_world_rad_s))
        record("tool_face_relative_linear_speed_m_s", np.linalg.norm(tool_face_linear-target_sample.grasp_linear_velocity_world_m_s))
        record("relative_angular_speed_rad_s", np.linalg.norm(flange_angular-target_sample.grasp_angular_velocity_world_rad_s))
        record("minimum_clearance_m", clearance)
        record("momentum", robot_momentum(model, post, base_body))
        record("base_twist", post.qvel[base_dof])

    arrays = {name: np.asarray(values) for name, values in records.items()}
    arrays.update(
        initial_qpos=initial_qpos, initial_qvel=initial_qvel,
        initial_momentum=initial_momentum, initial_ctrl=initial_ctrl,
        initial_qacc_warmstart=initial_qacc_warmstart,
        capture_time_s=np.asarray(trajectory.duration_s),
        terminal_arm_angle_rad=np.asarray(trajectory.final_arm_angle_rad),
    )
    trace_path = output/"trace.npz"
    np.savez_compressed(trace_path, **arrays)
    final_shape_error = abs((trajectory.final_arm_angle_rad-float(arrays["arm_angle_rad"][-1])+np.pi)%(2*np.pi)-np.pi)
    final_linear_speed = float(np.linalg.norm(arrays["flange_linear_velocity_world_m_s"][-1]))
    final_angular_speed = float(np.linalg.norm(arrays["flange_angular_velocity_world_rad_s"][-1]))
    task_jerk = np.diff(arrays["task_joint_acceleration_rad_s2"], axis=0)/task_dt
    momentum_delta = arrays["momentum"]-initial_momentum[None, :]
    tracking_last_01 = arrays["time"] >= trajectory.duration_s-0.1-1e-10
    gate = manifest["handoff_gates"]
    old_acceptance = precontact["acceptance"]
    metrics_values = {
        "terminal_position_error_m": float(arrays["position_error_m"][-1]),
        "terminal_orientation_error_rad": float(arrays["orientation_error_rad"][-1]),
        "terminal_arm_angle_error_rad": final_shape_error,
        "terminal_absolute_linear_speed_m_s": final_linear_speed,
        "terminal_absolute_angular_speed_rad_s": final_angular_speed,
        "terminal_tool_face_relative_linear_speed_m_s": float(arrays["tool_face_relative_linear_speed_m_s"][-1]),
        "terminal_relative_angular_speed_rad_s": float(arrays["relative_angular_speed_rad_s"][-1]),
        "terminal_flange_reference_linear_velocity_error_m_s": float(arrays["flange_reference_linear_velocity_error_m_s"][-1]),
        "terminal_flange_reference_angular_velocity_error_rad_s": float(arrays["flange_reference_angular_velocity_error_rad_s"][-1]),
        "last_0p1s_relative_linear_speed_max_m_s": float(np.max(arrays["tool_face_relative_linear_speed_m_s"][tracking_last_01])),
        "last_0p1s_relative_linear_speed_rms_m_s": float(np.sqrt(np.mean(arrays["tool_face_relative_linear_speed_m_s"][tracking_last_01]**2))),
        "last_0p1s_position_error_max_m": float(np.max(arrays["position_error_m"][tracking_last_01])),
        "last_0p1s_orientation_error_max_rad": float(np.max(arrays["orientation_error_rad"][tracking_last_01])),
        "tracking_position_rms_m": float(np.sqrt(np.mean(arrays["position_error_m"]**2))),
        "tracking_orientation_max_rad": float(np.max(arrays["orientation_error_rad"])),
        "minimum_clearance_m": float(np.min(arrays["minimum_clearance_m"])),
        "task_success_rate": float(np.mean(arrays["task_success"])),
        "torque_saturation_fraction": saturation_count/(7*steps),
        "maximum_momentum_delta": float(np.max(np.linalg.norm(momentum_delta, axis=1))),
        "maximum_command_joint_jerk_rad_s3": float(np.max(np.abs(task_jerk))),
        "velocity_bound_conflict_count": int(np.count_nonzero(arrays["task_velocity_bound_conflict"])),
        "initial_qpos_qvel_write_count_after_initialization": 0,
    }
    common = {
        "task_success_rate": metrics_values["task_success_rate"] >= float(old_acceptance["minimum_task_success_rate"]),
        "terminal_position_error": metrics_values["terminal_position_error_m"] <= float(gate["position_error_m"]),
        "terminal_orientation_error": metrics_values["terminal_orientation_error_rad"] <= float(gate["orientation_error_rad"]),
        "terminal_arm_angle_error": final_shape_error <= float(gate["arm_angle_error_rad"]),
        "minimum_clearance": metrics_values["minimum_clearance_m"] >= float(old_acceptance["minimum_clearance_m"]),
        "torque_saturation_fraction": metrics_values["torque_saturation_fraction"] <= float(old_acceptance["maximum_torque_saturation_fraction"]),
        "momentum_delta": metrics_values["maximum_momentum_delta"] <= float(old_acceptance["maximum_momentum_delta"]),
        "command_joint_jerk": metrics_values["maximum_command_joint_jerk_rad_s3"] <= float(precontact["controller"]["joint_jerk_limit_rad_s3"])+1e-8,
        "velocity_bound_compatibility": metrics_values["velocity_bound_conflict_count"] == 0,
    }
    if condition == "C0":
        specific = {
            "zero_terminal_linear_speed": final_linear_speed <= float(gate["C0_absolute_linear_speed_m_s"]),
            "zero_terminal_angular_speed": final_angular_speed <= float(gate["C0_absolute_angular_speed_rad_s"]),
        }
    else:
        specific = {
            "tool_face_relative_linear_speed": metrics_values["terminal_tool_face_relative_linear_speed_m_s"] <= float(gate["C1_tool_face_relative_linear_speed_m_s"]),
            "relative_angular_speed": metrics_values["terminal_relative_angular_speed_rad_s"] <= float(gate["C1_relative_angular_speed_rad_s"]),
        }
    acceptance = {**common, **specific, "common_passed": bool(all(common.values())),
                  "handoff_passed": bool(all(common.values()) and all(specific.values()))}
    config_snapshot = {
        "schema_version": "n110c_precontact_config_snapshot_v1",
        "condition": condition,
        "pairing_manifest_sha256": sha256(manifest_path),
        "precontact_config": precontact,
        "run_config": asdict(run_cfg),
        "terminal_reference": manifest["terminal_reference"],
        "implementation_identity": manifest["n110c_implementation_identity"],
    }
    snapshot_path = output/"config_snapshot.json"
    snapshot_path.write_text(json.dumps(config_snapshot, ensure_ascii=False, indent=2, default=_json_default)+"\n", encoding="utf-8")
    metrics = {
        "schema_version": "n110c_precontact_dynamics_v1",
        "condition": condition,
        "source_candidate": manifest["source_candidate"],
        "pairing_manifest_sha256": sha256(manifest_path),
        "config_snapshot_sha256": sha256(snapshot_path),
        "implementation_identity": manifest["n110c_implementation_identity"],
        "model_identity": spec.identity(),
        "controller": "frozen FPMFCHQP full; same parameters for C0/C1",
        "integrator": "RK4", "physics_steps": steps,
        "task_ticks": len(arrays["task_time"]),
        "elapsed_wall_s": time.perf_counter()-started,
        "trace_sha256": sha256(trace_path),
        "metrics": metrics_values,
        "acceptance": acceptance,
        "precontact_contact_policy": "contact disabled; full torque-level dynamics; target mocap at absolute time",
    }
    (output/"metrics.json").write_text(json.dumps(metrics, ensure_ascii=False, indent=2, default=_json_default)+"\n", encoding="utf-8")
    return metrics


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--condition", choices=("C0", "C1"), required=True)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_OUTPUT_ROOT/"pairing_manifest.json")
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    result = run_precontact(args.condition, manifest_path=args.manifest, output_dir=args.output_dir)
    print(json.dumps({"condition": result["condition"], "metrics": result["metrics"],
                      "acceptance": result["acceptance"]}, indent=2))


if __name__ == "__main__":
    main()

"""Independent torque replay and synchronized terminal check for N110C."""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from ..collision import build_collision_pairs, minimum_signed_distance
from ..model import default_model_spec, geom_id, site_id
from .config import load_fpmfc_config
from .handoff_contract import DEFAULT_OUTPUT_ROOT, load_manifest, repo_path, sha256
from .handoff_provenance import n110c_verification_identity
from .run_handoff_precontact import _site_twist, _trajectory
from .target import sync_mujoco_target, target_from_config


def validate_precontact(condition: str, *, manifest_path: Path | str = DEFAULT_OUTPUT_ROOT/"pairing_manifest.json",
                        output_dir: Path | str | None = None) -> dict[str, Any]:
    manifest_path = Path(manifest_path).resolve()
    manifest = load_manifest(manifest_path)
    if condition not in ("C0", "C1"):
        raise ValueError("condition must be C0 or C1")
    output = Path(output_dir or manifest_path.parent/"precontact"/condition).resolve()
    metrics = json.loads((output/"metrics.json").read_text(encoding="utf-8"))
    trace_path = output/"trace.npz"
    spec = default_model_spec()
    model = spec.compile_model()
    model.geom_pos[geom_id(model, "workspace_obstacle_0")] = [10.0, 10.0, 10.0]
    model.geom_contype[:] = 0
    model.geom_conaffinity[:] = 0
    data = mujoco.MjData(model)
    precontact = load_fpmfc_config(repo_path(manifest["precontact_config_path"]))
    target = target_from_config(precontact)
    trajectory = _trajectory(manifest, condition)
    flange = site_id(model, "flange_site")
    pairs = build_collision_pairs(model)
    stride = int(round(spec.task_period_s/spec.timestep_s))
    tolerance = manifest["replay_tolerances"]
    maximum = {name: 0.0 for name in (
        "qpos", "qvel", "flange_position_m", "flange_linear_twist",
        "flange_angular_twist", "target_grasp_position_m", "reference_position_m",
        "reference_linear_velocity_m_s", "reference_angular_velocity_rad_s",
        "clearance_m", "time_s",
    )}
    with np.load(trace_path, allow_pickle=False) as trace:
        data.qpos[:] = trace["initial_qpos"]
        data.qvel[:] = trace["initial_qvel"]
        data.ctrl[:] = trace["initial_ctrl"]
        data.qacc_warmstart[:] = trace["initial_qacc_warmstart"]
        initial_state_matches_manifest = (
            np.array_equal(data.qpos, manifest["initial_reference"]["initial_qpos"])
            and np.array_equal(data.qvel, manifest["initial_reference"]["initial_qvel"])
        )
        task_steps = np.asarray(trace["task_tick_physics_step"], dtype=int)
        count = len(trace["time"])
        task_schedule = np.array_equal(task_steps, np.arange(0, count, stride))
        pre_mocap_time_consistent = True
        for i, torque in enumerate(trace["torque"]):
            data.mocap_pos[:] = trace["mocap_pos_pre_step"][i]
            data.mocap_quat[:] = trace["mocap_quat_pre_step"][i]
            prescribed = target.sample(float(data.time))
            expected_mocap = copy.copy(data)
            sync_mujoco_target(model, expected_mocap, target, prescribed)
            pre_mocap_time_consistent &= np.max(np.abs(expected_mocap.mocap_pos-data.mocap_pos)) < 1e-12
            pre_mocap_time_consistent &= np.max(np.abs(expected_mocap.mocap_quat-data.mocap_quat)) < 1e-12
            # The controller forwards once at each task tick; the frozen
            # torque servo forwards once every physics step, both before the
            # new torque is applied. This reproduces solver warm-start input.
            if i % stride == 0:
                mujoco.mj_forward(model, data)
            mujoco.mj_forward(model, data)
            data.ctrl[:] = torque
            mujoco.mj_step(model, data)
            post = copy.copy(data)
            target_now = target.sample(float(data.time))
            sync_mujoco_target(model, post, target, target_now)
            mujoco.mj_forward(model, post)
            ref = trajectory.sample(float(data.time))
            flange_position = np.asarray(post.site_xpos[flange])
            linear, angular = _site_twist(model, post, flange)
            clearance, _ = minimum_signed_distance(model, post, pairs)
            errors = {
                "qpos": float(np.max(np.abs(post.qpos-trace["qpos"][i]))),
                "qvel": float(np.max(np.abs(post.qvel-trace["qvel"][i]))),
                "flange_position_m": float(np.linalg.norm(flange_position-trace["flange_position"][i])),
                "flange_linear_twist": float(np.linalg.norm(linear-trace["flange_linear_velocity_world_m_s"][i])),
                "flange_angular_twist": float(np.linalg.norm(angular-trace["flange_angular_velocity_world_rad_s"][i])),
                "target_grasp_position_m": float(np.linalg.norm(target_now.grasp_position_world_m-trace["target_grasp_position"][i])),
                "reference_position_m": float(np.linalg.norm(ref.position_world_m-trace["desired_position"][i])),
                "reference_linear_velocity_m_s": float(np.linalg.norm(ref.linear_velocity_world_m_s-trace["desired_linear_velocity_world_m_s"][i])),
                "reference_angular_velocity_rad_s": float(np.linalg.norm(ref.angular_velocity_world_rad_s-trace["desired_angular_velocity_world_rad_s"][i])),
                "clearance_m": abs(float(clearance)-float(trace["minimum_clearance_m"][i])),
                "time_s": abs(float(data.time)-float(trace["time"][i])),
            }
            for key, value in errors.items():
                maximum[key] = max(maximum[key], value)
        checks = {
            "trace_hash": sha256(trace_path) == metrics["trace_sha256"],
            "config_snapshot_hash": sha256(output/"config_snapshot.json") == metrics["config_snapshot_sha256"],
            "frozen_manifest_hash": sha256(manifest_path) == metrics["pairing_manifest_sha256"],
            "current_implementation_identity": metrics["implementation_identity"] == manifest["n110c_implementation_identity"],
            "initial_state_matches_manifest": initial_state_matches_manifest,
            "recorded_mocap_matches_absolute_time": bool(pre_mocap_time_consistent),
            "fixed_task_schedule": task_schedule and len(task_steps) == int(round(trajectory.duration_s/spec.task_period_s)),
            "qpos_strict_replay": maximum["qpos"] <= tolerance["qpos"],
            "qvel_strict_replay": maximum["qvel"] <= tolerance["qvel"],
            "flange_position_strict_replay": maximum["flange_position_m"] <= tolerance["flange_position_m"],
            "flange_twist_strict_replay": max(maximum["flange_linear_twist"], maximum["flange_angular_twist"]) <= tolerance["flange_twist"],
            "clearance_strict_replay": maximum["clearance_m"] <= tolerance["clearance_m"],
            "target_absolute_phase": maximum["target_grasp_position_m"] <= 1e-11,
            "reference_recomputation": max(maximum[k] for k in (
                "reference_position_m", "reference_linear_velocity_m_s", "reference_angular_velocity_rad_s",
            )) <= 1e-11,
            "time_grid": maximum["time_s"] <= 1e-12 and count == int(round(trajectory.duration_s/spec.timestep_s)),
            "source_history_available": all(key in trace.files for key in (
                "reference_q", "reference_dq", "reference_ddq",
                "task_joint_velocity_rad_s", "task_joint_acceleration_rad_s2", "terminal_arm_angle_rad",
            )),
        }
    result = {
        "schema_version": "n110c_precontact_replay_validation_v1",
        "condition": condition,
        "replay_passed": bool(all(checks.values())),
        "handoff_passed": bool(metrics["acceptance"]["handoff_passed"]),
        "checks": checks,
        "maximum_errors": maximum,
        "tolerances_frozen_before_runs": tolerance,
        "trace_sha256": sha256(trace_path),
        "metrics_sha256": sha256(output/"metrics.json"),
        "verification_identity": n110c_verification_identity(),
    }
    (output/"validation.json").write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    artifacts = {name: sha256(output/name) for name in (
        "trace.npz", "metrics.json", "config_snapshot.json", "validation.json",
    )}
    (output/"artifact_manifest.json").write_text(
        json.dumps({"schema_version": "n110c_precontact_artifacts_v1", "artifacts": artifacts}, ensure_ascii=False, indent=2)+"\n",
        encoding="utf-8",
    )
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--condition", choices=("C0", "C1"), required=True)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_OUTPUT_ROOT/"pairing_manifest.json")
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    result = validate_precontact(args.condition, manifest_path=args.manifest, output_dir=args.output_dir)
    print(json.dumps({"condition": result["condition"], "replay_passed": result["replay_passed"],
                      "handoff_passed": result["handoff_passed"], "checks": result["checks"],
                      "maximum_errors": result["maximum_errors"]}, indent=2))
    if not result["replay_passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

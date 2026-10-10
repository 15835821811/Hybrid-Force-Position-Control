"""Gate and run N110C contact comparisons using the frozen S controller."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from ..model import default_model_spec, geom_id
from .config import load_fpmfc_config
from .contact_config import load_contact_config
from .contact_model import default_contact_model_spec
from .contact_observation import ContactObserver
from .handoff_contract import DEFAULT_OUTPUT_ROOT, load_manifest, repo_path, sha256
from .run_contact_consistent import initialize_contact_state, run_consistent_contact_experiment


VARIANTS = ("rigid", "admittance", "admittance-no-shape")


def contact_preflight(condition: str, *, manifest_path: Path | str = DEFAULT_OUTPUT_ROOT/"pairing_manifest.json") -> dict[str, Any]:
    path = Path(manifest_path).resolve()
    manifest = load_manifest(path)
    if condition not in ("C0", "C1"):
        raise ValueError("condition must be C0 or C1")
    source = path.parent/"precontact"/condition
    metrics = json.loads((source/"metrics.json").read_text(encoding="utf-8"))
    validation = json.loads((source/"validation.json").read_text(encoding="utf-8"))
    source_qualified = bool(metrics["acceptance"]["handoff_passed"] and validation["replay_passed"])
    other = "C1" if condition == "C0" else "C0"
    other_source = path.parent/"precontact"/other
    other_metrics = json.loads((other_source/"metrics.json").read_text(encoding="utf-8"))
    other_validation = json.loads((other_source/"validation.json").read_text(encoding="utf-8"))
    both_qualified = source_qualified and bool(other_metrics["acceptance"]["handoff_passed"] and other_validation["replay_passed"])
    precontact = load_fpmfc_config(repo_path(manifest["precontact_config_path"]))
    contact_config = load_contact_config(repo_path(manifest["contact_config_path"]))
    spec = default_contact_model_spec()
    model = spec.compile_model()
    model.geom_pos[geom_id(model, "workspace_obstacle_0")] = [10.0, 10.0, 10.0]
    data = mujoco.MjData(model)
    spec.reset_home(model, data)
    transfer = initialize_contact_state(
        source_trace_path=source/"trace.npz", precontact_config=precontact,
        contact_spec=spec, contact_model=model, contact_data=data,
    )
    observer = ContactObserver(
        model, tool_face_recess_m=float(contact_config["interface"]["tool_pad_face_recess_m"]),
    )
    initial = observer.observe(data)
    source_spec = default_model_spec()
    source_model = source_spec.compile_model()
    source_base_qpos, source_base_dof = source_spec.base_slices(source_model)
    contact_base_qpos, contact_base_dof = spec.base_slices(model)
    source_joint_qpos, source_joint_dof = source_spec.joint_addresses(source_model)
    contact_joint_qpos, contact_joint_dof = spec.joint_addresses(model)
    with np.load(source/"trace.npz", allow_pickle=False) as trace:
        last_qpos = trace["qpos"][-1]
        last_qvel = trace["qvel"][-1]
        base_qpos_error = float(np.max(np.abs(last_qpos[source_base_qpos]-initial.qpos[contact_base_qpos])))
        base_qvel_error = float(np.max(np.abs(last_qvel[source_base_dof]-initial.qvel[contact_base_dof])))
        joint_qpos_error = float(np.max(np.abs(last_qpos[source_joint_qpos]-initial.qpos[contact_joint_qpos])))
        joint_qvel_error = float(np.max(np.abs(last_qvel[source_joint_dof]-initial.qvel[contact_joint_dof])))
        source_terminal_torque = trace["torque"][-1].tolist()
        source_reference_dq = trace["reference_dq"][-1].tolist()
        source_reference_ddq = trace["reference_ddq"][-1].tolist()
    gap_normal = float((initial.tool_face_position_world_m-initial.grasp_position_world_m) @ initial.contact_normal_world)
    relative_tool_speed = float(np.linalg.norm(initial.tool_face_linear_velocity_world_m_s-initial.grasp_linear_velocity_world_m_s))
    relative_angular_speed = float(np.linalg.norm(initial.relative_angular_velocity_world_rad_s))
    gate = manifest["handoff_gates"]
    checks = {
        "both_source_handoffs_and_replays_qualified": both_qualified,
        "base_qpos_preserved": base_qpos_error == 0.0,
        "base_qvel_preserved": base_qvel_error == 0.0,
        "joint_qpos_preserved": joint_qpos_error == 0.0,
        "joint_qvel_preserved": joint_qvel_error == 0.0,
        "initial_penetration": initial.penetration_m <= float(gate["initial_contact_max_penetration_m"]),
        "initial_contact_force": initial.measured_normal_force_n <= float(gate["initial_contact_max_force_n"]),
        "target_absolute_phase": max(transfer["target_transfer_errors"].values()) <= 2e-10,
    }
    result = {
        "schema_version": "n110c_contact_preflight_v1",
        "condition": condition, "pairing_manifest_sha256": sha256(path),
        "source_trace_sha256": sha256(source/"trace.npz"),
        "source_qualified": source_qualified, "both_qualified": both_qualified,
        "checks": checks, "passed": bool(all(checks.values())),
        "base_qpos_max_abs_difference": base_qpos_error,
        "base_qvel_max_abs_difference": base_qvel_error,
        "joint_qpos_max_abs_difference": joint_qpos_error,
        "joint_qvel_max_abs_difference": joint_qvel_error,
        "tool_face_to_grasp_normal_gap_m": gap_normal,
        "initial_penetration_m": initial.penetration_m,
        "initial_contact_force_n": initial.measured_normal_force_n,
        "initial_contact_count": initial.contact_count,
        "initial_tool_face_relative_linear_speed_m_s": relative_tool_speed,
        "initial_relative_angular_speed_rad_s": relative_angular_speed,
        "initial_flange_position_world_m": initial.flange_position_world_m.tolist(),
        "initial_tool_face_position_world_m": initial.tool_face_position_world_m.tolist(),
        "initial_grasp_position_world_m": initial.grasp_position_world_m.tolist(),
        "initial_target_grasp_rotation_world": initial.grasp_rotation_world.tolist(),
        "initial_target_grasp_linear_velocity_world_m_s": initial.grasp_linear_velocity_world_m_s.tolist(),
        "initial_target_grasp_angular_velocity_world_rad_s": initial.grasp_angular_velocity_world_rad_s.tolist(),
        "source_terminal_torque_nm": source_terminal_torque,
        "source_reference_dq_rad_s": source_reference_dq,
        "source_reference_ddq_rad_s2": source_reference_ddq,
        "transfer": transfer,
        "contact_local_time_s": 0.0,
        "target_absolute_time_s": manifest["source_candidate"]["capture_time_s"],
    }
    # Transfer contains arrays; remove those already stored in the source trace.
    result["transfer"] = {key: value for key, value in transfer.items()
                          if not isinstance(value, np.ndarray)}
    destination = path.parent/"contact"/condition
    destination.mkdir(parents=True, exist_ok=True)
    (destination/"preflight.json").write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    return result


def run_contact(condition: str, variant: str, *, manifest_path: Path | str = DEFAULT_OUTPUT_ROOT/"pairing_manifest.json") -> dict[str, Any]:
    if variant not in VARIANTS:
        raise ValueError(f"unknown variant: {variant}")
    manifest_path = Path(manifest_path).resolve()
    manifest = load_manifest(manifest_path)
    preflight = contact_preflight(condition, manifest_path=manifest_path)
    if not preflight["passed"]:
        raise RuntimeError(f"contact preflight failed for {condition}: {preflight['checks']}")
    source = manifest_path.parent/"precontact"/condition/"trace.npz"
    output = manifest_path.parent/"contact"/condition/variant
    result = run_consistent_contact_experiment(
        variant=variant, source_trace_path=source,
        contact_config_path=repo_path(manifest["contact_config_path"]),
        output_dir=output, mapping_mode="homogeneous",
    )
    snapshot_path = output/"config_snapshot.json"
    snapshot = json.loads(snapshot_path.read_text(encoding="utf-8"))
    snapshot["n110c_pairing_manifest_sha256"] = sha256(manifest_path)
    snapshot["n110c_implementation_identity"] = manifest["n110c_implementation_identity"]
    snapshot["n110c_condition"] = condition
    snapshot_path.write_text(json.dumps(snapshot, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    with np.load(source, allow_pickle=False) as pre, np.load(output/"trace.npz", allow_pickle=False) as contact:
        torque_jump = float(np.linalg.norm(contact["torque"][0]-pre["torque"][-1]))
        command_dq_jump = float(np.linalg.norm(contact["reference_dq"][0]-pre["reference_dq"][-1]))
        command_ddq_jump = float(np.linalg.norm(contact["reference_ddq"][0]-pre["reference_ddq"][-1]))
        detected_indices = np.flatnonzero(contact["contact_detected"])
        first_contact_time = float(contact["time"][detected_indices[0]]) if len(detected_indices) else None
    result["schema_version"] = "n110c_contact_dynamics_v1"
    result["n110c_condition"] = condition
    result["n110c_pairing_manifest_sha256"] = sha256(manifest_path)
    result["n110c_implementation_identity"] = manifest["n110c_implementation_identity"]
    result["n110c_contact_preflight"] = preflight
    result["metrics"]["first_contact_time_local_s"] = first_contact_time
    result["metrics"]["ever_contact"] = first_contact_time is not None
    result["handoff_command_jumps"] = {
        "last_precontact_to_first_contact_torque_norm_nm": torque_jump,
        "last_precontact_to_first_contact_reference_dq_norm_rad_s": command_dq_jump,
        "last_precontact_to_first_contact_reference_ddq_norm_rad_s2": command_ddq_jump,
        "initial_contact_feedback_torque": "zero, as in frozen S runner",
    }
    result["config_snapshot_sha256"] = sha256(snapshot_path)
    (output/"metrics.json").write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--condition", choices=("C0", "C1"), required=True)
    parser.add_argument("--variant", choices=VARIANTS)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_OUTPUT_ROOT/"pairing_manifest.json")
    parser.add_argument("--preflight-only", action="store_true")
    args = parser.parse_args()
    if args.preflight_only:
        result = contact_preflight(args.condition, manifest_path=args.manifest)
        print(json.dumps({"condition": args.condition, "preflight_passed": result["passed"],
                          "checks": result["checks"]}, indent=2))
        if not result["passed"]:
            raise SystemExit(1)
    else:
        if args.variant is None:
            parser.error("--variant is required unless --preflight-only")
        result = run_contact(args.condition, args.variant, manifest_path=args.manifest)
        print(json.dumps({"condition": args.condition, "variant": args.variant,
                          "metrics": result["metrics"], "acceptance": result["acceptance"]}, indent=2))


if __name__ == "__main__":
    main()

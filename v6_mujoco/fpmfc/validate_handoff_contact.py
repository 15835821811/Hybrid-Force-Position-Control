"""Verify N110C paired contact identity, target phase, and S torque replay."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from ..model import geom_id
from .contact_model import default_contact_model_spec
from .contact_observation import ContactObserver
from .contact_config import load_contact_config
from .handoff_contract import DEFAULT_OUTPUT_ROOT, load_manifest, repo_path, sha256
from .handoff_provenance import n110c_verification_identity
from .validate_contact_consistent import validate_consistent_output


def validate_contact(condition: str, variant: str, *,
                     manifest_path: Path | str = DEFAULT_OUTPUT_ROOT/"pairing_manifest.json") -> dict[str, Any]:
    path = Path(manifest_path).resolve()
    manifest = load_manifest(path)
    output = path.parent/"contact"/condition/variant
    metrics = json.loads((output/"metrics.json").read_text(encoding="utf-8"))
    source = path.parent/"precontact"/condition
    source_metrics = json.loads((source/"metrics.json").read_text(encoding="utf-8"))
    source_validation = json.loads((source/"validation.json").read_text(encoding="utf-8"))
    S_replay = validate_consistent_output(output)
    config = load_contact_config(repo_path(manifest["contact_config_path"]))
    spec = default_contact_model_spec()
    model = spec.compile_model()
    model.geom_pos[geom_id(model, "workspace_obstacle_0")] = [10.0, 10.0, 10.0]
    data = mujoco.MjData(model)
    with np.load(output/"trace.npz", allow_pickle=False) as trace:
        data.qpos[:] = trace["initial_qpos"]
        data.qvel[:] = trace["initial_qvel"]
        data.ctrl[:] = trace["initial_ctrl"]
        observer = ContactObserver(model, tool_face_recess_m=float(config["interface"]["tool_pad_face_recess_m"]))
        initial = observer.observe(data)
        terminal = manifest["terminal_reference"]
        target_position_error = float(np.linalg.norm(initial.grasp_position_world_m-terminal["target_grasp_position_world_m"]))
        target_rotation_error = float(np.linalg.norm(initial.grasp_rotation_world-terminal["flange_rotation_world"]))
        detected = np.asarray(trace["contact_detected"], dtype=bool)
        ever_contact = bool(np.any(detected))
        first_contact_time = float(trace["time"][np.flatnonzero(detected)[0]]) if ever_contact else None
        local_time_zero = abs(float(trace["feedback_time_s"][0])) <= 1e-12
        initial_desired_force_zero = abs(float(trace["command_desired_normal_force_n"][0])) <= 1e-12
        initial_feedback_control_zero = bool(np.array_equal(trace["feedback_ctrl_used_for_forward"][0], np.zeros(model.nu)))
        physics_steps = len(trace["time"])
    checks = {
        "S_independent_torque_replay": bool(S_replay["passed"]),
        "source_precontact_replay": bool(source_validation["replay_passed"]),
        "source_handoff_qualified": bool(source_metrics["acceptance"]["handoff_passed"]),
        "group_S_homogeneous": metrics["group"] == "S" and metrics["mapping_mode"] == "homogeneous",
        "condition_identity": metrics.get("n110c_condition") == condition,
        "frozen_manifest_hash": metrics.get("n110c_pairing_manifest_sha256") == sha256(path),
        "new_runtime_identity": metrics.get("n110c_implementation_identity") == manifest["n110c_implementation_identity"],
        "source_trace_hash": metrics["source_handoff"]["source_trace_sha256"] == sha256(source/"trace.npz"),
        "target_absolute_phase_position": target_position_error <= 1e-10,
        "target_absolute_phase_rotation": target_rotation_error <= 1e-10,
        "contact_local_time_zero": local_time_zero,
        "force_ramp_local_time_zero": initial_desired_force_zero,
        "S_initial_feedback_zero_held_torque": initial_feedback_control_zero,
        "contact_acquired_gate_consistency": ever_contact == bool(metrics["acceptance"]["contact_acquired"]),
        "fixed_1s_contact_grid": physics_steps == 500,
    }
    result = {
        **S_replay,
        "schema_version": "n110c_contact_validation_v1",
        "condition": condition,
        "n110c_checks": checks,
        "n110c_implementation_identity": manifest["n110c_implementation_identity"],
        "n110c_verification_identity": n110c_verification_identity(),
        "target_absolute_phase_errors": {"position_m": target_position_error,
                                         "rotation_frobenius": target_rotation_error},
        "first_contact_time_local_s": first_contact_time,
        "ever_contact": ever_contact,
        "passed": bool(S_replay["passed"] and all(checks.values())),
    }
    (output/"validation.json").write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    (output/"artifact_manifest.json").write_text(json.dumps({
        "schema_version": "n110c_contact_artifacts_v1",
        "verification_status": "VERIFIED" if result["passed"] else "FAILED",
        "artifacts": {name: sha256(output/name) for name in (
            "config_snapshot.json", "trace.npz", "metrics.json", "validation.json",
        )},
    }, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--condition", choices=("C0", "C1"), required=True)
    parser.add_argument("--variant", choices=("rigid", "admittance", "admittance-no-shape"), required=True)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_OUTPUT_ROOT/"pairing_manifest.json")
    args = parser.parse_args()
    result = validate_contact(args.condition, args.variant, manifest_path=args.manifest)
    print(json.dumps({"condition": args.condition, "variant": args.variant,
                      "passed": result["passed"], "n110c_checks": result["n110c_checks"],
                      "maximum_errors": result["maximum_errors"]}, indent=2))
    if not result["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

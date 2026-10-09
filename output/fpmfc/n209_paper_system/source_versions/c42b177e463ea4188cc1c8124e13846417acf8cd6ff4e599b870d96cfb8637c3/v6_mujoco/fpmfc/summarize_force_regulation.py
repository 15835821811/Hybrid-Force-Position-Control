"""Read-only four-cell audit and deterministic N110D contrast summary."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .force_regulation_contract import CELLS, OUTPUT_ROOT, load_manifest
from .handoff_contract import repo_path, sha256


INITIAL_FIELDS = (
    "initial_qpos", "initial_qvel", "initial_ctrl", "initial_qacc_warmstart",
    "initial_mocap_pos", "initial_mocap_quat", "initial_robot_momentum",
    "initial_total_momentum_world_origin",
)


def _read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def summarize(root: Path = OUTPUT_ROOT) -> dict:
    root = Path(root).resolve()
    manifest = load_manifest(root/"experiment_manifest.json")
    baseline = repo_path(manifest["baseline_dir"])
    original_config = _read(baseline/"metrics.json")["effective_contact_config"]
    records = {}
    initial_states = {}
    checks = {}
    for cell in CELLS:
        directory = baseline if cell == "D00" else root/cell
        metrics = _read(directory/"metrics.json")
        validation = _read(directory/"validation.json")
        decomposition = _read(root/"baseline_error_decomposition.json" if cell == "D00" else directory/"error_decomposition.json")
        with np.load(directory/"trace.npz", allow_pickle=False) as trace:
            initial_states[cell] = {key: trace[key].copy() for key in INITIAL_FIELDS}
            steps = len(trace["time"])
            task_ticks = len(trace["task_time"])
            trace_force = np.asarray(trace["measured_normal_force_n"], dtype=float)
            trace_desired = np.asarray(trace["desired_normal_force_n"], dtype=float)
            normal = np.asarray(trace["contact_normal_world"], dtype=float)
            expected_tracking = np.einsum("ij,ij->i", trace["desired_position"]-trace["flange_position"], normal)
            expected_gap = np.einsum("ij,ij->i", trace["tool_face_position_world_m"]-trace["target_grasp_position"], normal)
        tracking_dir = root if cell == "D00" else directory
        tracking_prefix = "baseline_" if cell == "D00" else ""
        tracking_record = _read(tracking_dir/f"{tracking_prefix}normal_tracking_diagnostics.json")
        with np.load(tracking_dir/f"{tracking_prefix}normal_tracking_diagnostics.npz", allow_pickle=False) as saved_tracking:
            tracking_values_match = (np.array_equal(saved_tracking["commanded_flange_normal_error_m"], expected_tracking)
                                     and np.array_equal(saved_tracking["tool_face_to_grasp_normal_gap_m"], expected_gap)
                                     and len(saved_tracking["post_time_s"]) == 500)
        effective = metrics["effective_contact_config"]
        intended = json.loads(json.dumps(effective))
        intended["force_control"]["stiffness_n_m"] = original_config["force_control"]["stiffness_n_m"]
        intended["force_control"]["reset_on_contact_loss"] = original_config["force_control"]["reset_on_contact_loss"]
        artifacts = _read(directory/"artifact_manifest.json")
        artifact_hashes = all(sha256(directory/name) == digest for name, digest in artifacts["artifacts"].items())
        a = metrics["acceptance"]
        checks[f"{cell}_trace_grid"] = steps == 500 and task_ticks == 50
        checks[f"{cell}_trace_force_recomputation"] = (
            abs(float(np.sqrt(np.mean((trace_force[-100:]-trace_desired[-100:])**2)))
                - metrics["metrics"]["steady_force_rmse_n"]) <= 1e-12
            and abs(float(np.sum(trace_force)*0.002)-metrics["metrics"]["normal_force_impulse_ns"]) <= 1e-12)
        checks[f"{cell}_only_intended_effective_config_changes"] = intended == original_config
        checks[f"{cell}_S_homogeneous_full_shape"] = metrics["group"] == "S" and metrics["mapping_mode"] == "homogeneous" and metrics["variant"] == "admittance"
        checks[f"{cell}_artifact_hashes"] = artifact_hashes
        checks[f"{cell}_per_step_normal_tracking"] = (
            tracking_values_match
            and tracking_record["source_trace_sha256"] == sha256(directory/"trace.npz")
            and tracking_record["derived_npz_sha256"] == sha256(tracking_dir/f"{tracking_prefix}normal_tracking_diagnostics.npz")
            and tracking_record["derivation_source_sha256"] == sha256(repo_path("v6_mujoco/fpmfc/derive_force_regulation_tracking.py")))
        checks[f"{cell}_independent_replay"] = bool(validation["passed"])
        checks[f"{cell}_bias_variance_identity"] = decomposition["full_window"]["bias_variance_identity_error_n2"] <= 1e-12
        checks[f"{cell}_common_gate"] = bool(a["common_passed"])
        if cell != "D00":
            checks[f"{cell}_manifest_hash"] = metrics["n110d"]["experiment_manifest_sha256"] == sha256(root/"experiment_manifest.json")
            checks[f"{cell}_runtime_identity"] = metrics["n110d"]["runtime_identity"] == manifest["runtime_identity"]
            checks[f"{cell}_verifier_addendum"] = validation["n110d_verification_identity"]["verifier_addendum_sha256"] == sha256(repo_path(validation["n110d_verification_identity"]["verifier_addendum_path"]))
        records[cell] = {
            "stiffness_n_m": float(effective["force_control"]["stiffness_n_m"]),
            "reset_on_contact_loss": bool(effective["force_control"]["reset_on_contact_loss"]),
            "metrics": metrics["metrics"], "acceptance": a,
            "window_decomposition": decomposition["full_window"],
            "full_stage_decomposition": decomposition["full_stage"],
            "inner_normal_execution": decomposition["inner_normal_execution"],
            "independent_replay_passed": bool(validation["passed"]),
            "trace_sha256": sha256(directory/"trace.npz"),
            "normal_tracking_diagnostics_sha256": tracking_record["derived_npz_sha256"],
        }
    reference = initial_states["D00"]
    initial_differences = {
        cell: {name: float(np.max(np.abs(values[name]-reference[name]))) for name in INITIAL_FIELDS}
        for cell, values in initial_states.items()
    }
    checks["same_C1_initial_state_all_four_cells"] = all(value == 0.0 for item in initial_differences.values() for value in item.values())
    checks["three_and_only_three_new_full_shape_runs"] = sum((root/cell/"trace.npz").exists() for cell in CELLS if cell != "D00") == 3
    eligible = [cell for cell in CELLS if records[cell]["acceptance"]["common_passed"] and records[cell]["acceptance"]["steady_force_tracking"]]
    optional = list(root.glob("D*_no_shape/trace.npz"))
    checks["optional_confirmation_budget_and_gate"] = (len(optional) <= 1 and (not optional or bool(eligible)))
    contrasts = {
        "D01_minus_D00": {}, "D10_minus_D00": {}, "D11_minus_D10": {},
        "two_factor_interaction": {},
    }
    for field in ("steady_force_rmse_n", "peak_normal_force_n", "normal_force_impulse_ns",
                  "contact_loss_events", "contact_detected_fraction"):
        v = {cell: records[cell]["metrics"][field] for cell in CELLS}
        contrasts["D01_minus_D00"][field] = v["D01"]-v["D00"]
        contrasts["D10_minus_D00"][field] = v["D10"]-v["D00"]
        contrasts["D11_minus_D10"][field] = v["D11"]-v["D10"]
        contrasts["two_factor_interaction"][field] = v["D11"]-v["D10"]-v["D01"]+v["D00"]
    result = {
        "schema_version": "n110d_paired_summary_v1",
        "experiment_manifest_sha256": sha256(root/"experiment_manifest.json"),
        "runtime_identity_sha256": manifest["runtime_identity"]["composite_sha256"],
        "archive_D00_reused": True,
        "new_closed_loop_runs": 3,
        "optional_no_shape_confirmation_runs": len(optional),
        "cells": records,
        "initial_state_max_abs_differences_from_D00": initial_differences,
        "contrasts": contrasts,
        "qualifying_cells": eligible,
        "force_tracking_gate_passed": bool(eligible),
        "checks": checks,
        "engineering_audit_passed": bool(all(checks.values())),
    }
    return result


def main() -> None:
    result = summarize()
    path = OUTPUT_ROOT/"paired_summary.json"
    path.write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    print(json.dumps({"engineering_audit_passed": result["engineering_audit_passed"],
                      "qualifying_cells": result["qualifying_cells"],
                      "checks": result["checks"]}, indent=2))
    if not result["engineering_audit_passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

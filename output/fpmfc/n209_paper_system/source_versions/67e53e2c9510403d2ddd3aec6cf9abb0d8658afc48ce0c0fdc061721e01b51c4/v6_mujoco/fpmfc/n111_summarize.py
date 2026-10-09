"""Independent read-only N111 paired results and dynamics summaries."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .handoff_contract import sha256
from .n111_contract import CELLS, OUTPUT_ROOT, load_manifest


def _intervals(active: np.ndarray, dt: float) -> list[dict]:
    intervals = []
    start = None
    for i, value in enumerate(active):
        if not value and start is None:
            start = i
        if (value or i == len(active)-1) and start is not None:
            end = i if value else i+1
            intervals.append({"start_s": float(start*dt), "end_s": float(end*dt),
                              "duration_s": float((end-start)*dt)})
            start = None
    return intervals


def summarize() -> dict:
    manifest = load_manifest()
    ledger = json.loads((OUTPUT_ROOT / "run_ledger.json").read_text(encoding="utf-8"))
    if [item["cell"] for item in ledger["attempts"]] != list(CELLS):
        raise RuntimeError("N111 three-run ledger is incomplete or reordered")
    rows = {}
    dynamics = {}
    from .contact_model import default_contact_model_spec
    spec = default_contact_model_spec()
    model = spec.compile_model()
    joint_dofs = spec.joint_addresses(model)[1]
    base_dofs = spec.base_slices(model)[1]
    detail_path = OUTPUT_ROOT / "dynamics_detail_summary.json"
    detail = json.loads(detail_path.read_text(encoding="utf-8")) if detail_path.exists() else None
    for cell, gamma in CELLS.items():
        output = OUTPUT_ROOT / cell
        metrics = json.loads((output / "metrics.json").read_text(encoding="utf-8"))
        validation = json.loads((output / "validation.json").read_text(encoding="utf-8"))
        chain = json.loads((output / "execution_chain_summary.json").read_text(encoding="utf-8"))
        if not validation["passed"] or metrics["n111"]["gamma"] != gamma:
            raise RuntimeError(f"unverified run: {cell}")
        with np.load(output / "trace.npz", allow_pickle=False) as t, \
             np.load(output / "servo_diagnostics.npz", allow_pickle=False) as s, \
             np.load(output / "prediction_diagnostics.npz", allow_pickle=False) as p:
            if len(t["time"]) != 500 or len(s["feedback_time_s"]) != 500:
                raise RuntimeError("unexpected trajectory length")
            error = np.asarray(t["desired_normal_force_n"][-100:]
                               - t["measured_normal_force_n"][-100:])
            force = np.asarray(t["measured_normal_force_n"])
            protected = s["protected_compensation_nm"]
            requested = s["requested_compensation_nm"]
            applied = s["applied_torque_nm"]
            limits = np.asarray(metrics["model_identity"]["torque_limits_nm"]) if "torque_limits_nm" in metrics["model_identity"] else None
            if limits is None:
                from .contact_model import default_contact_model_spec
                limits = default_contact_model_spec().torque_limits_nm
            if not np.array_equal(applied, t["torque"]):
                raise RuntimeError("torque diagnostic differs from trace")
            force_rmse = float(np.sqrt(np.mean(error**2)))
            if abs(force_rmse-metrics["metrics"]["steady_force_rmse_n"]) > 1e-12:
                raise RuntimeError("fixed-window force RMSE differs from run metrics")
            intervals = _intervals(np.asarray(t["contact_detected"], dtype=bool), 0.002)
            rows[cell] = {
                "gamma": gamma, "physics_steps": len(force), "task_ticks": len(t["task_time"]),
                "last_window_samples": len(error), "last_window_zero_force_samples": int(np.count_nonzero(force[-100:] == 0)),
                "force_rmse_n": force_rmse, "force_mean_error_n": float(np.mean(error)),
                "force_error_std_n": float(np.std(error)),
                "peak_normal_force_n": float(np.max(force)),
                "normal_execution_error_full_rms_m_s": chain["statistics"]["normal_reference_to_actual_m_s"]["full"]["rms"],
                "normal_execution_error_last_rms_m_s": chain["statistics"]["normal_reference_to_actual_m_s"]["last_0p2_s"]["rms"],
                "normal_execution_error_last_mean_m_s": chain["statistics"]["normal_reference_to_actual_m_s"]["last_0p2_s"]["mean"],
                "normal_task_to_hqp_last_rms_m_s": chain["statistics"]["normal_task_to_hqp_m_s"]["last_0p2_s"]["rms"],
                "normal_homogeneous_base_bias_last_rms_m_s": chain["statistics"]["normal_homogeneous_base_bias_m_s"]["last_0p2_s"]["rms"],
                "hqp_to_reference_joint_last_rms_rad_s": chain["statistics"]["joint_hqp_to_reference_rms_rad_s"]["last_0p2_s"]["rms"],
                "reference_to_actual_joint_last_rms_rad_s": chain["statistics"]["joint_reference_to_actual_rms_rad_s"]["last_0p2_s"]["rms"],
                "compensation_clipped_step_fraction": float(np.mean(np.any(np.abs(requested-protected) > 1e-12, axis=1))),
                "compensation_clipped_joint_step_fraction": float(np.mean(np.abs(requested-protected) > 1e-12)),
                "compensation_requested_max_nm": float(np.max(np.abs(requested))),
                "compensation_applied_max_nm": float(np.max(np.abs(protected))),
                "contact_reduced_load_max_nm": float(np.max(np.abs(s["reduced_contact_load_nm"]))),
                "torque_saturated_joint_step_fraction": float(np.mean(np.abs(applied) >= limits[None, :] - 1e-9)),
                "contact_loss_events": metrics["metrics"]["contact_loss_events"],
                "detected_false_intervals": intervals,
                "common_safety_passed": metrics["acceptance"]["common_passed"],
                "force_rmse_0p3_n_passed": force_rmse <= manifest["force_rmse_gate_n"],
                "independent_replay_passed": validation["passed"],
                "trace_sha256": sha256(output / "trace.npz"),
                "validation_sha256": sha256(output / "validation.json"),
            }
            dynamics[cell] = {
                "same_state_full_residual_max": float(np.max(s["full_dynamics_residual_max"])),
                "same_state_full_residual_p99": float(np.quantile(s["full_dynamics_residual_max"], .99)),
                "same_state_reduced_residual_max": float(np.max(s["reduced_dynamics_residual_max"])),
                "same_state_reduced_residual_p99": float(np.quantile(s["reduced_dynamics_residual_max"], .99)),
                "all_dof_constraint_minus_robot_interface_norm_max_including_target": float(np.max(s["other_generalized_force_norm"])),
                "robot_constraint_minus_interface_norm_max": (
                    detail["cells"][cell]["robot_constraint_minus_interface_norm"]["max"] if detail else None),
                "target_counterpart_constraint_norm_max": (
                    detail["cells"][cell]["target_constraint_reaction_norm"]["max"] if detail else None),
                "other_contact_step_count": int(np.count_nonzero(s["other_contact_count"])),
                "joint_limit_constraint_step_count": int(np.count_nonzero(s["joint_limit_constraint_count"])),
                "power_identity_max_w": float(np.max(np.abs(s["wrench_power_w"]-s["generalized_power_w"]))),
                "offline_next_step_joint_qvel_prediction_rms_rad_s": float(np.sqrt(np.mean(
                    p["qvel_next_minus_instantaneous_euler"][:, joint_dofs]**2))),
                "offline_next_step_base_linear_qvel_prediction_rms_m_s": float(np.sqrt(np.mean(
                    p["qvel_next_minus_instantaneous_euler"][:, base_dofs.start:base_dofs.start+3]**2))),
                "offline_next_step_base_angular_qvel_prediction_rms_rad_s": float(np.sqrt(np.mean(
                    p["qvel_next_minus_instantaneous_euler"][:, base_dofs.start+3:base_dofs.stop]**2))),
                "prediction_definition": validation["prediction_note"],
            }
    baseline = rows["gamma_0"]
    for cell in ("gamma_05", "gamma_10"):
        rows[cell]["delta_force_rmse_vs_gamma_zero_n"] = rows[cell]["force_rmse_n"] - baseline["force_rmse_n"]
        rows[cell]["delta_normal_execution_rms_vs_gamma_zero_m_s"] = rows[cell]["normal_execution_error_last_rms_m_s"] - baseline["normal_execution_error_last_rms_m_s"]
        rows[cell]["delta_force_std_vs_gamma_zero_n"] = rows[cell]["force_error_std_n"] - baseline["force_error_std_n"]
    summary = {
        "schema_version": "n111_contact_load_paired_summary_v1",
        "manifest_sha256": sha256(OUTPUT_ROOT / "experiment_manifest.json"),
        "ledger": ledger, "cells": rows,
        "qualification": {"engineering": all(row["independent_replay_passed"] for row in rows.values()),
                          "performance": {cell: row["common_safety_passed"] and row["force_rmse_0p3_n_passed"] for cell, row in rows.items()},
                          "candidate_for_independent_validation": None},
        "interpretation_limit": "one fixed C1 source and contact scenario, one deterministic run per gamma; same-state algebra and lag correlation do not prove force mechanism or robust improvement",
        "summarizer_identity": {"path": "v6_mujoco/fpmfc/n111_summarize.py", "sha256": sha256(__file__)},
    }
    (OUTPUT_ROOT / "paired_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    (OUTPUT_ROOT / "dynamics_residual_summary.json").write_text(json.dumps({
        "schema_version": "n111_dynamics_residual_summary_v1", "cells": dynamics,
        "note": "same-state residual checks algebra and the observed forward solve; all-DOF remainder includes target counterpart, while robot-only remainder excludes it; offline instantaneous-acceleration prediction differs from the RK4 2 ms average and is not a stability test",
    }, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    return summary


if __name__ == "__main__":
    print(json.dumps({key: {"rmse": row["force_rmse_n"],
                            "normal_execution_rms": row["normal_execution_error_last_rms_m_s"]}
                      for key, row in summarize()["cells"].items()}, indent=2))

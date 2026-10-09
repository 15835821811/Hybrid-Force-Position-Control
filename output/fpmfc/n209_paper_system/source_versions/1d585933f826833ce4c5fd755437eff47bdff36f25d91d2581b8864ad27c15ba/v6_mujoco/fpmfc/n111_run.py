"""Run one frozen N111 gamma through the unchanged S contact loop."""

from __future__ import annotations

import argparse
import copy
import json
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from unittest.mock import patch

import numpy as np

from ..model import PROJECT_ROOT
from . import run_contact_consistent as consistent
from .contact_config import load_contact_config
from .force_regulation_contract import load_config as load_parent_config, outer_config
from .force_regulation_outer import ExperimentalNormalAdmittance, diagnostics_arrays
from .handoff_contract import sha256
from .n111_contact_servo import ContactServo, RecordingObserver
from .n111_contract import CELLS, OUTPUT_ROOT, load_manifest
from .n111_execution_chain import reconstruct


def _ledger_update(cell: str, status: str, error: str | None = None) -> None:
    path = OUTPUT_ROOT / "run_ledger.json"
    value = json.loads(path.read_text(encoding="utf-8"))
    if status == "started":
        if len(value["attempts"]) >= value["budget"]:
            raise RuntimeError("three-run closed-loop budget exhausted")
        if any(item["cell"] == cell for item in value["attempts"]):
            raise RuntimeError("cell already attempted; preserve all failed traces")
        value["attempts"].append({"cell": cell, "status": "started",
                                  "started_utc": datetime.now(timezone.utc).isoformat()})
    else:
        item = next(item for item in value["attempts"] if item["cell"] == cell)
        item.update({"status": status, "finished_utc": datetime.now(timezone.utc).isoformat(),
                     "error": error})
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")


def _gate(cell: str, manifest: dict) -> None:
    order = list(CELLS)
    ledger = json.loads((OUTPUT_ROOT / "run_ledger.json").read_text(encoding="utf-8"))
    attempts = ledger["attempts"]
    if cell != order[len(attempts)]:
        raise RuntimeError("N111 gamma order or budget violation")
    if cell == "gamma_05":
        comparison = json.loads((OUTPUT_ROOT / "gamma_0" / "compatibility.json").read_text(encoding="utf-8"))
        validation = json.loads((OUTPUT_ROOT / "gamma_0" / "validation.json").read_text(encoding="utf-8"))
        if not comparison["passed"] or not validation["passed"]:
            raise RuntimeError("gamma=0 D10 compatibility or independent replay failed")
    if cell == "gamma_10":
        previous = json.loads((OUTPUT_ROOT / "gamma_05" / "metrics.json").read_text(encoding="utf-8"))
        validation = json.loads((OUTPUT_ROOT / "gamma_05" / "validation.json").read_text(encoding="utf-8"))
        if not previous["acceptance"]["common_passed"] or not validation["passed"]:
            raise RuntimeError("gamma=0.5 failed common safety or independent replay gate")


def _gamma_zero_compatibility(manifest: dict, output: Path) -> dict:
    keys = ("qpos", "qvel", "torque", "reference_q", "reference_dq", "reference_ddq",
            "measured_normal_force_n", "feedback_measured_normal_force_n",
            "flange_position", "contact_force_world_n", "contact_count",
            "task_joint_velocity_rad_s", "task_time")
    with np.load(PROJECT_ROOT / manifest["d10_trace_path"], allow_pickle=False) as old, \
         np.load(output / "trace.npz", allow_pickle=False) as new:
        max_errors = {key: float(np.max(np.abs(old[key]-new[key]))) for key in keys}
    old_metrics = json.loads((PROJECT_ROOT / manifest["d10_trace_path"]).parent.joinpath("metrics.json").read_text(encoding="utf-8"))
    new_metrics = json.loads((output / "metrics.json").read_text(encoding="utf-8"))
    metric_keys = ("steady_force_rmse_n", "peak_normal_force_n", "contact_loss_events",
                   "maximum_sustained_contact_loss_s", "torque_saturation_fraction")
    metric_errors = {key: abs(float(old_metrics["metrics"][key])-float(new_metrics["metrics"][key]))
                     for key in metric_keys}
    passed = all(value <= (manifest["gamma_zero_torque_atol"] if key == "torque"
                           else manifest["gamma_zero_state_atol"])
                 for key, value in max_errors.items()) and all(
                     value <= manifest["gamma_zero_state_atol"] for value in metric_errors.values())
    result = {"schema_version": "n111_gamma_zero_compatibility_v1", "passed": passed,
              "maximum_absolute_errors": max_errors, "metric_absolute_errors": metric_errors,
              "state_atol": manifest["gamma_zero_state_atol"],
              "torque_atol_nm": manifest["gamma_zero_torque_atol"]}
    (output / "compatibility.json").write_text(json.dumps(result, indent=2)+"\n", encoding="utf-8")
    return result


def run_cell(cell: str) -> dict:
    manifest = load_manifest()
    if cell not in CELLS:
        raise ValueError(cell)
    _gate(cell, manifest)
    output = OUTPUT_ROOT / cell
    if output.exists():
        raise FileExistsError(output)
    _ledger_update(cell, "started")
    try:
        original = load_contact_config(PROJECT_ROOT / manifest["contact_config_path"])
        effective = copy.deepcopy(original)
        effective["force_control"]["stiffness_n_m"] = 0.0
        effective["force_control"]["reset_on_contact_loss"] = True
        parent = load_parent_config()
        outer_cfg = outer_config(parent, "D10")
        observers: list[RecordingObserver] = []
        outers: list[ExperimentalNormalAdmittance] = []
        servo = ContactServo(CELLS[cell], cap_fraction=manifest["compensation_cap_fraction"])

        def make_observer(model, *, tool_face_recess_m):
            if observers:
                raise RuntimeError("unexpected second contact observer")
            instance = RecordingObserver(model, tool_face_recess_m=tool_face_recess_m)
            observers.append(instance)
            servo.observer = instance
            return instance

        def make_outer(config):
            if outers or config != outer_cfg:
                raise RuntimeError("unexpected outer-loop instance/config")
            instance = ExperimentalNormalAdmittance(config)
            outers.append(instance)
            return instance

        def patched_load(_path):
            return copy.deepcopy(effective)

        def patched_outer(_effective, *, timestep_s):
            if _effective != effective or timestep_s != outer_cfg.timestep_s:
                raise RuntimeError("contact config or timestep changed")
            return outer_cfg

        with patch.object(consistent, "load_contact_config", patched_load), \
             patch.object(consistent, "normal_admittance_config", patched_outer), \
             patch.object(consistent, "NormalAdmittance", make_outer), \
             patch.object(consistent, "ContactObserver", make_observer), \
             patch.object(consistent, "servo_torque", servo):
            result = consistent.run_consistent_contact_experiment(
                variant="admittance",
                source_trace_path=PROJECT_ROOT / manifest["source_trace_path"],
                contact_config_path=PROJECT_ROOT / manifest["contact_config_path"],
                output_dir=output, mapping_mode="homogeneous",
            )
        if (len(outers) != 1 or len(outers[0].rows) != 500
                or len(servo.rows) != 500 or observers[0].calls != 1001
                or result["physics_steps"] != 500 or result["task_ticks"] != 50):
            raise RuntimeError("observer, admittance, servo, or HQP call count changed")
        outer_arrays = diagnostics_arrays(outers[0])
        np.savez_compressed(output / "outer_diagnostics.npz", **outer_arrays)
        servo_arrays = servo.arrays()
        with np.load(output / "trace.npz", allow_pickle=False) as trace:
            if not (np.array_equal(servo_arrays["applied_torque_nm"], trace["torque"])
                    and np.array_equal(servo_arrays["feedback_time_s"], trace["feedback_time_s"])):
                raise RuntimeError("causal servo diagnostics differ from trace")
        np.savez_compressed(output / "servo_diagnostics.npz", **servo_arrays)
        compatibility = _gamma_zero_compatibility(manifest, output) if cell == "gamma_0" else None
        chain = reconstruct(output / "trace.npz", output)
        snapshot_path = output / "config_snapshot.json"
        snapshot = json.loads(snapshot_path.read_text(encoding="utf-8"))
        snapshot["n111"] = {"cell": cell, "gamma": CELLS[cell],
                            "compensation_cap_fraction": manifest["compensation_cap_fraction"],
                            "outer_config": asdict(outer_cfg),
                            "experiment_manifest_sha256": sha256(OUTPUT_ROOT / "experiment_manifest.json")}
        snapshot_path.write_text(json.dumps(snapshot, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
        result["schema_version"] = "n111_contact_load_v1"
        result["n111"] = {"cell": cell, "gamma": CELLS[cell],
                           "compensation_cap_fraction": manifest["compensation_cap_fraction"],
                           "runtime_identity": manifest["runtime_identity"],
                           "experiment_manifest_sha256": sha256(OUTPUT_ROOT / "experiment_manifest.json"),
                           "servo_diagnostics_sha256": sha256(output / "servo_diagnostics.npz"),
                           "outer_diagnostics_sha256": sha256(output / "outer_diagnostics.npz"),
                           "execution_chain_sha256": sha256(output / "execution_chain_summary.json")}
        result["config_snapshot_sha256"] = sha256(snapshot_path)
        result["n111_diagnostics"] = {
            "compensation_clipped_sample_fraction": float(np.mean(np.any(
                np.abs(servo_arrays["requested_compensation_nm"] - servo_arrays["protected_compensation_nm"]) > 1e-12, axis=1))),
            "torque_saturated_joint_step_fraction": result["metrics"]["torque_saturation_fraction"],
            "maximum_full_dynamics_residual": float(np.max(servo_arrays["full_dynamics_residual_max"])),
            "maximum_reduced_dynamics_residual": float(np.max(servo_arrays["reduced_dynamics_residual_max"])),
            "normal_execution_error_rms_m_s": chain["statistics"]["normal_reference_to_actual_m_s"]["last_0p2_s"]["rms"],
        }
        (output / "metrics.json").write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
        if compatibility is not None and not compatibility["passed"]:
            raise RuntimeError("gamma=0 failed frozen D10 compatibility; stop further runs")
        _ledger_update(cell, "completed")
        return result
    except Exception as exc:
        _ledger_update(cell, "failed", repr(exc))
        (output / "run_failure.json").write_text(json.dumps({"cell": cell, "error": repr(exc)}, indent=2)+"\n", encoding="utf-8") if output.exists() else None
        raise


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cell", choices=CELLS, required=True)
    args = parser.parse_args()
    result = run_cell(args.cell)
    print(json.dumps({"cell": args.cell, "metrics": result["metrics"], "acceptance": result["acceptance"]}, indent=2))


if __name__ == "__main__":
    main()

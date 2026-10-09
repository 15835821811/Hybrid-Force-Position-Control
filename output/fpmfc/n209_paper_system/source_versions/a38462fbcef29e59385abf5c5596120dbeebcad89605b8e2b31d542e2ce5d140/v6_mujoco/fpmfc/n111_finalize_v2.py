"""Finish the existing gamma=0 trace after a manifest-key postprocess typo.

This addendum never invokes the contact runner or MuJoCo stepping.  The failed
postprocessing attempt remains in run_failure.json and the run ledger.
"""

from __future__ import annotations

import json
from datetime import datetime, timezone

import numpy as np

from .force_regulation_contract import load_config as load_parent_config, outer_config
from .handoff_contract import sha256
from .n111_contract import OUTPUT_ROOT, load_manifest
from .n111_execution_chain import reconstruct
from .n111_run import _gamma_zero_compatibility


def finalize_existing_gamma_zero() -> dict:
    manifest = load_manifest()
    output = OUTPUT_ROOT / "gamma_0"
    failure = json.loads((output / "run_failure.json").read_text(encoding="utf-8"))
    if failure != {"cell": "gamma_0", "error": "KeyError('gamma_zero_torque_atol')"}:
        raise RuntimeError("unexpected failure; no automatic repair")
    with np.load(output / "trace.npz", allow_pickle=False) as trace, \
         np.load(output / "servo_diagnostics.npz", allow_pickle=False) as servo, \
         np.load(output / "outer_diagnostics.npz", allow_pickle=False) as outer:
        if not (len(trace["time"]) == len(servo["applied_torque_nm"]) == len(outer["x_after_step_m"]) == 500
                and np.array_equal(trace["torque"], servo["applied_torque_nm"])):
            raise RuntimeError("incomplete gamma=0 closed-loop artifacts")
        clipped_fraction = float(np.mean(np.any(
            np.abs(servo["requested_compensation_nm"]-servo["protected_compensation_nm"]) > 1e-12, axis=1)))
        full_residual = float(np.max(servo["full_dynamics_residual_max"]))
        reduced_residual = float(np.max(servo["reduced_dynamics_residual_max"]))
    corrected = dict(manifest)
    corrected["gamma_zero_torque_atol"] = manifest["gamma_zero_torque_atol_nm"]
    compatibility = _gamma_zero_compatibility(corrected, output)
    if not compatibility["passed"]:
        raise RuntimeError("gamma=0 does not match archived D10")
    compatibility["postprocessing_addendum_sha256"] = sha256(__file__)
    (output / "compatibility.json").write_text(json.dumps(compatibility, indent=2)+"\n", encoding="utf-8")
    chain = reconstruct(output / "trace.npz", output)
    snapshot_path = output / "config_snapshot.json"
    snapshot = json.loads(snapshot_path.read_text(encoding="utf-8"))
    snapshot["n111"] = {
        "cell": "gamma_0", "gamma": 0.0,
        "compensation_cap_fraction": manifest["compensation_cap_fraction"],
        "outer_config": vars(outer_config(load_parent_config(), "D10")),
        "experiment_manifest_sha256": sha256(OUTPUT_ROOT / "experiment_manifest.json"),
        "postprocessing_addendum_sha256": sha256(__file__),
    }
    snapshot_path.write_text(json.dumps(snapshot, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    metrics_path = output / "metrics.json"
    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    metrics["schema_version"] = "n111_contact_load_v1"
    metrics["n111"] = {
        "cell": "gamma_0", "gamma": 0.0,
        "compensation_cap_fraction": manifest["compensation_cap_fraction"],
        "runtime_identity": manifest["runtime_identity"],
        "experiment_manifest_sha256": sha256(OUTPUT_ROOT / "experiment_manifest.json"),
        "servo_diagnostics_sha256": sha256(output / "servo_diagnostics.npz"),
        "outer_diagnostics_sha256": sha256(output / "outer_diagnostics.npz"),
        "execution_chain_sha256": sha256(output / "execution_chain_summary.json"),
        "postprocessing_addendum": {"path": "v6_mujoco/fpmfc/n111_finalize_v2.py",
                                   "sha256": sha256(__file__),
                                   "reason": "frozen runner used manifest key without _nm suffix after the 500-step trace completed"},
    }
    metrics["config_snapshot_sha256"] = sha256(snapshot_path)
    metrics["n111_diagnostics"] = {
        "compensation_clipped_sample_fraction": clipped_fraction,
        "torque_saturated_joint_step_fraction": metrics["metrics"]["torque_saturation_fraction"],
        "maximum_full_dynamics_residual": full_residual,
        "maximum_reduced_dynamics_residual": reduced_residual,
        "normal_execution_error_rms_m_s": chain["statistics"]["normal_reference_to_actual_m_s"]["last_0p2_s"]["rms"],
    }
    metrics_path.write_text(json.dumps(metrics, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    ledger_path = OUTPUT_ROOT / "run_ledger.json"
    ledger = json.loads(ledger_path.read_text(encoding="utf-8"))
    if len(ledger["attempts"]) != 1 or ledger["attempts"][0]["status"] != "failed":
        raise RuntimeError("unexpected run ledger before repair")
    ledger["attempts"][0]["status"] = "completed_after_postprocessing_repair"
    ledger["attempts"][0]["repair_utc"] = datetime.now(timezone.utc).isoformat()
    ledger["attempts"][0]["repair_addendum_sha256"] = sha256(__file__)
    ledger_path.write_text(json.dumps(ledger, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    return metrics


if __name__ == "__main__":
    print(json.dumps({"force_rmse_n": finalize_existing_gamma_zero()["metrics"]["steady_force_rmse_n"]}, indent=2))

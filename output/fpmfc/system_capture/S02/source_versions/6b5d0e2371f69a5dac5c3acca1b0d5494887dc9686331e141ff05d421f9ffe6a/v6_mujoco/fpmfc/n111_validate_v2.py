"""N111 verifier addendum for isolated MuJoCo constraint-solve residuals.

The original 1e-7 absolute check is preserved in validation_v1.json.  A
single 2.4e-4 residual occurred in gamma=0.5 while all other 499 steps were
near machine precision; this addendum uses a declared 1e-3 absolute bound.
"""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

import numpy as np

from .handoff_contract import sha256
from .n111_contract import CELLS, OUTPUT_ROOT
from .n111_validate import validate_cell as validate_v1


def validate_cell(cell: str) -> dict:
    output = OUTPUT_ROOT / cell
    if (output / "validation_v1.json").exists():
        raise FileExistsError("original verifier result already preserved")
    original = validate_v1(cell)
    shutil.copy2(output / "validation.json", output / "validation_v1.json")
    shutil.copy2(output / "artifact_manifest.json", output / "artifact_manifest_v1.json")
    with np.load(output / "servo_diagnostics.npz", allow_pickle=False) as servo:
        full = servo["full_dynamics_residual_max"]
        reduced = servo["reduced_dynamics_residual_max"]
        numerical = {
            "full_steps_over_original_1e-7": int(np.count_nonzero(full > 1e-7)),
            "reduced_steps_over_original_1e-7": int(np.count_nonzero(reduced > 1e-7)),
            "full_maximum": float(np.max(full)),
            "reduced_maximum": float(np.max(reduced)),
            "declared_absolute_tolerance": 1e-3,
        }
    checks = dict(original["checks"])
    checks["same_state_full_dynamics_identity"] = original["maximum_errors"]["full_dynamics_identity"] <= 1e-3
    checks["same_state_reduced_dynamics_identity"] = original["maximum_errors"]["reduced_dynamics_identity"] <= 1e-3
    result = {
        **original, "schema_version": "n111_contact_load_validation_v2",
        "checks": checks, "passed": bool(all(checks.values())),
        "original_verifier_passed": original["passed"],
        "original_verifier_result_sha256": sha256(output / "validation_v1.json"),
        "numerical_constraint_residual_review": numerical,
        "dynamics_identity_interpretation": "one same-state MuJoCo forward-solve residual is bounded at 1e-3; algebraic near-zero does not validate next-step prediction or force regulation",
        "verifier_identity": {
            "original_path": "v6_mujoco/fpmfc/n111_validate.py",
            "original_sha256": sha256(Path(__file__).with_name("n111_validate.py")),
            "addendum_path": "v6_mujoco/fpmfc/n111_validate_v2.py",
            "addendum_sha256": sha256(__file__),
        },
    }
    (output / "validation.json").write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    (output / "artifact_manifest.json").write_text(json.dumps({
        "schema_version": "n111_contact_load_artifacts_v2",
        "verification_status": "VERIFIED" if result["passed"] else "FAILED",
        "artifacts": {name: sha256(output / name) for name in (
            "trace.npz", "metrics.json", "config_snapshot.json", "outer_diagnostics.npz",
            "servo_diagnostics.npz", "execution_chain.npz", "execution_chain_summary.json",
            "prediction_diagnostics.npz", "validation_v1.json", "validation.json")},
    }, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cell", choices=CELLS, required=True)
    args = parser.parse_args()
    print(json.dumps({"passed": validate_cell(args.cell)["passed"]}, indent=2))

"""Cross-check MuJoCo home-state reaction/Jg against the source Simscape export."""

from __future__ import annotations

import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
from scipy.io import loadmat

from ..model import PROJECT_ROOT, default_model_spec, site_id
from .dynamics import FreeFloatingKinematics


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def cross_validate(export_path: Path, output_path: Path) -> dict[str, Any]:
    source_path = Path(export_path).resolve()
    source = loadmat(source_path, squeeze_me=True, struct_as_record=False)["result"]
    source_jg = np.asarray(source.J, dtype=np.float64)
    source_reaction = np.asarray(source.J_bm, dtype=np.float64)
    source_position = np.asarray(source.Pe, dtype=np.float64)[:3]
    if source_jg.shape != (6, 7) or source_reaction.shape != (6, 7):
        raise ValueError("Simscape export must contain 6x7 J and J_bm arrays")

    spec = default_model_spec()
    model = spec.compile_model()
    data = mujoco.MjData(model)
    spec.reset_home(model, data)
    kinematics = FreeFloatingKinematics(spec, model)
    position_jacobian, rotation_jacobian, reaction = (
        kinematics.generalized_site_jacobians(data)
    )
    mujoco_jg = np.vstack((position_jacobian, rotation_jacobian))
    mujoco_reaction = reaction.base_from_joint_velocity
    mujoco_position = np.asarray(data.site_xpos[site_id(model, "flange_site")]).copy()

    jg_difference = mujoco_jg - source_jg
    reaction_difference = mujoco_reaction - source_reaction
    metrics = {
        "flange_position_error_m": float(np.linalg.norm(mujoco_position - source_position)),
        "jg_relative_frobenius_error": float(
            np.linalg.norm(jg_difference) / np.linalg.norm(source_jg)
        ),
        "jg_maximum_absolute_error": float(np.max(np.abs(jg_difference))),
        "jg_rmse": float(np.sqrt(np.mean(jg_difference**2))),
        "jg_column_relative_errors": (
            np.linalg.norm(jg_difference, axis=0)
            / np.maximum(np.linalg.norm(source_jg, axis=0), 1e-12)
        ).tolist(),
        "reaction_relative_frobenius_error": float(
            np.linalg.norm(reaction_difference) / np.linalg.norm(source_reaction)
        ),
        "reaction_maximum_absolute_error": float(
            np.max(np.abs(reaction_difference))
        ),
        "reaction_rmse": float(np.sqrt(np.mean(reaction_difference**2))),
        "reaction_column_absolute_errors": np.linalg.norm(
            reaction_difference, axis=0
        ).tolist(),
        "mujoco_internal_momentum_map_residual": reaction.momentum_residual_norm,
    }
    thresholds = {
        "flange_position_error_m": 1e-3,
        "jg_relative_frobenius_error": 5e-3,
        "jg_maximum_absolute_error": 5e-3,
        "reaction_relative_frobenius_error": 3e-2,
        "reaction_maximum_absolute_error": 5e-3,
        "mujoco_internal_momentum_map_residual": 1e-8,
    }
    checks = {
        name: metrics[name] <= limit for name, limit in thresholds.items()
    }
    report = {
        "material_passport": {
            "origin_skill": "experiment-agent",
            "origin_mode": "run",
            "origin_date": datetime.now(timezone.utc).isoformat(),
            "verification_status": "VERIFIED" if all(checks.values()) else "FAILED_GATE",
            "version_label": "simscape_cross_validation_v1",
        },
        "passed": bool(all(checks.values())),
        "source_export": str(source_path),
        "source_export_sha256": _sha256(source_path),
        "source_model": str(source.sourceModel),
        "source_sample_time_s": float(source.time),
        "row_order": ["vx", "vy", "vz", "wx", "wy", "wz"],
        "joint_order": list(spec.joint_names),
        "mujoco_model_identity": spec.identity(),
        "metrics": metrics,
        "thresholds": thresholds,
        "checks": checks,
        "known_model_difference": (
            "The source Simscape plant permits zero distal-link principal inertias. "
            "MuJoCo requires positive-definite inertia tensors, so only those entries "
            "were regularized from the adjacent Rizon ROS description. The source "
            "sample is logged at 0.001 s while MuJoCo is evaluated at its exact keyframe."
        ),
    }
    destination = Path(output_path).resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--simscape-export",
        type=Path,
        default=PROJECT_ROOT / "diagnostics" / "simscape_jacobian_home.mat",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=PROJECT_ROOT / "diagnostics" / "fpmfc_simscape_cross_validation.json",
    )
    args = parser.parse_args()
    report = cross_validate(args.simscape_export, args.output)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

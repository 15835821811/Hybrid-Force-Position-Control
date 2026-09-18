"""Independently replay and validate an FPMFC pre-contact torque trace."""

from __future__ import annotations

import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from ..collision import build_collision_pairs, minimum_signed_distance
from ..model import PROJECT_ROOT, default_model_spec, geom_id, site_id


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def replay_capture_trace(trace_path: Path) -> dict[str, float]:
    """Replay only stored torques, independently of controller/planner code."""

    with np.load(trace_path, allow_pickle=False) as archive:
        trace = {key: archive[key] for key in archive.files}
    spec = default_model_spec()
    model = spec.compile_model()
    model.geom_pos[geom_id(model, "workspace_obstacle_0")] = [10.0, 10.0, 10.0]
    model.geom_contype[:] = 0
    model.geom_conaffinity[:] = 0
    data = mujoco.MjData(model)
    data.qpos[:] = trace["initial_qpos"]
    data.qvel[:] = trace.get("initial_qvel", np.zeros(model.nv))
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)
    flange = site_id(model, "flange_site")
    pairs = build_collision_pairs(model)
    maximum_qpos_error = 0.0
    maximum_qvel_error = 0.0
    maximum_flange_error = 0.0
    maximum_clearance_error = 0.0
    recomputed_minimum_clearance = float("inf")
    for index, torque in enumerate(trace["torque"]):
        data.ctrl[:] = torque
        mujoco.mj_step(model, data)
        maximum_qpos_error = max(
            maximum_qpos_error,
            float(np.max(np.abs(np.asarray(data.qpos) - trace["qpos"][index]))),
        )
        maximum_qvel_error = max(
            maximum_qvel_error,
            float(np.max(np.abs(np.asarray(data.qvel) - trace["qvel"][index]))),
        )
        maximum_flange_error = max(
            maximum_flange_error,
            float(
                np.max(
                    np.abs(
                        np.asarray(data.site_xpos[flange])
                        - trace["flange_position"][index]
                    )
                )
            ),
        )
        clearance, _pair = minimum_signed_distance(model, data, pairs)
        maximum_clearance_error = max(
            maximum_clearance_error,
            abs(clearance - float(trace["minimum_clearance_m"][index])),
        )
        recomputed_minimum_clearance = min(recomputed_minimum_clearance, clearance)
    return {
        "maximum_qpos_error": maximum_qpos_error,
        "maximum_qvel_error": maximum_qvel_error,
        "maximum_flange_position_error_m": maximum_flange_error,
        "maximum_clearance_error_m": maximum_clearance_error,
        "recomputed_minimum_clearance_m": recomputed_minimum_clearance,
    }


def validate_capture_output(output_dir: Path) -> dict[str, Any]:
    output = Path(output_dir).resolve()
    metrics_path = output / "metrics.json"
    trace_path = output / "trace.npz"
    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    replay = replay_capture_trace(trace_path)
    checks = {
        "trace_hash_matches_metrics": _sha256(trace_path)
        == metrics["trace"]["sha256"],
        "reported_candidate_passed_acceptance": bool(metrics["acceptance"]["passed"]),
        "deterministic_qpos_replay": replay["maximum_qpos_error"] <= 1e-11,
        "deterministic_qvel_replay": replay["maximum_qvel_error"] <= 1e-10,
        "deterministic_flange_replay": replay["maximum_flange_position_error_m"]
        <= 1e-11,
        "deterministic_clearance_replay": replay["maximum_clearance_error_m"]
        <= 1e-10,
    }
    report = {
        "material_passport": {
            "origin_skill": "experiment-agent",
            "origin_mode": "run",
            "origin_date": datetime.now(timezone.utc).isoformat(),
            "verification_status": "VERIFIED" if all(checks.values()) else "FAILED_GATE",
            "version_label": "validation_v1",
        },
        "passed": bool(all(checks.values())),
        "checks": checks,
        "replay": replay,
        "metrics_path": str(metrics_path),
        "metrics_sha256": _sha256(metrics_path),
        "trace_path": str(trace_path),
        "trace_sha256": _sha256(trace_path),
    }
    validation_path = output / "validation.json"
    validation_path.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=PROJECT_ROOT / "output" / "fpmfc" / "precontact" / "manual_candidate",
    )
    args = parser.parse_args()
    report = validate_capture_output(args.output_dir)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

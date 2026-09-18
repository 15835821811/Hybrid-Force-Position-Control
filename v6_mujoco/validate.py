"""Independent torque-trace replay and migration acceptance checks."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from .collision import build_collision_pairs, minimum_signed_distance
from .model import PROJECT_ROOT, SIMSCAPE_HOME_FLANGE_POSITION_M, default_model_spec, geom_id, site_id


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def replay_trace(trace_path: Path) -> dict[str, float]:
    # Materialize once: repeatedly indexing an NpzFile would decompress the
    # same member on every physics step and make replay needlessly quadratic.
    with np.load(trace_path, allow_pickle=False) as archive:
        trace = {key: archive[key] for key in archive.files}
    spec = default_model_spec()
    model = spec.compile_model()
    model.geom_pos[geom_id(model, "workspace_obstacle_0")] = trace["obstacle_position"]
    model.geom_contype[:] = 0
    model.geom_conaffinity[:] = 0
    data = mujoco.MjData(model)
    data.qpos[:] = trace["initial_qpos"]
    data.qvel[:] = trace["initial_qvel"]
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)
    pairs = build_collision_pairs(model)
    flange = site_id(model, "flange_site")
    maximum_qpos_error = 0.0
    maximum_qvel_error = 0.0
    maximum_flange_error = 0.0
    maximum_clearance_error = 0.0
    recomputed_minimum = float("inf")
    clearance_stride = int(round(spec.task_period_s / spec.timestep_s))
    clearance_sample_count = 0
    for index, torque in enumerate(trace["torque"]):
        data.ctrl[:] = torque
        mujoco.mj_step(model, data)
        maximum_qpos_error = max(maximum_qpos_error, float(np.max(np.abs(data.qpos - trace["qpos"][index]))))
        maximum_qvel_error = max(maximum_qvel_error, float(np.max(np.abs(data.qvel - trace["qvel"][index]))))
        maximum_flange_error = max(maximum_flange_error, float(np.max(np.abs(data.site_xpos[flange] - trace["flange_position"][index]))))
        if index % clearance_stride == 0 or index == len(trace["torque"]) - 1:
            clearance, _ = minimum_signed_distance(model, data, pairs)
            maximum_clearance_error = max(maximum_clearance_error, abs(clearance - float(trace["minimum_clearance"][index])))
            recomputed_minimum = min(recomputed_minimum, clearance)
            clearance_sample_count += 1
    return {
        "maximum_qpos_error": maximum_qpos_error,
        "maximum_qvel_error": maximum_qvel_error,
        "maximum_flange_position_error_m": maximum_flange_error,
        "maximum_clearance_error_m": maximum_clearance_error,
        "recomputed_minimum_clearance_m": recomputed_minimum,
        "recomputed_clearance_samples_50hz": clearance_sample_count,
    }


def validate_output(output_dir: Path) -> dict[str, Any]:
    output_dir = Path(output_dir).resolve()
    metrics_path = output_dir / "migration_metrics.json"
    manifest_path = output_dir / "artifact_manifest.json"
    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    spec = default_model_spec()
    model = spec.compile_model()
    data = mujoco.MjData(model)
    spec.reset_home(model, data)
    flange = site_id(model, "flange_site")
    home_fk_error = float(np.linalg.norm(data.site_xpos[flange] - SIMSCAPE_HOME_FLANGE_POSITION_M))
    replay_reports = []
    trace_hashes_valid = True
    for scenario in metrics["scenarios"]:
        trace_path = PROJECT_ROOT / scenario["trace"]
        trace_hashes_valid &= _sha256(trace_path) == scenario["trace_sha256"]
        replay_reports.append({"scenario_id": scenario["scenario_id"], **replay_trace(trace_path)})
    identity_valid = manifest["model_identity"] == spec.identity()
    checks = {
        "model_identity_and_artifact_hashes": bool(identity_valid and trace_hashes_valid and _sha256(metrics_path) == manifest["metrics_sha256"]),
        "simscape_home_fk_within_1_mm": home_fk_error <= 0.001,
        "physics_500_hz_and_task_50_hz": abs(spec.timestep_s - 0.002) <= 1e-12 and abs(spec.task_period_s - 0.020) <= 1e-12,
        "one_qp_succeeded_each_task_tick": metrics["aggregate"]["qp_success_rate_min"] == 1.0,
        "no_state_teleport_after_initialization": all(item["qpos_write_count_after_initialization"] == 0 and item["qvel_write_count_after_initialization"] == 0 for item in metrics["scenarios"]),
        "signed_clearance_above_5_mm": metrics["aggregate"]["minimum_signed_clearance_m"] >= 0.005,
        "tracking_max_below_50_mm": metrics["aggregate"]["tracking_max_m_max"] <= 0.050,
        "deterministic_torque_replay": all(
            item["maximum_qpos_error"] <= 1e-11
            and item["maximum_qvel_error"] <= 1e-10
            and item["maximum_flange_position_error_m"] <= 1e-11
            and item["maximum_clearance_error_m"] <= 1e-10
            for item in replay_reports
        ),
    }
    report = {
        "passed": all(checks.values()),
        "checks": checks,
        "home_fk_error_m": home_fk_error,
        "replay": replay_reports,
        "acceptance_thresholds": {
            "home_fk_error_m": 0.001,
            "minimum_signed_clearance_m": 0.005,
            "maximum_tracking_error_m": 0.050,
        },
    }
    validation_path = output_dir / "validation.json"
    validation_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    manifest["validation"] = str(validation_path.resolve().relative_to(PROJECT_ROOT.resolve()))
    manifest["validation_sha256"] = _sha256(validation_path)
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=PROJECT_ROOT / "output")
    args = parser.parse_args()
    report = validate_output(args.output_dir)
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

"""Independent N110D torque replay, outer-balance, and force-window checks."""

from __future__ import annotations

import argparse
import copy
import json
from dataclasses import asdict
from pathlib import Path
from unittest.mock import patch

import numpy as np

from . import validate_contact_consistent as frozen_validation
from .contact_config import load_contact_config
from .force_regulation_analysis import analyze_trace
from .force_regulation_contract import CELLS, OUTPUT_ROOT, load_config, load_manifest, outer_config
from .force_regulation_outer import ExperimentalNormalAdmittance
from .handoff_contract import repo_path, sha256


def _independent_outer_checks(diagnostics: dict[str, np.ndarray], config) -> dict[str, float | bool]:
    a = diagnostics
    error = a["desired_force_n"] - a["measured_force_n"]
    raw_acceleration = (error - config.damping_n_s_m*a["v_after_reset_m_s"]
                        - config.stiffness_n_m*a["x_after_reset_m"]) / config.virtual_mass_kg
    raw_velocity = a["v_after_reset_m_s"] + config.timestep_s*raw_acceleration
    bounded_velocity = np.clip(raw_velocity, -config.maximum_velocity_m_s, config.maximum_velocity_m_s)
    raw_offset = a["x_after_reset_m"] + config.timestep_s*bounded_velocity
    bounded_offset = np.clip(raw_offset, -config.maximum_offset_m, config.maximum_offset_m)
    boundary_zero = (bounded_offset != raw_offset) & (np.sign(bounded_velocity) == np.sign(bounded_offset))
    bounded_velocity = np.where(boundary_zero, 0.0, bounded_velocity)
    previous_x = np.r_[0.0, a["x_after_step_m"][:-1]]
    previous_v = np.r_[0.0, a["v_after_step_m_s"][:-1]]
    detected = False
    expected_loss = []
    for value in a["measured_force_n"]:
        prior = detected
        detected = value > config.release_force_n if detected else value >= config.detection_force_n
        expected_loss.append(prior and not detected)
    expected_loss = np.asarray(expected_loss, dtype=bool)
    expected_reset = expected_loss & config.reset_on_contact_loss
    expected_after_reset_x = np.where(expected_reset, 0.0, previous_x)
    expected_after_reset_v = np.where(expected_reset, 0.0, previous_v)
    maximum_error = max(float(np.max(np.abs(a["acceleration_m_s2"]-raw_acceleration))),
                        float(np.max(np.abs(a["velocity_unclipped_m_s"]-raw_velocity))),
                        float(np.max(np.abs(a["offset_unclipped_m"]-raw_offset))),
                        float(np.max(np.abs(a["x_after_step_m"]-bounded_offset))),
                        float(np.max(np.abs(a["v_after_step_m_s"]-bounded_velocity))),
                        float(np.max(np.abs(a["x_before_reset_m"]-previous_x))),
                        float(np.max(np.abs(a["v_before_reset_m_s"]-previous_v))),
                        float(np.max(np.abs(a["x_after_reset_m"]-expected_after_reset_x))),
                        float(np.max(np.abs(a["v_after_reset_m_s"]-expected_after_reset_v))))
    return {
        "maximum_discrete_outer_equation_error": maximum_error,
        "independent_discrete_update": maximum_error <= 1e-12,
        "contact_loss_events_exact": bool(np.array_equal(a["loss_event"], expected_loss)),
        "reset_policy_exact": bool(np.array_equal(a["reset_applied"], expected_reset)),
        "velocity_limit_flags_exact": bool(np.array_equal(a["velocity_clipped"], raw_velocity != np.clip(raw_velocity, -config.maximum_velocity_m_s, config.maximum_velocity_m_s))),
        "offset_limit_flags_exact": bool(np.array_equal(a["offset_clipped"], raw_offset != bounded_offset)),
        "boundary_velocity_flags_exact": bool(np.array_equal(a["boundary_velocity_zeroed"], boundary_zero)),
        "offset_bounded": bool(np.max(np.abs(a["x_after_step_m"])) <= config.maximum_offset_m),
        "velocity_bounded": bool(np.max(np.abs(a["v_after_step_m_s"])) <= config.maximum_velocity_m_s),
    }


def validate_cell(cell: str, *, no_shape_confirmation: bool = False,
                  manifest_path: Path = OUTPUT_ROOT/"experiment_manifest.json") -> dict:
    manifest_path = Path(manifest_path).resolve()
    manifest = load_manifest(manifest_path)
    config = load_config(repo_path(manifest["config_path"]))
    if cell not in CELLS or cell == "D00":
        raise ValueError("D00 uses its archived validation")
    output = manifest_path.parent / (f"{cell}_no_shape" if no_shape_confirmation else cell)
    metrics = json.loads((output/"metrics.json").read_text(encoding="utf-8"))
    original = load_contact_config(repo_path(manifest["contact_config_path"]))
    modified = copy.deepcopy(original)
    modified["force_control"]["stiffness_n_m"] = float(config["conditions"][cell]["stiffness_n_m"])
    modified["force_control"]["reset_on_contact_loss"] = bool(config["conditions"][cell]["reset_on_contact_loss"])
    outer_cfg = outer_config(config, cell)

    def patched_load(_path):
        return copy.deepcopy(modified)

    def patched_outer_config(_config, *, timestep_s):
        if timestep_s != outer_cfg.timestep_s:
            raise ValueError("unexpected timestep")
        return outer_cfg

    with patch.object(frozen_validation, "load_contact_config", patched_load), \
         patch.object(frozen_validation, "normal_admittance_config", patched_outer_config), \
         patch.object(frozen_validation, "NormalAdmittance", ExperimentalNormalAdmittance):
        replay = frozen_validation.validate_consistent_output(output)
    analysis, independently_reconstructed = analyze_trace(output/"trace.npz", outer_cfg)
    with np.load(output/"outer_diagnostics.npz", allow_pickle=False) as archive:
        saved = {key: archive[key] for key in archive.files}
    checks = {
        "frozen_S_independent_torque_replay": bool(replay["passed"]),
        "S_homogeneous_full_shape_or_declared_confirmation": metrics["group"] == "S"
        and metrics["mapping_mode"] == "homogeneous"
        and metrics["variant"] == ("admittance-no-shape" if no_shape_confirmation else "admittance"),
        "frozen_manifest_sha256": metrics["n110d"]["experiment_manifest_sha256"] == sha256(manifest_path),
        "new_runtime_identity": metrics["n110d"]["runtime_identity"] == manifest["runtime_identity"],
        "new_outer_config": metrics["n110d"]["outer_config"] == asdict(outer_cfg),
        "outer_diagnostics_sha256": metrics["n110d"]["outer_diagnostics_sha256"] == sha256(output/"outer_diagnostics.npz"),
        "causal_diagnostics_match_reconstruction": all(np.array_equal(saved[key], independently_reconstructed[key]) for key in saved),
        "full_window_force_rmse": abs(analysis["full_window"]["force_rmse_n"]-metrics["metrics"]["steady_force_rmse_n"]) <= 1e-12,
        "all_window_samples_and_bias_variance": analysis["window"]["last_samples"] == 100
        and analysis["full_window"]["bias_variance_identity_error_n2"] <= 1e-12,
        "source_trace_hash": metrics["source_handoff"]["source_trace_sha256"] == manifest["source_trace_sha256"],
        "no_state_overwrite": metrics["qpos_write_count_after_initialization"] == 0
        and metrics["qvel_write_count_after_initialization"] == 0,
    }
    with np.load(output/"trace.npz", allow_pickle=False) as trace, \
         np.load(repo_path(manifest["baseline_dir"])/"trace.npz", allow_pickle=False) as baseline:
        checks["same_C1_initial_state_as_D00"] = all(np.array_equal(trace[key], baseline[key]) for key in (
            "initial_qpos", "initial_qvel", "initial_ctrl", "initial_qacc_warmstart",
            "initial_mocap_pos", "initial_mocap_quat", "initial_robot_momentum"))
    independent = _independent_outer_checks(saved, outer_cfg)
    checks.update({key: value for key, value in independent.items() if isinstance(value, bool)})
    result = {
        **replay, "schema_version": "n110d_force_regulation_validation_v1",
        "cell": cell, "variant": metrics["variant"],
        "n110d_checks": checks, "independent_outer": independent,
        "n110d_verification_identity": manifest["verification_identity"],
        "passed": bool(all(checks.values())),
    }
    (output/"error_decomposition.json").write_text(json.dumps(analysis, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    (output/"validation.json").write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    (output/"artifact_manifest.json").write_text(json.dumps({
        "schema_version": "n110d_contact_artifacts_v1",
        "verification_status": "VERIFIED" if result["passed"] else "FAILED",
        "artifacts": {name: sha256(output/name) for name in (
            "trace.npz", "metrics.json", "config_snapshot.json", "outer_diagnostics.npz",
            "error_decomposition.json", "validation.json")},
    }, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cell", choices=CELLS, required=True)
    parser.add_argument("--no-shape-confirmation", action="store_true")
    parser.add_argument("--manifest", type=Path, default=OUTPUT_ROOT/"experiment_manifest.json")
    args = parser.parse_args()
    result = validate_cell(args.cell, no_shape_confirmation=args.no_shape_confirmation,
                           manifest_path=args.manifest)
    print(json.dumps({"cell": args.cell, "passed": result["passed"],
                      "n110d_checks": result["n110d_checks"],
                      "maximum_errors": result["maximum_errors"]}, indent=2))
    if not result["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

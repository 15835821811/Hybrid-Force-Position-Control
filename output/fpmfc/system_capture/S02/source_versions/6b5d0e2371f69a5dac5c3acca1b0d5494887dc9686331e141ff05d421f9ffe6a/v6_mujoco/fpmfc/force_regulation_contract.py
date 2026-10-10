"""Freeze the N110D 2x2 contrast before any new contact rollout."""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml

from ..model import PROJECT_ROOT
from .contact_config import load_contact_config, normal_admittance_config
from .force_regulation_outer import ForceRegulationConfig
from .force_regulation_provenance import runtime_identity, verification_identity
from .handoff_contract import DEFAULT_OUTPUT_ROOT as N110C_ROOT, load_manifest as load_n110c_manifest, repo_path, sha256


CONFIG_PATH = PROJECT_ROOT / "configs/fpmfc_n110_force_regulation.yaml"
OUTPUT_ROOT = PROJECT_ROOT / "output/fpmfc/n110/force_regulation"
CELLS = ("D00", "D01", "D10", "D11")


def load_config(path: Path = CONFIG_PATH) -> dict:
    config = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if config.get("schema_version") != "n110d_force_regulation_v1":
        raise ValueError("unexpected N110D schema")
    if config["baseline_commit"] != "dd80807948778e8a77a0d7277553a5f6501d8067":
        raise ValueError("N110D baseline commit changed")
    if config["mapping_mode"] != "homogeneous" or config["variant"] != "admittance":
        raise ValueError("only S full-shape admittance is permitted")
    if set(config["conditions"]) != set(CELLS):
        raise ValueError("2x2 cells missing")
    expected = ((2500.0, True), (2500.0, False), (0.0, True), (0.0, False))
    for name, (stiffness, reset) in zip(CELLS, expected, strict=True):
        cell = config["conditions"][name]
        if float(cell["stiffness_n_m"]) != stiffness or bool(cell["reset_on_contact_loss"]) != reset:
            raise ValueError(f"cell {name} differs from frozen 2x2 design")
        if cell["source"] != ("archive_only" if name == "D00" else "new_run"):
            raise ValueError("D00 must reuse archive, other cells each run once")
    for key, expected_value in (("virtual_mass_kg", 1.0), ("explicit_damping_n_s_m", 100.0),
                                ("maximum_offset_m", 0.003), ("maximum_velocity_m_s", 0.020)):
        if float(config[key]) != expected_value:
            raise ValueError(f"unexpected {key}")
    if config["run_budget"] != {"new_full_shape_contact_trajectories": 3,
                                "optional_no_shape_confirmation_trajectories": 1,
                                "no_parameter_search": True}:
        raise ValueError("unexpected run budget")
    contact = load_contact_config(repo_path(config["contact_config"]))
    force = contact["force_control"]
    old = normal_admittance_config(contact, timestep_s=0.002)
    if not (float(force["virtual_mass_kg"]) == 1.0 and float(force["stiffness_n_m"]) == 2500.0
            and old.damping_n_s_m == 100.0 and force["reset_on_contact_loss"] is True
            and float(force["maximum_offset_m"]) == 0.003
            and float(force["maximum_velocity_m_s"]) == 0.020):
        raise ValueError("D00 is not parameter-equivalent to archived C1 admittance")
    if config["source_trace"] != "output/fpmfc/n110/live_twist_handoff/precontact/C1/trace.npz":
        raise ValueError("source must be the frozen C1 precontact trace")
    return config


def outer_config(config: dict, cell: str, *, timestep_s: float = 0.002) -> ForceRegulationConfig:
    frozen = load_contact_config(repo_path(config["contact_config"]))
    interface = frozen["interface"]
    values = config["conditions"][cell]
    return ForceRegulationConfig(
        virtual_mass_kg=float(config["virtual_mass_kg"]),
        damping_n_s_m=float(config["explicit_damping_n_s_m"]),
        stiffness_n_m=float(values["stiffness_n_m"]), timestep_s=timestep_s,
        maximum_offset_m=float(config["maximum_offset_m"]),
        maximum_velocity_m_s=float(config["maximum_velocity_m_s"]),
        detection_force_n=float(interface["contact_detection_force_n"]),
        release_force_n=float(interface["contact_release_force_n"]),
        reset_on_contact_loss=bool(values["reset_on_contact_loss"]),
    )


def prepare_manifest(config_path: Path = CONFIG_PATH, output_root: Path = OUTPUT_ROOT) -> dict:
    config_path, output_root = Path(config_path).resolve(), Path(output_root).resolve()
    config = load_config(config_path)
    path = output_root / "experiment_manifest.json"
    if path.exists():
        raise FileExistsError(path)
    parent = load_n110c_manifest(N110C_ROOT / "pairing_manifest.json")
    source = repo_path(config["source_trace"])
    baseline = repo_path(config["baseline_dir"])
    baseline_metrics = json.loads((baseline / "metrics.json").read_text(encoding="utf-8"))
    baseline_validation = json.loads((baseline / "validation.json").read_text(encoding="utf-8"))
    source_metrics = json.loads((source.parent / "metrics.json").read_text(encoding="utf-8"))
    source_validation = json.loads((source.parent / "validation.json").read_text(encoding="utf-8"))
    baseline_diagnostic = json.loads((output_root / "baseline_error_decomposition.json").read_text(encoding="utf-8"))
    if not (source_metrics["acceptance"]["handoff_passed"] and source_validation["replay_passed"]
            and baseline_validation["passed"] and baseline_metrics["acceptance"]["common_passed"]
            and sha256(source) == baseline_metrics["source_handoff"]["source_trace_sha256"]):
        raise ValueError("archived C1 source and D00 baseline are not qualified")
    equivalence = {
        "offset_reconstruction_exact": baseline_diagnostic["outer_reconstruction"]["maximum_offset_error_m"] == 0.0,
        "velocity_reconstruction_exact": baseline_diagnostic["outer_reconstruction"]["maximum_velocity_error_m_s"] == 0.0,
        "explicit_damping_matches_archive": baseline_diagnostic["outer_reconstruction"]["explicit_damping_n_s_m"] == 100.0,
        "fixed_window_metric_matches_archive": baseline_diagnostic["full_window"]["force_rmse_n"] == baseline_metrics["metrics"]["steady_force_rmse_n"],
    }
    if not all(equivalence.values()):
        raise ValueError(f"D00 entry is not equivalent to archived C1 admittance: {equivalence}")
    with np.load(source, allow_pickle=False) as trace:
        initial_qpos = trace["qpos"][-1].tolist()
        initial_qvel = trace["qvel"][-1].tolist()
    manifest = {
        "schema_version": "n110d_frozen_2x2_contract_v1",
        "frozen_at_utc": datetime.now(timezone.utc).isoformat(),
        "baseline_commit": config["baseline_commit"],
        "config_path": str(config_path.relative_to(PROJECT_ROOT)),
        "config_sha256": sha256(config_path),
        "source_trace_path": config["source_trace"], "source_trace_sha256": sha256(source),
        "source_terminal_qpos": initial_qpos, "source_terminal_qvel": initial_qvel,
        "source_metrics_sha256": sha256(source.parent / "metrics.json"),
        "source_validation_sha256": sha256(source.parent / "validation.json"),
        "baseline_dir": config["baseline_dir"],
        "baseline_trace_sha256": sha256(baseline / "trace.npz"),
        "baseline_metrics_sha256": sha256(baseline / "metrics.json"),
        "baseline_validation_sha256": sha256(baseline / "validation.json"),
        "D00_archive_equivalence_checks": equivalence,
        "parent_n110c_manifest_sha256": sha256(N110C_ROOT / "pairing_manifest.json"),
        "parent_n110c_runtime_sha256": parent["n110c_implementation_identity"]["composite_sha256"],
        "contact_config_path": config["contact_config"],
        "contact_config_sha256": sha256(repo_path(config["contact_config"])),
        "runtime_identity": runtime_identity(),
        "verification_identity": verification_identity(),
        "cells": config["conditions"],
        "virtual_mass_kg": config["virtual_mass_kg"],
        "explicit_damping_n_s_m": config["explicit_damping_n_s_m"],
        "maximum_offset_m": config["maximum_offset_m"],
        "maximum_velocity_m_s": config["maximum_velocity_m_s"],
        "mapping_mode": config["mapping_mode"],
        "selection": config["selection"],
        "run_budget": config["run_budget"],
        "force_target_n": 3.0, "ramp_duration_s": 0.5,
        "contact_duration_s": 1.0, "evaluation_window_s": 0.2,
        "physics_timestep_s": 0.002, "hqp_period_s": 0.020,
        "baseline_error_decomposition_sha256": sha256(output_root / "baseline_error_decomposition.json"),
        "baseline_outer_diagnostics_sha256": sha256(output_root / "baseline_outer_diagnostics.npz"),
    }
    output_root.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    return manifest


def load_manifest(path: Path = OUTPUT_ROOT / "experiment_manifest.json") -> dict:
    path = Path(path).resolve()
    result = json.loads(path.read_text(encoding="utf-8"))
    if result.get("schema_version") != "n110d_frozen_2x2_contract_v1":
        raise ValueError("unexpected N110D manifest schema")
    config = load_config(repo_path(result["config_path"]))
    if sha256(repo_path(result["config_path"])) != result["config_sha256"]:
        raise ValueError("N110D config changed after freeze")
    if sha256(repo_path(config["source_trace"])) != result["source_trace_sha256"]:
        raise ValueError("C1 source trace changed")
    if sha256(repo_path(config["contact_config"])) != result["contact_config_sha256"]:
        raise ValueError("contact config changed")
    if runtime_identity() != result["runtime_identity"] or verification_identity() != result["verification_identity"]:
        raise ValueError("N110D source identity changed after freeze")
    if sha256(path.parent / "baseline_error_decomposition.json") != result["baseline_error_decomposition_sha256"]:
        raise ValueError("baseline diagnostic changed after freeze")
    if sha256(path.parent / "baseline_outer_diagnostics.npz") != result["baseline_outer_diagnostics_sha256"]:
        raise ValueError("baseline outer terms changed after freeze")
    if sha256(N110C_ROOT / "pairing_manifest.json") != result["parent_n110c_manifest_sha256"]:
        raise ValueError("N110C pairing contract changed")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=CONFIG_PATH)
    parser.add_argument("--output-root", type=Path, default=OUTPUT_ROOT)
    args = parser.parse_args()
    result = prepare_manifest(args.config, args.output_root)
    print(json.dumps({"manifest": str(args.output_root / "experiment_manifest.json"),
                      "runtime_sha256": result["runtime_identity"]["composite_sha256"]}, indent=2))


if __name__ == "__main__":
    main()

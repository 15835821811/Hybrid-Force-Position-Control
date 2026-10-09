"""Run exactly the new N110D cells through the unchanged S contact loop."""

from __future__ import annotations

import argparse
import copy
import json
from dataclasses import asdict
from pathlib import Path
from unittest.mock import patch

import numpy as np

from . import run_contact_consistent as consistent
from .contact_config import load_contact_config
from .force_regulation_contract import CELLS, OUTPUT_ROOT, load_config, load_manifest, outer_config
from .force_regulation_outer import ExperimentalNormalAdmittance, diagnostics_arrays
from .handoff_contract import repo_path, sha256


def _selected_passing_cell(root: Path, manifest: dict) -> str | None:
    candidates = []
    for name in CELLS:
        directory = repo_path(manifest["baseline_dir"]) if name == "D00" else root/name
        metrics_path = directory/"metrics.json"
        if not metrics_path.exists():
            continue
        metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
        if name != "D00":
            validation_path = directory/"validation.json"
            if not validation_path.exists() or not json.loads(validation_path.read_text(encoding="utf-8"))["passed"]:
                continue
        if metrics["acceptance"]["common_passed"] and metrics["acceptance"]["steady_force_tracking"]:
            m = metrics["metrics"]
            candidates.append((m["steady_force_rmse_n"], m["peak_normal_force_n"],
                               m["contact_loss_events"], m["maximum_sustained_contact_loss_s"], name))
    if not candidates:
        return None
    best_rmse = min(item[0] for item in candidates)
    close = [item for item in candidates if item[0]-best_rmse <= manifest["selection"]["near_tie_rmse_n"]]
    return min(close, key=lambda item: item[1:])[4]


def run_cell(cell: str, *, no_shape_confirmation: bool = False,
             manifest_path: Path = OUTPUT_ROOT/"experiment_manifest.json") -> dict:
    manifest_path = Path(manifest_path).resolve()
    manifest = load_manifest(manifest_path)
    config = load_config(repo_path(manifest["config_path"]))
    if cell not in CELLS or cell == "D00":
        raise ValueError("only D01/D10/D11 create new full-shape trajectories")
    root = manifest_path.parent
    variant = "admittance-no-shape" if no_shape_confirmation else "admittance"
    if no_shape_confirmation:
        if _selected_passing_cell(root, manifest) != cell:
            raise ValueError("no-shape confirmation requires the frozen best qualifying cell")
        if any((root/f"{name}_no_shape").exists() for name in CELLS):
            raise FileExistsError("the one permitted no-shape confirmation already exists")
        output = root/f"{cell}_no_shape"
    else:
        output = root/cell
        if sum((root/name/"trace.npz").exists() for name in CELLS if name != "D00") >= 3:
            raise RuntimeError("the three full-shape N110D runs are already used")
    if output.exists():
        raise FileExistsError(output)
    original = load_contact_config(repo_path(manifest["contact_config_path"]))
    modified = copy.deepcopy(original)
    values = config["conditions"][cell]
    modified["force_control"]["stiffness_n_m"] = float(values["stiffness_n_m"])
    modified["force_control"]["reset_on_contact_loss"] = bool(values["reset_on_contact_loss"])
    unchanged = copy.deepcopy(modified)
    unchanged["force_control"]["stiffness_n_m"] = original["force_control"]["stiffness_n_m"]
    unchanged["force_control"]["reset_on_contact_loss"] = original["force_control"]["reset_on_contact_loss"]
    if unchanged != original:
        raise RuntimeError("unplanned effective contact config difference")
    outer_cfg = outer_config(config, cell)
    instances: list[ExperimentalNormalAdmittance] = []

    def patched_load(_path: Path) -> dict:
        return copy.deepcopy(modified)

    def patched_config(effective: dict, *, timestep_s: float):
        if effective != modified or timestep_s != outer_cfg.timestep_s:
            raise RuntimeError("unexpected outer-loop configuration or task cadence")
        return outer_cfg

    def make_outer(cfg):
        if cfg != outer_cfg or instances:
            raise RuntimeError("unexpected number of normal outer loops")
        instance = ExperimentalNormalAdmittance(cfg)
        instances.append(instance)
        return instance

    with patch.object(consistent, "load_contact_config", patched_load), \
         patch.object(consistent, "normal_admittance_config", patched_config), \
         patch.object(consistent, "NormalAdmittance", make_outer):
        result = consistent.run_consistent_contact_experiment(
            variant=variant, source_trace_path=repo_path(manifest["source_trace_path"]),
            contact_config_path=repo_path(manifest["contact_config_path"]),
            output_dir=output, mapping_mode="homogeneous",
        )
    if len(instances) != 1 or len(instances[0].rows) != 500:
        raise RuntimeError("outer loop did not update exactly once per physics step")
    arrays = diagnostics_arrays(instances[0])
    with np.load(output/"trace.npz", allow_pickle=False) as trace:
        for left, right in (("x_after_step_m", "command_normal_offset_m"),
                            ("v_after_step_m_s", "command_normal_offset_velocity_m_s"),
                            ("measured_force_n", "feedback_measured_normal_force_n"),
                            ("desired_force_n", "command_desired_normal_force_n")):
            if not np.array_equal(arrays[left], trace[right]):
                raise RuntimeError(f"outer diagnostic differs from contact trace: {left}")
        if float(np.max(np.abs(arrays["feedback_time_s"]-trace["feedback_time_s"]))) > 1e-11:
            raise RuntimeError("outer diagnostic time differs from causal feedback")
    np.savez_compressed(output/"outer_diagnostics.npz", **arrays)
    snapshot_path = output/"config_snapshot.json"
    snapshot = json.loads(snapshot_path.read_text(encoding="utf-8"))
    snapshot["n110d"] = {"cell": cell, "variant": variant, "outer_config": asdict(outer_cfg),
                          "experiment_manifest_sha256": sha256(manifest_path),
                          "runtime_identity": manifest["runtime_identity"]}
    snapshot_path.write_text(json.dumps(snapshot, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    result["schema_version"] = "n110d_force_regulation_contact_v1"
    result["n110d"] = {"cell": cell, "variant": variant, "outer_config": asdict(outer_cfg),
                       "experiment_manifest_sha256": sha256(manifest_path),
                       "runtime_identity": manifest["runtime_identity"],
                       "outer_diagnostics_sha256": sha256(output/"outer_diagnostics.npz")}
    result["config_snapshot_sha256"] = sha256(snapshot_path)
    (output/"metrics.json").write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cell", choices=CELLS, required=True)
    parser.add_argument("--no-shape-confirmation", action="store_true")
    parser.add_argument("--manifest", type=Path, default=OUTPUT_ROOT/"experiment_manifest.json")
    args = parser.parse_args()
    result = run_cell(args.cell, no_shape_confirmation=args.no_shape_confirmation,
                      manifest_path=args.manifest)
    print(json.dumps({"cell": args.cell, "variant": result["variant"],
                      "metrics": result["metrics"], "acceptance": result["acceptance"]}, indent=2))


if __name__ == "__main__":
    main()

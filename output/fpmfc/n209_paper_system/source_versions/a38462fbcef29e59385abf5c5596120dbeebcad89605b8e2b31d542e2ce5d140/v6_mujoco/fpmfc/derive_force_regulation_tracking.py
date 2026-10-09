"""Save per-step normal execution errors derived from immutable traces."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .force_regulation_contract import CELLS, OUTPUT_ROOT, load_manifest
from .handoff_contract import repo_path, sha256


def derive(cell: str, root: Path = OUTPUT_ROOT) -> dict:
    root = Path(root).resolve()
    manifest = load_manifest(root/"experiment_manifest.json")
    if cell not in CELLS:
        raise ValueError(cell)
    source = repo_path(manifest["baseline_dir"]) if cell == "D00" else root/cell
    destination = root if cell == "D00" else source
    prefix = "baseline_" if cell == "D00" else ""
    array_path = destination/f"{prefix}normal_tracking_diagnostics.npz"
    record_path = destination/f"{prefix}normal_tracking_diagnostics.json"
    if array_path.exists() or record_path.exists():
        raise FileExistsError(f"normal tracking result already exists for {cell}")
    with np.load(source/"trace.npz", allow_pickle=False) as trace:
        normal = np.asarray(trace["contact_normal_world"], dtype=float)
        desired = np.asarray(trace["desired_position"], dtype=float)
        flange = np.asarray(trace["flange_position"], dtype=float)
        tool_face = np.asarray(trace["tool_face_position_world_m"], dtype=float)
        grasp = np.asarray(trace["target_grasp_position"], dtype=float)
        arrays = {
            "post_time_s": np.asarray(trace["time"], dtype=float),
            "commanded_flange_normal_error_m": np.einsum("ij,ij->i", desired-flange, normal),
            "tool_face_to_grasp_normal_gap_m": np.einsum("ij,ij->i", tool_face-grasp, normal),
            "command_normal_offset_m": np.asarray(trace["command_normal_offset_m"], dtype=float),
            "physical_penetration_m": np.asarray(trace["penetration_m"], dtype=float),
            "post_measured_force_n": np.asarray(trace["measured_normal_force_n"], dtype=float),
            "post_desired_force_n": np.asarray(trace["desired_normal_force_n"], dtype=float),
            "post_force_error_n": np.asarray(trace["desired_normal_force_n"]-trace["measured_normal_force_n"], dtype=float),
            "geometric_contact": np.asarray(trace["contact_count"] > 0, dtype=bool),
            "force_detected": np.asarray(trace["contact_detected"], dtype=bool),
            "true_zero_force": np.asarray(trace["measured_normal_force_n"] == 0.0, dtype=bool),
        }
    if any(value.shape != (500,) for value in arrays.values()):
        raise ValueError("expected exactly 500 same-time post-step samples")
    np.savez_compressed(array_path, **arrays)
    result = {
        "schema_version": "n110d_per_step_normal_tracking_v1",
        "cell": cell,
        "source_trace_sha256": sha256(source/"trace.npz"),
        "derived_npz_sha256": sha256(array_path),
        "derivation_source_sha256": sha256(Path(__file__)),
        "time_and_reference": "post-step world-frame flange normal error (desired flange minus physical flange) projected on target grasp +Z; tool-face gap is separately projected; neither is physical penetration",
        "samples": 500,
    }
    record_path.write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cell", choices=CELLS, required=True)
    args = parser.parse_args()
    print(json.dumps(derive(args.cell), indent=2))


if __name__ == "__main__":
    main()

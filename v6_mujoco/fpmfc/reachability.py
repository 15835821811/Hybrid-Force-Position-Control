"""Scan capture-time/arm-shape feasibility with the kinematic FPMFC rollout."""

from __future__ import annotations

import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from ..model import PROJECT_ROOT
from .config import DEFAULT_CONFIG_PATH, load_fpmfc_config
from .rollout import PrecontactRolloutEvaluator


def _effective_config_sha256(config: Mapping[str, Any]) -> str:
    encoded = json.dumps(
        config, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def scan_reachability(
    config: Mapping[str, Any],
    *,
    times_s: list[float],
    arm_angles_rad: list[float],
    output: Path,
) -> dict[str, Any]:
    if not times_s or not arm_angles_rad:
        raise ValueError("reachability scan needs at least one time and arm angle")
    evaluator = PrecontactRolloutEvaluator(config)
    rows: list[dict[str, Any]] = []
    for arm_angle in arm_angles_rad:
        for capture_time in times_s:
            result = evaluator.evaluate(float(capture_time), float(arm_angle))
            row = result.to_dict()
            rows.append(row)
            print(
                f"T={capture_time:.6f} psi={arm_angle:.6f} "
                f"feasible={result.feasible} objective={result.objective:.9g} "
                f"position_mm={1000.0 * result.terminal_position_error_m:.5f} "
                f"shape_deg={np.rad2deg(result.terminal_arm_angle_error_rad):.5f}",
                flush=True,
            )
    feasible_rows = [row for row in rows if row["feasible"]]
    report = {
        "material_passport": {
            "origin_skill": "experiment-agent",
            "origin_mode": "run",
            "origin_date": datetime.now(timezone.utc).isoformat(),
            "verification_status": "KINEMATIC_ONLY",
            "version_label": "reachability_scan_v1",
        },
        "effective_config_sha256": _effective_config_sha256(config),
        "times_s": list(map(float, times_s)),
        "arm_angles_rad": list(map(float, arm_angles_rad)),
        "candidate_count": len(rows),
        "feasible_count": len(feasible_rows),
        "feasible_fraction": len(feasible_rows) / len(rows),
        "feasible_capture_time_range_s": (
            [
                min(row["capture_time_s"] for row in feasible_rows),
                max(row["capture_time_s"] for row in feasible_rows),
            ]
            if feasible_rows
            else None
        ),
        "candidates": rows,
    }
    output_path = Path(output).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(f"wrote={output_path}", flush=True)
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--time-start", type=float, default=8.0)
    parser.add_argument("--time-stop", type=float, default=25.0)
    parser.add_argument("--time-step", type=float, default=1.0)
    parser.add_argument("--arm-angles", nargs="+", type=float, required=True)
    parser.add_argument(
        "--output",
        type=Path,
        default=PROJECT_ROOT / "output" / "fpmfc" / "reachability" / "scan.json",
    )
    args = parser.parse_args()
    if args.time_step <= 0.0 or args.time_stop < args.time_start:
        raise ValueError("invalid time grid")
    count = int(np.floor((args.time_stop - args.time_start) / args.time_step + 1e-12))
    times = [args.time_start + index * args.time_step for index in range(count + 1)]
    if times[-1] < args.time_stop - 1e-12:
        times.append(args.time_stop)
    config = load_fpmfc_config(args.config)
    scan_reachability(
        config,
        times_s=times,
        arm_angles_rad=args.arm_angles,
        output=args.output,
    )


if __name__ == "__main__":
    main()

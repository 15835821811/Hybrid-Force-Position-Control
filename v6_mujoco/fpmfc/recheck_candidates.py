"""Counterfactually re-evaluate saved PSO candidates under a new clearance margin.

This is a decision diagnostic only: it holds every saved ``(T_c, psi_f)`` pair
fixed and therefore cannot substitute for rerunning PSO under the new contract.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from ..model import PROJECT_ROOT
from .config import DEFAULT_CONFIG_PATH, apply_runtime_overrides, load_fpmfc_config
from .rollout import PrecontactRolloutEvaluator


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _effective_config_sha256(config: dict[str, Any]) -> str:
    encoded = json.dumps(
        config, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def recheck_optimizer_candidates(
    optimizer_json: Path,
    *,
    config_path: Path,
    planning_clearance_m: float,
    output_path: Path,
    top_k: int = 5,
) -> dict[str, Any]:
    optimizer_path = Path(optimizer_json).resolve()
    config_file = Path(config_path).resolve()
    payload = json.loads(optimizer_path.read_text(encoding="utf-8"))
    if payload.get("config_sha256") and payload["config_sha256"] != _sha256(config_file):
        raise ValueError("optimizer result and requested source configuration hashes differ")
    if top_k < 1:
        raise ValueError("top_k must be positive")

    source = load_fpmfc_config(config_file)
    original_overrides = payload.get("runtime_overrides") or {}
    effective = apply_runtime_overrides(
        source,
        objective_weights=original_overrides.get("objective_weights"),
        planning_clearance_m=float(planning_clearance_m),
    )
    evaluator = PrecontactRolloutEvaluator(effective)
    candidates = payload["optimizer"].get("top_candidates", [])[:top_k]
    if not candidates:
        raise ValueError("optimizer JSON contains no saved top candidates")

    rows: list[dict[str, Any]] = []
    for rank, candidate in enumerate(candidates, start=1):
        capture_time, arm_angle = map(float, candidate["position"])
        original = candidate["rollout"]
        rechecked = evaluator.evaluate(capture_time, arm_angle)
        rows.append(
            {
                "original_rank": rank,
                "capture_time_s": capture_time,
                "terminal_arm_angle_rad": arm_angle,
                "original_objective": float(candidate["objective"]),
                "original_feasible": bool(original["feasible"]),
                "original_minimum_clearance_m": original["minimum_clearance_m"],
                "rechecked_objective": float(rechecked.objective),
                "rechecked_feasible": bool(rechecked.feasible),
                "rechecked_minimum_clearance_m": rechecked.minimum_clearance_m,
                "rechecked_constraint_penalty": rechecked.constraint_penalty,
                "rechecked_rollout": rechecked.to_dict(),
            }
        )

    feasible = [row for row in rows if row["rechecked_feasible"]]
    feasible_ranking = sorted(feasible, key=lambda row: row["rechecked_objective"])
    for rank, row in enumerate(feasible_ranking, start=1):
        row["rechecked_rank"] = rank
    acceptance_clearance = float(
        effective["acceptance"].get(
            "minimum_clearance_m", effective["controller"]["minimum_clearance_m"]
        )
    )
    report = {
        "material_passport": {
            "origin_skill": "experiment-agent",
            "origin_mode": "run",
            "origin_date": datetime.now(timezone.utc).isoformat(),
            "verification_status": "COUNTERFACTUAL_DIAGNOSTIC",
            "version_label": "candidate_clearance_recheck_v1",
        },
        "scope_warning": (
            "Fixed-candidate re-evaluation only; this does not replace PSO under the new clearance."
        ),
        "optimizer_json": str(optimizer_path),
        "optimizer_json_sha256": _sha256(optimizer_path),
        "source_config": str(config_file),
        "source_config_sha256": _sha256(config_file),
        "original_effective_config_sha256": payload.get("effective_config_sha256"),
        "rechecked_effective_config_sha256": _effective_config_sha256(effective),
        "planning_clearance_m": float(effective["controller"]["minimum_clearance_m"]),
        "acceptance_clearance_m": acceptance_clearance,
        "candidate_count": len(rows),
        "rechecked_feasible_count": len(feasible),
        "candidates": rows,
        "feasible_ranking": feasible_ranking,
    }
    output = Path(output_path).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--optimizer-json", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--planning-clearance", type=float, required=True)
    parser.add_argument("--top-k", type=int, default=5)
    parser.add_argument(
        "--output",
        type=Path,
        default=PROJECT_ROOT
        / "output"
        / "fpmfc"
        / "optimization"
        / "candidate_clearance_recheck.json",
    )
    args = parser.parse_args()
    report = recheck_optimizer_candidates(
        args.optimizer_json,
        config_path=args.config,
        planning_clearance_m=args.planning_clearance,
        output_path=args.output,
        top_k=args.top_k,
    )
    print(
        json.dumps(
            {
                "candidate_count": report["candidate_count"],
                "rechecked_feasible_count": report["rechecked_feasible_count"],
                "planning_clearance_m": report["planning_clearance_m"],
                "acceptance_clearance_m": report["acceptance_clearance_m"],
                "output": str(Path(args.output).resolve()),
            },
            indent=2,
            ensure_ascii=False,
        )
    )


if __name__ == "__main__":
    main()

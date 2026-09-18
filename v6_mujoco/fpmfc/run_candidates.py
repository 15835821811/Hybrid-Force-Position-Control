"""Replay the top candidates from one PSO seed in 500 Hz MuJoCo dynamics."""

from __future__ import annotations

import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from ..model import PROJECT_ROOT
from .config import DEFAULT_CONFIG_PATH, apply_runtime_overrides, load_fpmfc_config
from .provenance import implementation_identity
from .run_capture import run_precontact_candidate
from .validate_capture import validate_capture_output


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


def _effective_config_from_optimizer_payload(
    payload: dict[str, Any], config_file: Path
) -> dict[str, Any]:
    """Reconstruct and verify the exact config used by the optimizer."""

    source = load_fpmfc_config(config_file)
    overrides = payload.get("runtime_overrides") or {}
    known = {"objective_weights", "planning_clearance_m"}
    unknown = sorted(key for key, value in overrides.items() if key not in known and value is not None)
    if unknown:
        raise ValueError(f"unsupported optimizer runtime overrides: {unknown}")
    effective = apply_runtime_overrides(
        source,
        objective_weights=overrides.get("objective_weights"),
        planning_clearance_m=overrides.get("planning_clearance_m"),
    )
    expected_hash = payload.get("effective_config_sha256")
    actual_hash = _effective_config_sha256(effective)
    if expected_hash and expected_hash != actual_hash:
        raise ValueError(
            "optimizer effective configuration cannot be reconstructed from its runtime overrides"
        )
    stored_config = payload.get("effective_config")
    if stored_config is not None and stored_config != effective:
        raise ValueError(
            "optimizer effective configuration snapshot differs from the reconstructed config"
        )
    return effective


def _planning_and_dynamic_qualified(row: dict[str, Any]) -> bool:
    """Require both planning feasibility and torque-level acceptance."""

    return bool(
        row["kinematic_feasible"]
        and row["dynamic_acceptance_passed"]
        and row["independent_replay_passed"]
    )


def replay_optimizer_candidates(
    optimizer_json: Path,
    *,
    config_path: Path,
    output_root: Path,
    top_k: int = 5,
) -> dict[str, Any]:
    optimizer_path = Path(optimizer_json).resolve()
    payload = json.loads(optimizer_path.read_text(encoding="utf-8"))
    config_file = Path(config_path).resolve()
    if payload.get("config_sha256") and payload["config_sha256"] != _sha256(config_file):
        raise ValueError("optimizer result and requested configuration hashes differ")
    config = _effective_config_from_optimizer_payload(payload, config_file)
    code_identity = implementation_identity()
    optimizer_code_identity = payload.get("implementation_identity")
    if (
        optimizer_code_identity is not None
        and optimizer_code_identity.get("composite_sha256")
        != code_identity["composite_sha256"]
    ):
        raise ValueError(
            "optimizer and dynamic replay implementation identities differ"
        )
    candidates = payload["optimizer"].get("top_candidates", [])
    if not candidates:
        raise ValueError("optimizer JSON contains no saved top candidates")
    if top_k < 1:
        raise ValueError("top_k must be positive")
    selected = candidates[: min(int(top_k), len(candidates))]
    root = Path(output_root).resolve()
    root.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, Any]] = []
    for rank, candidate in enumerate(selected, start=1):
        capture_time, arm_angle = map(float, candidate["position"])
        run_dir = root / f"candidate_{rank:02d}"
        dynamic = run_precontact_candidate(
            config,
            capture_time_s=capture_time,
            terminal_arm_angle_rad=arm_angle,
            output_dir=run_dir,
        )
        validation = validate_capture_output(run_dir)
        row = {
                "kinematic_rank": rank,
                "capture_time_s": capture_time,
                "terminal_arm_angle_rad": arm_angle,
                "kinematic_objective": float(candidate["objective"]),
                "kinematic_feasible": bool(candidate["rollout"]["feasible"]),
                "dynamic_objective": float(dynamic["metrics"]["dynamic_objective"]),
                "dynamic_acceptance_passed": bool(dynamic["acceptance"]["passed"]),
                "independent_replay_passed": bool(validation["passed"]),
                "metrics_path": str(run_dir / "metrics.json"),
                "validation_path": str(run_dir / "validation.json"),
            }
        row["planning_and_dynamic_qualified"] = _planning_and_dynamic_qualified(row)
        rows.append(row)
    valid_rows = [row for row in rows if _planning_and_dynamic_qualified(row)]
    dynamically_accepted_rows = [
        row
        for row in rows
        if row["dynamic_acceptance_passed"] and row["independent_replay_passed"]
    ]
    ranked = sorted(valid_rows, key=lambda row: row["dynamic_objective"])
    for rank, row in enumerate(ranked, start=1):
        row["dynamic_rank"] = rank
    report = {
        "material_passport": {
            "origin_skill": "experiment-agent",
            "origin_mode": "run",
            "origin_date": datetime.now(timezone.utc).isoformat(),
            "verification_status": (
                "VERIFIED" if len(valid_rows) == len(rows) else "PARTIALLY_VERIFIED"
            ),
            "version_label": "candidate_replay_v1",
        },
        "optimizer_json": str(optimizer_path),
        "optimizer_json_sha256": _sha256(optimizer_path),
        "config_path": str(config_file),
        "config_sha256": _sha256(config_file),
        "effective_config_sha256": _effective_config_sha256(config),
        "runtime_overrides": payload.get("runtime_overrides") or {},
        "implementation_identity": code_identity,
        "optimizer_implementation_identity_present": optimizer_code_identity is not None,
        "candidate_count": len(rows),
        "accepted_candidate_count": len(valid_rows),
        "dynamic_accepted_candidate_count": len(dynamically_accepted_rows),
        "planning_and_dynamic_qualified_candidate_count": len(valid_rows),
        "candidates": rows,
        "dynamic_ranking": ranked,
    }
    summary_path = root / "candidate_replay_summary.json"
    summary_path.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--optimizer-json", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--top-k", type=int, default=5)
    parser.add_argument(
        "--output-root",
        type=Path,
        default=PROJECT_ROOT / "output" / "fpmfc" / "precontact" / "optimizer_candidates",
    )
    args = parser.parse_args()
    report = replay_optimizer_candidates(
        args.optimizer_json,
        config_path=args.config,
        output_root=args.output_root,
        top_k=args.top_k,
    )
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()

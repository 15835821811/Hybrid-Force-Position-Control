"""Freeze, run, and audit the N073 multi-seed controller ablation."""

from __future__ import annotations

import argparse
import concurrent.futures
import csv
import hashlib
import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

import numpy as np
from scipy.stats import wilcoxon

from ..model import PROJECT_ROOT
from .compare_precontact import compare_runs
from .config import load_fpmfc_config
from .provenance import implementation_identity
from .run_capture import run_precontact_candidate
from .validate_capture import validate_capture_output


VARIANTS = ("full", "no-shape", "no-base-reaction")
VARIANT_DIRECTORIES = {
    "full": "full",
    "no-shape": "no_shape",
    "no-base-reaction": "no_base_reaction",
}
VARIANT_LABELS = {
    "full": "Full",
    "no-shape": "No shape",
    "no-base-reaction": "No base reaction",
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _canonical_sha256(value: Any) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _effective_config_sha256(config: dict[str, Any]) -> str:
    return _canonical_sha256(config)


def _is_close(left: float, right: float, *, tolerance: float = 1e-12) -> bool:
    return abs(float(left) - float(right)) <= tolerance


def _all_finite(value: Any) -> bool:
    if isinstance(value, dict):
        return all(_all_finite(item) for item in value.values())
    if isinstance(value, list):
        return all(_all_finite(item) for item in value)
    if isinstance(value, float):
        return math.isfinite(value)
    return True


def _replay_integrity(validation: dict[str, Any], metrics_path: Path) -> bool:
    checks = validation.get("checks", {})
    deterministic = [
        bool(result)
        for name, result in checks.items()
        if name.startswith("deterministic_") or name == "trace_hash_matches_metrics"
    ]
    return bool(
        validation.get("metrics_sha256") == _sha256(metrics_path)
        and deterministic
        and all(deterministic)
    )


def _select_first_qualified(candidates: Iterable[dict[str, Any]]) -> dict[str, Any]:
    """Select in saved kinematic order, never from dynamic_ranking."""

    for candidate in candidates:
        if candidate.get("planning_and_dynamic_qualified") is True:
            return candidate
    raise ValueError("candidate summary contains no jointly qualified candidate")


def _source_row(
    *,
    seed: int,
    summary_path: Path,
    expected_config_sha256: str,
    expected_implementation_sha256: str,
) -> dict[str, Any]:
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    if summary.get("effective_config_sha256") != expected_config_sha256:
        raise ValueError(f"seed {seed}: effective configuration hash mismatch")
    if (
        summary.get("implementation_identity", {}).get("composite_sha256")
        != expected_implementation_sha256
    ):
        raise ValueError(f"seed {seed}: implementation identity mismatch")

    candidate = _select_first_qualified(summary.get("candidates", []))
    metrics_path = Path(candidate["metrics_path"]).resolve()
    validation_path = Path(candidate["validation_path"]).resolve()
    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    validation = json.loads(validation_path.read_text(encoding="utf-8"))
    source_candidate = metrics["candidate"]
    if not (
        metrics.get("acceptance", {}).get("passed") is True
        and validation.get("passed") is True
        and _replay_integrity(validation, metrics_path)
        and metrics.get("effective_config_sha256") == expected_config_sha256
        and metrics.get("implementation_identity", {}).get("composite_sha256")
        == expected_implementation_sha256
        and _is_close(
            source_candidate["requested_capture_time_s"], candidate["capture_time_s"]
        )
        and _is_close(
            source_candidate["terminal_arm_angle_rad"],
            candidate["terminal_arm_angle_rad"],
        )
    ):
        raise ValueError(f"seed {seed}: selected source candidate failed provenance audit")

    trace_path = Path(metrics["trace"]["path"]).resolve()
    if not trace_path.is_file() or _sha256(trace_path) != metrics["trace"]["sha256"]:
        raise ValueError(f"seed {seed}: source trace hash mismatch")
    return {
        "seed": int(seed),
        "selection_rule": "first_planning_and_dynamic_qualified_in_candidates",
        "kinematic_rank": int(candidate["kinematic_rank"]),
        "requested_capture_time_s": float(candidate["capture_time_s"]),
        "source_executed_capture_time_s": float(
            source_candidate["executed_capture_time_s"]
        ),
        "terminal_arm_angle_rad": float(candidate["terminal_arm_angle_rad"]),
        "source_summary_path": str(summary_path),
        "source_summary_sha256": _sha256(summary_path),
        "source_metrics_path": str(metrics_path),
        "source_metrics_sha256": _sha256(metrics_path),
        "source_validation_path": str(validation_path),
        "source_validation_sha256": _sha256(validation_path),
        "source_trace_path": str(trace_path),
        "source_trace_sha256": _sha256(trace_path),
        "source_target": metrics["target"],
        "source_model_identity": metrics["model_identity"],
        "source_dynamic_run_config": metrics["dynamic_run_config"],
        "source_physics_steps": int(metrics["physics_steps"]),
        "source_task_ticks": int(metrics["task_ticks"]),
    }


def build_candidate_manifest(
    source_root: Path,
    *,
    config_path: Path,
    output_root: Path,
    seeds: Iterable[int] = range(10),
) -> tuple[dict[str, Any], Path]:
    """Create the immutable ten-row N073 selection manifest."""

    source = Path(source_root).resolve()
    config_file = Path(config_path).resolve()
    output = Path(output_root).resolve()
    config = load_fpmfc_config(config_file)
    effective_hash = _effective_config_sha256(config)
    code_identity = implementation_identity()
    rows = [
        _source_row(
            seed=int(seed),
            summary_path=(
                source
                / f"seed_{int(seed):02d}"
                / "candidate_replay_summary.json"
            ).resolve(),
            expected_config_sha256=effective_hash,
            expected_implementation_sha256=code_identity["composite_sha256"],
        )
        for seed in seeds
    ]
    if len(rows) != 10 or [row["seed"] for row in rows] != list(range(10)):
        raise ValueError("formal N073 manifest must contain seeds 0 through 9 exactly once")
    selection_payload = {
        "selection_rule": "first_planning_and_dynamic_qualified_in_candidates",
        "effective_config_sha256": effective_hash,
        "implementation_composite_sha256": code_identity["composite_sha256"],
        "rows": rows,
    }
    manifest = {
        "material_passport": {
            "origin_skill": "experiment-plan",
            "origin_mode": "run",
            "origin_date": datetime.now(timezone.utc).isoformat(),
            "verification_status": "VERIFIED",
            "version_label": "n073_candidate_manifest_v1",
        },
        "experiment_id": "N073",
        "scope": "controller_ablation_on_preplanned_full_controller_candidates",
        "source_root": str(source),
        "config_path": str(config_file),
        "config_file_sha256": _sha256(config_file),
        "effective_config_sha256": effective_hash,
        "implementation_identity": code_identity,
        "selection_rule": selection_payload["selection_rule"],
        "row_count": len(rows),
        "rows": rows,
        "selection_sha256": _canonical_sha256(selection_payload),
    }
    output.mkdir(parents=True, exist_ok=True)
    manifest_path = output / "candidate_manifest.json"
    if manifest_path.exists():
        existing = json.loads(manifest_path.read_text(encoding="utf-8"))
        if existing.get("selection_sha256") != manifest["selection_sha256"]:
            raise ValueError(
                "existing N073 manifest differs from the newly audited selection; "
                "refusing to overwrite immutable candidates"
            )
        return existing, manifest_path
    manifest_path.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    return manifest, manifest_path


def _run_directory(output_root: Path, seed: int, variant: str) -> Path:
    return output_root / f"seed_{seed:02d}" / VARIANT_DIRECTORIES[variant]


def _completed_run_matches(
    run_dir: Path,
    *,
    row: dict[str, Any],
    variant: str,
    implementation_sha256: str,
) -> bool:
    metrics_path = run_dir / "metrics.json"
    trace_path = run_dir / "trace.npz"
    validation_path = run_dir / "validation.json"
    if not all(path.is_file() for path in (metrics_path, trace_path, validation_path)):
        return False
    try:
        metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
        validation = json.loads(validation_path.read_text(encoding="utf-8"))
    except (OSError, ValueError, TypeError):
        return False
    candidate = metrics.get("candidate", {})
    return bool(
        metrics.get("controller_variant") == variant
        and metrics.get("implementation_identity", {}).get("composite_sha256")
        == implementation_sha256
        and _is_close(
            candidate.get("requested_capture_time_s", float("inf")),
            row["requested_capture_time_s"],
        )
        and _is_close(
            candidate.get("terminal_arm_angle_rad", float("inf")),
            row["terminal_arm_angle_rad"],
        )
        and metrics.get("trace", {}).get("sha256") == _sha256(trace_path)
        and _replay_integrity(validation, metrics_path)
    )


def run_controller_ablation(
    manifest: dict[str, Any],
    *,
    config_path: Path,
    output_root: Path,
    parallelism: int = 1,
) -> None:
    if parallelism < 1:
        raise ValueError("parallelism must be positive")
    config = load_fpmfc_config(Path(config_path).resolve())
    if _effective_config_sha256(config) != manifest["effective_config_sha256"]:
        raise ValueError("execution config differs from frozen N073 manifest")
    code_hash = implementation_identity()["composite_sha256"]
    if code_hash != manifest["implementation_identity"]["composite_sha256"]:
        raise ValueError("implementation differs from frozen N073 manifest")
    root = Path(output_root).resolve()
    jobs: list[tuple[dict[str, Any], str, Path]] = []
    for row in manifest["rows"]:
        for variant in VARIANTS:
            run_dir = _run_directory(root, int(row["seed"]), variant)
            if _completed_run_matches(
                run_dir,
                row=row,
                variant=variant,
                implementation_sha256=code_hash,
            ):
                print(
                    f"skip-N073 seed={row['seed']} variant={variant} path={run_dir}",
                    flush=True,
                )
                continue
            if run_dir.exists() and any(run_dir.iterdir()):
                raise ValueError(
                    f"incomplete or mismatched output exists at {run_dir}; "
                    "refusing to overwrite evidence"
                )
            jobs.append((row, variant, run_dir))

    def execute(job: tuple[dict[str, Any], str, Path]) -> None:
        row, variant, run_dir = job
        print(f"start-N073 seed={row['seed']} variant={variant}", flush=True)
        run_precontact_candidate(
            config,
            capture_time_s=float(row["requested_capture_time_s"]),
            terminal_arm_angle_rad=float(row["terminal_arm_angle_rad"]),
            output_dir=run_dir,
            controller_variant=variant,
        )
        validate_capture_output(run_dir)
        print(f"complete-N073 seed={row['seed']} variant={variant}", flush=True)

    if parallelism == 1:
        for job in jobs:
            execute(job)
    else:
        with concurrent.futures.ThreadPoolExecutor(
            max_workers=min(parallelism, max(1, len(jobs)))
        ) as executor:
            futures = [executor.submit(execute, job) for job in jobs]
            for future in concurrent.futures.as_completed(futures):
                future.result()


def _variant_config_contract(
    configs: dict[str, dict[str, Any]],
) -> bool:
    reference = json.loads(json.dumps(configs["full"]))
    reference_shape = float(reference["controller"]["shape_weight"])
    reference_base = float(reference["controller"]["base_reaction_weight"])

    def stripped(config: dict[str, Any]) -> dict[str, Any]:
        result = json.loads(json.dumps(config))
        result["controller"].pop("shape_weight", None)
        result["controller"].pop("base_reaction_weight", None)
        return result

    expected = {
        "full": (reference_shape, reference_base),
        "no-shape": (0.0, reference_base),
        "no-base-reaction": (reference_shape, 0.0),
    }
    return all(
        stripped(config) == stripped(reference)
        and (
            float(config["controller"]["shape_weight"]),
            float(config["controller"]["base_reaction_weight"]),
        )
        == expected[variant]
        for variant, config in configs.items()
    )


def audit_controller_ablation(
    manifest: dict[str, Any],
    *,
    manifest_path: Path,
    output_root: Path,
    comparison_root: Path,
) -> dict[str, Any]:
    root = Path(output_root).resolve()
    comparison = Path(comparison_root).resolve()
    comparison.mkdir(parents=True, exist_ok=True)
    expected_code_hash = manifest["implementation_identity"]["composite_sha256"]
    result_rows: list[dict[str, Any]] = []
    seed_audits: list[dict[str, Any]] = []

    for source_row in manifest["rows"]:
        seed = int(source_row["seed"])
        payloads: dict[str, dict[str, Any]] = {}
        validations: dict[str, dict[str, Any]] = {}
        metrics_paths: dict[str, Path] = {}
        for variant in VARIANTS:
            run_dir = _run_directory(root, seed, variant)
            metrics_path = run_dir / "metrics.json"
            validation_path = run_dir / "validation.json"
            payloads[variant] = json.loads(metrics_path.read_text(encoding="utf-8"))
            validations[variant] = json.loads(
                validation_path.read_text(encoding="utf-8")
            )
            metrics_paths[variant] = metrics_path

        comparison_report = compare_runs(
            [
                (VARIANT_LABELS[variant], metrics_paths[variant])
                for variant in VARIANTS
            ],
            output_dir=comparison / f"seed_{seed:02d}",
            mode="controller-ablation",
        )
        requested_candidates_exact = all(
            _is_close(
                payload["candidate"]["requested_capture_time_s"],
                source_row["requested_capture_time_s"],
            )
            and _is_close(
                payload["candidate"]["terminal_arm_angle_rad"],
                source_row["terminal_arm_angle_rad"],
            )
            for payload in payloads.values()
        )
        executed_times = [
            float(payload["candidate"]["executed_capture_time_s"])
            for payload in payloads.values()
        ]
        exact_variant_set = {
            payload["controller_variant"] for payload in payloads.values()
        } == set(VARIANTS)
        replay_integrity = {
            variant: _replay_integrity(validations[variant], metrics_paths[variant])
            for variant in VARIANTS
        }
        seed_contract = {
            "requested_candidate_exact": requested_candidates_exact,
            "executed_time_within_one_physics_step": bool(
                max(executed_times) - min(executed_times) <= 0.002 + 1e-12
            ),
            "exact_variant_set": exact_variant_set,
            "same_target": all(
                payload["target"] == payloads["full"]["target"]
                for payload in payloads.values()
            ),
            "same_model_identity": all(
                payload["model_identity"] == payloads["full"]["model_identity"]
                for payload in payloads.values()
            ),
            "implementation_identity_frozen": all(
                payload["implementation_identity"]["composite_sha256"]
                == expected_code_hash
                for payload in payloads.values()
            ),
            "same_dynamic_run_config": all(
                payload["dynamic_run_config"]
                == payloads["full"]["dynamic_run_config"]
                for payload in payloads.values()
            ),
            "same_step_and_tick_counts": len(
                {
                    (int(payload["physics_steps"]), int(payload["task_ticks"]))
                    for payload in payloads.values()
                }
            )
            == 1,
            "all_payloads_finite": all(_all_finite(payload) for payload in payloads.values()),
            "all_replay_integrity_passed": all(replay_integrity.values()),
            "only_declared_weight_changes": _variant_config_contract(
                {variant: payload["effective_config"] for variant, payload in payloads.items()}
            ),
            "per_seed_comparison_contract": all(
                comparison_report["comparison_contract"].values()
            ),
        }
        seed_audits.append(
            {
                "seed": seed,
                "kinematic_rank": int(source_row["kinematic_rank"]),
                "contract": seed_contract,
                "passed": all(seed_contract.values()),
                "comparison_path": str(
                    comparison / f"seed_{seed:02d}" / "precontact_comparison.json"
                ),
            }
        )
        for variant in VARIANTS:
            payload = payloads[variant]
            metrics = payload["metrics"]
            qualified = bool(payload["acceptance"]["passed"] and replay_integrity[variant])
            result_rows.append(
                {
                    "seed": seed,
                    "kinematic_rank": int(source_row["kinematic_rank"]),
                    "variant": variant,
                    "requested_capture_time_s": float(
                        payload["candidate"]["requested_capture_time_s"]
                    ),
                    "executed_capture_time_s": float(
                        payload["candidate"]["executed_capture_time_s"]
                    ),
                    "terminal_arm_angle_rad": float(
                        payload["candidate"]["terminal_arm_angle_rad"]
                    ),
                    "acceptance_passed": bool(payload["acceptance"]["passed"]),
                    "replay_integrity_passed": replay_integrity[variant],
                    "qualified": qualified,
                    "failed_acceptance_checks": [
                        name
                        for name, passed in payload["acceptance"].items()
                        if name != "passed" and passed is False
                    ],
                    "maximum_base_angular_velocity_rad_s": float(
                        metrics["maximum_base_angular_velocity_rad_s"]
                    ),
                    "rms_base_angular_velocity_rad_s": float(
                        metrics["rms_base_angular_velocity_rad_s"]
                    ),
                    "maximum_base_orientation_drift_rad": float(
                        metrics["maximum_base_orientation_drift_rad"]
                    ),
                    "terminal_position_error_m": float(
                        metrics["terminal_position_error_m"]
                    ),
                    "terminal_orientation_error_rad": float(
                        metrics["terminal_orientation_error_rad"]
                    ),
                    "terminal_arm_angle_error_rad": float(
                        metrics["terminal_arm_angle_error_rad"]
                    ),
                    "minimum_clearance_m": float(metrics["minimum_clearance_m"]),
                    "torque_saturation_fraction": float(
                        metrics["torque_saturation_fraction"]
                    ),
                    "metrics_path": str(metrics_paths[variant]),
                    "metrics_sha256": _sha256(metrics_paths[variant]),
                    "validation_path": str(metrics_paths[variant].parent / "validation.json"),
                    "validation_sha256": _sha256(
                        metrics_paths[variant].parent / "validation.json"
                    ),
                    "seed_contract_passed": all(seed_contract.values()),
                }
            )

    paired_summary: dict[str, Any] = {}
    for variant in VARIANTS[1:]:
        pairs: list[dict[str, Any]] = []
        for seed in range(10):
            full = next(
                row
                for row in result_rows
                if row["seed"] == seed and row["variant"] == "full"
            )
            ablated = next(
                row
                for row in result_rows
                if row["seed"] == seed and row["variant"] == variant
            )
            if not (
                full["qualified"]
                and ablated["qualified"]
                and full["seed_contract_passed"]
                and ablated["seed_contract_passed"]
            ):
                continue
            pairs.append({"seed": seed, "full": full, "ablation": ablated})
        metric_summary: dict[str, Any] = {}
        for metric in (
            "maximum_base_angular_velocity_rad_s",
            "rms_base_angular_velocity_rad_s",
            "maximum_base_orientation_drift_rad",
        ):
            full_values = np.asarray([pair["full"][metric] for pair in pairs])
            ablated_values = np.asarray([pair["ablation"][metric] for pair in pairs])
            improvements = np.asarray(
                [
                    100.0 * (ablated - full) / ablated
                    for full, ablated in zip(full_values, ablated_values)
                    if ablated != 0.0
                ]
            )
            p_value = None
            if len(pairs) >= 2 and not np.allclose(full_values, ablated_values):
                p_value = float(
                    wilcoxon(full_values, ablated_values, alternative="two-sided").pvalue
                )
            metric_summary[metric] = {
                "full_mean": None if not len(pairs) else float(np.mean(full_values)),
                "ablation_mean": (
                    None if not len(pairs) else float(np.mean(ablated_values))
                ),
                "full_improvement_mean_percent": (
                    None if improvements.size == 0 else float(np.mean(improvements))
                ),
                "full_improvement_median_percent": (
                    None if improvements.size == 0 else float(np.median(improvements))
                ),
                "full_improvement_min_percent": (
                    None if improvements.size == 0 else float(np.min(improvements))
                ),
                "full_improvement_max_percent": (
                    None if improvements.size == 0 else float(np.max(improvements))
                ),
                "wilcoxon_two_sided_p": p_value,
            }
        paired_summary[variant] = {
            "qualified_pair_count": len(pairs),
            "paired_metrics": metric_summary,
            "paired_seeds": [int(pair["seed"]) for pair in pairs],
        }

    expected_rows = len(manifest["rows"]) * len(VARIANTS)
    selection_payload = {
        "selection_rule": manifest["selection_rule"],
        "effective_config_sha256": manifest["effective_config_sha256"],
        "implementation_composite_sha256": manifest["implementation_identity"][
            "composite_sha256"
        ],
        "rows": manifest["rows"],
    }
    source_evidence_unchanged = all(
        _sha256(Path(row[path_key]).resolve()) == row[hash_key]
        for row in manifest["rows"]
        for path_key, hash_key in (
            ("source_summary_path", "source_summary_sha256"),
            ("source_metrics_path", "source_metrics_sha256"),
            ("source_validation_path", "source_validation_sha256"),
            ("source_trace_path", "source_trace_sha256"),
        )
    )
    suite_contract = {
        "candidate_manifest_selection_hash_valid": manifest["selection_sha256"]
        == _canonical_sha256(selection_payload),
        "source_evidence_unchanged": source_evidence_unchanged,
        "exactly_30_runs": len(result_rows) == expected_rows == 30,
        "ten_seed_contracts_passed": len(seed_audits) == 10
        and all(row["passed"] for row in seed_audits),
        "all_full_runs_qualified": all(
            row["qualified"] for row in result_rows if row["variant"] == "full"
        ),
    }
    report = {
        "material_passport": {
            "origin_skill": "experiment-plan",
            "origin_mode": "run",
            "origin_date": datetime.now(timezone.utc).isoformat(),
            "verification_status": (
                "VERIFIED" if all(suite_contract.values()) else "FAILED_GATE"
            ),
            "version_label": "n073_controller_ablation_summary_v1",
        },
        "experiment_id": "N073",
        "claim_scope": (
            "Effect of deleting one online level-2 objective on trajectories "
            "preplanned and frozen by the full controller"
        ),
        "manifest_path": str(Path(manifest_path).resolve()),
        "manifest_sha256": _sha256(Path(manifest_path).resolve()),
        "output_root": str(root),
        "comparison_root": str(comparison),
        "suite_contract": suite_contract,
        "variant_counts": {
            variant: {
                "run_count": sum(row["variant"] == variant for row in result_rows),
                "accepted_count": sum(
                    row["variant"] == variant and row["acceptance_passed"]
                    for row in result_rows
                ),
                "qualified_count": sum(
                    row["variant"] == variant and row["qualified"]
                    for row in result_rows
                ),
                "failure_counts_by_gate": {
                    gate: sum(
                        row["variant"] == variant
                        and gate in row["failed_acceptance_checks"]
                        for row in result_rows
                    )
                    for gate in sorted(
                        {
                            gate
                            for row in result_rows
                            if row["variant"] == variant
                            for gate in row["failed_acceptance_checks"]
                        }
                    )
                },
            }
            for variant in VARIANTS
        },
        "paired_summary": paired_summary,
        "seed_audits": seed_audits,
        "runs": result_rows,
    }
    summary_path = comparison / "controller_ablation_summary.json"
    summary_path.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    csv_path = comparison / "controller_ablation_runs.csv"
    with csv_path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(result_rows[0]))
        writer.writeheader()
        writer.writerows(result_rows)
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source-root",
        type=Path,
        default=(
            PROJECT_ROOT
            / "output"
            / "fpmfc"
            / "precontact"
            / "n055_smooth_jerk80_rk4_formal_planning45"
            / "joint"
        ),
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=PROJECT_ROOT / "configs" / "fpmfc_paper_planning45_effective.yaml",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=(
            PROJECT_ROOT
            / "output"
            / "fpmfc"
            / "precontact"
            / "n073_controller_ablation_rk4_formal_planning45"
        ),
    )
    parser.add_argument(
        "--comparison-root",
        type=Path,
        default=(
            PROJECT_ROOT
            / "output"
            / "fpmfc"
            / "comparison"
            / "n073_controller_ablation_rk4_formal_planning45"
        ),
    )
    parser.add_argument("--parallel", type=int, default=1)
    parser.add_argument(
        "--manifest-only",
        action="store_true",
        help="Freeze and audit the ten source candidates without running dynamics.",
    )
    args = parser.parse_args()
    manifest, manifest_path = build_candidate_manifest(
        args.source_root,
        config_path=args.config,
        output_root=args.output_root,
    )
    if args.manifest_only:
        print(
            json.dumps(
                {
                    "manifest_path": str(manifest_path),
                    "manifest_sha256": _sha256(manifest_path),
                    "selection_sha256": manifest["selection_sha256"],
                    "row_count": manifest["row_count"],
                },
                indent=2,
                ensure_ascii=False,
            )
        )
        return
    run_controller_ablation(
        manifest,
        config_path=args.config,
        output_root=args.output_root,
        parallelism=args.parallel,
    )
    report = audit_controller_ablation(
        manifest,
        manifest_path=manifest_path,
        output_root=args.output_root,
        comparison_root=args.comparison_root,
    )
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()

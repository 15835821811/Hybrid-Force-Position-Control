"""Create a gate-aware FPMFC pre-contact comparison from validated runs."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from ..model import PROJECT_ROOT


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _parse_method(value: str) -> tuple[str, Path]:
    if "=" not in value:
        raise argparse.ArgumentTypeError("method must use LABEL=METRICS_JSON")
    label, path = value.split("=", 1)
    if not label.strip():
        raise argparse.ArgumentTypeError("method label cannot be empty")
    return label.strip(), Path(path)


def _latex_escape(value: str) -> str:
    return (
        value.replace("\\", r"\textbackslash{}")
        .replace("&", r"\&")
        .replace("%", r"\%")
        .replace("_", r"\_")
    )


def _format(value: float | None, scale: float = 1.0, digits: int = 3) -> str:
    return "--" if value is None else f"{scale * float(value):.{digits}f}"


def _effective_config_sha256(config: dict[str, Any]) -> str:
    encoded = json.dumps(
        config, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _controller_ablation_config_contract(
    configs: list[dict[str, Any] | None], variants: list[str]
) -> bool:
    """Allow only the two declared level-2 weight changes across runs."""

    if any(config is None for config in configs):
        return False
    if not variants or variants[0] != "full":
        return False
    if not any(variant != "full" for variant in variants[1:]):
        return False
    allowed_variants = {"full", "no-shape", "no-base-reaction"}
    if any(variant not in allowed_variants for variant in variants):
        return False

    concrete = [config for config in configs if config is not None]
    reference = concrete[0]
    reference_shape = float(reference["controller"]["shape_weight"])
    reference_base = float(reference["controller"]["base_reaction_weight"])

    def stripped(config: dict[str, Any]) -> dict[str, Any]:
        result = json.loads(json.dumps(config))
        result["controller"].pop("shape_weight", None)
        result["controller"].pop("base_reaction_weight", None)
        return result

    reference_stripped = stripped(reference)
    for config, variant in zip(concrete, variants):
        if stripped(config) != reference_stripped:
            return False
        shape_weight = float(config["controller"]["shape_weight"])
        base_weight = float(config["controller"]["base_reaction_weight"])
        expected = {
            "full": (reference_shape, reference_base),
            "no-shape": (0.0, reference_base),
            "no-base-reaction": (reference_shape, 0.0),
        }[variant]
        if (shape_weight, base_weight) != expected:
            return False
    return True


def compare_runs(
    methods: list[tuple[str, Path]],
    *,
    output_dir: Path,
    mode: str,
) -> dict[str, Any]:
    if len(methods) < 2:
        raise ValueError("at least two methods are required")
    if len({label for label, _path in methods}) != len(methods):
        raise ValueError("method labels must be unique")
    if mode not in {"fixed-time", "reoptimized-time", "controller-ablation"}:
        raise ValueError(
            "comparison mode must be fixed-time, reoptimized-time, "
            "or controller-ablation"
        )

    rows: list[dict[str, Any]] = []
    model_contracts: list[str] = []
    effective_config_hashes: list[str] = []
    effective_configs: list[dict[str, Any] | None] = []
    controller_variants: list[str] = []
    config_snapshot_hash_validity: list[bool] = []
    targets: list[dict[str, Any]] = []
    for label, metrics_value in methods:
        metrics_path = Path(metrics_value).resolve()
        validation_path = metrics_path.parent / "validation.json"
        metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
        validation = json.loads(validation_path.read_text(encoding="utf-8"))
        validation_hash_valid = validation.get("metrics_sha256") == _sha256(metrics_path)
        deterministic_checks = [
            value
            for key, value in validation.get("checks", {}).items()
            if key.startswith("deterministic_") or key == "trace_hash_matches_metrics"
        ]
        replay_integrity = bool(
            validation_hash_valid and deterministic_checks and all(deterministic_checks)
        )
        values = metrics["metrics"]
        candidate = metrics["candidate"]
        accepted = bool(metrics["acceptance"]["passed"])
        row = {
            "method": label,
            "metrics_path": str(metrics_path),
            "metrics_sha256": _sha256(metrics_path),
            "validation_path": str(validation_path),
            "replay_integrity_passed": replay_integrity,
            "acceptance_passed": accepted,
            "qualification": "qualified" if accepted and replay_integrity else "unqualified",
            "controller_variant": str(metrics.get("controller_variant", "full")),
            "requested_capture_time_s": float(candidate["requested_capture_time_s"]),
            "executed_capture_time_s": float(candidate["executed_capture_time_s"]),
            "terminal_arm_angle_rad": float(candidate["terminal_arm_angle_rad"]),
            "dynamic_objective": float(values["dynamic_objective"]),
            "constraint_penalty": float(values["constraint_penalty"]),
            "maximum_base_angular_velocity_rad_s": float(
                values["maximum_base_angular_velocity_rad_s"]
            ),
            "rms_base_angular_velocity_rad_s": float(
                values["rms_base_angular_velocity_rad_s"]
            ),
            "maximum_base_orientation_drift_rad": float(
                values["maximum_base_orientation_drift_rad"]
            ),
            "terminal_position_error_m": float(values["terminal_position_error_m"]),
            "terminal_orientation_error_rad": float(
                values["terminal_orientation_error_rad"]
            ),
            "terminal_arm_angle_error_rad": float(
                values["terminal_arm_angle_error_rad"]
            ),
            "minimum_clearance_m": values["minimum_clearance_m"],
            "torque_saturation_fraction": float(values["torque_saturation_fraction"]),
        }
        rows.append(row)
        model_contracts.append(metrics["model_identity"]["runtime_contract_sha256"])
        effective_config_hashes.append(
            str(metrics.get("effective_config_sha256", f"missing:{metrics_path}"))
        )
        config_snapshot = metrics.get("effective_config")
        effective_configs.append(config_snapshot)
        controller_variants.append(row["controller_variant"])
        config_snapshot_hash_validity.append(
            isinstance(config_snapshot, dict)
            and _effective_config_sha256(config_snapshot)
            == metrics.get("effective_config_sha256")
        )
        targets.append(metrics["target"])

    same_model = len(set(model_contracts)) == 1
    same_effective_config = len(set(effective_config_hashes)) == 1 and not any(
        value.startswith("missing:") for value in effective_config_hashes
    )
    same_target = all(target == targets[0] for target in targets[1:])
    capture_times = np.asarray([row["executed_capture_time_s"] for row in rows])
    fixed_time_valid = bool(
        mode not in {"fixed-time", "controller-ablation"}
        or np.max(capture_times) - np.min(capture_times) <= 2e-3 + 1e-12
    )
    controller_ablation_config_valid = _controller_ablation_config_contract(
        effective_configs, controller_variants
    )
    if mode == "controller-ablation":
        configuration_contract_valid = bool(
            controller_ablation_config_valid and all(config_snapshot_hash_validity)
        )
    else:
        configuration_contract_valid = same_effective_config
    comparison_contract = {
        "same_model_contract": same_model,
        "configuration_contract_valid": configuration_contract_valid,
        "same_target_contract": same_target,
        "fixed_time_within_one_physics_step": fixed_time_valid,
        "all_replay_integrity_passed": all(row["replay_integrity_passed"] for row in rows),
    }
    comparison_diagnostics = {
        "same_effective_config": same_effective_config,
        "all_config_snapshot_hashes_valid": all(config_snapshot_hash_validity),
        "controller_ablation_differences_only": controller_ablation_config_valid,
        "controller_variants": controller_variants,
    }
    optimized = rows[0]
    for row in rows:
        if (
            row is optimized
            or not optimized["acceptance_passed"]
            or not row["acceptance_passed"]
            or not optimized["replay_integrity_passed"]
            or not row["replay_integrity_passed"]
            or not all(comparison_contract.values())
        ):
            row["base_angular_velocity_improvement_vs_baseline_percent"] = None
        else:
            baseline = row["maximum_base_angular_velocity_rad_s"]
            row["base_angular_velocity_improvement_vs_baseline_percent"] = (
                100.0
                * (baseline - optimized["maximum_base_angular_velocity_rad_s"])
                / baseline
            )

    output = Path(output_dir).resolve()
    output.mkdir(parents=True, exist_ok=True)
    report = {
        "material_passport": {
            "origin_skill": "experiment-agent",
            "origin_mode": "run",
            "origin_date": datetime.now(timezone.utc).isoformat(),
            "verification_status": (
                "VERIFIED"
                if all(comparison_contract.values())
                and all(row["acceptance_passed"] for row in rows)
                else "PARTIALLY_VERIFIED"
            ),
            "version_label": "precontact_comparison_v1",
        },
        "mode": mode,
        "comparison_contract": comparison_contract,
        "comparison_diagnostics": comparison_diagnostics,
        "method_order": [row["method"] for row in rows],
        "methods": rows,
        "claim_gate": {
            "all_methods_qualified": all(
                row["qualification"] == "qualified" for row in rows
            ),
            "relative_improvement_claim_allowed": bool(
                all(comparison_contract.values())
                and all(row["qualification"] == "qualified" for row in rows)
            ),
        },
    }
    json_path = output / "precontact_comparison.json"
    json_path.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    fieldnames = list(rows[0])
    csv_path = output / "precontact_comparison.csv"
    with csv_path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    table_path = output / "TABLE_precontact_comparison.tex"
    table_lines = [
        r"\begin{table*}[t]",
        r"\centering",
        r"\caption{Validated pre-contact comparison in the free-floating Flexiv model. A dash denotes an unqualified comparison rather than a zero value.}",
        r"\label{tab:precontact_comparison}",
        r"\begin{tabular}{lcccccccc}",
        r"\toprule",
        r"Method & Feasible & $T_c$ (s) & $\psi_f$ (rad) & $\max\|\omega_b\|$ (rad/s) & RMS $\|\omega_b\|$ & $e_p(T_c)$ (mm) & $d_{\min}$ (mm) & Improvement \\",
        r"\midrule",
    ]
    for row in rows:
        improvement = row["base_angular_velocity_improvement_vs_baseline_percent"]
        table_lines.append(
            " & ".join(
                (
                    _latex_escape(row["method"]),
                    "yes" if row["qualification"] == "qualified" else "no",
                    _format(row["executed_capture_time_s"], digits=3),
                    _format(row["terminal_arm_angle_rad"], digits=3),
                    _format(row["maximum_base_angular_velocity_rad_s"], digits=5),
                    _format(row["rms_base_angular_velocity_rad_s"], digits=5),
                    _format(row["terminal_position_error_m"], scale=1000.0, digits=3),
                    _format(row["minimum_clearance_m"], scale=1000.0, digits=1),
                    "--" if improvement is None else f"{improvement:.2f}\\%",
                )
            )
            + r" \\"
        )
    table_lines.extend((r"\bottomrule", r"\end{tabular}", r"\end{table*}"))
    table_path.write_text("\n".join(table_lines) + "\n", encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--methods", nargs="+", type=_parse_method, required=True)
    parser.add_argument(
        "--mode",
        choices=("fixed-time", "reoptimized-time", "controller-ablation"),
        required=True,
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=PROJECT_ROOT / "output" / "fpmfc" / "comparison",
    )
    args = parser.parse_args()
    report = compare_runs(args.methods, output_dir=args.output_dir, mode=args.mode)
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()

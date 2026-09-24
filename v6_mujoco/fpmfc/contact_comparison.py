"""Aggregate the replay-verified N102--N104 authoritative contact runs."""

from __future__ import annotations

import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from ..model import PROJECT_ROOT


DEFAULT_RUNS = {
    "rigid": PROJECT_ROOT / "output/fpmfc/contact/n102_rigid_authoritative",
    "admittance": PROJECT_ROOT / "output/fpmfc/contact/n103_admittance_authoritative",
    "admittance-no-shape": PROJECT_ROOT
    / "output/fpmfc/contact/n104_admittance_no_shape_authoritative",
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_verified(path: Path, expected_variant: str) -> dict[str, Any]:
    metrics_path = path / "metrics.json"
    validation_path = path / "validation.json"
    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    validation = json.loads(validation_path.read_text(encoding="utf-8"))
    if metrics["variant"] != expected_variant or validation["variant"] != expected_variant:
        raise ValueError(f"variant mismatch under {path}")
    if not validation["passed"]:
        raise ValueError(f"contact validation did not pass under {path}")
    return {
        "output_dir": str(path.resolve().relative_to(PROJECT_ROOT.resolve())),
        "metrics_path": str(metrics_path.resolve().relative_to(PROJECT_ROOT.resolve())),
        "metrics_sha256": _sha256(metrics_path),
        "validation_path": str(
            validation_path.resolve().relative_to(PROJECT_ROOT.resolve())
        ),
        "validation_sha256": _sha256(validation_path),
        "metrics": metrics,
        "validation": validation,
    }


def _lower_is_better_improvement(reference: float, candidate: float) -> float:
    return 100.0 * (float(reference) - float(candidate)) / float(reference)


def build_contact_comparison(
    *, run_dirs: dict[str, Path] = DEFAULT_RUNS, output_path: Path
) -> dict[str, Any]:
    runs = {
        variant: _load_verified(Path(run_dirs[variant]), variant)
        for variant in DEFAULT_RUNS
    }
    identity_keys = (
        "contact_config_sha256",
        "precontact_config_sha256",
        "model_identity",
        "implementation_identity",
    )
    identity_match = all(
        runs["rigid"]["metrics"][key] == runs[variant]["metrics"][key]
        for variant in ("admittance", "admittance-no-shape")
        for key in identity_keys
    )
    source_match = all(
        runs["rigid"]["metrics"]["source_handoff"]["source_trace_sha256"]
        == runs[variant]["metrics"]["source_handoff"]["source_trace_sha256"]
        for variant in ("admittance", "admittance-no-shape")
    )
    if not identity_match or not source_match:
        raise ValueError("authoritative contact runs do not share one frozen contract")

    metric_names = (
        "peak_normal_force_n",
        "normal_force_impulse_ns",
        "steady_force_rmse_n",
        "maximum_penetration_m",
        "maximum_base_angular_speed_rad_s",
        "robot_contact_angular_impulse_about_initial_com_norm_nms",
    )
    compact = {
        variant: {
            **{name: float(item["metrics"]["metrics"][name]) for name in metric_names},
            "contact_loss_events": int(
                item["metrics"]["metrics"]["contact_loss_events"]
            ),
            "maximum_sustained_contact_loss_s": float(
                item["metrics"]["metrics"]["maximum_sustained_contact_loss_s"]
            ),
            "common_passed": bool(item["metrics"]["acceptance"]["common_passed"]),
            "force_tracking_passed": item["metrics"]["acceptance"][
                "steady_force_tracking"
            ],
            "overall_passed": bool(item["metrics"]["acceptance"]["passed"]),
            "replay_verified": bool(item["validation"]["passed"]),
        }
        for variant, item in runs.items()
    }
    rigid = compact["rigid"]
    admittance = compact["admittance"]
    no_shape = compact["admittance-no-shape"]
    comparison = {
        "material_passport": {
            "origin_skill": "experiment-result-to-claim",
            "origin_mode": "run",
            "origin_date": datetime.now(timezone.utc).isoformat(),
            "verification_status": "VERIFIED",
            "version_label": "n102_n104_contact_comparison_v1",
        },
        "experiment_ids": ["N102", "N103", "N104"],
        "scope": "one_second_unilateral_contact_transient_on_custom_free_target",
        "contract_checks": {
            "same_config_model_implementation": identity_match,
            "same_source_handoff": source_match,
            "all_independent_replays_verified": all(
                item["validation"]["passed"] for item in runs.values()
            ),
        },
        "runs": compact,
        "admittance_vs_rigid_lower_is_better_improvement_percent": {
            name: _lower_is_better_improvement(rigid[name], admittance[name])
            for name in metric_names
        },
        "admittance_with_shape_vs_without_shape_lower_is_better_improvement_percent": {
            name: _lower_is_better_improvement(no_shape[name], admittance[name])
            for name in metric_names
        },
        "artifact_provenance": {
            variant: {
                key: value
                for key, value in item.items()
                if key not in {"metrics", "validation"}
            }
            for variant, item in runs.items()
        },
        "gated_interpretation": {
            "rigid_common_safety_and_contact_gate": rigid["common_passed"],
            "admittance_common_safety_and_contact_gate": admittance[
                "common_passed"
            ],
            "admittance_force_tracking_gate": admittance[
                "force_tracking_passed"
            ],
            "no_shape_common_safety_and_contact_gate": no_shape[
                "common_passed"
            ],
            "no_shape_force_tracking_gate": no_shape["force_tracking_passed"],
            "supports_peak_force_reduction_claim": admittance[
                "peak_normal_force_n"
            ]
            < rigid["peak_normal_force_n"],
            "supports_impulse_reduction_claim": admittance[
                "normal_force_impulse_ns"
            ]
            < rigid["normal_force_impulse_ns"],
            "supports_angular_impulse_reduction_claim": admittance[
                "robot_contact_angular_impulse_about_initial_com_norm_nms"
            ]
            < rigid["robot_contact_angular_impulse_about_initial_com_norm_nms"],
        },
    }
    output = Path(output_path).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(comparison, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return comparison


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=PROJECT_ROOT
        / "output/fpmfc/contact/n102_n104_authoritative_comparison.json",
    )
    args = parser.parse_args()
    result = build_contact_comparison(output_path=args.output)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()

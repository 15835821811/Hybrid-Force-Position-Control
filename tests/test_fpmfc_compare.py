"""Claim-gate tests for validated pre-contact comparisons."""

from __future__ import annotations

import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from v6_mujoco.fpmfc.compare_precontact import compare_runs


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_run(
    root: Path,
    name: str,
    *,
    base_speed: float,
    accepted: bool,
    capture_time: float = 10.0,
    controller_variant: str = "full",
    controller_weights: tuple[float, float] = (1.0, 1.0),
    target_mode: str = "same-target",
) -> Path:
    directory = root / name
    directory.mkdir()
    effective_config = {
        "controller": {
            "shape_weight": controller_weights[0],
            "base_reaction_weight": controller_weights[1],
            "unchanged": 7,
        },
        "target": {"mode": target_mode},
    }
    effective_config_hash = hashlib.sha256(
        json.dumps(
            effective_config,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
        ).encode("utf-8")
    ).hexdigest()
    metrics = {
        "acceptance": {"passed": accepted},
        "candidate": {
            "requested_capture_time_s": capture_time,
            "executed_capture_time_s": capture_time,
            "terminal_arm_angle_rad": 0.1,
        },
        "model_identity": {"runtime_contract_sha256": "same-model"},
        "effective_config_sha256": effective_config_hash,
        "effective_config": effective_config,
        "controller_variant": controller_variant,
        "target": {"mode": target_mode},
        "metrics": {
            "dynamic_objective": base_speed**2,
            "constraint_penalty": 0.0 if accepted else 1.0,
            "maximum_base_angular_velocity_rad_s": base_speed,
            "rms_base_angular_velocity_rad_s": 0.5 * base_speed,
            "maximum_base_orientation_drift_rad": 0.01,
            "terminal_position_error_m": 1e-5,
            "terminal_orientation_error_rad": 1e-5,
            "terminal_arm_angle_error_rad": 1e-5,
            "minimum_clearance_m": 0.1,
            "torque_saturation_fraction": 0.0,
        },
    }
    metrics_path = directory / "metrics.json"
    metrics_path.write_text(json.dumps(metrics), encoding="utf-8")
    validation = {
        "metrics_sha256": _sha256(metrics_path),
        "checks": {
            "trace_hash_matches_metrics": True,
            "deterministic_qpos_replay": True,
            "deterministic_qvel_replay": True,
            "deterministic_flange_replay": True,
            "deterministic_clearance_replay": True,
        },
    }
    (directory / "validation.json").write_text(
        json.dumps(validation), encoding="utf-8"
    )
    return metrics_path


class TestFPMFCComparison(unittest.TestCase):
    def test_improvement_requires_qualified_baseline(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            optimized = _write_run(root, "optimized", base_speed=0.02, accepted=True)
            baseline = _write_run(root, "baseline", base_speed=0.04, accepted=True)
            failed = _write_run(root, "failed", base_speed=0.08, accepted=False)
            report = compare_runs(
                [("Optimized", optimized), ("Baseline", baseline), ("Failed", failed)],
                output_dir=root / "comparison",
                mode="fixed-time",
            )
            self.assertFalse(report["claim_gate"]["relative_improvement_claim_allowed"])
            self.assertAlmostEqual(
                report["methods"][1][
                    "base_angular_velocity_improvement_vs_baseline_percent"
                ],
                50.0,
            )
            self.assertIsNone(
                report["methods"][2][
                    "base_angular_velocity_improvement_vs_baseline_percent"
                ]
            )
            self.assertTrue(
                (root / "comparison" / "TABLE_precontact_comparison.tex").is_file()
            )

    def test_controller_ablation_allows_only_declared_weight_change(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            full = _write_run(root, "full", base_speed=0.02, accepted=True)
            no_shape = _write_run(
                root,
                "no_shape",
                base_speed=0.04,
                accepted=True,
                controller_variant="no-shape",
                controller_weights=(0.0, 1.0),
            )
            report = compare_runs(
                [("Full", full), ("No shape", no_shape)],
                output_dir=root / "comparison",
                mode="controller-ablation",
            )
            self.assertTrue(report["comparison_contract"]["configuration_contract_valid"])
            self.assertFalse(report["comparison_diagnostics"]["same_effective_config"])
            self.assertTrue(report["claim_gate"]["relative_improvement_claim_allowed"])

    def test_controller_ablation_rejects_unrelated_config_change(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            full = _write_run(root, "full", base_speed=0.02, accepted=True)
            invalid = _write_run(
                root,
                "invalid",
                base_speed=0.04,
                accepted=True,
                controller_variant="no-shape",
                controller_weights=(0.0, 1.0),
                target_mode="changed-inside-effective-config",
            )
            # Keep the external target contract unchanged so the failure is
            # attributable specifically to the effective-config audit.
            invalid_payload = json.loads(invalid.read_text(encoding="utf-8"))
            invalid_payload["target"] = {"mode": "same-target"}
            invalid.write_text(json.dumps(invalid_payload), encoding="utf-8")
            validation_path = invalid.parent / "validation.json"
            validation = json.loads(validation_path.read_text(encoding="utf-8"))
            validation["metrics_sha256"] = _sha256(invalid)
            validation_path.write_text(json.dumps(validation), encoding="utf-8")
            report = compare_runs(
                [("Full", full), ("Invalid", invalid)],
                output_dir=root / "comparison",
                mode="controller-ablation",
            )
            self.assertFalse(report["comparison_contract"]["configuration_contract_valid"])
            self.assertFalse(report["claim_gate"]["relative_improvement_claim_allowed"])


if __name__ == "__main__":
    unittest.main()

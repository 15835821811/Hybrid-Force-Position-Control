"""Regression tests for multi-seed FPMFC aggregation and resume checks."""

from __future__ import annotations

import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from v6_mujoco.fpmfc.config import apply_runtime_overrides, load_fpmfc_config
from v6_mujoco.fpmfc.pso_suite import _result_matches, aggregate_suite
from v6_mujoco.fpmfc.run_candidates import (
    _effective_config_from_optimizer_payload,
    _effective_config_sha256,
    _planning_and_dynamic_qualified,
)
from v6_mujoco.fpmfc.recheck_candidates import recheck_optimizer_candidates
from v6_mujoco.fpmfc.run_suite_candidates import _dynamic_result_matches
from v6_mujoco.fpmfc.provenance import implementation_identity
from v6_mujoco.model import PROJECT_ROOT


def _payload(seed: int, objective: float, config_hash: str = "abc") -> dict:
    return {
        "config_sha256": config_hash,
        "effective_config_sha256": "effective-abc",
        "fixed_arm_angle_rad": None,
        "implementation_identity": {
            "composite_sha256": "implementation-abc"
        },
        "optimizer": {
            "seed": seed,
            "population": 20,
            "generations_requested": 50,
            "generations_completed": 23,
            "evaluations": 460,
            "best_objective": objective,
            "best_position": [18.0 + seed, 0.5],
            "best_rollout": {
                "feasible": True,
                "maximum_base_angular_velocity_rad_s": 0.02,
                "alignment_angle_rad": 0.3,
                "minimum_clearance_m": 0.08,
            },
        },
    }


class TestFPMFCSuite(unittest.TestCase):
    def test_dynamic_suite_resume_contract_checks_optimizer_hash(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            optimizer = root / "optimizer.json"
            optimizer.write_text(
                json.dumps({"optimizer": {"top_candidates": [{}, {}]}}),
                encoding="utf-8",
            )
            config = root / "config.yaml"
            config.write_text("config", encoding="utf-8")
            summary = root / "candidate_replay_summary.json"
            summary.write_text(
                json.dumps(
                    {
                        "optimizer_json_sha256": hashlib.sha256(
                            optimizer.read_bytes()
                        ).hexdigest(),
                        "config_sha256": hashlib.sha256(config.read_bytes()).hexdigest(),
                        "candidate_count": 2,
                        "implementation_identity": implementation_identity(),
                    }
                ),
                encoding="utf-8",
            )
            self.assertTrue(
                _dynamic_result_matches(
                    summary,
                    optimizer_path=optimizer,
                    config_path=config,
                    top_k=5,
                )
            )
            optimizer.write_text("changed", encoding="utf-8")
            self.assertFalse(
                _dynamic_result_matches(
                    summary,
                    optimizer_path=optimizer,
                    config_path=config,
                    top_k=5,
                )
            )

    def test_suite_rejects_nonpositive_parallelism(self) -> None:
        from v6_mujoco.fpmfc.pso_suite import run_suite

        with self.assertRaises(ValueError):
            run_suite(
                config_path=PROJECT_ROOT / "configs" / "fpmfc_paper.yaml",
                output_root=PROJECT_ROOT / "output" / "unused",
                variants=["joint"],
                seeds=[0],
                population=2,
                generations=1,
                overwrite=False,
                parallelism=0,
            )

    def test_dynamic_ranking_requires_planning_feasibility(self) -> None:
        row = {
            "kinematic_feasible": False,
            "dynamic_acceptance_passed": True,
            "independent_replay_passed": True,
        }
        self.assertFalse(_planning_and_dynamic_qualified(row))
        row["kinematic_feasible"] = True
        self.assertTrue(_planning_and_dynamic_qualified(row))

    def test_clearance_recheck_rejects_wrong_source_config(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            optimizer_path = root / "optimizer.json"
            optimizer_path.write_text(
                json.dumps({"config_sha256": "wrong"}), encoding="utf-8"
            )
            with self.assertRaises(ValueError):
                recheck_optimizer_candidates(
                    optimizer_path,
                    config_path=PROJECT_ROOT / "configs" / "fpmfc_paper.yaml",
                    planning_clearance_m=0.045,
                    output_path=root / "result.json",
                )

    def test_candidate_replay_reconstructs_runtime_overrides(self) -> None:
        config_path = PROJECT_ROOT / "configs" / "fpmfc_paper.yaml"
        effective = apply_runtime_overrides(
            load_fpmfc_config(config_path),
            objective_weights=(2.0, 1.0),
            planning_clearance_m=0.045,
        )
        payload = {
            "runtime_overrides": {
                "objective_weights": [2.0, 1.0],
                "planning_clearance_m": 0.045,
            },
            "effective_config_sha256": _effective_config_sha256(effective),
            "effective_config": effective,
        }
        reconstructed = _effective_config_from_optimizer_payload(payload, config_path)
        self.assertEqual(reconstructed, effective)
        payload["effective_config"] = {
            **effective,
            "experiment_id": "tampered-snapshot",
        }
        with self.assertRaises(ValueError):
            _effective_config_from_optimizer_payload(payload, config_path)
        payload["effective_config"] = effective
        payload["effective_config_sha256"] = "wrong"
        with self.assertRaises(ValueError):
            _effective_config_from_optimizer_payload(payload, config_path)

    def test_resume_contract_and_seed_aggregation(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            variant = root / "joint"
            variant.mkdir()
            for seed, objective in ((0, 3.0), (1, 1.0)):
                path = variant / f"pso_seed_{seed:02d}.json"
                path.write_text(json.dumps(_payload(seed, objective)), encoding="utf-8")
            self.assertTrue(
                _result_matches(
                    variant / "pso_seed_00.json",
                    config_sha256="abc",
                    effective_config_sha256="effective-abc",
                    seed=0,
                    population=20,
                    generations=50,
                    fixed_angle=None,
                )
            )
            self.assertFalse(
                _result_matches(
                    variant / "pso_seed_00.json",
                    config_sha256="different",
                    effective_config_sha256="effective-abc",
                    seed=0,
                    population=20,
                    generations=50,
                    fixed_angle=None,
                )
            )
            report = aggregate_suite(root, ["joint"], [0, 1])
            joint = report["variants"]["joint"]
            self.assertEqual(joint["feasible_seed_count"], 2)
            self.assertEqual(joint["selected_seed"], 1)
            self.assertAlmostEqual(joint["best_objective_mean"], 2.0)
            self.assertTrue((root / "pso_suite_summary.json").is_file())


if __name__ == "__main__":
    unittest.main()

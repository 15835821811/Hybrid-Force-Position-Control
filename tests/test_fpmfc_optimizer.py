"""Determinism and boundary regression tests for the reproduction PSO."""

from __future__ import annotations

import unittest
from types import SimpleNamespace

import numpy as np

from v6_mujoco.fpmfc.optimizer import particle_swarm_optimize


class TestFPMFCOptimizer(unittest.TestCase):
    @staticmethod
    def _evaluate(capture_time_s: float, arm_angle_rad: float) -> SimpleNamespace:
        objective = (capture_time_s - 4.0) ** 2 + 3.0 * (arm_angle_rad + 0.3) ** 2
        return SimpleNamespace(objective=float(objective))

    def _run(self, seed: int):
        return particle_swarm_optimize(
            self._evaluate,
            np.asarray([[1.0, 8.0], [-2.0, 2.0]]),
            seed=seed,
            population=12,
            generations=80,
            inertia=0.7,
            cognitive=1.5,
            social=1.7,
            velocity_fraction=0.15,
            convergence_tolerance=1e-10,
            stagnation_generations=12,
        )

    def test_seed_reproducibility_and_bounds(self) -> None:
        first = self._run(17)
        second = self._run(17)
        np.testing.assert_array_equal(first.best_position, second.best_position)
        np.testing.assert_array_equal(first.history, second.history)
        self.assertGreaterEqual(first.best_position[0], 1.0)
        self.assertLessEqual(first.best_position[0], 8.0)
        self.assertGreaterEqual(first.best_position[1], -2.0)
        self.assertLessEqual(first.best_position[1], 2.0)
        self.assertLess(first.best_objective, 1e-5)
        self.assertTrue(np.all(first.history_feasible))
        self.assertEqual(first.top_positions.shape, (5, 2))
        self.assertTrue(np.all(np.diff(first.top_objectives) >= 0.0))
        self.assertEqual(len(first.top_rollouts), 5)

    def test_invalid_bounds_are_rejected(self) -> None:
        with self.assertRaises(ValueError):
            particle_swarm_optimize(
                self._evaluate,
                np.asarray([[2.0, 1.0], [-2.0, 2.0]]),
                seed=0,
                population=2,
                generations=1,
                inertia=0.9,
                cognitive=1.5,
                social=2.0,
                velocity_fraction=0.1,
                convergence_tolerance=1e-5,
                stagnation_generations=1,
            )

    def test_one_dimension_can_be_fixed_exactly(self) -> None:
        result = particle_swarm_optimize(
            self._evaluate,
            np.asarray([[1.0, 8.0], [0.25, 0.25]]),
            seed=5,
            population=8,
            generations=30,
            inertia=0.7,
            cognitive=1.5,
            social=1.7,
            velocity_fraction=0.15,
            convergence_tolerance=1e-10,
            stagnation_generations=8,
        )
        self.assertEqual(float(result.best_position[1]), 0.25)
        self.assertTrue(np.all(result.top_positions[:, 1] == 0.25))

    def test_feasible_candidates_dominate_lower_objective_infeasible_candidates(self) -> None:
        def evaluate(capture_time_s: float, arm_angle_rad: float) -> SimpleNamespace:
            del arm_angle_rad
            feasible = capture_time_s >= 0.5
            return SimpleNamespace(
                objective=float(100.0 + capture_time_s if feasible else capture_time_s),
                feasible=feasible,
            )

        result = particle_swarm_optimize(
            evaluate,
            np.asarray([[0.0, 1.0], [0.0, 0.0]]),
            seed=0,
            population=20,
            generations=1,
            inertia=0.7,
            cognitive=1.5,
            social=1.7,
            velocity_fraction=0.15,
            convergence_tolerance=1e-10,
            stagnation_generations=2,
        )
        self.assertTrue(result.best_rollout.feasible)
        self.assertTrue(all(rollout.feasible for rollout in result.top_rollouts))
        self.assertTrue(bool(result.history_feasible[-1]))

    def test_objective_order_is_preserved_when_no_candidate_is_feasible(self) -> None:
        def evaluate(capture_time_s: float, arm_angle_rad: float) -> SimpleNamespace:
            return SimpleNamespace(
                objective=float(capture_time_s**2 + arm_angle_rad**2),
                feasible=False,
            )

        result = particle_swarm_optimize(
            evaluate,
            np.asarray([[0.0, 1.0], [0.0, 1.0]]),
            seed=3,
            population=12,
            generations=5,
            inertia=0.7,
            cognitive=1.5,
            social=1.7,
            velocity_fraction=0.15,
            convergence_tolerance=1e-10,
            stagnation_generations=4,
        )
        self.assertFalse(result.best_rollout.feasible)
        self.assertFalse(np.any(result.history_feasible))
        self.assertTrue(np.all(np.diff(result.top_objectives) >= 0.0))

    def test_nonfinite_first_evaluation_cannot_poison_global_best(self) -> None:
        evaluations = 0

        def evaluate(capture_time_s: float, arm_angle_rad: float) -> SimpleNamespace:
            nonlocal evaluations
            evaluations += 1
            objective = (
                float("nan")
                if evaluations == 1
                else float(capture_time_s**2 + arm_angle_rad**2)
            )
            return SimpleNamespace(objective=objective, feasible=True)

        result = particle_swarm_optimize(
            evaluate,
            np.asarray([[0.0, 1.0], [0.0, 1.0]]),
            seed=7,
            population=12,
            generations=5,
            inertia=0.7,
            cognitive=1.5,
            social=1.7,
            velocity_fraction=0.15,
            convergence_tolerance=1e-10,
            stagnation_generations=4,
        )
        self.assertTrue(np.isfinite(result.best_objective))
        self.assertTrue(np.all(np.isfinite(result.top_objectives)))


if __name__ == "__main__":
    unittest.main()

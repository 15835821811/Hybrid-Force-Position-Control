"""Regression tests for the Simscape-to-MuJoCo migration contract."""

from __future__ import annotations

import unittest

import mujoco
import numpy as np

from v6_mujoco.collision import build_collision_pairs
from v6_mujoco.hierarchical_qp import HierarchicalVelocityQP, QPConfig
from v6_mujoco.irregular_waypoints import build_target
from v6_mujoco.model import SIMSCAPE_HOME_FLANGE_POSITION_M, default_model_spec, geom_id, site_id


class MigrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.spec = default_model_spec()
        cls.model = cls.spec.compile_model()
        cls.data = mujoco.MjData(cls.model)
        cls.spec.reset_home(cls.model, cls.data)
        cls.pairs = build_collision_pairs(cls.model)
        cls.qp = HierarchicalVelocityQP(cls.spec, cls.model, cls.pairs)

    def test_active_distance_barrier_solves_within_task_period(self) -> None:
        obstacle = geom_id(self.model, "workspace_obstacle_0")
        original = self.model.geom_pos[obstacle].copy()
        self.model.geom_pos[obstacle] = [0.62, 0.04, 0.48]
        mujoco.mj_forward(self.model, self.data)
        flange = site_id(self.model, "flange_site")
        result = self.qp.solve(
            self.data,
            target_position=self.data.site_xpos[flange].copy(),
            target_velocity=np.zeros(3),
            target_rotation=self.data.site_xmat[flange].reshape(3, 3).copy(),
        )
        self.model.geom_pos[obstacle] = original
        mujoco.mj_forward(self.model, self.data)
        self.assertTrue(result.success)
        self.assertGreater(result.active_clearance_constraints, 0)
        self.assertLess(result.full_latency_s, self.spec.task_period_s)

    def test_model_contract_and_home_fk(self) -> None:
        self.assertEqual((self.model.nq, self.model.nv, self.model.nu), (14, 13, 7))
        self.assertAlmostEqual(float(self.model.opt.timestep), 0.002)
        self.assertLess(np.linalg.norm(self.model.opt.gravity), 1e-12)
        self.assertEqual(
            int(self.model.opt.integrator), int(mujoco.mjtIntegrator.mjINT_RK4)
        )
        flange = site_id(self.model, "flange_site")
        self.assertLess(np.linalg.norm(self.data.site_xpos[flange] - SIMSCAPE_HOME_FLANGE_POSITION_M), 0.001)

    def test_reaction_map_zeroes_base_momentum(self) -> None:
        mapping, residual = self.qp.reaction_velocity_map(self.data)
        self.assertEqual(mapping.shape, (13, 7))
        self.assertLess(residual, 1e-10)

    def test_irregular_reference_is_seed_deterministic(self) -> None:
        flange = site_id(self.model, "flange_site")
        first = build_target(self.data.site_xpos[flange], self.data.site_xmat[flange], 7)
        second = build_target(self.data.site_xpos[flange], self.data.site_xmat[flange], 7)
        np.testing.assert_array_equal(first.waypoint_points_w, second.waypoint_points_w)
        self.assertEqual(first.metadata["waypoint_count"], 7)

    def test_signed_distance_gradient_matches_finite_difference(self) -> None:
        probe = HierarchicalVelocityQP(
            self.spec,
            self.model,
            self.pairs,
            QPConfig(clearance_activation_m=0.5, clearance_query_max_m=1.0),
        )
        mapping, _ = probe.reaction_velocity_map(self.data)
        pair = min(self.pairs, key=lambda item: probe.clearance_gradient_for_pair(self.data, item, mapping)[0])
        distance, gradient = probe.clearance_gradient_for_pair(self.data, pair, mapping)
        self.assertGreater(np.linalg.norm(gradient), 1e-5)
        direction = np.asarray([0.2, -0.3, 0.4, -0.1, 0.5, -0.2, 0.1])
        direction /= np.linalg.norm(direction)
        epsilon = 1e-6
        original = self.data.qpos.copy()
        velocity = mapping @ direction
        mujoco.mj_integratePos(self.model, self.data.qpos, velocity, epsilon)
        mujoco.mj_forward(self.model, self.data)
        shifted, _ = probe.clearance_gradient_for_pair(self.data, pair, mapping)
        self.data.qpos[:] = original
        mujoco.mj_forward(self.model, self.data)
        self.assertAlmostEqual((shifted - distance) / epsilon, float(gradient @ direction), delta=2e-3)

    def test_online_package_has_no_learning_or_oracle_dependency(self) -> None:
        import pathlib
        root = pathlib.Path(__file__).resolve().parents[1] / "v6_mujoco"
        text = "\n".join(path.read_text(encoding="utf-8") for path in root.glob("*.py"))
        self.assertNotIn("import torch", text)
        self.assertNotIn("diffusion", text.lower())
        self.assertNotIn("oracle", text.lower())


if __name__ == "__main__":
    unittest.main()

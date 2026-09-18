"""Regression checks for smooth saturation and third-order joint limits."""

from __future__ import annotations

import unittest

import mujoco
import numpy as np

from v6_mujoco.collision import build_collision_pairs
from v6_mujoco.hierarchical_qp import (
    HierarchicalVelocityQP,
    QPConfig,
    jerk_limited_safe_speed,
    jerk_limited_stopping_distance,
    smooth_saturate_norm,
)
from v6_mujoco.model import default_model_spec


class SmoothSaturationTests(unittest.TestCase):
    def test_radial_saturation_is_bounded_and_direction_preserving(self) -> None:
        limit = 0.45
        vector = np.asarray([0.8, -0.4, 0.2])
        saturated = smooth_saturate_norm(vector, limit, 0.90)
        self.assertLess(float(np.linalg.norm(saturated)), limit)
        np.testing.assert_allclose(
            saturated / np.linalg.norm(saturated),
            vector / np.linalg.norm(vector),
            atol=1e-14,
        )

    def test_radial_saturation_has_c2_shoulder(self) -> None:
        limit = 0.45
        ratio = 0.90
        shoulder = ratio * limit
        step = 1e-6

        def magnitude(radius: float) -> float:
            return float(
                np.linalg.norm(
                    smooth_saturate_norm(np.asarray([radius, 0.0, 0.0]), limit, ratio)
                )
            )

        self.assertEqual(magnitude(0.5 * shoulder), 0.5 * shoulder)
        left_slope = (magnitude(shoulder) - magnitude(shoulder - step)) / step
        right_slope = (magnitude(shoulder + step) - magnitude(shoulder)) / step
        curvature = (
            magnitude(shoulder + step)
            - 2.0 * magnitude(shoulder)
            + magnitude(shoulder - step)
        ) / step**2
        self.assertAlmostEqual(left_slope, 1.0, places=9)
        self.assertAlmostEqual(right_slope, 1.0, places=7)
        self.assertAlmostEqual(curvature, 0.0, delta=2e-3)


class JerkBoundTests(unittest.TestCase):
    def setUp(self) -> None:
        self.spec = default_model_spec()
        self.model = self.spec.compile_model()
        self.data = mujoco.MjData(self.model)
        self.spec.reset_home(self.model, self.data)
        self.config = QPConfig(joint_jerk_limit_rad_s3=40.0)
        self.qp = HierarchicalVelocityQP(
            self.spec,
            self.model,
            build_collision_pairs(self.model),
            self.config,
        )

    def test_velocity_bounds_limit_discrete_jerk(self) -> None:
        q = np.asarray(self.data.qpos[self.qp.qpos_ids])
        previous_acceleration = self.qp.previous_acceleration.copy()
        for _ in range(8):
            _lower, upper = self.qp._bounds(q)
            previous_velocity = self.qp.previous_velocity.copy()
            candidate = upper.copy()
            acceleration = (
                candidate - previous_velocity
            ) / self.config.task_period_s
            jerk = (
                acceleration - previous_acceleration
            ) / self.config.task_period_s
            self.assertLessEqual(
                float(np.max(np.abs(jerk))),
                self.config.joint_jerk_limit_rad_s3 + 1e-10,
            )
            self.assertLessEqual(
                float(np.max(np.abs(acceleration))),
                float(np.max(self.spec.acceleration_limits_rad_s2)) + 1e-10,
            )
            self.qp._commit_velocity(candidate)
            previous_acceleration = acceleration

    def test_speed_bound_brakes_acceleration_before_hard_limit(self) -> None:
        q = np.asarray(self.data.qpos[self.qp.qpos_ids]).copy()
        maximum_speed = self.config.velocity_limit_scale * self.spec.velocity_limits_rad_s
        for _ in range(300):
            lower, upper = self.qp._bounds(q)
            self.assertFalse(np.any(self.qp.last_bound_conflicts))
            previous_velocity = self.qp.previous_velocity.copy()
            previous_acceleration = self.qp.previous_acceleration.copy()
            candidate = upper.copy()
            acceleration = (
                candidate - previous_velocity
            ) / self.config.task_period_s
            jerk = (
                acceleration - previous_acceleration
            ) / self.config.task_period_s
            self.assertLessEqual(
                float(np.max(np.abs(jerk))),
                self.config.joint_jerk_limit_rad_s3 + 1e-8,
            )
            self.assertTrue(np.all(candidate <= maximum_speed + 1e-10))
            self.qp._commit_velocity(candidate)

    def test_reset_clears_velocity_and_acceleration_history(self) -> None:
        self.qp.previous_velocity[:] = 1.0
        self.qp.previous_acceleration[:] = 2.0
        self.qp.reset()
        np.testing.assert_array_equal(self.qp.previous_velocity, np.zeros(7))
        np.testing.assert_array_equal(self.qp.previous_acceleration, np.zeros(7))

    def test_jerk_aware_speed_cap_has_enough_stopping_distance(self) -> None:
        distance = 0.40
        acceleration = -1.0
        safe_speed = jerk_limited_safe_speed(
            distance,
            acceleration,
            speed_limit=3.0,
            acceleration_limit=4.0,
            jerk_limit=40.0,
        )
        stopping_distance = jerk_limited_stopping_distance(
            safe_speed,
            acceleration,
            acceleration_limit=4.0,
            jerk_limit=40.0,
        )
        self.assertAlmostEqual(stopping_distance, distance, places=8)

    def test_jerk_and_position_bounds_remain_compatible_during_braking(self) -> None:
        q = np.asarray(self.data.qpos[self.qp.qpos_ids]).copy()
        joint = 6
        upper_position = self.qp.joint_upper[joint] - self.config.joint_position_margin_rad
        for _ in range(300):
            lower, upper = self.qp._bounds(q)
            self.assertTrue(np.all(lower <= upper + 1e-12))
            previous_velocity = self.qp.previous_velocity.copy()
            previous_acceleration = self.qp.previous_acceleration.copy()
            candidate = np.clip(previous_velocity, lower, upper)
            candidate[joint] = upper[joint]
            acceleration = (
                candidate - previous_velocity
            ) / self.config.task_period_s
            jerk = (
                acceleration - previous_acceleration
            ) / self.config.task_period_s
            self.assertLessEqual(
                abs(float(jerk[joint])),
                self.config.joint_jerk_limit_rad_s3 + 1e-8,
            )
            q[joint] += (
                0.5
                * (previous_velocity[joint] + candidate[joint])
                * self.config.task_period_s
            )
            self.assertLessEqual(q[joint], upper_position + 1e-8)
            self.qp._commit_velocity(candidate)


if __name__ == "__main__":
    unittest.main()

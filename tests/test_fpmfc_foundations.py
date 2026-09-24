"""Foundational checks for the adaptive Flexiv FPMFC reproduction."""

from __future__ import annotations

import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from v6_mujoco.fpmfc.config import (
    apply_runtime_overrides,
    load_fpmfc_config,
    validate_fpmfc_config,
)
from v6_mujoco.fpmfc.dynamics import FreeFloatingKinematics, rotation_delta_world
from v6_mujoco.fpmfc.rollout import PrecontactRolloutEvaluator
from v6_mujoco.fpmfc.run_capture import run_precontact_candidate
from v6_mujoco.fpmfc.shape import ArmShapeKinematics, unwrap_angle
from v6_mujoco.fpmfc.target import sync_mujoco_target, target_from_config
from v6_mujoco.fpmfc.trajectory import PoseShapeTrajectory, quintic_time_scaling
from v6_mujoco.model import body_id, default_model_spec, site_id


class FPMFCFoundationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = load_fpmfc_config()
        self.spec = default_model_spec()
        self.model = self.spec.compile_model()
        self.data = mujoco.MjData(self.model)
        self.spec.reset_home(self.model, self.data)
        self.kinematics = FreeFloatingKinematics(self.spec, self.model)
        shape_config = self.config["shape"]
        self.shape = ArmShapeKinematics(
            self.spec,
            self.model,
            self.data,
            shoulder_joint=shape_config["shoulder_joint"],
            elbow_joint=shape_config["elbow_joint"],
            wrist_joint=shape_config["wrist_joint"],
            singularity_margin=float(shape_config["singularity_margin"]),
        )

    def test_configuration_provenance_and_model_contract_match(self) -> None:
        model_config = self.config["model"]
        np.testing.assert_allclose(
            np.deg2rad(model_config["home_joint_position_deg"]),
            self.spec.home_joint_position,
            atol=1e-12,
        )
        base = body_id(self.model, "base_link_0")
        self.assertAlmostEqual(float(self.model.body_mass[base]), model_config["base_mass_kg"])
        np.testing.assert_allclose(
            self.model.body_inertia[base],
            model_config["base_diagonal_inertia_kg_m2"],
            rtol=0.0,
            atol=1e-9,
        )
        self.assertEqual(model_config["integrator"], "RK4")
        self.assertEqual(self.spec.integrator, "RK4")
        self.assertEqual(
            int(self.model.opt.integrator), int(mujoco.mjtIntegrator.mjINT_RK4)
        )

    def test_rk4_replay_preserves_momentum_for_former_failure_branch(self) -> None:
        effective = apply_runtime_overrides(
            self.config, planning_clearance_m=0.045
        )
        with TemporaryDirectory() as directory:
            result = run_precontact_candidate(
                effective,
                capture_time_s=12.559110859785916,
                terminal_arm_angle_rad=2.8186331817200623,
                output_dir=Path(directory),
            )
        self.assertTrue(result["acceptance"]["momentum_delta"])
        self.assertTrue(result["acceptance"]["passed"])
        self.assertLess(result["metrics"]["maximum_momentum_delta"], 1e-5)

    def test_planning_clearance_override_preserves_acceptance_threshold(self) -> None:
        effective = apply_runtime_overrides(
            self.config,
            objective_weights=(2.0, 1.0),
            planning_clearance_m=0.045,
        )
        self.assertAlmostEqual(effective["controller"]["minimum_clearance_m"], 0.045)
        self.assertAlmostEqual(effective["acceptance"]["minimum_clearance_m"], 0.04)
        self.assertEqual(effective["optimization"]["objective_weights"], [2.0, 1.0])
        self.assertEqual(
            effective["provenance"]["controller.minimum_clearance_m"], "calibrated"
        )
        self.assertNotIn("minimum_clearance_m", self.config["acceptance"])
        self.assertAlmostEqual(self.config["controller"]["minimum_clearance_m"], 0.04)
        with self.assertRaises(ValueError):
            apply_runtime_overrides(self.config, planning_clearance_m=0.035)

    def test_planning_shape_margin_is_stricter_than_dynamic_acceptance(self) -> None:
        planning_limit = float(
            self.config["optimization"]["planning_terminal_arm_angle_error_rad"]
        )
        acceptance_limit = float(
            self.config["acceptance"]["terminal_arm_angle_error_rad"]
        )
        self.assertAlmostEqual(np.rad2deg(planning_limit), 0.75)
        self.assertLess(planning_limit, acceptance_limit)

        invalid = apply_runtime_overrides(self.config)
        invalid["optimization"]["planning_terminal_arm_angle_error_rad"] = (
            acceptance_limit + 1e-6
        )
        with self.assertRaises(ValueError):
            validate_fpmfc_config(invalid)

        effective = apply_runtime_overrides(self.config, planning_clearance_m=0.045)
        rollout = PrecontactRolloutEvaluator(effective).evaluate(8.0, -1.5)
        self.assertGreater(rollout.terminal_arm_angle_error_rad, planning_limit)
        self.assertLess(rollout.terminal_arm_angle_error_rad, acceptance_limit)
        self.assertFalse(rollout.feasible)
        self.assertGreater(rollout.constraint_penalty, 0.0)

    def test_reaction_map_zeroes_base_momentum_over_random_states(self) -> None:
        rng = np.random.default_rng(20260915)
        qpos_ids, _ = self.spec.joint_addresses(self.model)
        joint_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in self.spec.joint_names
        ]
        lower = np.asarray([self.model.jnt_range[index, 0] for index in joint_ids])
        upper = np.asarray([self.model.jnt_range[index, 1] for index in joint_ids])
        original = self.data.qpos.copy()
        try:
            for _ in range(100):
                self.data.qpos[qpos_ids] = rng.uniform(lower + 0.05, upper - 0.05)
                reaction = self.kinematics.reaction_map(self.data)
                self.assertLess(reaction.momentum_residual_norm, 1e-8)
        finally:
            self.data.qpos[:] = original
            mujoco.mj_forward(self.model, self.data)

    def test_generalized_site_jacobian_matches_free_floating_difference(self) -> None:
        jacobian_position, jacobian_rotation, reaction = (
            self.kinematics.generalized_site_jacobians(self.data)
        )
        direction = np.asarray([0.3, -0.2, 0.4, -0.1, 0.25, -0.35, 0.15])
        direction /= np.linalg.norm(direction)
        site = site_id(self.model, "flange_site")
        initial_position = np.asarray(self.data.site_xpos[site]).copy()
        initial_rotation = np.asarray(self.data.site_xmat[site]).reshape(3, 3).copy()
        original = self.data.qpos.copy()
        step = 1e-7
        try:
            mujoco.mj_integratePos(
                self.model,
                self.data.qpos,
                reaction.full_velocity_from_joint_velocity @ direction,
                step,
            )
            mujoco.mj_forward(self.model, self.data)
            shifted_position = np.asarray(self.data.site_xpos[site]).copy()
            shifted_rotation = np.asarray(self.data.site_xmat[site]).reshape(3, 3).copy()
        finally:
            self.data.qpos[:] = original
            mujoco.mj_forward(self.model, self.data)
        numerical_linear = (shifted_position - initial_position) / step
        numerical_angular = rotation_delta_world(initial_rotation, shifted_rotation) / step
        np.testing.assert_allclose(jacobian_position @ direction, numerical_linear, atol=2e-6)
        np.testing.assert_allclose(jacobian_rotation @ direction, numerical_angular, atol=2e-6)

    def test_arm_shape_is_base_invariant_and_jacobian_converges(self) -> None:
        initial = self.shape.sample(self.data)
        self.assertFalse(initial.singular)
        original = self.data.qpos.copy()
        base_qpos_slice, _ = self.spec.base_slices(self.model)
        try:
            self.data.qpos[base_qpos_slice.start : base_qpos_slice.start + 3] += [0.2, -0.3, 0.1]
            self.data.qpos[base_qpos_slice.start + 3 : base_qpos_slice.stop] = Rotation.from_euler(
                "xyz", [0.4, -0.2, 0.3]
            ).as_quat()[[3, 0, 1, 2]]
            mujoco.mj_forward(self.model, self.data)
            moved = self.shape.sample(self.data)
            self.assertAlmostEqual(initial.angle_rad, moved.angle_rad, delta=1e-10)
        finally:
            self.data.qpos[:] = original
            mujoco.mj_forward(self.model, self.data)
        coarse = self.shape.jacobian(self.data, 1e-5)
        fine = self.shape.jacobian(self.data, 1e-6)
        np.testing.assert_allclose(coarse, fine, rtol=2e-5, atol=2e-5)
        self.assertGreater(float(np.linalg.norm(fine)), 1e-3)
        np.testing.assert_array_equal(self.data.qpos, original)

    def test_arm_shape_changes_along_generalized_task_nullspace(self) -> None:
        jacobian_position, jacobian_rotation, _ = self.kinematics.generalized_site_jacobians(
            self.data
        )
        generalized = np.vstack((jacobian_position, jacobian_rotation))
        _u, _s, vh = np.linalg.svd(generalized, full_matrices=True)
        null_direction = vh[-1]
        shape_jacobian = self.shape.jacobian(self.data)
        self.assertLess(np.linalg.norm(generalized @ null_direction), 1e-10)
        self.assertGreater(abs(float(shape_jacobian @ null_direction)), 1e-3)

    def test_prescribed_target_twist_matches_position_difference(self) -> None:
        target = target_from_config(self.config)
        self.assertEqual(target.geometry, "cube")
        self.assertAlmostEqual(target.side_length_m, 0.30)
        initial = target.sample(0.0)
        np.testing.assert_allclose(
            initial.center_position_world_m,
            [0.95, 0.14, 0.95],
            atol=1e-12,
        )
        np.testing.assert_allclose(
            initial.grasp_position_world_m,
            [0.80, 0.14, 0.95],
            atol=1e-12,
        )
        target_outward_normal = (
            initial.grasp_position_world_m - initial.center_position_world_m
        ) / (0.5 * target.side_length_m)
        np.testing.assert_allclose(
            initial.grasp_rotation_world[:, 2],
            -target_outward_normal,
            atol=1e-12,
        )
        flange_position = np.asarray(
            self.data.site_xpos[
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "flange_site")
            ]
        )
        self.assertAlmostEqual(
            float(np.linalg.norm(initial.grasp_position_world_m - flange_position)),
            0.533,
            places=3,
        )
        mocap_id = sync_mujoco_target(self.model, self.data, target, initial)
        mujoco.mj_forward(self.model, self.data)
        target_geom = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "tumbling_target_geom"
        )
        np.testing.assert_allclose(self.model.geom_size[target_geom], [0.15, 0.15, 0.15])
        np.testing.assert_allclose(self.data.mocap_pos[mocap_id], [0.95, 0.14, 0.95])
        later = target.sample(2.0)
        sync_mujoco_target(self.model, self.data, target, later)
        mujoco.mj_forward(self.model, self.data)
        target_body = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "tumbling_target"
        )
        np.testing.assert_allclose(
            self.data.xpos[target_body], later.center_position_world_m, atol=1e-12
        )
        np.testing.assert_allclose(
            self.data.xmat[target_body].reshape(3, 3),
            later.center_rotation_world,
            atol=1e-12,
        )
        time_s = 7.3
        step = 1e-6
        sample = target.sample(time_s)
        plus = target.sample(time_s + step)
        minus = target.sample(time_s - step)
        numerical_velocity = (
            plus.grasp_position_world_m - minus.grasp_position_world_m
        ) / (2.0 * step)
        numerical_angular = rotation_delta_world(
            minus.grasp_rotation_world, plus.grasp_rotation_world
        ) / (2.0 * step)
        np.testing.assert_allclose(sample.grasp_linear_velocity_world_m_s, numerical_velocity, atol=1e-9)
        np.testing.assert_allclose(sample.grasp_angular_velocity_world_rad_s, numerical_angular, atol=1e-9)
        self.assertAlmostEqual(
            np.rad2deg(np.linalg.norm(sample.angular_velocity_world_rad_s)), 5.0, places=12
        )

    def test_quintic_pose_shape_trajectory_has_c2_boundaries(self) -> None:
        for time_s, expected in ((-1.0, 0.0), (0.0, 0.0), (4.0, 1.0), (5.0, 1.0)):
            scalar, velocity, acceleration = quintic_time_scaling(time_s, 4.0)
            self.assertEqual(scalar, expected)
            self.assertEqual(velocity, 0.0)
            self.assertEqual(acceleration, 0.0)
        initial_rotation = Rotation.from_euler("xyz", [0.1, -0.2, 0.3]).as_matrix()
        final_rotation = Rotation.from_euler("xyz", [-0.4, 0.25, 0.7]).as_matrix()
        trajectory = PoseShapeTrajectory(
            initial_position_world_m=np.asarray([0.4, -0.1, 0.5]),
            final_position_world_m=np.asarray([0.8, 0.2, 0.7]),
            initial_rotation_world=initial_rotation,
            final_rotation_world=final_rotation,
            initial_arm_angle_rad=-0.2,
            final_arm_angle_rad=0.6,
            duration_s=4.0,
        )
        start = trajectory.sample(0.0)
        finish = trajectory.sample(4.0)
        np.testing.assert_allclose(start.position_world_m, trajectory.initial_position_world_m)
        np.testing.assert_allclose(finish.position_world_m, trajectory.final_position_world_m)
        np.testing.assert_allclose(start.rotation_world, initial_rotation, atol=1e-12)
        np.testing.assert_allclose(finish.rotation_world, final_rotation, atol=1e-12)
        self.assertEqual(np.linalg.norm(start.linear_velocity_world_m_s), 0.0)
        self.assertEqual(np.linalg.norm(finish.angular_velocity_world_rad_s), 0.0)
        self.assertEqual(start.arm_angle_velocity_rad_s, 0.0)
        self.assertEqual(finish.arm_angle_acceleration_rad_s2, 0.0)

    def test_unwrap_angle_removes_branch_cut_jump(self) -> None:
        previous = np.deg2rad(179.0)
        current_wrapped = np.deg2rad(-179.0)
        self.assertAlmostEqual(np.rad2deg(unwrap_angle(previous, current_wrapped)), 181.0)


if __name__ == "__main__":
    unittest.main()

"""Hierarchy and timing checks for the adaptive FPMFC controller."""

from __future__ import annotations

import unittest

import mujoco
import numpy as np

from v6_mujoco.collision import build_collision_pairs
from v6_mujoco.fpmfc.config import load_fpmfc_config
from v6_mujoco.fpmfc.controller import FPMFCControllerConfig, FPMFCHQP
from v6_mujoco.fpmfc.shape import ArmShapeKinematics
from v6_mujoco.model import default_model_spec, site_id


class FPMFCControllerTests(unittest.TestCase):
    def setUp(self) -> None:
        config = load_fpmfc_config()
        self.spec = default_model_spec()
        self.model = self.spec.compile_model()
        self.data = mujoco.MjData(self.model)
        self.spec.reset_home(self.model, self.data)
        shape_config = config["shape"]
        self.shape = ArmShapeKinematics(
            self.spec,
            self.model,
            self.data,
            shoulder_joint=shape_config["shoulder_joint"],
            elbow_joint=shape_config["elbow_joint"],
            wrist_joint=shape_config["wrist_joint"],
            singularity_margin=float(shape_config["singularity_margin"]),
        )
        controller_config = config["controller"]
        self.control = FPMFCControllerConfig(
            position_gain=float(controller_config["position_gain"]),
            orientation_gain=float(controller_config["orientation_gain"]),
            shape_gain=float(controller_config["shape_gain"]),
            shape_weight=float(controller_config["shape_weight"]),
            base_reaction_weight=float(controller_config["base_reaction_weight"]),
            level1_position_tolerance_m_s=float(
                controller_config["level1_position_tolerance_m_s"]
            ),
            level1_angular_tolerance_rad_s=float(
                controller_config["level1_angular_tolerance_rad_s"]
            ),
        )
        self.controller = FPMFCHQP(
            self.spec,
            self.model,
            build_collision_pairs(self.model),
            self.shape,
            controller_config=self.control,
        )
        flange = site_id(self.model, "flange_site")
        self.position = np.asarray(self.data.site_xpos[flange]).copy()
        self.rotation = np.asarray(self.data.site_xmat[flange]).reshape(3, 3).copy()
        self.arm_angle = self.shape.sample(self.data).angle_rad

    def solve(self, target_arm_angle_rad: float):
        return self.controller.solve_fpmfc(
            self.data,
            target_position=self.position,
            target_velocity=np.zeros(3),
            target_rotation=self.rotation,
            target_angular_velocity=np.zeros(3),
            target_arm_angle_rad=target_arm_angle_rad,
            target_arm_angle_velocity_rad_s=0.0,
        )

    def test_zero_error_command_is_stationary(self) -> None:
        result = self.solve(self.arm_angle)
        self.assertTrue(result.success, (result.primary_status, result.secondary_status))
        self.assertLess(np.linalg.norm(result.joint_velocity), 1e-7)
        self.assertLess(result.full_latency_s, self.spec.task_period_s)
        self.assertLess(result.momentum_map_residual_norm, 1e-10)

    def test_shape_motion_respects_locked_end_effector_task(self) -> None:
        result = self.solve(self.arm_angle + 0.05)
        self.assertTrue(result.success, (result.primary_status, result.secondary_status))
        self.assertGreater(np.linalg.norm(result.joint_velocity), 1e-4)
        shape_jacobian = self.shape.jacobian(self.data)
        self.assertGreater(float(shape_jacobian @ result.joint_velocity), 0.0)
        self.assertLessEqual(
            result.hierarchy_linear_degradation_m_s,
            np.sqrt(3.0) * self.control.level1_position_tolerance_m_s + 2e-5,
        )
        self.assertLessEqual(
            result.hierarchy_angular_degradation_rad_s,
            np.sqrt(3.0) * self.control.level1_angular_tolerance_rad_s + 2e-5,
        )
        self.assertLess(result.full_latency_s, self.spec.task_period_s)


if __name__ == "__main__":
    unittest.main()

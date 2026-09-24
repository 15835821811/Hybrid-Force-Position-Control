"""Contact-wrench sign/reference and normal-admittance tests."""

from __future__ import annotations

import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

import mujoco
import numpy as np

from v6_mujoco.fpmfc.contact import (
    NormalAdmittance,
    aggregate_contact_wrench,
    maximum_interface_penetration,
)
from v6_mujoco.fpmfc.contact_config import (
    load_contact_config,
    normal_admittance_config,
)
from v6_mujoco.fpmfc.contact_model import default_contact_model_spec
from v6_mujoco.fpmfc.config import load_fpmfc_config
from v6_mujoco.fpmfc.run_contact import (
    _quintic_ramp,
    initialize_contact_state,
)
from v6_mujoco.fpmfc.target import target_from_config
from v6_mujoco.model import default_model_spec, site_id


CONTACT_XML = """
<mujoco>
  <option timestep="0.001" gravity="0 0 -9.81"/>
  <worldbody>
    <geom name="floor" type="plane" size="1 1 0.1"/>
    <body name="box" pos="0 0 0.049">
      <freejoint/>
      <geom name="box_geom" type="box" size="0.05 0.05 0.05" mass="2"/>
    </body>
  </worldbody>
</mujoco>
"""


class ContactWrenchTests(unittest.TestCase):
    def setUp(self) -> None:
        self.model = mujoco.MjModel.from_xml_string(CONTACT_XML)
        self.data = mujoco.MjData(self.model)
        for _ in range(2000):
            mujoco.mj_step(self.model, self.data)
        self.floor = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "floor"
        )
        self.box = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "box_geom"
        )

    def test_wrench_sign_matches_force_on_selected_geom(self) -> None:
        box_wrench = aggregate_contact_wrench(
            self.model,
            self.data,
            selected_geom_ids=[self.box],
            counterpart_geom_ids=[self.floor],
            reference_point_world_m=[0.0, 0.0, float(self.data.qpos[2])],
        )
        floor_wrench = aggregate_contact_wrench(
            self.model,
            self.data,
            selected_geom_ids=[self.floor],
            counterpart_geom_ids=[self.box],
            reference_point_world_m=[0.0, 0.0, float(self.data.qpos[2])],
        )
        self.assertEqual(box_wrench.contact_count, 4)
        self.assertAlmostEqual(box_wrench.force_world_n[2], 2.0 * 9.81, places=8)
        np.testing.assert_allclose(
            floor_wrench.force_world_n, -box_wrench.force_world_n, atol=1e-10
        )

    def test_reference_point_shift_obeys_wrench_transport(self) -> None:
        reference_a = np.array([0.0, 0.0, float(self.data.qpos[2])])
        reference_b = reference_a + np.array([0.1, -0.2, 0.05])
        wrench_a = aggregate_contact_wrench(
            self.model,
            self.data,
            selected_geom_ids=[self.box],
            counterpart_geom_ids=[self.floor],
            reference_point_world_m=reference_a,
        )
        wrench_b = aggregate_contact_wrench(
            self.model,
            self.data,
            selected_geom_ids=[self.box],
            counterpart_geom_ids=[self.floor],
            reference_point_world_m=reference_b,
        )
        expected_b = wrench_a.torque_world_nm + np.cross(
            reference_a - reference_b, wrench_a.force_world_n
        )
        np.testing.assert_allclose(wrench_b.torque_world_nm, expected_b, atol=1e-10)

    def test_active_contact_penetration_is_nonnegative(self) -> None:
        penetration = maximum_interface_penetration(
            self.model,
            self.data,
            selected_geom_ids=[self.box],
            counterpart_geom_ids=[self.floor],
        )
        self.assertGreaterEqual(penetration, 0.0)
        self.assertLess(penetration, 1e-3)


class ContactModelContractTests(unittest.TestCase):
    def test_physical_target_and_interface_match_contact_config(self) -> None:
        spec = default_contact_model_spec()
        model = spec.compile_model()
        data = mujoco.MjData(model)
        spec.reset_home(model, data)
        target_qpos, target_dof = spec.target_slices(model)
        base_qpos, base_dof = spec.base_slices(model)
        self.assertEqual((model.nq, model.nv, model.nu), (21, 19, 7))
        self.assertEqual(target_qpos.stop - target_qpos.start, 7)
        self.assertEqual(target_dof.stop - target_dof.start, 6)
        self.assertEqual(base_qpos.stop - base_qpos.start, 7)
        self.assertEqual(base_dof.stop - base_dof.start, 6)
        np.testing.assert_allclose(data.qpos[target_qpos][:3], [0.95, 0.14, 0.95])
        pad = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "gripper_contact_pad")
        target = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "target_contact_plate")
        np.testing.assert_allclose(model.geom_margin[[pad, target]], 0.0)
        self.assertAlmostEqual(float(model.geom_pos[pad, 2]), -0.0052)
        self.assertEqual(data.ncon, 0)

    def test_named_state_transfer_preserves_target_grasp_pose_and_twist(self) -> None:
        precontact_config = load_fpmfc_config(
            Path("configs/fpmfc_paper_planning45_effective.yaml")
        )
        target_sample = target_from_config(precontact_config).sample(8.0)
        source_spec = default_model_spec()
        source_model = source_spec.compile_model()
        source_data = mujoco.MjData(source_model)
        source_spec.reset_home(source_model, source_data)
        with TemporaryDirectory() as temporary_directory:
            trace_path = Path(temporary_directory) / "trace.npz"
            np.savez_compressed(
                trace_path,
                qpos=np.asarray(source_data.qpos)[None, :],
                qvel=np.asarray(source_data.qvel)[None, :],
                reference_q=source_spec.home_joint_position[None, :],
                reference_dq=np.zeros((1, 7)),
                reference_ddq=np.zeros((1, 7)),
                task_joint_velocity_rad_s=np.zeros((1, 7)),
                task_joint_acceleration_rad_s2=np.zeros((1, 7)),
                capture_time_s=np.asarray(8.0),
                terminal_arm_angle_rad=np.asarray(0.25),
                target_center_position=target_sample.center_position_world_m[None, :],
                target_center_rotation=target_sample.center_rotation_world[None, :, :],
                target_grasp_position=target_sample.grasp_position_world_m[None, :],
                target_grasp_rotation=target_sample.grasp_rotation_world[None, :, :],
            )
            contact_spec = default_contact_model_spec()
            contact_model = contact_spec.compile_model()
            contact_data = mujoco.MjData(contact_model)
            contact_spec.reset_home(contact_model, contact_data)
            result = initialize_contact_state(
                source_trace_path=trace_path,
                precontact_config=precontact_config,
                contact_spec=contact_spec,
                contact_model=contact_model,
                contact_data=contact_data,
            )
        self.assertLess(max(result["target_transfer_errors"].values()), 2e-10)
        grasp = site_id(contact_model, "target_grasp_site")
        np.testing.assert_allclose(
            contact_data.site_xpos[grasp],
            target_sample.grasp_position_world_m,
            atol=2e-12,
        )


class ContactReferenceTests(unittest.TestCase):
    def test_quintic_ramp_has_zero_endpoint_velocity(self) -> None:
        self.assertEqual(_quintic_ramp(0.0, 0.5), (0.0, 0.0))
        self.assertEqual(_quintic_ramp(0.5, 0.5), (1.0, 0.0))
        value, rate = _quintic_ramp(0.25, 0.5)
        self.assertAlmostEqual(value, 0.5)
        self.assertGreater(rate, 0.0)


class NormalAdmittanceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.contact_config = load_contact_config()
        self.control = NormalAdmittance(
            normal_admittance_config(self.contact_config, timestep_s=0.002)
        )

    def test_zero_force_error_is_stationary(self) -> None:
        for _ in range(100):
            state = self.control.step(desired_force_n=3.0, measured_force_n=3.0)
        self.assertEqual(state.offset_m, 0.0)
        self.assertEqual(state.velocity_m_s, 0.0)

    def test_critical_step_converges_without_overshoot(self) -> None:
        offsets = []
        times = []
        for index in range(2500):
            offsets.append(
                self.control.step(desired_force_n=3.0, measured_force_n=0.0).offset_m
            )
            times.append((index + 1) * self.control.config.timestep_s)
        desired_force = float(self.contact_config["force_control"]["desired_normal_force_n"])
        steady_offset = desired_force / self.control.config.stiffness_n_m
        self.assertAlmostEqual(offsets[-1], steady_offset, places=7)
        self.assertLessEqual(max(offsets), steady_offset + 1e-9)
        self.assertTrue(all(np.diff(offsets) >= -1e-12))
        omega_n = np.sqrt(
            self.control.config.stiffness_n_m
            / self.control.config.virtual_mass_kg
        )
        times_array = np.asarray(times)
        exact = steady_offset * (
            1.0 - (1.0 + omega_n * times_array) * np.exp(-omega_n * times_array)
        )
        self.assertLess(np.max(np.abs(np.asarray(offsets) - exact)), 3.0e-5)

    def test_limits_and_reset_are_deterministic(self) -> None:
        for _ in range(1000):
            state = self.control.step(desired_force_n=100.0, measured_force_n=0.0)
        self.assertLessEqual(abs(state.offset_m), 0.003)
        self.assertLessEqual(abs(state.velocity_m_s), 0.02)
        reset = self.control.reset()
        self.assertEqual(reset.offset_m, 0.0)
        self.assertEqual(reset.velocity_m_s, 0.0)


if __name__ == "__main__":
    unittest.main()

"""N110 diagnostics: coordinate conventions and archived contact evidence."""

from __future__ import annotations

import numpy as np
import pytest

from v6_mujoco.model import PROJECT_ROOT, geom_id
from v6_mujoco.fpmfc.audit_contact import (
    CONTACTS, SOURCE, _admittance, _handoff, _npz_is_real, _phase,
    _sampling_and_velocity, point_twist, velocity_map_residual,
)
from v6_mujoco.fpmfc.contact_config import load_contact_config
from v6_mujoco.fpmfc.contact_model import default_contact_model_spec


def test_world_point_twist_includes_rotational_offset():
    result = point_twist(np.zeros(3), np.array([0.0, 2.0, 0.0]),
                         np.array([1.0, 0.0, 0.0]), np.array([0.0, 0.0, 3.0]))
    np.testing.assert_allclose(result, [-5.0, 0.0, 0.0], atol=1e-15)


def test_velocity_residual_is_exactly_explained_by_base_bias():
    linear = np.array([[1.0, 0.0, 2.0], [0.0, 1.0, -1.0], [0.0, 0.0, 1.0]])
    angular = np.array([[0.0, 1.0, 2.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    base_ids, arm_ids = np.array([0, 1]), np.array([2])
    mapping = np.array([[0.2], [-0.3]])
    joint = 0.7
    bias = np.array([0.04, -0.02])
    qvel = np.r_[mapping[:, 0]*joint + bias, joint]
    residual = velocity_map_residual(linear, angular, qvel, base_ids, arm_ids, mapping)
    np.testing.assert_allclose(residual["base_bias"], bias)
    np.testing.assert_allclose(residual["linear_error"], linear[:, base_ids] @ bias)
    np.testing.assert_allclose(residual["angular_error"], angular[:, base_ids] @ bias)
    np.testing.assert_allclose(residual["linear_repaired_error"], 0.0, atol=1e-15)
    np.testing.assert_allclose(residual["angular_repaired_error"], 0.0, atol=1e-15)


def test_contact_phase_classification_includes_loss_and_recontact():
    counts = np.array([0, 1, 1, 0, 1])
    assert [_phase(counts, i) for i in range(len(counts))] == [
        "pre_contact", "first_contact", "sustained_contact", "contact_loss", "recontact"
    ]


def _archives_available():
    return all(_npz_is_real(PROJECT_ROOT / path / "trace.npz") for path in (SOURCE, *CONTACTS.values()))


@pytest.mark.skipif(not _archives_available(), reason="four N110 LFS archives are not hydrated")
def test_handoff_named_joint_mapping_and_measured_relative_twist(tmp_path):
    spec = default_contact_model_spec()
    model = spec.compile_model()
    model.geom_pos[geom_id(model, "workspace_obstacle_0")] = [10.0, 10.0, 10.0]
    result = _handoff(tmp_path, model, spec)
    assert result["source_to_contact_base_qpos_max_abs"] == 0.0
    assert result["source_to_contact_base_qvel_max_abs"] == 0.0
    assert result["source_to_contact_joint_qpos_max_abs"] == 0.0
    assert result["source_to_contact_joint_qvel_max_abs"] == 0.0
    assert result["flange_to_grasp_linear_speed_m_s"] > 0.01
    assert result["relative_angular_speed_rad_s"] > 0.08
    assert result["controller_history_vs_reference_velocity_rad_s"] == 0.0


@pytest.mark.skipif(not _archives_available(), reason="four N110 LFS archives are not hydrated")
def test_poststep_copy_forward_exposes_sampling_and_velocity_bias(tmp_path):
    spec = default_contact_model_spec()
    model = spec.compile_model()
    model.geom_pos[geom_id(model, "workspace_obstacle_0")] = [10.0, 10.0, 10.0]
    sampling, velocity = _sampling_and_velocity(tmp_path, model, spec)
    for name in CONTACTS:
        sample = sampling["variants"][name]
        mapped = velocity["variants"][name]
        assert sample["replay_qpos_max_abs"] < 1e-8
        assert sample["replay_qvel_max_abs"] < 1e-8
        assert sample["status"] == "FAIL"
        assert sample["fresh_poststep_vs_logged"]["force_n"]["max"] > 0.01
        assert sample["fresh_recomputed_metrics_descriptive_only"]["contact_count_disagree_steps"] == 0
        assert mapped["phases"]["pre_contact"]["linear_m_s"]["max"] < 1e-8
        assert mapped["phases"]["sustained_contact"]["linear_m_s"]["max"] > 0.004
        assert mapped["overall"]["linear_repaired_m_s"]["max"] < 1e-12


@pytest.mark.skipif(not _archives_available(), reason="four N110 LFS archives are not hydrated")
def test_admittance_reset_reconstruction_and_full_window_rmse(tmp_path):
    spec = default_contact_model_spec()
    model = spec.compile_model()
    result = _admittance(tmp_path, load_contact_config(), spec.timestep_s, model)["variants"]
    assert len(result["admittance"]["admittance_reset_times_s"]) == 3
    assert len(result["admittance-no-shape"]["admittance_reset_times_s"]) == 6
    assert result["rigid"]["admittance_reset_times_s"] == []
    assert result["admittance"]["maximum_reconstructed_state_error"] == 0.0
    assert result["admittance-no-shape"]["maximum_reconstructed_state_error"] == 0.0
    assert all(row["detection_reconstruction_mismatch_steps"] == [] for row in result.values())
    assert result["admittance"]["steady_window_samples"] == 100
    assert result["admittance"]["steady_force_rmse_all_samples_n"] == pytest.approx(1.2383079734442908)
    assert result["admittance"]["robot_contact_angular_impulse_proxy_nms"] != result["admittance"]["base_link_angular_momentum_change_norm_kg_m2_s"]

"""The paired policy is derived from one archived candidate before contact."""

import json

import numpy as np

from v6_mujoco.fpmfc.handoff_contract import (
    DEFAULT_HANDOFF_CONFIG, load_handoff_config, load_manifest,
    prepare_manifest, source_candidate,
)
from v6_mujoco.fpmfc.run_handoff_precontact import _trajectory


def test_candidate_is_read_exactly_from_hydrated_N073_archive():
    config = load_handoff_config(DEFAULT_HANDOFF_CONFIG)
    candidate = source_candidate(config)
    assert candidate["capture_time_s"] == 8.0
    assert candidate["terminal_arm_angle_rad"] == -1.492877399060744
    assert len(candidate["initial_qpos"]) == 14
    assert len(candidate["initial_qvel"]) == 13


def test_manifest_freezes_same_initial_pose_terminal_pose_time_and_shape(tmp_path):
    manifest = prepare_manifest(output_root=tmp_path)
    loaded = load_manifest(tmp_path/"pairing_manifest.json")
    assert loaded == manifest
    C0, C1 = _trajectory(manifest, "C0"), _trajectory(manifest, "C1")
    for t in (0.0, 8.0):
        a, b = C0.sample(t), C1.sample(t)
        np.testing.assert_allclose(a.position_world_m, b.position_world_m, atol=1e-14)
        np.testing.assert_allclose(a.rotation_world, b.rotation_world, atol=1e-14)
        assert abs(a.arm_angle_rad-b.arm_angle_rad) < 1e-14
    np.testing.assert_array_equal(C0.sample(0.0).linear_velocity_world_m_s, np.zeros(3))
    np.testing.assert_array_equal(C1.sample(0.0).angular_velocity_world_rad_s, np.zeros(3))
    assert np.linalg.norm(C1.sample(8.0).linear_velocity_world_m_s) > 0.01
    assert np.linalg.norm(C1.sample(8.0).angular_velocity_world_rad_s) > 0.08
    assert manifest["terminal_reference_contract"]["target_absolute_time"] == "archived_capture_time"
    assert manifest["terminal_reference_contract"]["contact_local_time"] == "resets_to_zero_at_handoff"
    assert manifest["fixed_contact_mapping_mode"] == "homogeneous"
    assert manifest["run_budget"]["contact_dynamics_trajectories_if_qualified"] == 6
    assert json.loads((tmp_path/"pairing_manifest.json").read_text(encoding="utf-8")) == manifest

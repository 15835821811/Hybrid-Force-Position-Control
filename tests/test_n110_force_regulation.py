"""N110D discrete outer-loop and archived C1 interface regressions."""

import json

import numpy as np

from v6_mujoco.fpmfc.contact import NormalAdmittance
from v6_mujoco.fpmfc.contact_config import load_contact_config, normal_admittance_config
from v6_mujoco.fpmfc.force_regulation_analysis import analyze_trace
from v6_mujoco.fpmfc.force_regulation_contract import CONFIG_PATH, load_config, outer_config
from v6_mujoco.fpmfc.force_regulation_outer import ExperimentalNormalAdmittance, ForceRegulationConfig, replay_outer_loop
from v6_mujoco.fpmfc.handoff_contract import DEFAULT_OUTPUT_ROOT, load_manifest, repo_path, sha256


def _config(*, stiffness=0.0, reset=False):
    return ForceRegulationConfig(1.0, 100.0, stiffness, 0.002, 0.003, 0.020, 0.20, 0.10, reset)


def test_explicit_damping_and_frozen_d00_discrete_equivalence():
    config = load_config(CONFIG_PATH)
    assert outer_config(config, "D00").damping_n_s_m == 100.0
    assert outer_config(config, "D10").damping_n_s_m == 100.0
    original = NormalAdmittance(normal_admittance_config(load_contact_config(), timestep_s=0.002))
    trial = ExperimentalNormalAdmittance(outer_config(config, "D00"))
    with np.load(repo_path(config["baseline_dir"])/"trace.npz", allow_pickle=False) as trace:
        detected = False
        for desired, measured, x, v in zip(trace["command_desired_normal_force_n"],
                                           trace["feedback_measured_normal_force_n"],
                                           trace["command_normal_offset_m"],
                                           trace["command_normal_offset_velocity_m_s"], strict=True):
            prior = detected
            detected = measured > 0.10 if detected else measured >= 0.20
            if prior and not detected:
                original.reset()
                trial.reset()
            old = original.step(desired_force_n=float(desired), measured_force_n=float(measured))
            new = trial.step(desired_force_n=float(desired), measured_force_n=float(measured))
            assert old == new
            assert new.offset_m == x and new.velocity_m_s == v
    assert len(trial.rows) == 500


def test_zero_stiffness_zero_input_velocity_decays_but_offset_remains():
    outer = ExperimentalNormalAdmittance(_config())
    outer.state = type(outer.state)(offset_m=0.001, velocity_m_s=0.010)
    state = outer.step(desired_force_n=0.0, measured_force_n=0.0)
    assert state.velocity_m_s == 0.008
    assert state.offset_m == 0.001016
    for _ in range(250):
        state = outer.step(desired_force_n=0.0, measured_force_n=0.0)
    assert abs(state.velocity_m_s) < 1e-20
    assert state.offset_m > 0.001


def test_no_contact_persistent_error_stays_bounded_even_without_reset():
    outer = ExperimentalNormalAdmittance(_config(stiffness=0.0, reset=False))
    for _ in range(5000):
        outer.step(desired_force_n=3.0, measured_force_n=0.0)
    assert outer.state.offset_m == 0.003
    assert outer.state.velocity_m_s == 0.0
    assert any(row["offset_clipped"] for row in outer.rows)
    assert all(abs(row["x_after_step_m"]) <= 0.003 for row in outer.rows)


def test_reset_policy_is_only_hard_clear_at_loss_event():
    a = ExperimentalNormalAdmittance(_config(stiffness=2500.0, reset=True))
    b = ExperimentalNormalAdmittance(_config(stiffness=2500.0, reset=False))
    for measured in (0.3, 0.3, 0.3):
        assert a.step(desired_force_n=3.0, measured_force_n=measured) == b.step(desired_force_n=3.0, measured_force_n=measured)
    a.reset()
    a.step(desired_force_n=3.0, measured_force_n=0.0)
    b.step(desired_force_n=3.0, measured_force_n=0.0)
    assert a.rows[-1]["loss_event"] and b.rows[-1]["loss_event"]
    assert a.rows[-1]["reset_applied"] and not b.rows[-1]["reset_applied"]
    assert a.rows[-1]["x_after_reset_m"] == a.rows[-1]["v_after_reset_m_s"] == 0.0
    assert b.rows[-1]["x_after_reset_m"] != 0.0
    assert a.rows[-1]["measured_force_n"] == b.rows[-1]["measured_force_n"] == 0.0


def test_force_sign_clock_reference_point_and_full_window_decomposition():
    positive = ExperimentalNormalAdmittance(_config())
    assert positive.step(desired_force_n=3.0, measured_force_n=0.0).offset_m > 0
    negative = ExperimentalNormalAdmittance(_config())
    assert negative.step(desired_force_n=0.0, measured_force_n=3.0).offset_m < 0
    config = load_config(CONFIG_PATH)
    path = repo_path(config["baseline_dir"])/"trace.npz"
    analysis, arrays = analyze_trace(path, outer_config(config, "D00"))
    metrics = json.loads((path.parent/"metrics.json").read_text(encoding="utf-8"))
    assert analysis["full_window"]["force_rmse_n"] == metrics["metrics"]["steady_force_rmse_n"]
    assert analysis["full_window"]["bias_variance_identity_error_n2"] < 1e-12
    assert abs(analysis["full_window"]["mse_contribution_geometric_contact_n2"]
               + analysis["full_window"]["mse_contribution_geometric_loss_n2"]
               - analysis["full_window"]["force_error_mse_n2"]) < 1e-12
    assert arrays["feedback_time_s"][0] == 0.0
    assert arrays["feedback_time_s"][-1] == 0.998
    with np.load(path, allow_pickle=False) as trace:
        expected = trace["target_grasp_position"] + trace["contact_normal_world"] * (
            0.0002 + trace["command_normal_offset_m"][:, None])
        np.testing.assert_allclose(trace["desired_position"], expected, atol=1e-12, rtol=0)
        np.testing.assert_allclose(arrays["measured_force_n"], trace["feedback_measured_normal_force_n"], atol=0, rtol=0)
    assert analysis["full_stage"]["desired_force_impulse_ns"] > analysis["full_stage"]["actual_force_impulse_ns"]


def test_historical_sources_and_C1_identity_are_unchanged():
    parent = load_manifest(DEFAULT_OUTPUT_ROOT/"pairing_manifest.json")
    config = load_config(CONFIG_PATH)
    assert sha256(repo_path(config["source_trace"])) == json.loads(
        (repo_path(config["baseline_dir"])/"metrics.json").read_text(encoding="utf-8"))[
            "source_handoff"]["source_trace_sha256"]
    assert sha256(repo_path(config["contact_config"])) == parent["contact_config_sha256"]
    assert parent["n110c_implementation_identity"]["composite_sha256"]

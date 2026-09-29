"""Audit the finite N110C pair using saved physical trajectories only."""

import numpy as np
import pytest

from v6_mujoco.fpmfc.handoff_contract import DEFAULT_OUTPUT_ROOT
from v6_mujoco.fpmfc.summarize_handoff import INITIAL_FIELDS, VARIANTS, summarize


@pytest.fixture(scope="module")
def paired():
    if not (DEFAULT_OUTPUT_ROOT / "precontact/C0/trace.npz").is_file():
        pytest.skip("N110C LFS result traces are not hydrated")
    return summarize()


def test_source_history_phase_and_same_condition_initial_states(paired):
    assert paired["engineering_audit_passed"]
    assert paired["run_count"] == {"precontact_dynamics": 2, "contact_dynamics": 6}
    assert paired["between_condition_initial_state_max_abs_differences"] == {
        "initial_qpos": 0.0, "initial_qvel": 0.0,
    }
    for condition in ("C0", "C1"):
        source = paired["conditions"][condition]
        assert all(source["source_controller_history_transfer_checks"].values())
        assert all(source["contact_preflight"]["state_transfer_checks"].values())
        assert source["contact_preflight"]["target_absolute_time_s"] == 8.0
        assert source["contact_preflight"]["contact_local_time_s"] == 0.0
        for variant in VARIANTS:
            assert all(paired["same_condition_initial_state_max_abs_differences"][condition][variant][key] == 0
                       for key in INITIAL_FIELDS)
            assert all(paired["contact"][condition][variant]["trace_checks"].values())


def test_live_twist_uses_relative_gate_and_force_window_keeps_failures(paired):
    assert paired["checks"]["C1_relative_gate_passes"]
    assert paired["checks"]["C1_exceeds_old_absolute_zero_speed_gate"]
    c0 = paired["conditions"]["C0"]["metrics"]
    c1 = paired["conditions"]["C1"]["metrics"]
    assert c1["terminal_tool_face_relative_linear_speed_m_s"] < 0.001
    assert c1["terminal_tool_face_relative_linear_speed_m_s"] < c0["terminal_tool_face_relative_linear_speed_m_s"]
    assert c1["terminal_relative_angular_speed_rad_s"] < np.deg2rad(0.2)
    for condition in ("C0", "C1"):
        for variant in ("admittance", "admittance-no-shape"):
            result = paired["contact"][condition][variant]
            assert result["independent_torque_replay_passed"]
            assert result["trace_checks"]["force_rmse_all_last_100_samples"]
            assert not result["acceptance"]["steady_force_tracking"]

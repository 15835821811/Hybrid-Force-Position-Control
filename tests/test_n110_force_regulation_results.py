"""Saved N110D 2x2 contrast and verification gates."""

import pytest

from v6_mujoco.fpmfc.force_regulation_contract import OUTPUT_ROOT
from v6_mujoco.fpmfc.summarize_force_regulation import summarize


@pytest.fixture(scope="module")
def paired():
    if not (OUTPUT_ROOT/"D11/trace.npz").is_file():
        pytest.skip("N110D LFS traces are not hydrated")
    return summarize()


def test_same_C1_initial_state_and_three_replayed_runs(paired):
    assert paired["engineering_audit_passed"]
    assert paired["new_closed_loop_runs"] == 3
    assert paired["optional_no_shape_confirmation_runs"] == 0
    assert all(v == 0.0 for fields in paired["initial_state_max_abs_differences_from_D00"].values()
               for v in fields.values())
    assert all(paired["cells"][name]["independent_replay_passed"] for name in ("D00", "D01", "D10", "D11"))


def test_force_gate_failure_and_decomposition_are_preserved(paired):
    assert paired["qualifying_cells"] == []
    assert not paired["force_tracking_gate_passed"]
    assert all(paired["cells"][name]["acceptance"]["common_passed"] for name in ("D00", "D01", "D10", "D11"))
    assert all(not paired["cells"][name]["acceptance"]["steady_force_tracking"] for name in ("D00", "D01", "D10", "D11"))
    d = paired["cells"]["D00"]["window_decomposition"]
    assert d["true_zero_force_samples"] == 5
    assert d["below_detection_samples"] == 9
    assert d["geometric_loss_samples"] == 0
    assert d["zero_force_rmse_lower_bound_n"] > 0.3

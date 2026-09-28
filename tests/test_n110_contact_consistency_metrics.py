"""Force metrics retain contact-loss samples and the archived quadrature."""

import numpy as np

from v6_mujoco.fpmfc.contact_consistency_metrics import endpoint_force_metrics, hysteresis_loss


def test_fixed_window_includes_zero_force_and_uses_right_endpoints():
    force = np.array([0.0, 1.0, 0.0, 2.0])
    desired = np.ones(4)
    metrics = endpoint_force_metrics(force, desired, 0.002, 3)
    assert metrics["peak_normal_force_n"] == 2.0
    assert metrics["normal_force_impulse_ns"] == 0.006
    assert metrics["steady_force_rmse_n"] == np.sqrt(2.0/3.0)


def test_loss_hysteresis_keeps_entire_interval():
    detected, events, maximum_loss = hysteresis_loss(
        np.array([0.0, 0.6, 0.2, 0.0, 0.7]), 0.5, 0.3, 0.002,
    )
    np.testing.assert_array_equal(detected, [False, True, False, False, True])
    assert events == 1
    assert maximum_loss == 0.004

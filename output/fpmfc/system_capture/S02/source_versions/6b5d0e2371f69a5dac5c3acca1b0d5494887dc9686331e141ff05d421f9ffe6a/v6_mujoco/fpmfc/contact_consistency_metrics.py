"""Fixed N110 force-window, contact-loss, and endpoint-quadrature rules."""

from __future__ import annotations

import numpy as np


def hysteresis_loss(force: np.ndarray, detection: float, release: float, dt: float) -> tuple[np.ndarray, int, float]:
    """Classify every sample, including loss intervals, without selection."""
    detected = False
    ever = False
    losses = 0
    active_loss = 0
    longest = 0
    sequence = []
    for value in np.asarray(force, dtype=float):
        prior = detected
        detected = value > release if detected else value >= detection
        ever = ever or detected
        if not detected and ever:
            losses += int(prior)
            active_loss += 1
            longest = max(longest, active_loss)
        else:
            active_loss = 0
        sequence.append(detected)
    return np.asarray(sequence, dtype=bool), losses, longest * dt


def endpoint_force_metrics(force: np.ndarray, desired: np.ndarray, dt: float, window: int) -> dict[str, float]:
    """Right-endpoint rectangle impulse and all-sample fixed-window RMSE."""
    actual = np.asarray(force, dtype=float)
    reference = np.asarray(desired, dtype=float)
    if actual.shape != reference.shape or actual.ndim != 1 or not len(actual):
        raise ValueError("force and desired must have the same nonempty 1-D shape")
    if not 1 <= window <= len(actual):
        raise ValueError("window must be within the sample count")
    return {
        "peak_normal_force_n": float(np.max(actual)),
        "normal_force_impulse_ns": float(np.sum(actual) * dt),
        "steady_force_rmse_n": float(np.sqrt(np.mean((actual[-window:] - reference[-window:]) ** 2))),
    }

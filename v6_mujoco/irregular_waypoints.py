"""Seeded irregular-waypoint reference adapted to the Flexiv workspace."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


CIRCLE_CENTER_W = np.asarray([0.650, -0.110, 0.550], dtype=np.float64)
CIRCLE_RADIUS_M = 0.150
WAYPOINT_COUNT = 7
TRANSITION_DURATION_S = 4.5
PATH_DURATION_S = 21.0


def _minimum_jerk(value: float) -> tuple[float, float]:
    tau = float(np.clip(value, 0.0, 1.0))
    return (
        tau**3 * (10.0 - 15.0 * tau + 6.0 * tau**2),
        30.0 * tau**2 * (1.0 - tau) ** 2,
    )


def _polyline_sample(points: np.ndarray, fractions: np.ndarray) -> np.ndarray:
    vectors = np.diff(points, axis=0)
    lengths = np.linalg.norm(vectors, axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    total = float(cumulative[-1])
    result = np.zeros((len(fractions), 3), dtype=np.float64)
    for index, distance in enumerate(np.asarray(fractions) * total):
        segment = int(np.clip(np.searchsorted(cumulative, distance, side="right") - 1, 0, len(lengths) - 1))
        alpha = (distance - cumulative[segment]) / max(float(lengths[segment]), 1e-12)
        result[index] = points[segment] + alpha * vectors[segment]
    return result


def _template() -> tuple[np.ndarray, np.ndarray]:
    fractions = np.asarray([0.0, 0.08, 0.17, 0.30, 0.43, 0.58, 0.72, 0.87, 1.0])
    offsets = np.asarray(
        [
            [0.000, 0.000, 0.000], [0.010, -0.008, 0.006],
            [-0.014, 0.010, 0.018], [0.016, 0.018, -0.004],
            [-0.018, -0.012, 0.016], [0.014, 0.014, 0.008],
            [-0.012, 0.020, -0.006], [0.008, -0.016, 0.012],
            [-0.006, 0.012, -0.004],
        ],
        dtype=np.float64,
    )
    return fractions, offsets


def build_waypoints(rotation_world: np.ndarray, seed: int) -> tuple[np.ndarray, dict[str, Any]]:
    """Preserve V6 draw order/ranges while translating its circle into reach."""

    rotation = np.asarray(rotation_world, dtype=np.float64).reshape(3, 3)
    fractions, offsets = _template()
    start = rotation[:, 2]
    counterclockwise = -rotation[:, 1]
    anchors = []
    for fraction, offset in zip(fractions, offsets):
        theta = 2.0 * np.pi * fraction
        point = CIRCLE_CENTER_W + CIRCLE_RADIUS_M * (
            np.cos(theta) * start + np.sin(theta) * counterclockwise
        )
        anchors.append(point + rotation @ offset)
    nominal = _polyline_sample(np.asarray(anchors), np.linspace(0.0, 1.0, WAYPOINT_COUNT))

    rng = np.random.default_rng(int(seed))
    radial_scales = 1.0 + rng.uniform(-0.10, 0.40, size=WAYPOINT_COUNT)
    front_back_scales = rng.uniform(0.6, 2.0, size=WAYPOINT_COUNT)
    front_back_offsets = rng.uniform(-0.030, 0.020, size=WAYPOINT_COUNT)
    local = (rotation.T @ (nominal - CIRCLE_CENTER_W).T).T
    local[:, 0] = local[:, 0] * front_back_scales + front_back_offsets
    local[:, 1:] *= radial_scales[:, None]
    points = CIRCLE_CENTER_W + (rotation @ local.T).T
    metadata = {
        "preset": "v6_irregular_generalization_adapted_to_flexiv",
        "random_seed": int(seed),
        "circle_center_w": CIRCLE_CENTER_W.tolist(),
        "circle_radius_m": CIRCLE_RADIUS_M,
        "arc_angle_deg": 360.0,
        "waypoint_count": WAYPOINT_COUNT,
        "radial_expansion_range": [-0.10, 0.40],
        "front_back_scale_range": [0.6, 2.0],
        "front_back_jitter_range_m": [-0.030, 0.020],
        "radial_scales": radial_scales.tolist(),
        "front_back_scales": front_back_scales.tolist(),
        "front_back_offsets_m": front_back_offsets.tolist(),
        "waypoint_points_m": points.tolist(),
    }
    return points, metadata


@dataclass(frozen=True)
class IrregularWaypointTarget:
    initial_position_w: np.ndarray
    waypoint_points_w: np.ndarray
    target_rotation_world: np.ndarray
    transition_duration_s: float
    path_duration_s: float
    segment_durations_s: np.ndarray
    metadata: dict[str, Any]

    @property
    def end_time_s(self) -> float:
        return self.transition_duration_s + self.path_duration_s

    def sample(self, time_s: float) -> tuple[np.ndarray, np.ndarray]:
        time_s = max(float(time_s), 0.0)
        if time_s < self.transition_duration_s:
            tau = time_s / self.transition_duration_s
            alpha, alpha_tau_dot = _minimum_jerk(tau)
            delta = self.waypoint_points_w[0] - self.initial_position_w
            return self.initial_position_w + alpha * delta, alpha_tau_dot * delta / self.transition_duration_s
        elapsed = time_s - self.transition_duration_s
        if elapsed >= self.path_duration_s:
            return self.waypoint_points_w[-1].copy(), np.zeros(3)
        cumulative = np.concatenate(([0.0], np.cumsum(self.segment_durations_s)))
        segment = int(np.clip(np.searchsorted(cumulative, elapsed, side="right") - 1, 0, len(self.segment_durations_s) - 1))
        duration = float(self.segment_durations_s[segment])
        alpha, alpha_tau_dot = _minimum_jerk((elapsed - cumulative[segment]) / duration)
        delta = self.waypoint_points_w[segment + 1] - self.waypoint_points_w[segment]
        return self.waypoint_points_w[segment] + alpha * delta, alpha_tau_dot * delta / duration

    def to_dict(self) -> dict[str, Any]:
        return {
            "mode": "irregular_waypoints",
            "reference_profile": "minimum_jerk_c2",
            "initial_position_w": self.initial_position_w.tolist(),
            "target_rotation_world": self.target_rotation_world.tolist(),
            "transition_duration_s": self.transition_duration_s,
            "path_duration_s": self.path_duration_s,
            "path_start_s": self.transition_duration_s,
            "path_end_s": self.end_time_s,
            "segment_durations_s": self.segment_durations_s.tolist(),
            **self.metadata,
        }


def build_target(initial_position_w: np.ndarray, rotation_world: np.ndarray, seed: int) -> IrregularWaypointTarget:
    points, metadata = build_waypoints(rotation_world, seed)
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    durations = PATH_DURATION_S * lengths / max(float(np.sum(lengths)), 1e-12)
    return IrregularWaypointTarget(
        np.asarray(initial_position_w, dtype=np.float64).copy(),
        points,
        np.asarray(rotation_world, dtype=np.float64).reshape(3, 3).copy(),
        TRANSITION_DURATION_S,
        PATH_DURATION_S,
        durations,
        metadata,
    )

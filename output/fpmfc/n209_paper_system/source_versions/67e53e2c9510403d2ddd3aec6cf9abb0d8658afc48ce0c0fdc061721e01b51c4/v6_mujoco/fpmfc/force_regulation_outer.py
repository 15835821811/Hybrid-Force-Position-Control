"""Bounded experimental normal outer loop for the N110D 2x2 comparison.

Positive x increases the flange target along the target grasp normal.  This
module never modifies the frozen NormalAdmittance class or MuJoCo material.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .contact import NormalAdmittanceState


@dataclass(frozen=True)
class ForceRegulationConfig:
    virtual_mass_kg: float
    damping_n_s_m: float
    stiffness_n_m: float
    timestep_s: float
    maximum_offset_m: float
    maximum_velocity_m_s: float
    detection_force_n: float
    release_force_n: float
    reset_on_contact_loss: bool

    def __post_init__(self) -> None:
        positive = ("virtual_mass_kg", "damping_n_s_m", "timestep_s",
                    "maximum_offset_m", "maximum_velocity_m_s", "detection_force_n")
        for name in positive:
            if not np.isfinite(getattr(self, name)) or float(getattr(self, name)) <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if not np.isfinite(self.stiffness_n_m) or self.stiffness_n_m < 0:
            raise ValueError("virtual stiffness must be finite and nonnegative")
        if not np.isfinite(self.release_force_n) or not 0 <= self.release_force_n < self.detection_force_n:
            raise ValueError("invalid contact hysteresis thresholds")


class ExperimentalNormalAdmittance:
    """Same semi-implicit bounded update as NormalAdmittance, with diagnostics."""

    def __init__(self, config: ForceRegulationConfig):
        self.config = config
        self.state = NormalAdmittanceState()
        self.detected = False
        self._reset_before = None
        self.rows: list[dict[str, float | bool]] = []

    def reset(self) -> NormalAdmittanceState:
        self._reset_before = self.state
        self.state = NormalAdmittanceState()
        return self.state

    def step(self, *, desired_force_n: float, measured_force_n: float) -> NormalAdmittanceState:
        desired = float(desired_force_n)
        measured = float(measured_force_n)
        if not np.isfinite(desired) or not np.isfinite(measured):
            raise ValueError("forces must be finite")
        cfg = self.config
        previous_detection = self.detected
        self.detected = measured > cfg.release_force_n if previous_detection else measured >= cfg.detection_force_n
        loss_event = previous_detection and not self.detected
        reset_applied = self._reset_before is not None
        if reset_applied != (loss_event and cfg.reset_on_contact_loss):
            raise RuntimeError("outer-loop reset differs from frozen S contact-loss policy")
        before = self._reset_before if reset_applied else self.state
        after_reset = self.state
        error = desired - measured
        mass_acceleration = error - cfg.damping_n_s_m * after_reset.velocity_m_s - cfg.stiffness_n_m * after_reset.offset_m
        acceleration = mass_acceleration / cfg.virtual_mass_kg
        velocity_unclipped = after_reset.velocity_m_s + cfg.timestep_s * acceleration
        velocity = float(np.clip(velocity_unclipped, -cfg.maximum_velocity_m_s, cfg.maximum_velocity_m_s))
        velocity_clipped = velocity != velocity_unclipped
        offset_unclipped = after_reset.offset_m + cfg.timestep_s * velocity
        offset = float(np.clip(offset_unclipped, -cfg.maximum_offset_m, cfg.maximum_offset_m))
        boundary_velocity_zeroed = offset != offset_unclipped and np.sign(velocity) == np.sign(offset)
        if boundary_velocity_zeroed:
            velocity = 0.0
        self.state = NormalAdmittanceState(offset_m=offset, velocity_m_s=velocity)
        self.rows.append({
            "feedback_time_s": len(self.rows) * cfg.timestep_s,
            "desired_force_n": desired, "measured_force_n": measured, "force_error_n": error,
            "detected_before": previous_detection, "detected_after": self.detected,
            "loss_event": loss_event, "reset_applied": reset_applied,
            "x_before_reset_m": before.offset_m, "v_before_reset_m_s": before.velocity_m_s,
            "x_after_reset_m": after_reset.offset_m, "v_after_reset_m_s": after_reset.velocity_m_s,
            "mass_times_acceleration_n": mass_acceleration,
            "damping_force_n": cfg.damping_n_s_m * after_reset.velocity_m_s,
            "spring_force_n": cfg.stiffness_n_m * after_reset.offset_m,
            "acceleration_m_s2": acceleration,
            "velocity_unclipped_m_s": velocity_unclipped,
            "offset_unclipped_m": offset_unclipped,
            "velocity_clipped": velocity_clipped,
            "offset_clipped": offset != offset_unclipped,
            "boundary_velocity_zeroed": boundary_velocity_zeroed,
            "x_after_step_m": offset, "v_after_step_m_s": velocity,
        })
        self._reset_before = None
        return self.state


def replay_outer_loop(trace: dict[str, np.ndarray], config: ForceRegulationConfig) -> ExperimentalNormalAdmittance:
    """Reconstruct state from recorded causal pre-step feedback, no dynamics run."""
    outer = ExperimentalNormalAdmittance(config)
    for desired, measured in zip(trace["command_desired_normal_force_n"],
                                 trace["feedback_measured_normal_force_n"], strict=True):
        if outer.detected and float(measured) <= config.release_force_n and config.reset_on_contact_loss:
            outer.reset()
        outer.step(desired_force_n=float(desired), measured_force_n=float(measured))
    return outer


def diagnostics_arrays(outer: ExperimentalNormalAdmittance) -> dict[str, np.ndarray]:
    return {key: np.asarray([row[key] for row in outer.rows]) for key in outer.rows[0]}

"""Causal contact wrench compensation for the frozen S contact loop.

Only the already synchronized t_k observation enters the feedback term.  The
underlying S servo performs its original forward solve exactly once.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass, field

import mujoco
import numpy as np

from ..model import site_id
from ..run import servo_torque as frozen_servo_torque
from .contact_observation import ContactObserver, ContactObservation


def transport_wrench(force: np.ndarray, torque_at_old: np.ndarray,
                     old_point: np.ndarray, new_point: np.ndarray) -> np.ndarray:
    """World torque about ``new_point`` for a wrench at ``old_point``."""
    return torque_at_old + np.cross(old_point - new_point, force)


def reduced_contact_load(mass: np.ndarray, linear_jacobian: np.ndarray,
                         angular_jacobian: np.ndarray, force: np.ndarray,
                         torque: np.ndarray, base_ids: np.ndarray,
                         joint_ids: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    if not all(np.all(np.isfinite(x)) for x in
               (mass, linear_jacobian, angular_jacobian, force, torque)):
        raise ValueError("nonfinite contact compensation input")
    full = linear_jacobian.T @ force + angular_jacobian.T @ torque
    bb = mass[np.ix_(base_ids, base_ids)]
    qb = mass[np.ix_(joint_ids, base_ids)]
    return full, full[joint_ids] - qb @ np.linalg.solve(bb, full[base_ids])


def bounded_compensation(raw_torque: np.ndarray, reduced_force: np.ndarray,
                         gamma: float, limits: np.ndarray,
                         cap_fraction: float = 0.10) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if not (np.isfinite(gamma) and gamma in (0.0, 0.5, 1.0)
            and np.isfinite(cap_fraction) and 0.0 < cap_fraction <= 1.0
            and all(np.all(np.isfinite(x)) for x in (raw_torque, reduced_force, limits))
            and np.all(limits > 0)):
        raise ValueError("nonfinite or invalid contact compensation input")
    requested = gamma * reduced_force
    protected = np.clip(requested, -cap_fraction * limits, cap_fraction * limits)
    return requested, protected, np.clip(raw_torque - protected, -limits, limits)


class RecordingObserver(ContactObserver):
    """Retain the most recent private observation, without writing to MjData."""

    def __init__(self, model: mujoco.MjModel, *, tool_face_recess_m: float) -> None:
        super().__init__(model, tool_face_recess_m=tool_face_recess_m)
        self.latest: ContactObservation | None = None
        self.calls = 0

    def observe(self, main_data: mujoco.MjData) -> ContactObservation:
        result = super().observe(main_data)
        self.latest = result
        self.calls += 1
        return result


@dataclass
class ContactServo:
    gamma: float
    cap_fraction: float = 0.10
    observer: RecordingObserver | None = None
    rows: list[dict[str, np.ndarray | float | int]] = field(default_factory=list)

    def __call__(self, model, data, dof_ids, base_dof_slice, qpos_ids,
                 reference_q, reference_dq, feedforward_ddq, torque_limits,
                 natural_frequency, acceleration_limit, full_mass):
        observation = self.observer.latest if self.observer is not None else None
        if observation is None or abs(observation.time_s - float(data.time)) > 1e-12:
            raise RuntimeError("no causal t_k contact observation")
        if not (np.array_equal(observation.qpos, data.qpos)
                and np.array_equal(observation.qvel, data.qvel)):
            raise RuntimeError("contact feedback state differs from servo state")
        inputs = (reference_q, reference_dq, feedforward_ddq, torque_limits,
                  observation.contact_force_world_n,
                  observation.contact_torque_at_flange_world_nm)
        if not all(np.all(np.isfinite(value)) for value in inputs):
            raise ValueError("nonfinite contact servo input at t_k")
        if not np.isfinite(natural_frequency) or not np.isfinite(acceleration_limit):
            raise ValueError("nonfinite contact servo gain")
        original, desired = frozen_servo_torque(
            model, data, dof_ids, base_dof_slice, qpos_ids, reference_q,
            reference_dq, feedforward_ddq, torque_limits, natural_frequency,
            acceleration_limit, full_mass,
        )
        base = np.arange(base_dof_slice.start, base_dof_slice.stop)
        bb = full_mass[np.ix_(base, base)]
        qb = full_mass[np.ix_(dof_ids, base)]
        base_acc = -np.linalg.solve(
            bb, full_mass[np.ix_(base, dof_ids)] @ desired
            + data.qfrc_bias[base] - data.qfrc_passive[base],
        )
        original_raw = (qb @ base_acc
                        + full_mass[np.ix_(dof_ids, dof_ids)] @ desired
                        + data.qfrc_bias[dof_ids] - data.qfrc_passive[dof_ids])
        if not np.array_equal(np.clip(original_raw, -torque_limits, torque_limits), original):
            raise RuntimeError("original raw torque reconstruction changed")

        # Both Jacobian and wrench are world-frame, linear then angular, about
        # flange_site.  The copy was solved with the preceding held torque.
        observed = observation.forward_data
        mass = np.zeros((model.nv, model.nv), dtype=np.float64)
        mujoco.mj_fullM(model, mass, observed.qM)
        jp = np.zeros((3, model.nv), dtype=np.float64)
        jr = np.zeros((3, model.nv), dtype=np.float64)
        mujoco.mj_jacSite(model, observed, jp, jr, site_id(model, "flange_site"))
        full_force, reduced = reduced_contact_load(
            mass, jp, jr, observation.contact_force_world_n,
            observation.contact_torque_at_flange_world_nm, base, dof_ids,
        )
        requested, protected, torque = bounded_compensation(
            original_raw, reduced, self.gamma, torque_limits, self.cap_fraction,
        )
        if self.gamma == 0.0 and not np.array_equal(torque, original):
            raise RuntimeError("gamma=0 changed original S servo torque")

        # Identity of the preceding held-torque forward solve.  Residual from
        # all other constraints is listed separately from the interface load.
        h = observed.qfrc_bias - observed.qfrc_passive - observed.qfrc_applied
        full_residual = (mass @ observed.qacc + h - observed.qfrc_actuator
                         - observed.qfrc_constraint)
        other = observed.qfrc_constraint - full_force
        red_mass = (mass[np.ix_(dof_ids, dof_ids)]
                    - qb @ np.linalg.solve(bb, mass[np.ix_(base, dof_ids)]))
        red_h = h[dof_ids] - qb @ np.linalg.solve(bb, h[base])
        red_other = other[dof_ids] - qb @ np.linalg.solve(bb, other[base])
        red_residual = (red_mass @ observed.qacc[dof_ids] + red_h
                        - observed.qfrc_actuator[dof_ids] - reduced - red_other)
        interface_pair = {self.observer.pad, self.observer.plate}
        other_contacts = sum(
            {int(observed.contact[i].geom1), int(observed.contact[i].geom2)} != interface_pair
            for i in range(observed.ncon)
        )
        limit_code = int(mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT)
        joint_limits = int(np.count_nonzero(np.asarray(observed.efc_type[:observed.nefc]) == limit_code))
        self.rows.append({
            "feedback_time_s": observation.time_s,
            "original_raw_torque_nm": original_raw.copy(),
            "original_bounded_torque_nm": original.copy(),
            "requested_compensation_nm": requested.copy(),
            "protected_compensation_nm": protected.copy(),
            "reduced_contact_load_nm": reduced.copy(),
            "full_contact_generalized_force": full_force.copy(),
            "applied_torque_nm": torque.copy(),
            "desired_acceleration_rad_s2": desired.copy(),
            "full_dynamics_residual_max": float(np.max(np.abs(full_residual))),
            "reduced_dynamics_residual_max": float(np.max(np.abs(red_residual))),
            "other_generalized_force_norm": float(np.linalg.norm(other)),
            "other_contact_count": int(other_contacts),
            "joint_limit_constraint_count": joint_limits,
            "wrench_power_w": float(observation.contact_force_world_n @ (jp @ observed.qvel)
                                     + observation.contact_torque_at_flange_world_nm @ (jr @ observed.qvel)),
            "generalized_power_w": float(full_force @ observed.qvel),
            "pre_qvel": observation.qvel.copy(),
            "held_torque_nm": observation.ctrl_used_for_forward.copy(),
        })
        return torque, desired

    def arrays(self) -> dict[str, np.ndarray]:
        return {key: np.asarray([row[key] for row in self.rows]) for key in self.rows[0]}

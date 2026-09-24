"""Contact-wrench aggregation and scalar normal-admittance primitives."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import mujoco
import numpy as np


def _vector3(value: np.ndarray | Iterable[float], name: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must be a finite three-vector")
    return array


@dataclass(frozen=True)
class ContactWrench:
    """Net wrench on selected geoms, expressed in the world frame."""

    force_world_n: np.ndarray
    torque_world_nm: np.ndarray
    reference_point_world_m: np.ndarray
    contact_count: int

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "force_world_n",
            _vector3(self.force_world_n, "force_world_n").copy(),
        )
        object.__setattr__(
            self,
            "torque_world_nm",
            _vector3(self.torque_world_nm, "torque_world_nm").copy(),
        )
        object.__setattr__(
            self,
            "reference_point_world_m",
            _vector3(
                self.reference_point_world_m, "reference_point_world_m"
            ).copy(),
        )
        if int(self.contact_count) < 0:
            raise ValueError("contact_count must be nonnegative")
        object.__setattr__(self, "contact_count", int(self.contact_count))


def maximum_interface_penetration(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    selected_geom_ids: Iterable[int],
    counterpart_geom_ids: Iterable[int],
) -> float:
    """Return the largest active-contact penetration for one geom interface.

    The active ``mjContact.dist`` value is used rather than the general convex
    ``mj_geomDistance`` query.  The latter can jump to the translation needed
    to separate fully overlapping finite solids and is not a surface
    compression measure once one thin interface crosses the other.
    """

    selected = {int(value) for value in selected_geom_ids}
    counterparts = {int(value) for value in counterpart_geom_ids}
    if not selected or not counterparts:
        raise ValueError("both interface geom sets must be nonempty")
    if any(value < 0 or value >= model.ngeom for value in selected | counterparts):
        raise ValueError("interface geom ids contain an invalid value")
    penetration = 0.0
    for contact_index in range(int(data.ncon)):
        contact = data.contact[contact_index]
        geom0, geom1 = map(int, contact.geom)
        if not (
            (geom0 in selected and geom1 in counterparts)
            or (geom1 in selected and geom0 in counterparts)
        ):
            continue
        penetration = max(penetration, -float(contact.dist))
    return max(0.0, penetration)


def aggregate_contact_wrench(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    selected_geom_ids: Iterable[int],
    counterpart_geom_ids: Iterable[int] | None = None,
    reference_point_world_m: np.ndarray | Iterable[float],
) -> ContactWrench:
    """Return the net wrench applied to ``selected_geom_ids``.

    ``mj_contactForce`` returns the force and torque on contact geom 2 in the
    contact frame.  ``mjContact.frame`` stores world-frame axes by row, so the
    local wrench is rotated with ``frame.T``.  Contacts where the selected geom
    is geom 1 are negated.  Per-contact torque is shifted from the contact point
    to ``reference_point_world_m`` before summation.
    """

    selected = {int(value) for value in selected_geom_ids}
    if not selected:
        raise ValueError("selected_geom_ids cannot be empty")
    if any(value < 0 or value >= model.ngeom for value in selected):
        raise ValueError("selected_geom_ids contains an invalid geom id")
    counterparts = (
        None
        if counterpart_geom_ids is None
        else {int(value) for value in counterpart_geom_ids}
    )
    if counterparts is not None and any(
        value < 0 or value >= model.ngeom for value in counterparts
    ):
        raise ValueError("counterpart_geom_ids contains an invalid geom id")
    reference = _vector3(reference_point_world_m, "reference_point_world_m")
    net_force = np.zeros(3, dtype=np.float64)
    net_torque = np.zeros(3, dtype=np.float64)
    count = 0
    local_wrench = np.zeros(6, dtype=np.float64)

    for contact_index in range(int(data.ncon)):
        contact = data.contact[contact_index]
        geom0, geom1 = map(int, contact.geom)
        if geom0 in selected and (counterparts is None or geom1 in counterparts):
            sign = -1.0
        elif geom1 in selected and (
            counterparts is None or geom0 in counterparts
        ):
            sign = 1.0
        else:
            continue

        local_wrench[:] = 0.0
        mujoco.mj_contactForce(model, data, contact_index, local_wrench)
        contact_frame_world = np.asarray(contact.frame, dtype=np.float64).reshape(3, 3)
        force_world = sign * (contact_frame_world.T @ local_wrench[:3])
        torque_at_contact_world = sign * (
            contact_frame_world.T @ local_wrench[3:]
        )
        contact_position_world = np.asarray(contact.pos, dtype=np.float64)
        torque_at_reference_world = torque_at_contact_world + np.cross(
            contact_position_world - reference, force_world
        )
        net_force += force_world
        net_torque += torque_at_reference_world
        count += 1

    return ContactWrench(
        force_world_n=net_force,
        torque_world_nm=net_torque,
        reference_point_world_m=reference,
        contact_count=count,
    )


@dataclass(frozen=True)
class NormalAdmittanceConfig:
    """Parameters for ``M xdd + B xd + K x = F_des - F_meas``."""

    virtual_mass_kg: float
    damping_n_s_m: float
    stiffness_n_m: float
    timestep_s: float
    maximum_offset_m: float
    maximum_velocity_m_s: float

    def __post_init__(self) -> None:
        for name in (
            "virtual_mass_kg",
            "damping_n_s_m",
            "stiffness_n_m",
            "timestep_s",
            "maximum_offset_m",
            "maximum_velocity_m_s",
        ):
            value = float(getattr(self, name))
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
            object.__setattr__(self, name, value)


@dataclass(frozen=True)
class NormalAdmittanceState:
    offset_m: float = 0.0
    velocity_m_s: float = 0.0

    def __post_init__(self) -> None:
        if not np.isfinite(self.offset_m) or not np.isfinite(self.velocity_m_s):
            raise ValueError("admittance state must be finite")


class NormalAdmittance:
    """Bounded semi-implicit discretization of a scalar normal admittance.

    Positive offset is defined along the caller's commanded contact direction.
    Consequently, a positive ``desired_force_n - measured_force_n`` increases
    penetration command.  The caller remains responsible for choosing and
    documenting that direction in world or tool coordinates.
    """

    def __init__(self, config: NormalAdmittanceConfig) -> None:
        self.config = config
        self.state = NormalAdmittanceState()

    def reset(self) -> NormalAdmittanceState:
        self.state = NormalAdmittanceState()
        return self.state

    def step(
        self, *, desired_force_n: float, measured_force_n: float
    ) -> NormalAdmittanceState:
        desired = float(desired_force_n)
        measured = float(measured_force_n)
        if not np.isfinite(desired) or not np.isfinite(measured):
            raise ValueError("desired and measured forces must be finite")
        cfg = self.config
        acceleration = (
            desired
            - measured
            - cfg.damping_n_s_m * self.state.velocity_m_s
            - cfg.stiffness_n_m * self.state.offset_m
        ) / cfg.virtual_mass_kg
        velocity = float(
            np.clip(
                self.state.velocity_m_s + cfg.timestep_s * acceleration,
                -cfg.maximum_velocity_m_s,
                cfg.maximum_velocity_m_s,
            )
        )
        offset_unclipped = self.state.offset_m + cfg.timestep_s * velocity
        offset = float(
            np.clip(
                offset_unclipped,
                -cfg.maximum_offset_m,
                cfg.maximum_offset_m,
            )
        )
        if offset != offset_unclipped and np.sign(velocity) == np.sign(offset):
            velocity = 0.0
        self.state = NormalAdmittanceState(offset_m=offset, velocity_m_s=velocity)
        return self.state

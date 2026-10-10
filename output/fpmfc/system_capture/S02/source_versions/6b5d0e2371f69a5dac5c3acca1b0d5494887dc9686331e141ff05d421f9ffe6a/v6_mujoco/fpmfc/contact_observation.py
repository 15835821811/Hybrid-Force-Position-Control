"""Synchronized, non-invasive observations for the contact-only experiment.

The complete MjData object is copied before mj_forward. MuJoCo's Python copy
operation carries integrator state, applied inputs, solver warm start, mocap,
and constraint state; the original MjData is never forwarded or written here.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass

import mujoco
import numpy as np

from ..model import geom_id, site_id
from .contact import aggregate_contact_wrench, maximum_interface_penetration
from .dynamics import rotation_distance_rad


_CALLBACKS = (
    "control", "passive", "sensor", "contactfilter", "act_bias", "act_dyn",
    "act_gain", "time",
)


def assert_observation_safe(model: mujoco.MjModel) -> None:
    """Reject callbacks/plugins whose repeated mj_forward could have side effects."""
    active = [name for name in _CALLBACKS if getattr(mujoco, f"get_mjcb_{name}")() is not None]
    if active or int(model.nplugin) != 0:
        raise RuntimeError(f"observation requires isolated callbacks/plugins: callbacks={active}, plugins={model.nplugin}")


def _site_twist(model: mujoco.MjModel, data: mujoco.MjData, site: int) -> tuple[np.ndarray, np.ndarray]:
    linear = np.zeros((3, model.nv), dtype=np.float64)
    angular = np.zeros((3, model.nv), dtype=np.float64)
    mujoco.mj_jacSite(model, data, linear, angular, site)
    return linear @ data.qvel, angular @ data.qvel


@dataclass(frozen=True)
class ContactObservation:
    # `forward_data` is a private complete MjData copy after synchronization.
    forward_data: mujoco.MjData
    time_s: float
    qpos: np.ndarray
    qvel: np.ndarray
    act: np.ndarray
    plugin_state: np.ndarray
    ctrl_used_for_forward: np.ndarray
    qfrc_applied: np.ndarray
    xfrc_applied: np.ndarray
    qacc_warmstart_used_for_forward: np.ndarray
    mocap_pos: np.ndarray
    mocap_quat: np.ndarray
    eq_active: np.ndarray
    flange_position_world_m: np.ndarray
    flange_rotation_world: np.ndarray
    flange_linear_velocity_world_m_s: np.ndarray
    flange_angular_velocity_world_rad_s: np.ndarray
    grasp_position_world_m: np.ndarray
    grasp_rotation_world: np.ndarray
    grasp_linear_velocity_world_m_s: np.ndarray
    grasp_angular_velocity_world_rad_s: np.ndarray
    tool_face_position_world_m: np.ndarray
    tool_face_linear_velocity_world_m_s: np.ndarray
    contact_normal_world: np.ndarray
    contact_force_world_n: np.ndarray
    contact_torque_at_flange_world_nm: np.ndarray
    measured_normal_force_n: float
    contact_count: int
    penetration_m: float
    relative_position_world_m: np.ndarray
    relative_linear_velocity_world_m_s: np.ndarray
    relative_angular_velocity_world_rad_s: np.ndarray
    relative_orientation_error_rad: float


class ContactObserver:
    """Observe one model with fixed geom/site identities and point conventions."""

    def __init__(self, model: mujoco.MjModel, *, tool_face_recess_m: float) -> None:
        assert_observation_safe(model)
        self.model = model
        self.flange = site_id(model, "flange_site")
        self.grasp = site_id(model, "target_grasp_site")
        self.pad = geom_id(model, "gripper_contact_pad")
        self.plate = geom_id(model, "target_contact_plate")
        self.tool_face_recess_m = float(tool_face_recess_m)
        if not np.isfinite(self.tool_face_recess_m) or self.tool_face_recess_m < 0.0:
            raise ValueError("tool face recess must be finite and nonnegative")

    def contract(self) -> dict[str, object]:
        return {
            "schema_version": "fpmfc_contact_observation_v1",
            "time_definition": "data.time at current generalized state; all derived fields from mj_forward on full MjData copy",
            "forward_control_definition": "held torque already applied in preceding physics step; initial value zero",
            "world_frame_fields": "site pose/twist, contact wrench/normal, relative pose/twist",
            "tool_face_definition": "flange origin minus local flange +Z times tool_face_recess_m",
            "tool_face_recess_m": self.tool_face_recess_m,
            "contact_interface_geoms": ["gripper_contact_pad", "target_contact_plate"],
            "wrench_reference_point": "flange_site world position at observation time",
            "penetration_definition": "maximum active interface -mjContact.dist clipped at zero",
            "normal_force_definition": "max(0, -force_on_pad_world dot grasp_site local +Z in world)",
            "integrator_state_copy": "copy.copy(MjData), including time/qpos/qvel/act/plugin_state/ctrl/applied forces/qacc_warmstart/mocap/equality state",
            "model_disableflags": int(self.model.opt.disableflags),
            "model_enableflags": int(self.model.opt.enableflags),
            "integrator": int(self.model.opt.integrator),
            "timestep_s": float(self.model.opt.timestep),
        }

    def observe(self, main_data: mujoco.MjData) -> ContactObservation:
        assert_observation_safe(self.model)
        observed = copy.copy(main_data)
        # Save the inputs to the forward solve before it can update warm start.
        warmstart = np.asarray(observed.qacc_warmstart).copy()
        control = np.asarray(observed.ctrl).copy()
        qfrc = np.asarray(observed.qfrc_applied).copy()
        xfrc = np.asarray(observed.xfrc_applied).copy()
        mocap_pos = np.asarray(observed.mocap_pos).copy()
        mocap_quat = np.asarray(observed.mocap_quat).copy()
        eq_active = np.asarray(observed.eq_active).copy()
        act = np.asarray(observed.act).copy()
        plugin_state = np.asarray(observed.plugin_state).copy()
        mujoco.mj_forward(self.model, observed)
        fp = np.asarray(observed.site_xpos[self.flange]).copy()
        fr = np.asarray(observed.site_xmat[self.flange]).reshape(3, 3).copy()
        gp = np.asarray(observed.site_xpos[self.grasp]).copy()
        gr = np.asarray(observed.site_xmat[self.grasp]).reshape(3, 3).copy()
        fv, fw = _site_twist(self.model, observed, self.flange)
        gv, gw = _site_twist(self.model, observed, self.grasp)
        tool_face = fp - fr[:, 2] * self.tool_face_recess_m
        tool_velocity = fv + np.cross(fw, tool_face - fp)
        normal = gr[:, 2].copy()
        wrench = aggregate_contact_wrench(
            self.model, observed, selected_geom_ids=[self.pad],
            counterpart_geom_ids=[self.plate], reference_point_world_m=fp,
        )
        penetration = maximum_interface_penetration(
            self.model, observed, selected_geom_ids=[self.pad],
            counterpart_geom_ids=[self.plate],
        )
        return ContactObservation(
            forward_data=observed,
            time_s=float(observed.time), qpos=np.asarray(observed.qpos).copy(),
            qvel=np.asarray(observed.qvel).copy(), act=act, plugin_state=plugin_state,
            ctrl_used_for_forward=control, qfrc_applied=qfrc, xfrc_applied=xfrc,
            qacc_warmstart_used_for_forward=warmstart,
            mocap_pos=mocap_pos, mocap_quat=mocap_quat, eq_active=eq_active,
            flange_position_world_m=fp, flange_rotation_world=fr,
            flange_linear_velocity_world_m_s=fv, flange_angular_velocity_world_rad_s=fw,
            grasp_position_world_m=gp, grasp_rotation_world=gr,
            grasp_linear_velocity_world_m_s=gv, grasp_angular_velocity_world_rad_s=gw,
            tool_face_position_world_m=tool_face,
            tool_face_linear_velocity_world_m_s=tool_velocity,
            contact_normal_world=normal,
            contact_force_world_n=wrench.force_world_n.copy(),
            contact_torque_at_flange_world_nm=wrench.torque_world_nm.copy(),
            measured_normal_force_n=max(0.0, -float(wrench.force_world_n @ normal)),
            contact_count=wrench.contact_count, penetration_m=penetration,
            relative_position_world_m=fp - gp,
            relative_linear_velocity_world_m_s=fv - gv,
            relative_angular_velocity_world_rad_s=fw - gw,
            relative_orientation_error_rad=rotation_distance_rad(gr, fr),
        )


def observation_arrays(observation: ContactObservation) -> dict[str, np.ndarray | float | int]:
    """Serializable synchronized fields for trace rows."""
    o = observation
    return {
        "time": o.time_s, "qpos": o.qpos, "qvel": o.qvel,
        "ctrl_used_for_forward": o.ctrl_used_for_forward,
        "qfrc_applied": o.qfrc_applied, "xfrc_applied": o.xfrc_applied,
        "qacc_warmstart_used_for_forward": o.qacc_warmstart_used_for_forward,
        "act": o.act, "plugin_state": o.plugin_state,
        "mocap_pos": o.mocap_pos, "mocap_quat": o.mocap_quat,
        "eq_active": o.eq_active,
        "flange_position": o.flange_position_world_m,
        "flange_rotation": o.flange_rotation_world,
        "flange_linear_velocity_world_m_s": o.flange_linear_velocity_world_m_s,
        "flange_angular_velocity_world_rad_s": o.flange_angular_velocity_world_rad_s,
        "target_grasp_position": o.grasp_position_world_m,
        "target_grasp_rotation": o.grasp_rotation_world,
        "target_grasp_linear_velocity_world_m_s": o.grasp_linear_velocity_world_m_s,
        "target_grasp_angular_velocity_world_rad_s": o.grasp_angular_velocity_world_rad_s,
        "tool_face_position_world_m": o.tool_face_position_world_m,
        "tool_face_linear_velocity_world_m_s": o.tool_face_linear_velocity_world_m_s,
        "contact_normal_world": o.contact_normal_world,
        "contact_force_world_n": o.contact_force_world_n,
        "contact_torque_at_flange_world_nm": o.contact_torque_at_flange_world_nm,
        "measured_normal_force_n": o.measured_normal_force_n,
        "contact_count": o.contact_count, "penetration_m": o.penetration_m,
        "relative_position_world_m": o.relative_position_world_m,
        "relative_linear_velocity_world_m_s": o.relative_linear_velocity_world_m_s,
        "relative_angular_velocity_world_rad_s": o.relative_angular_velocity_world_rad_s,
        "relative_orientation_error_rad": o.relative_orientation_error_rad,
        "relative_position_error_m": float(np.linalg.norm(o.relative_position_world_m)),
        "relative_linear_speed_m_s": float(np.linalg.norm(o.relative_linear_velocity_world_m_s)),
        "relative_angular_speed_rad_s": float(np.linalg.norm(o.relative_angular_velocity_world_rad_s)),
    }

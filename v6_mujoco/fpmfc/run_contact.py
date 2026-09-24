"""Run the physical-target contact stage from an audited pre-contact trace.

The contact extension is deliberately separate from the frozen pre-contact
model.  A named-joint transfer initializes the physical target, free base and
seven Flexiv joints from the terminal sample of one N073 ``full`` trace.  The
runner then applies either a ramped rigid normal offset or the scalar normal
admittance while retaining the same pose/shape HQP and torque servo.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping

import mujoco
import numpy as np

from ..collision import build_collision_pairs
from ..model import PROJECT_ROOT, body_id, default_model_spec, geom_id, site_id
from ..run import robot_momentum, servo_torque
from .config import load_fpmfc_config
from .contact import (
    NormalAdmittance,
    aggregate_contact_wrench,
    maximum_interface_penetration,
)
from .contact_config import (
    DEFAULT_CONTACT_CONFIG_PATH,
    load_contact_config,
    normal_admittance_config,
)
from .contact_model import ContactModelSpec, default_contact_model_spec
from .contact_provenance import contact_implementation_identity
from .controller import FPMFCHQP
from .dynamics import rotation_distance_rad
from .run_capture import (
    DynamicRunConfig,
    _constraint_config,
    _controller_config,
    controller_variant_config,
)
from .shape import ArmShapeKinematics
from .target import target_from_config


DEFAULT_SOURCE_TRACE = (
    PROJECT_ROOT
    / "output"
    / "fpmfc"
    / "precontact"
    / "n073_controller_ablation_rk4_formal_planning45"
    / "seed_00"
    / "full"
    / "trace.npz"
)
CONTACT_VARIANTS = ("rigid", "admittance", "admittance-no-shape")


@dataclass(frozen=True)
class ContactRunConfig:
    reference_tracking_band_rad: float = 0.012
    servo_natural_frequency_rad_s: float = 34.0
    servo_acceleration_limit_rad_s2: float = 70.0


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _mapping_sha256(value: Mapping[str, Any]) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _json_default(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


def _matrix_to_mujoco_quaternion(rotation: np.ndarray) -> np.ndarray:
    quaternion = np.zeros(4, dtype=np.float64)
    mujoco.mju_mat2Quat(
        quaternion, np.asarray(rotation, dtype=np.float64).reshape(9)
    )
    return quaternion


def _quintic_ramp(time_s: float, duration_s: float) -> tuple[float, float]:
    if time_s <= 0.0:
        return 0.0, 0.0
    if time_s >= duration_s:
        return 1.0, 0.0
    phase = float(time_s / duration_s)
    value = 10.0 * phase**3 - 15.0 * phase**4 + 6.0 * phase**5
    derivative = (
        30.0 * phase**2 - 60.0 * phase**3 + 30.0 * phase**4
    ) / duration_s
    return value, derivative


def _site_twist(
    model: mujoco.MjModel, data: mujoco.MjData, site: int
) -> tuple[np.ndarray, np.ndarray]:
    linear_jacobian = np.zeros((3, model.nv), dtype=np.float64)
    angular_jacobian = np.zeros((3, model.nv), dtype=np.float64)
    mujoco.mj_jacSite(
        model, data, linear_jacobian, angular_jacobian, int(site)
    )
    velocity = np.asarray(data.qvel)
    return linear_jacobian @ velocity, angular_jacobian @ velocity


def _subtree_momentum_about_world_origin(
    model: mujoco.MjModel, data: mujoco.MjData, root_body: int
) -> np.ndarray:
    """Return spatial momentum about the fixed world origin for one subtree."""

    mujoco.mj_subtreeVel(model, data)
    mass = float(model.body_subtreemass[root_body])
    linear = mass * np.asarray(data.subtree_linvel[root_body], dtype=np.float64)
    angular_com = np.asarray(data.subtree_angmom[root_body], dtype=np.float64)
    center = np.asarray(data.subtree_com[root_body], dtype=np.float64)
    return np.concatenate((linear, angular_com + np.cross(center, linear)))


def initialize_contact_state(
    *,
    source_trace_path: Path | str,
    precontact_config: Mapping[str, Any],
    contact_spec: ContactModelSpec,
    contact_model: mujoco.MjModel,
    contact_data: mujoco.MjData,
) -> dict[str, Any]:
    """Transfer a terminal pre-contact state using named joint addresses.

    MuJoCo stores the rotational velocity of a free joint in the body's local
    frame.  The prescribed target angular velocity is therefore rotated from
    world into the target frame before it is assigned.
    """

    source_path = Path(source_trace_path).resolve()
    if not source_path.is_file():
        raise FileNotFoundError(source_path)
    with np.load(source_path, allow_pickle=False) as source:
        required = {
            "qpos",
            "qvel",
            "reference_q",
            "reference_dq",
            "reference_ddq",
            "task_joint_velocity_rad_s",
            "task_joint_acceleration_rad_s2",
            "capture_time_s",
            "terminal_arm_angle_rad",
            "target_center_position",
            "target_center_rotation",
            "target_grasp_position",
            "target_grasp_rotation",
        }
        missing = sorted(required - set(source.files))
        if missing:
            raise ValueError(f"source trace is missing arrays: {missing}")
        source_terminal_qpos = np.asarray(source["qpos"][-1], dtype=np.float64)
        source_terminal_qvel = np.asarray(source["qvel"][-1], dtype=np.float64)
        reference_q = np.asarray(source["reference_q"][-1], dtype=np.float64)
        reference_dq = np.asarray(source["reference_dq"][-1], dtype=np.float64)
        reference_ddq = np.asarray(source["reference_ddq"][-1], dtype=np.float64)
        controller_velocity = np.asarray(
            source["task_joint_velocity_rad_s"][-1], dtype=np.float64
        )
        controller_acceleration = np.asarray(
            source["task_joint_acceleration_rad_s2"][-1], dtype=np.float64
        )
        capture_time_s = float(source["capture_time_s"])
        terminal_arm_angle_rad = float(source["terminal_arm_angle_rad"])
        recorded_center_position = np.asarray(
            source["target_center_position"][-1], dtype=np.float64
        )
        recorded_center_rotation = np.asarray(
            source["target_center_rotation"][-1], dtype=np.float64
        )
        recorded_grasp_position = np.asarray(
            source["target_grasp_position"][-1], dtype=np.float64
        )
        recorded_grasp_rotation = np.asarray(
            source["target_grasp_rotation"][-1], dtype=np.float64
        )

    source_spec = default_model_spec()
    source_model = source_spec.compile_model()
    if source_terminal_qpos.shape != (source_model.nq,) or source_terminal_qvel.shape != (
        source_model.nv,
    ):
        raise ValueError("source terminal state dimensions do not match the source model")
    source_base_qpos, source_base_dof = source_spec.base_slices(source_model)
    contact_base_qpos, contact_base_dof = contact_spec.base_slices(contact_model)
    source_joint_qpos, source_joint_dof = source_spec.joint_addresses(source_model)
    contact_joint_qpos, contact_joint_dof = contact_spec.joint_addresses(contact_model)
    target_qpos, target_dof = contact_spec.target_slices(contact_model)

    contact_data.qpos[contact_base_qpos] = source_terminal_qpos[source_base_qpos]
    contact_data.qvel[contact_base_dof] = source_terminal_qvel[source_base_dof]
    contact_data.qpos[contact_joint_qpos] = source_terminal_qpos[source_joint_qpos]
    contact_data.qvel[contact_joint_dof] = source_terminal_qvel[source_joint_dof]

    target = target_from_config(precontact_config)
    target_sample = target.sample(capture_time_s)
    if not (
        np.allclose(
            recorded_center_position,
            target_sample.center_position_world_m,
            atol=2e-12,
        )
        and np.allclose(
            recorded_center_rotation,
            target_sample.center_rotation_world,
            atol=2e-12,
        )
        and np.allclose(
            recorded_grasp_position,
            target_sample.grasp_position_world_m,
            atol=2e-12,
        )
        and np.allclose(
            recorded_grasp_rotation,
            target_sample.grasp_rotation_world,
            atol=2e-12,
        )
    ):
        raise ValueError("source trace target state differs from the frozen target model")

    contact_data.qpos[target_qpos.start : target_qpos.start + 3] = (
        target_sample.center_position_world_m
    )
    contact_data.qpos[target_qpos.start + 3 : target_qpos.stop] = (
        _matrix_to_mujoco_quaternion(target_sample.center_rotation_world)
    )
    contact_data.qvel[target_dof.start : target_dof.start + 3] = (
        target_sample.center_linear_velocity_world_m_s
    )
    contact_data.qvel[target_dof.start + 3 : target_dof.stop] = (
        target_sample.center_rotation_world.T
        @ target_sample.angular_velocity_world_rad_s
    )
    contact_data.time = 0.0
    contact_data.ctrl[:] = 0.0
    mujoco.mj_forward(contact_model, contact_data)

    grasp_site = site_id(contact_model, "target_grasp_site")
    grasp_linear_velocity, grasp_angular_velocity = _site_twist(
        contact_model, contact_data, grasp_site
    )
    errors = {
        "center_position_m": float(
            np.linalg.norm(
                np.asarray(contact_data.qpos[target_qpos][:3])
                - target_sample.center_position_world_m
            )
        ),
        "grasp_position_m": float(
            np.linalg.norm(
                np.asarray(contact_data.site_xpos[grasp_site])
                - target_sample.grasp_position_world_m
            )
        ),
        "grasp_rotation_frobenius": float(
            np.linalg.norm(
                np.asarray(contact_data.site_xmat[grasp_site]).reshape(3, 3)
                - target_sample.grasp_rotation_world
            )
        ),
        "grasp_linear_velocity_m_s": float(
            np.linalg.norm(
                grasp_linear_velocity
                - target_sample.grasp_linear_velocity_world_m_s
            )
        ),
        "grasp_angular_velocity_rad_s": float(
            np.linalg.norm(
                grasp_angular_velocity
                - target_sample.grasp_angular_velocity_world_rad_s
            )
        ),
    }
    if max(errors.values()) > 2e-10:
        raise RuntimeError(f"contact-state transfer failed: {errors}")
    return {
        "source_trace_path": str(source_path),
        "source_trace_sha256": _sha256(source_path),
        "capture_time_s": capture_time_s,
        "terminal_arm_angle_rad": terminal_arm_angle_rad,
        "reference_q": reference_q,
        "reference_dq": reference_dq,
        "reference_ddq": reference_ddq,
        "controller_velocity": controller_velocity,
        "controller_acceleration": controller_acceleration,
        "target_transfer_errors": errors,
    }


def run_contact_experiment(
    *,
    variant: str,
    source_trace_path: Path | str = DEFAULT_SOURCE_TRACE,
    contact_config_path: Path | str = DEFAULT_CONTACT_CONFIG_PATH,
    output_dir: Path | str,
    run_config: ContactRunConfig = ContactRunConfig(),
    overwrite: bool = False,
) -> dict[str, Any]:
    """Run and persist one rigid or admittance contact experiment."""

    if variant not in CONTACT_VARIANTS:
        raise ValueError(f"variant must be one of {CONTACT_VARIANTS}")
    contact_config_path = Path(contact_config_path).resolve()
    contact_config = load_contact_config(contact_config_path)
    precontact_config_path = (PROJECT_ROOT / contact_config["precontact_config"]).resolve()
    precontact_config = load_fpmfc_config(precontact_config_path)
    controller_variant = "no-shape" if variant == "admittance-no-shape" else "full"
    controller_config_values = controller_variant_config(
        precontact_config, controller_variant
    )

    output = Path(output_dir).resolve()
    trace_path = output / "trace.npz"
    metrics_path = output / "metrics.json"
    if not overwrite and (trace_path.exists() or metrics_path.exists()):
        raise FileExistsError(f"contact output already exists: {output}")
    output.mkdir(parents=True, exist_ok=True)

    spec = default_contact_model_spec()
    if contact_config_path != Path(spec.contact_config_path).resolve():
        spec = ContactModelSpec(contact_config_path=contact_config_path)
    model = spec.compile_model()
    obstacle = geom_id(model, "workspace_obstacle_0")
    model.geom_pos[obstacle] = [10.0, 10.0, 10.0]
    data = mujoco.MjData(model)
    spec.reset_home(model, data)

    shape_values = precontact_config["shape"]
    # Construct at the same home pose as the A-stage so the fixed reference
    # axis defining the geometric arm angle cannot switch at the handoff.
    shape = ArmShapeKinematics(
        spec,
        model,
        data,
        shoulder_joint=shape_values["shoulder_joint"],
        elbow_joint=shape_values["elbow_joint"],
        wrist_joint=shape_values["wrist_joint"],
        singularity_margin=float(shape_values["singularity_margin"]),
    )
    transfer = initialize_contact_state(
        source_trace_path=source_trace_path,
        precontact_config=precontact_config,
        contact_spec=spec,
        contact_model=model,
        contact_data=data,
    )

    controller = FPMFCHQP(
        spec,
        model,
        build_collision_pairs(model),
        shape,
        controller_config=_controller_config(controller_config_values),
        constraint_config=_constraint_config(controller_config_values),
    )
    controller.previous_velocity = np.asarray(
        transfer["controller_velocity"], dtype=np.float64
    ).copy()
    controller.previous_acceleration = np.asarray(
        transfer["controller_acceleration"], dtype=np.float64
    ).copy()

    qpos_ids, dof_ids = spec.joint_addresses(model)
    base_qpos_slice, base_dof_slice = spec.base_slices(model)
    target_qpos_slice, _target_dof_slice = spec.target_slices(model)
    flange_site = site_id(model, "flange_site")
    target_grasp_site = site_id(model, "target_grasp_site")
    pad_geom = geom_id(model, "gripper_contact_pad")
    target_geom = geom_id(model, "target_contact_plate")
    base_body = body_id(model, "base_link_0")
    target_body = body_id(model, "tumbling_target")

    force_values = contact_config["force_control"]
    interface_values = contact_config["interface"]
    acceptance_values = contact_config["acceptance"]
    duration_s = float(force_values["contact_stage_duration_s"])
    ramp_duration_s = float(force_values["force_ramp_duration_s"])
    desired_force_final_n = float(force_values["desired_normal_force_n"])
    rigid_offset_m = float(force_values["rigid_normal_offset_m"])
    tool_face_recess_m = float(interface_values["tool_pad_face_recess_m"])
    physics_steps = int(round(duration_s / spec.timestep_s))
    task_stride = int(round(spec.task_period_s / spec.timestep_s))

    initial_qpos = np.asarray(data.qpos).copy()
    initial_qvel = np.asarray(data.qvel).copy()
    initial_base_pose = np.asarray(data.qpos[base_qpos_slice]).copy()
    initial_robot_momentum = robot_momentum(model, data, base_body)
    initial_total_momentum = (
        _subtree_momentum_about_world_origin(model, data, base_body)
        + _subtree_momentum_about_world_origin(model, data, target_body)
    )
    initial_robot_com = np.asarray(data.subtree_com[base_body]).copy()
    reference_q = np.asarray(transfer["reference_q"], dtype=np.float64).copy()
    command_velocity = np.asarray(
        transfer["controller_velocity"], dtype=np.float64
    ).copy()
    segment_start_velocity = command_velocity.copy()
    segment_step = 0
    full_mass = np.zeros((model.nv, model.nv), dtype=np.float64)
    admittance = NormalAdmittance(
        normal_admittance_config(contact_config, timestep_s=spec.timestep_s)
    )

    detected = False
    ever_detected = False
    loss_events = 0
    current_loss_steps = 0
    maximum_loss_steps = 0
    saturation_samples = 0
    contact_linear_impulse = np.zeros(3, dtype=np.float64)
    contact_angular_impulse = np.zeros(3, dtype=np.float64)
    current_command_offset = 0.0
    current_command_offset_velocity = 0.0
    current_desired_force = 0.0

    physics_keys = (
        "time",
        "qpos",
        "qvel",
        "torque",
        "reference_q",
        "reference_dq",
        "reference_ddq",
        "flange_position",
        "flange_rotation",
        "target_center_position",
        "target_grasp_position",
        "target_grasp_rotation",
        "desired_position",
        "desired_rotation",
        "command_normal_offset_m",
        "command_normal_offset_velocity_m_s",
        "desired_normal_force_n",
        "measured_normal_force_n",
        "contact_force_world_n",
        "contact_torque_at_flange_world_nm",
        "contact_count",
        "contact_detected",
        "penetration_m",
        "relative_position_error_m",
        "relative_orientation_error_rad",
        "relative_linear_speed_m_s",
        "arm_angle_rad",
        "base_qpos",
        "base_twist",
        "robot_momentum",
        "total_momentum_world_origin",
        "contact_linear_impulse_ns",
        "contact_angular_impulse_nms",
    )
    task_keys = (
        "task_time",
        "task_success",
        "task_primary_feasible",
        "task_secondary_feasible",
        "task_primary_status",
        "task_secondary_status",
        "task_latency_s",
        "task_minimum_constraint_slack",
        "task_hierarchy_linear_degradation_m_s",
        "task_hierarchy_angular_degradation_rad_s",
        "task_joint_velocity_rad_s",
    )
    physics_log: dict[str, list[Any]] = {key: [] for key in physics_keys}
    task_log: dict[str, list[Any]] = {key: [] for key in task_keys}

    started = time.perf_counter()
    for step in range(physics_steps):
        current_time = float(data.time)
        flange_position = np.asarray(data.site_xpos[flange_site]).copy()
        flange_rotation = np.asarray(data.site_xmat[flange_site]).reshape(3, 3).copy()
        grasp_position = np.asarray(data.site_xpos[target_grasp_site]).copy()
        grasp_rotation = np.asarray(data.site_xmat[target_grasp_site]).reshape(3, 3).copy()
        contact_direction = grasp_rotation[:, 2].copy()
        grasp_linear_velocity, grasp_angular_velocity = _site_twist(
            model, data, target_grasp_site
        )
        current_wrench = aggregate_contact_wrench(
            model,
            data,
            selected_geom_ids=[pad_geom],
            counterpart_geom_ids=[target_geom],
            reference_point_world_m=flange_position,
        )
        measured_force = max(
            0.0,
            -float(current_wrench.force_world_n @ contact_direction),
        )
        was_detected = detected
        if detected:
            detected = measured_force > float(interface_values["contact_release_force_n"])
        else:
            detected = measured_force >= float(interface_values["contact_detection_force_n"])
        if detected:
            ever_detected = True
            current_loss_steps = 0
        elif ever_detected:
            if was_detected:
                loss_events += 1
                if bool(force_values["reset_on_contact_loss"]):
                    admittance.reset()
            current_loss_steps += 1
            maximum_loss_steps = max(maximum_loss_steps, current_loss_steps)

        ramp, ramp_rate = _quintic_ramp(current_time, ramp_duration_s)
        current_desired_force = desired_force_final_n * ramp
        if variant == "rigid":
            current_command_offset = rigid_offset_m * ramp
            current_command_offset_velocity = rigid_offset_m * ramp_rate
        else:
            state = admittance.step(
                desired_force_n=current_desired_force,
                measured_force_n=measured_force,
            )
            current_command_offset = state.offset_m
            current_command_offset_velocity = state.velocity_m_s

        flange_normal_offset = tool_face_recess_m + current_command_offset
        desired_position = grasp_position + contact_direction * flange_normal_offset
        desired_velocity = (
            grasp_linear_velocity
            + contact_direction * current_command_offset_velocity
            + np.cross(grasp_angular_velocity, contact_direction)
            * flange_normal_offset
        )
        desired_rotation = grasp_rotation

        if step % task_stride == 0:
            result = controller.solve_fpmfc(
                data,
                target_position=desired_position,
                target_velocity=desired_velocity,
                target_rotation=desired_rotation,
                target_angular_velocity=grasp_angular_velocity,
                target_arm_angle_rad=float(transfer["terminal_arm_angle_rad"]),
                target_arm_angle_velocity_rad_s=0.0,
            )
            segment_start_velocity = command_velocity.copy()
            command_velocity = result.joint_velocity.copy()
            segment_step = 0
            task_log["task_time"].append(current_time)
            task_log["task_success"].append(result.success)
            task_log["task_primary_feasible"].append(result.primary_feasible)
            task_log["task_secondary_feasible"].append(result.secondary_feasible)
            task_log["task_primary_status"].append(result.primary_status)
            task_log["task_secondary_status"].append(result.secondary_status)
            task_log["task_latency_s"].append(result.full_latency_s)
            task_log["task_minimum_constraint_slack"].append(
                result.minimum_constraint_slack
            )
            task_log["task_hierarchy_linear_degradation_m_s"].append(
                result.hierarchy_linear_degradation_m_s
            )
            task_log["task_hierarchy_angular_degradation_rad_s"].append(
                result.hierarchy_angular_degradation_rad_s
            )
            task_log["task_joint_velocity_rad_s"].append(command_velocity.copy())

        interpolation = float(segment_step + 1) / task_stride
        reference_dq = segment_start_velocity + interpolation * (
            command_velocity - segment_start_velocity
        )
        reference_ddq = (
            command_velocity - segment_start_velocity
        ) / spec.task_period_s
        reference_q = np.clip(
            reference_q + reference_dq * spec.timestep_s,
            controller.joint_lower,
            controller.joint_upper,
        )
        measured_q = np.asarray(data.qpos[qpos_ids])
        reference_q = np.clip(
            reference_q,
            measured_q - run_config.reference_tracking_band_rad,
            measured_q + run_config.reference_tracking_band_rad,
        )
        torque, _desired_acceleration = servo_torque(
            model,
            data,
            dof_ids,
            base_dof_slice,
            qpos_ids,
            reference_q,
            reference_dq,
            reference_ddq,
            spec.torque_limits_nm,
            run_config.servo_natural_frequency_rad_s,
            run_config.servo_acceleration_limit_rad_s2,
            full_mass,
        )
        saturation_samples += int(
            np.count_nonzero(np.abs(torque) >= spec.torque_limits_nm - 1e-9)
        )
        data.ctrl[:] = torque
        mujoco.mj_step(model, data)
        segment_step += 1

        now = float(data.time)
        flange_position = np.asarray(data.site_xpos[flange_site]).copy()
        flange_rotation = np.asarray(data.site_xmat[flange_site]).reshape(3, 3).copy()
        grasp_position = np.asarray(data.site_xpos[target_grasp_site]).copy()
        grasp_rotation = np.asarray(data.site_xmat[target_grasp_site]).reshape(3, 3).copy()
        contact_direction = grasp_rotation[:, 2].copy()
        flange_linear_velocity, _flange_angular_velocity = _site_twist(
            model, data, flange_site
        )
        grasp_linear_velocity, _grasp_angular_velocity = _site_twist(
            model, data, target_grasp_site
        )
        wrench = aggregate_contact_wrench(
            model,
            data,
            selected_geom_ids=[pad_geom],
            counterpart_geom_ids=[target_geom],
            reference_point_world_m=flange_position,
        )
        normal_force = max(0.0, -float(wrench.force_world_n @ contact_direction))
        penetration = maximum_interface_penetration(
            model,
            data,
            selected_geom_ids=[pad_geom],
            counterpart_geom_ids=[target_geom],
        )
        robot_com = np.asarray(data.subtree_com[base_body]).copy()
        torque_about_initial_robot_com = wrench.torque_world_nm + np.cross(
            flange_position - initial_robot_com, wrench.force_world_n
        )
        contact_linear_impulse += wrench.force_world_n * spec.timestep_s
        contact_angular_impulse += (
            torque_about_initial_robot_com * spec.timestep_s
        )
        robot_system_momentum = robot_momentum(model, data, base_body)
        total_momentum = (
            _subtree_momentum_about_world_origin(model, data, base_body)
            + _subtree_momentum_about_world_origin(model, data, target_body)
        )
        current_shape = shape.sample(data).angle_rad
        desired_position_now = (
            grasp_position
            + contact_direction * (tool_face_recess_m + current_command_offset)
        )

        physics_log["time"].append(now)
        physics_log["qpos"].append(np.asarray(data.qpos).copy())
        physics_log["qvel"].append(np.asarray(data.qvel).copy())
        physics_log["torque"].append(torque.copy())
        physics_log["reference_q"].append(reference_q.copy())
        physics_log["reference_dq"].append(reference_dq.copy())
        physics_log["reference_ddq"].append(reference_ddq.copy())
        physics_log["flange_position"].append(flange_position)
        physics_log["flange_rotation"].append(flange_rotation)
        physics_log["target_center_position"].append(
            np.asarray(data.qpos[target_qpos_slice][:3]).copy()
        )
        physics_log["target_grasp_position"].append(grasp_position)
        physics_log["target_grasp_rotation"].append(grasp_rotation)
        physics_log["desired_position"].append(desired_position_now)
        physics_log["desired_rotation"].append(grasp_rotation.copy())
        physics_log["command_normal_offset_m"].append(current_command_offset)
        physics_log["command_normal_offset_velocity_m_s"].append(
            current_command_offset_velocity
        )
        physics_log["desired_normal_force_n"].append(current_desired_force)
        physics_log["measured_normal_force_n"].append(normal_force)
        physics_log["contact_force_world_n"].append(wrench.force_world_n.copy())
        physics_log["contact_torque_at_flange_world_nm"].append(
            wrench.torque_world_nm.copy()
        )
        physics_log["contact_count"].append(wrench.contact_count)
        physics_log["contact_detected"].append(detected)
        physics_log["penetration_m"].append(penetration)
        physics_log["relative_position_error_m"].append(
            float(np.linalg.norm(desired_position_now - flange_position))
        )
        physics_log["relative_orientation_error_rad"].append(
            rotation_distance_rad(grasp_rotation, flange_rotation)
        )
        physics_log["relative_linear_speed_m_s"].append(
            float(np.linalg.norm(flange_linear_velocity - grasp_linear_velocity))
        )
        physics_log["arm_angle_rad"].append(current_shape)
        physics_log["base_qpos"].append(
            np.asarray(data.qpos[base_qpos_slice]).copy()
        )
        physics_log["base_twist"].append(
            np.asarray(data.qvel[base_dof_slice]).copy()
        )
        physics_log["robot_momentum"].append(robot_system_momentum)
        physics_log["total_momentum_world_origin"].append(total_momentum)
        physics_log["contact_linear_impulse_ns"].append(
            contact_linear_impulse.copy()
        )
        physics_log["contact_angular_impulse_nms"].append(
            contact_angular_impulse.copy()
        )

    elapsed_wall_s = time.perf_counter() - started
    arrays = {
        key: np.asarray(value)
        for key, value in {**physics_log, **task_log}.items()
    }
    arrays["initial_qpos"] = initial_qpos
    arrays["initial_qvel"] = initial_qvel
    arrays["initial_base_pose"] = initial_base_pose
    arrays["initial_robot_momentum"] = initial_robot_momentum
    arrays["initial_total_momentum_world_origin"] = initial_total_momentum
    arrays["source_trace_sha256"] = np.asarray(transfer["source_trace_sha256"])
    np.savez_compressed(trace_path, **arrays)

    force = np.asarray(arrays["measured_normal_force_n"], dtype=np.float64)
    desired_force = np.asarray(arrays["desired_normal_force_n"], dtype=np.float64)
    penetration = np.asarray(arrays["penetration_m"], dtype=np.float64)
    task_success_rate = float(np.mean(arrays["task_success"]))
    steady_steps = max(
        1,
        int(
            round(
                float(force_values["steady_evaluation_window_s"])
                / spec.timestep_s
            )
        ),
    )
    steady_force_rmse = float(
        np.sqrt(np.mean((force[-steady_steps:] - desired_force[-steady_steps:]) ** 2))
    )
    base_angular_speed = np.linalg.norm(
        np.asarray(arrays["base_twist"])[:, 3:], axis=1
    )
    robot_momentum_delta = (
        np.asarray(arrays["robot_momentum"]) - initial_robot_momentum[None, :]
    )
    total_momentum_delta = (
        np.asarray(arrays["total_momentum_world_origin"])
        - initial_total_momentum[None, :]
    )
    all_numeric_finite = all(
        np.all(np.isfinite(value))
        for value in arrays.values()
        if np.issubdtype(np.asarray(value).dtype, np.number)
    )
    torque_saturation_fraction = saturation_samples / (7.0 * physics_steps)
    maximum_loss_s = maximum_loss_steps * spec.timestep_s
    common_acceptance = {
        "all_numeric_finite": bool(all_numeric_finite),
        "contact_acquired": bool(ever_detected),
        "peak_contact_force": float(np.max(force))
        <= float(acceptance_values["maximum_peak_contact_force_n"]),
        "maximum_penetration": float(np.max(penetration))
        <= float(acceptance_values["maximum_penetration_m"]),
        "maximum_sustained_contact_loss": maximum_loss_s
        <= float(acceptance_values["maximum_sustained_contact_loss_s"]),
        "task_success_rate": task_success_rate
        >= float(precontact_config["acceptance"]["minimum_task_success_rate"]),
        "torque_saturation_fraction": torque_saturation_fraction
        <= float(
            precontact_config["acceptance"]["maximum_torque_saturation_fraction"]
        ),
        "total_momentum_drift": float(
            np.max(np.linalg.norm(total_momentum_delta, axis=1))
        )
        <= float(precontact_config["acceptance"]["maximum_momentum_delta"]),
    }
    force_tracking_gate = steady_force_rmse <= (
        float(acceptance_values["maximum_steady_force_rmse_fraction"])
        * desired_force_final_n
    )
    acceptance = {
        **common_acceptance,
        "common_passed": bool(all(common_acceptance.values())),
        "steady_force_tracking": (
            None if variant == "rigid" else bool(force_tracking_gate)
        ),
    }
    acceptance["passed"] = bool(
        acceptance["common_passed"]
        and (variant == "rigid" or force_tracking_gate)
    )

    metrics = {
        "material_passport": {
            "origin_skill": "experiment-plan",
            "origin_mode": "run",
            "origin_date": datetime.now(timezone.utc).isoformat(),
            "verification_status": "UNVERIFIED",
            "version_label": "contact_result_v1",
        },
        "experiment_id": contact_config["experiment_id"],
        "scope": "custom_flexiv_physical_target_contact_extension",
        "variant": variant,
        "controller_variant": controller_variant,
        "contact_config_path": str(contact_config_path.relative_to(PROJECT_ROOT)),
        "contact_config_sha256": _sha256(contact_config_path),
        "effective_contact_config_sha256": _mapping_sha256(contact_config),
        "effective_contact_config": contact_config,
        "precontact_config_path": str(precontact_config_path.relative_to(PROJECT_ROOT)),
        "precontact_config_sha256": _sha256(precontact_config_path),
        "effective_controller_config": controller_config_values["controller"],
        "source_handoff": {
            key: value
            for key, value in transfer.items()
            if key
            not in {
                "reference_q",
                "reference_dq",
                "reference_ddq",
                "controller_velocity",
                "controller_acceleration",
            }
        },
        "model_identity": spec.identity(),
        "implementation_identity": contact_implementation_identity(),
        "run_config": asdict(run_config),
        "scene_runtime_overrides": {
            "workspace_obstacle_position_world_m": [10.0, 10.0, 10.0]
        },
        "physics_steps": physics_steps,
        "task_ticks": int(len(arrays["task_time"])),
        "elapsed_wall_s": elapsed_wall_s,
        "qpos_write_count_after_initialization": 0,
        "qvel_write_count_after_initialization": 0,
        "metrics": {
            "peak_normal_force_n": float(np.max(force)),
            "normal_force_impulse_ns": float(np.sum(force) * spec.timestep_s),
            "steady_force_rmse_n": steady_force_rmse,
            "steady_force_rmse_fraction": steady_force_rmse
            / desired_force_final_n,
            "maximum_penetration_m": float(np.max(penetration)),
            "contact_loss_events": int(loss_events),
            "maximum_sustained_contact_loss_s": maximum_loss_s,
            "contact_detected_fraction": float(
                np.mean(np.asarray(arrays["contact_detected"], dtype=np.float64))
            ),
            "final_normal_force_n": float(force[-1]),
            "task_success_rate": task_success_rate,
            "torque_saturation_fraction": torque_saturation_fraction,
            "maximum_base_angular_speed_rad_s": float(
                np.max(base_angular_speed)
            ),
            "rms_base_angular_speed_rad_s": float(
                np.sqrt(np.mean(base_angular_speed**2))
            ),
            "robot_contact_linear_impulse_norm_ns": float(
                np.linalg.norm(contact_linear_impulse)
            ),
            "robot_contact_angular_impulse_about_initial_com_norm_nms": float(
                np.linalg.norm(contact_angular_impulse)
            ),
            "robot_momentum_change_max": float(
                np.max(np.linalg.norm(robot_momentum_delta, axis=1))
            ),
            "total_momentum_drift_max": float(
                np.max(np.linalg.norm(total_momentum_delta, axis=1))
            ),
            "relative_position_error_max_m": float(
                np.max(arrays["relative_position_error_m"])
            ),
            "relative_orientation_error_max_rad": float(
                np.max(arrays["relative_orientation_error_rad"])
            ),
            "relative_linear_speed_max_m_s": float(
                np.max(arrays["relative_linear_speed_m_s"])
            ),
            "maximum_command_normal_offset_m": float(
                np.max(np.abs(arrays["command_normal_offset_m"]))
            ),
        },
        "acceptance": acceptance,
        "trace": str(trace_path.relative_to(PROJECT_ROOT)),
    }
    metrics["trace_sha256"] = _sha256(trace_path)
    metrics_path.write_text(
        json.dumps(metrics, ensure_ascii=False, indent=2, default=_json_default),
        encoding="utf-8",
    )
    return metrics


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--variant", choices=CONTACT_VARIANTS, required=True)
    parser.add_argument("--source-trace", type=Path, default=DEFAULT_SOURCE_TRACE)
    parser.add_argument(
        "--contact-config", type=Path, default=DEFAULT_CONTACT_CONFIG_PATH
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    result = run_contact_experiment(
        variant=args.variant,
        source_trace_path=args.source_trace,
        contact_config_path=args.contact_config,
        output_dir=args.output_dir,
        overwrite=args.overwrite,
    )
    print(json.dumps({"variant": result["variant"], "metrics": result["metrics"], "acceptance": result["acceptance"]}, indent=2))


if __name__ == "__main__":
    main()

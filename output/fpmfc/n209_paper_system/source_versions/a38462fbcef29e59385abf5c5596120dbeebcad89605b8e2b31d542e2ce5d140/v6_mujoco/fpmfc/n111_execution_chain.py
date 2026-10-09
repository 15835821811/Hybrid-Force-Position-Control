"""Read-only reconstruction of the normal execution chain from an S trace."""

from __future__ import annotations

import json
from pathlib import Path

import mujoco
import numpy as np

from ..model import PROJECT_ROOT, site_id
from .contact_config import load_contact_config
from .contact_model import default_contact_model_spec
from .contact_observation import ContactObserver
from .controller import FPMFCControllerConfig
from ..hierarchical_qp import rotation_error_vector_world
from .handoff_contract import sha256


def statistics(values: np.ndarray, mask: np.ndarray) -> dict[str, float | int]:
    selected = np.asarray(values)[mask]
    selected = selected[np.isfinite(selected)]
    return {"count": int(len(selected)), "mean": float(np.mean(selected)),
            "rms": float(np.sqrt(np.mean(selected**2))),
            "maximum_absolute": float(np.max(np.abs(selected)))} if len(selected) else {"count": 0}


def _smooth_saturate_norm(value: np.ndarray, limit: float, transition: float) -> np.ndarray:
    # Use the frozen controller implementation to reproduce its precise rule.
    from .controller import smooth_saturate_norm
    return smooth_saturate_norm(value, limit, transition)


def reconstruct(trace_path: Path, output_dir: Path) -> dict:
    """Recreate task-tick commands and physics-step execution at t_k."""
    trace_path, output_dir = Path(trace_path).resolve(), Path(output_dir).resolve()
    with np.load(trace_path, allow_pickle=False) as z:
        trace = {key: z[key] for key in z.files}
    metrics = json.loads((trace_path.parent / "metrics.json").read_text(encoding="utf-8"))
    cfg = metrics["effective_controller_config"]
    defaults = FPMFCControllerConfig()
    contact = load_contact_config(PROJECT_ROOT / "configs/fpmfc_contact.yaml")
    recess = float(contact["interface"]["tool_pad_face_recess_m"])
    spec = default_contact_model_spec()
    model = spec.compile_model()
    data = mujoco.MjData(model)
    observer = ContactObserver(model, tool_face_recess_m=recess)
    flange_id = site_id(model, "flange_site")
    _, joints = spec.joint_addresses(model)
    base = np.arange(spec.base_slices(model)[1].start, spec.base_slices(model)[1].stop)
    nsteps = len(trace["time"])
    tick_lookup = {int(step): i for i, step in enumerate(trace["task_tick_physics_step"])}
    fields = {key: [] for key in (
        "time_s", "task_tick", "normal_raw_to_limit_m_s", "tangent_raw_to_limit_m_s",
        "angular_raw_to_limit_rad_s", "normal_task_to_hqp_m_s", "tangent_task_to_hqp_m_s",
        "angular_task_to_hqp_rad_s", "normal_homogeneous_base_bias_m_s",
        "tangent_homogeneous_base_bias_m_s", "angular_homogeneous_base_bias_rad_s",
        "joint_hqp_to_reference_rms_rad_s", "joint_reference_to_actual_rms_rad_s",
        "normal_reference_to_actual_m_s", "tangent_reference_to_actual_m_s",
        "angular_reference_to_actual_rad_s", "normal_tool_relative_actual_m_s",
        "normal_tool_relative_desired_m_s", "contact_force_feedback_n",
        "geometric_contact_count", "raw_normal_command_m_s", "limited_normal_command_m_s",
        "hqp_normal_achieved_m_s", "actual_flange_normal_m_s",
    )}
    nan = float("nan")
    for step in range(nsteps):
        data.time = float(trace["feedback_time_s"][step])
        data.qpos[:] = trace["feedback_qpos"][step]
        data.qvel[:] = trace["feedback_qvel"][step]
        data.ctrl[:] = trace["feedback_ctrl_used_for_forward"][step]
        source = step - 1
        data.qacc_warmstart[:] = (trace["qacc_warmstart_used_for_forward"][source]
                                   if source >= 0 else trace["initial_qacc_warmstart"])
        data.mocap_pos[:] = (trace["mocap_pos"][source] if source >= 0 else trace["initial_mocap_pos"])
        data.mocap_quat[:] = (trace["mocap_quat"][source] if source >= 0 else trace["initial_mocap_quat"])
        o = observer.observe(data)
        n = o.contact_normal_world
        jp = np.zeros((3, model.nv)); jr = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, o.forward_data, jp, jr, flange_id)
        actual_flange = np.r_[o.flange_linear_velocity_world_m_s,
                              o.flange_angular_velocity_world_rad_s]
        desired_relative = (n * trace["command_normal_offset_velocity_m_s"][step]
                            + np.cross(o.grasp_angular_velocity_world_rad_s, n)
                            * trace["command_normal_offset_m"][step])
        actual_relative = o.tool_face_linear_velocity_world_m_s - o.grasp_linear_velocity_world_m_s
        relative_error = desired_relative - actual_relative
        reference_joint = trace["reference_dq"][step]
        actual_joint = o.qvel[joints]
        # Reference to measured flange twist uses the same observed base velocity;
        # this isolates the joint tracking contribution. Tool-face relative error
        # above keeps the full base, target, and omega-cross-r effects.
        reference_flange = actual_flange + np.r_[jp[:, joints] @ (reference_joint-actual_joint),
                                                 jr[:, joints] @ (reference_joint-actual_joint)]
        reference_error = reference_flange - actual_flange
        tick = step in tick_lookup
        raw, limited, predicted, bias = (np.full(6, nan) for _ in range(4))
        active_tick = max(index for index in tick_lookup if index <= step)
        hqp_joint_difference = float(np.sqrt(np.mean((
            trace["task_joint_velocity_rad_s"][tick_lookup[active_tick]]-reference_joint)**2)))
        if tick:
            target_position = (o.grasp_position_world_m
                               + n * (recess + trace["command_normal_offset_m"][step]))
            target_velocity = (o.grasp_linear_velocity_world_m_s
                               + n * trace["command_normal_offset_velocity_m_s"][step]
                               + np.cross(o.grasp_angular_velocity_world_rad_s, n)
                               * (recess + trace["command_normal_offset_m"][step]))
            raw_linear = target_velocity + float(cfg["position_gain"]) * (target_position-o.flange_position_world_m)
            magnitude = float(np.linalg.norm(raw_linear))
            limited_linear = raw_linear * min(1.0, float(cfg.get("linear_speed_limit_m_s", defaults.linear_speed_limit_m_s))/magnitude) if magnitude else raw_linear
            raw_angular = (o.grasp_angular_velocity_world_rad_s
                           + float(cfg["orientation_gain"])
                           * rotation_error_vector_world(o.grasp_rotation_world, o.flange_rotation_world))
            limited_angular = _smooth_saturate_norm(
                raw_angular, float(cfg.get("angular_speed_limit_rad_s", defaults.angular_speed_limit_rad_s)),
                float(cfg["angular_saturation_transition_ratio"]),
            )
            raw = np.r_[raw_linear, raw_angular]
            limited = np.r_[limited_linear, limited_angular]
            predicted = trace["task_predicted_flange_twist_world"][tick_lookup[step]]
            bias = np.r_[jp[:, base], jr[:, base]] @ trace["task_model_base_bias"][tick_lookup[step]]
        normal = lambda x: float(n @ x)
        tangent = lambda x: float(np.linalg.norm(x - n*(n@x)))
        scalars = {
            "time_s": o.time_s, "task_tick": tick,
            "normal_raw_to_limit_m_s": normal(raw[:3]-limited[:3]) if tick else nan,
            "tangent_raw_to_limit_m_s": tangent(raw[:3]-limited[:3]) if tick else nan,
            "angular_raw_to_limit_rad_s": float(np.linalg.norm(raw[3:]-limited[3:])) if tick else nan,
            "normal_task_to_hqp_m_s": normal(limited[:3]-predicted[:3]) if tick else nan,
            "tangent_task_to_hqp_m_s": tangent(limited[:3]-predicted[:3]) if tick else nan,
            "angular_task_to_hqp_rad_s": float(np.linalg.norm(limited[3:]-predicted[3:])) if tick else nan,
            "normal_homogeneous_base_bias_m_s": normal(bias[:3]) if tick else nan,
            "tangent_homogeneous_base_bias_m_s": tangent(bias[:3]) if tick else nan,
            "angular_homogeneous_base_bias_rad_s": float(np.linalg.norm(bias[3:])) if tick else nan,
            "joint_hqp_to_reference_rms_rad_s": hqp_joint_difference,
            "joint_reference_to_actual_rms_rad_s": float(np.sqrt(np.mean((reference_joint-actual_joint)**2))),
            "normal_reference_to_actual_m_s": normal(relative_error),
            "tangent_reference_to_actual_m_s": tangent(relative_error),
            "angular_reference_to_actual_rad_s": float(np.linalg.norm(reference_error[3:])),
            "normal_tool_relative_actual_m_s": normal(actual_relative),
            "normal_tool_relative_desired_m_s": normal(desired_relative),
            "contact_force_feedback_n": float(trace["feedback_measured_normal_force_n"][step]),
            "geometric_contact_count": o.contact_count,
            "raw_normal_command_m_s": normal(raw[:3]) if tick else nan,
            "limited_normal_command_m_s": normal(limited[:3]) if tick else nan,
            "hqp_normal_achieved_m_s": normal(predicted[:3]) if tick else nan,
            "actual_flange_normal_m_s": normal(actual_flange[:3]),
        }
        for key in fields:
            fields[key].append(scalars[key])
    arrays = {key: np.asarray(value) for key, value in fields.items()}
    full = np.ones(nsteps, dtype=bool)
    window = np.arange(nsteps) >= nsteps - 100
    if int(np.count_nonzero(window)) != 100:
        raise RuntimeError("original evaluation window does not contain 100 poststep samples")
    report = {
        "schema_version": "n111_execution_chain_v1", "source_trace_sha256": sha256(trace_path),
        "sampling": "t_k pre-step feedback; HQP terms only at 50 task ticks; window selected by corresponding post-step time > 0.8 s",
        "normal_definition": "target grasp local +Z in world; tool-face velocity includes flange omega cross (tool-face minus flange)",
        "normal_reference_to_actual_definition": "desired tool-face minus target-grasp relative velocity, projected on target normal; includes base and target motion",
        "normal_homogeneous_base_bias_definition": "full flange Jacobian base columns times nu_b minus homogeneous base-map times measured joint velocity; offline only",
        "statistics": {key: {"full": statistics(value, full), "last_0p2_s": statistics(value, window)}
                       for key, value in arrays.items() if key not in ("time_s", "task_tick")},
        "contact_switches": int(np.count_nonzero(np.diff(arrays["geometric_contact_count"] > 0))),
        "force_error_lag_correlations": {},
    }
    force_change = np.diff(arrays["contact_force_feedback_n"])
    execution = arrays["normal_reference_to_actual_m_s"][1:]
    for lag in range(-5, 6):
        x, y = (force_change[-lag:], execution[:len(execution)+lag]) if lag < 0 else (force_change[:len(force_change)-lag or None], execution[lag:])
        report["force_error_lag_correlations"][str(lag)] = float(np.corrcoef(x, y)[0, 1]) if np.std(x) and np.std(y) else None
    output_dir.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output_dir / "execution_chain.npz", **arrays)
    (output_dir / "execution_chain_summary.json").write_text(json.dumps(report, indent=2, ensure_ascii=False)+"\n", encoding="utf-8")
    return report

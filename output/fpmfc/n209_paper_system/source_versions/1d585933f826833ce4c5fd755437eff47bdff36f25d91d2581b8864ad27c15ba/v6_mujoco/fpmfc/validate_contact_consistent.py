"""Independent torque replay and synchronized-observation validation for N110."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from ..model import PROJECT_ROOT, geom_id
from .contact import NormalAdmittance
from .contact_config import load_contact_config, normal_admittance_config
from .contact_consistency_metrics import endpoint_force_metrics, hysteresis_loss
from .contact_consistency_provenance import n110_implementation_identity, n110_verification_identity
from .contact_model import default_contact_model_spec
from .contact_observation import ContactObserver
from .run_contact import _quintic_ramp
from .run_contact_consistent import _sha256


NUMERICAL_TOLERANCES = {
    "qpos": 1e-8, "qvel": 1e-8, "site_position_m": 1e-9,
    "site_twist": 1e-8, "force_n": 1e-5, "penetration_m": 1e-8,
    "normal": 1e-9, "same_time_s": 1e-12,
}


def _max_norm(first: np.ndarray, second: np.ndarray) -> float:
    return float(np.max(np.linalg.norm(np.asarray(first)-np.asarray(second), axis=1)))


def validate_consistent_output(output_dir: Path | str) -> dict[str, Any]:
    output = Path(output_dir).resolve()
    metrics_path, trace_path = output / "metrics.json", output / "trace.npz"
    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    config_snapshot = output / "config_snapshot.json"
    config = load_contact_config()
    spec = default_contact_model_spec()
    model = spec.compile_model()
    model.geom_pos[geom_id(model, "workspace_obstacle_0")] = [10.0, 10.0, 10.0]
    observer = ContactObserver(model, tool_face_recess_m=float(config["interface"]["tool_pad_face_recess_m"]))
    data = mujoco.MjData(model)
    with np.load(trace_path, allow_pickle=False) as t:
        required = {
            "initial_qpos", "initial_qvel", "initial_ctrl", "initial_qfrc_applied",
            "initial_xfrc_applied", "initial_qacc_warmstart", "initial_mocap_pos",
            "initial_mocap_quat", "initial_eq_active", "time", "qpos", "qvel",
            "torque", "flange_position", "flange_linear_velocity_world_m_s",
            "flange_angular_velocity_world_rad_s", "target_grasp_position",
            "target_grasp_linear_velocity_world_m_s", "target_grasp_angular_velocity_world_rad_s",
            "contact_normal_world", "measured_normal_force_n", "desired_normal_force_n",
            "feedback_time_s", "feedback_qpos", "feedback_qvel",
            "feedback_ctrl_used_for_forward", "feedback_measured_normal_force_n",
            "contact_count", "penetration_m", "task_tick_physics_step",
        }
        missing = sorted(required-set(t.files))
        if missing:
            raise ValueError(f"missing trace fields: {missing}")
        data.qpos[:] = t["initial_qpos"]
        data.qvel[:] = t["initial_qvel"]
        data.ctrl[:] = t["initial_ctrl"]
        data.qfrc_applied[:] = t["initial_qfrc_applied"]
        data.xfrc_applied[:] = t["initial_xfrc_applied"]
        data.qacc_warmstart[:] = t["initial_qacc_warmstart"]
        data.mocap_pos[:] = t["initial_mocap_pos"]
        data.mocap_quat[:] = t["initial_mocap_quat"]
        data.eq_active[:] = t["initial_eq_active"]
        if "initial_act" in t:
            data.act[:] = t["initial_act"]
        if "initial_plugin_state" in t:
            data.plugin_state[:] = t["initial_plugin_state"]
        data.time = 0.0
        # The first observation forwards a copy. Keep the replay's main
        # warm-start/integrator state exactly as archived before mj_step.
        errors = {key: 0.0 for key in (
            "qpos", "qvel", "flange_position_m", "grasp_position_m",
            "flange_linear_twist", "flange_angular_twist", "grasp_linear_twist",
            "grasp_angular_twist", "contact_normal", "force_n", "penetration_m",
            "feedback_force_n", "feedback_state", "feedback_control", "time_s",
        )}
        contact_count_exact = True
        desired_time_exact = True
        feedback_causal = True
        observed_forces = []
        feedback_forces = []
        for i, torque in enumerate(t["torque"]):
            before = observer.observe(data)
            feedback_forces.append(before.measured_normal_force_n)
            errors["feedback_force_n"] = max(errors["feedback_force_n"], abs(before.measured_normal_force_n-float(t["feedback_measured_normal_force_n"][i])))
            errors["feedback_state"] = max(errors["feedback_state"], float(np.max(np.abs(before.qpos-t["feedback_qpos"][i]))), float(np.max(np.abs(before.qvel-t["feedback_qvel"][i]))))
            errors["feedback_control"] = max(errors["feedback_control"], float(np.max(np.abs(before.ctrl_used_for_forward-t["feedback_ctrl_used_for_forward"][i]))))
            errors["time_s"] = max(errors["time_s"], abs(before.time_s-float(t["feedback_time_s"][i])))
            if i == 0:
                feedback_causal &= np.array_equal(before.ctrl_used_for_forward, t["initial_ctrl"])
            else:
                feedback_causal &= np.array_equal(before.ctrl_used_for_forward, t["torque"][i-1])
                feedback_causal &= abs(before.time_s-float(t["time"][i-1])) <= NUMERICAL_TOLERANCES["same_time_s"]
            # Reproduce the frozen controller/servo forward-call cadence on
            # the replay state. Both calls use only the held prior torque;
            # their solver warm-start updates matter in sustained contact.
            if i % int(round(spec.task_period_s/spec.timestep_s)) == 0:
                mujoco.mj_forward(model, data)
            mujoco.mj_forward(model, data)
            data.ctrl[:] = torque
            mujoco.mj_step(model, data)
            post = observer.observe(data)
            observed_forces.append(post.measured_normal_force_n)
            errors["qpos"] = max(errors["qpos"], float(np.max(np.abs(post.qpos-t["qpos"][i]))))
            errors["qvel"] = max(errors["qvel"], float(np.max(np.abs(post.qvel-t["qvel"][i]))))
            errors["flange_position_m"] = max(errors["flange_position_m"], float(np.linalg.norm(post.flange_position_world_m-t["flange_position"][i])))
            errors["grasp_position_m"] = max(errors["grasp_position_m"], float(np.linalg.norm(post.grasp_position_world_m-t["target_grasp_position"][i])))
            errors["flange_linear_twist"] = max(errors["flange_linear_twist"], float(np.linalg.norm(post.flange_linear_velocity_world_m_s-t["flange_linear_velocity_world_m_s"][i])))
            errors["flange_angular_twist"] = max(errors["flange_angular_twist"], float(np.linalg.norm(post.flange_angular_velocity_world_rad_s-t["flange_angular_velocity_world_rad_s"][i])))
            errors["grasp_linear_twist"] = max(errors["grasp_linear_twist"], float(np.linalg.norm(post.grasp_linear_velocity_world_m_s-t["target_grasp_linear_velocity_world_m_s"][i])))
            errors["grasp_angular_twist"] = max(errors["grasp_angular_twist"], float(np.linalg.norm(post.grasp_angular_velocity_world_rad_s-t["target_grasp_angular_velocity_world_rad_s"][i])))
            errors["contact_normal"] = max(errors["contact_normal"], float(np.linalg.norm(post.contact_normal_world-t["contact_normal_world"][i])))
            errors["force_n"] = max(errors["force_n"], abs(post.measured_normal_force_n-float(t["measured_normal_force_n"][i])))
            errors["penetration_m"] = max(errors["penetration_m"], abs(post.penetration_m-float(t["penetration_m"][i])))
            errors["time_s"] = max(errors["time_s"], abs(post.time_s-float(t["time"][i])))
            contact_count_exact &= post.contact_count == int(t["contact_count"][i])
            ramp = _quintic_ramp(post.time_s, float(config["force_control"]["force_ramp_duration_s"]))[0]
            desired_time_exact &= abs(float(t["desired_normal_force_n"][i]) - float(config["force_control"]["desired_normal_force_n"])*ramp) <= 1e-12
        force = np.asarray(t["measured_normal_force_n"], dtype=float)
        desired = np.asarray(t["desired_normal_force_n"], dtype=float)
        window = int(round(float(config["force_control"]["steady_evaluation_window_s"])/spec.timestep_s))
        recomputed = endpoint_force_metrics(force, desired, spec.timestep_s, window)
        metric_errors = {key: abs(float(metrics["metrics"][key])-value) for key, value in recomputed.items()}
        detected, loss_events, maximum_loss_s = hysteresis_loss(
            force, float(config["interface"]["contact_detection_force_n"]),
            float(config["interface"]["contact_release_force_n"]), spec.timestep_s,
        )
        task_steps = np.asarray(t["task_tick_physics_step"], dtype=int)
        task_schedule = np.array_equal(task_steps, np.arange(0, len(force), int(round(spec.task_period_s/spec.timestep_s))))
        state_count = len(force)
        admittance_reconstruction_max = 0.0
        if metrics["variant"] != "rigid":
            admittance = NormalAdmittance(normal_admittance_config(config, timestep_s=spec.timestep_s))
            active = False
            for i in range(state_count):
                measured = float(t["feedback_measured_normal_force_n"][i])
                prior = active
                active = measured > float(config["interface"]["contact_release_force_n"]) if active else measured >= float(config["interface"]["contact_detection_force_n"])
                if prior and not active and bool(config["force_control"]["reset_on_contact_loss"]):
                    admittance.reset()
                state = admittance.step(
                    desired_force_n=float(t["command_desired_normal_force_n"][i]),
                    measured_force_n=measured,
                )
                admittance_reconstruction_max = max(
                    admittance_reconstruction_max,
                    abs(state.offset_m-float(t["command_normal_offset_m"][i])),
                    abs(state.velocity_m_s-float(t["command_normal_offset_velocity_m_s"][i])),
                )
        checks = {
            "trace_sha256": _sha256(trace_path) == metrics["trace_sha256"],
            "config_snapshot_sha256": _sha256(config_snapshot) == metrics["config_snapshot_sha256"],
            "current_n110_implementation_identity": n110_implementation_identity() == metrics["implementation_identity"],
            "model_identity": spec.identity() == metrics["model_identity"],
            "source_trace_sha256": _sha256(Path(metrics["source_handoff"]["source_trace_path"])) == metrics["source_handoff"]["source_trace_sha256"],
            "qpos_original_replay_tolerance": errors["qpos"] <= NUMERICAL_TOLERANCES["qpos"],
            "qvel_original_replay_tolerance": errors["qvel"] <= NUMERICAL_TOLERANCES["qvel"],
            "site_position_numerical_tolerance": max(errors["flange_position_m"], errors["grasp_position_m"]) <= NUMERICAL_TOLERANCES["site_position_m"],
            "site_twist_numerical_tolerance": max(errors[k] for k in ("flange_linear_twist", "flange_angular_twist", "grasp_linear_twist", "grasp_angular_twist")) <= NUMERICAL_TOLERANCES["site_twist"],
            "normal_numerical_tolerance": errors["contact_normal"] <= NUMERICAL_TOLERANCES["normal"],
            "force_original_replay_tolerance": errors["force_n"] <= NUMERICAL_TOLERANCES["force_n"],
            "penetration_original_replay_tolerance": errors["penetration_m"] <= NUMERICAL_TOLERANCES["penetration_m"],
            "contact_count_exact": bool(contact_count_exact),
            "synchronized_time": errors["time_s"] <= NUMERICAL_TOLERANCES["same_time_s"],
            "desired_force_synchronized_time": bool(desired_time_exact),
            "feedback_uses_previous_control": bool(feedback_causal),
            "feedback_state_and_force_replay": errors["feedback_state"] <= NUMERICAL_TOLERANCES["qvel"] and errors["feedback_force_n"] <= NUMERICAL_TOLERANCES["force_n"] and errors["feedback_control"] == 0.0,
            "fixed_task_schedule": bool(task_schedule) and len(task_steps) == 50,
            "admittance_once_per_physics_step": admittance_reconstruction_max <= 1e-12,
            "metric_recomputation": max(metric_errors.values()) <= 1e-12,
            "contact_loss_recomputed": loss_events == int(metrics["metrics"]["contact_loss_events"]) and abs(maximum_loss_s-float(metrics["metrics"]["maximum_sustained_contact_loss_s"])) <= 1e-12,
            "all_samples_in_force_window": len(force[-window:]) == window,
        }
    validation = {
        "schema_version": "n110_contact_consistency_validation_v1",
        "variant": metrics["variant"], "group": metrics["group"],
        "numerical_tolerances_fixed_before_closed_loop_runs": NUMERICAL_TOLERANCES,
        "checks": checks, "passed": bool(all(checks.values())),
        "maximum_errors": errors, "metric_recomputation_absolute_errors": metric_errors,
        "admittance_state_reconstruction_max_error": admittance_reconstruction_max,
        "physics_steps": state_count, "task_ticks": len(task_steps),
        "trace_sha256": _sha256(trace_path),
        "implementation_identity": n110_implementation_identity(),
        "verification_identity": n110_verification_identity(),
    }
    (output / "validation.json").write_text(json.dumps(validation, ensure_ascii=False, indent=2), encoding="utf-8")
    manifest = {
        "schema_version": "n110_contact_consistency_artifact_manifest_v1",
        "verification_status": "VERIFIED" if validation["passed"] else "FAILED",
        "artifacts": {name: _sha256(output / name) for name in (
            "config_snapshot.json", "trace.npz", "metrics.json", "validation.json",
        )},
    }
    (output / "artifact_manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    return validation


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    result = validate_consistent_output(args.output_dir)
    print(json.dumps({"group": result["group"], "variant": result["variant"],
                      "passed": result["passed"], "checks": result["checks"],
                      "maximum_errors": result["maximum_errors"]}, indent=2))
    if not result["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

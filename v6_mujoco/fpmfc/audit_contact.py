"""Read-only N110 audit of four frozen pre-contact/contact archives.

Only files below --output-dir are written. Archived metrics and traces are
never edited; validators receive copies in a separate replay staging area.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import platform
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from importlib import metadata
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from ..model import PROJECT_ROOT, body_id, default_model_spec, geom_id, site_id
from .contact import NormalAdmittance, aggregate_contact_wrench, maximum_interface_penetration
from .contact_config import load_contact_config, normal_admittance_config
from .contact_model import default_contact_model_spec
from .contact_provenance import contact_implementation_identity
from .provenance import implementation_identity
from .validate_capture import validate_capture_output
from .validate_contact import validate_contact_output


SOURCE = Path("output/fpmfc/precontact/n073_controller_ablation_rk4_formal_planning45/seed_00/full")
CONTACTS = {
    "rigid": Path("output/fpmfc/contact/n102_rigid_authoritative"),
    "admittance": Path("output/fpmfc/contact/n103_admittance_authoritative"),
    "admittance-no-shape": Path("output/fpmfc/contact/n104_admittance_no_shape_authoritative"),
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _save(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")


def _cmd(*arguments: str) -> str:
    result = subprocess.run(arguments, cwd=PROJECT_ROOT, check=True, capture_output=True, text=True)
    return result.stdout.strip()


def _array(value: np.ndarray) -> list[float]:
    return np.asarray(value, dtype=float).tolist()


def _norm(value: np.ndarray) -> float:
    return float(np.linalg.norm(value))


def _npz_is_real(path: Path) -> bool:
    if not path.is_file():
        return False
    with path.open("rb") as stream:
        return stream.read(4) == b"PK\x03\x04"


def preflight(output: Path) -> dict[str, Any]:
    archives = {"precontact": SOURCE, **CONTACTS}
    checks: dict[str, Any] = {}
    for name, relative in archives.items():
        directory = PROJECT_ROOT / relative
        metrics_path, trace_path = directory / "metrics.json", directory / "trace.npz"
        row: dict[str, Any] = {"path": str(directory), "trace_real_npz": _npz_is_real(trace_path)}
        if metrics_path.is_file() and row["trace_real_npz"]:
            metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
            expected = metrics["trace"]["sha256"] if name == "precontact" else metrics["trace_sha256"]
            row.update(trace_sha256=_sha256(trace_path), trace_hash_match=_sha256(trace_path) == expected)
            row["implementation_identity_match"] = metrics["implementation_identity"] == (
                implementation_identity() if name == "precontact" else contact_implementation_identity()
            )
            if name != "precontact":
                stored = Path(metrics["source_handoff"]["source_trace_path"])
                resolved = PROJECT_ROOT / SOURCE / "trace.npz"
                row["stored_source_path"] = str(stored)
                row["source_path_mapping"] = {"archived": str(stored), "local": str(resolved)}
                row["source_hash_match_via_mapping"] = _sha256(resolved) == metrics["source_handoff"]["source_trace_sha256"]
                row["model_identity_match"] = default_contact_model_spec().identity() == metrics["model_identity"]
        checks[name] = row
    result = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "git": {"head": _cmd("git", "rev-parse", "HEAD"), "branch": _cmd("git", "branch", "--show-current"), "status_short": _cmd("git", "status", "--short")},
        "environment": {"platform": platform.platform(), "python": sys.version, "executable": sys.executable,
                        "packages": {p: metadata.version(p) for p in ("numpy", "scipy", "mujoco", "matplotlib", "Pillow", "PyYAML", "pytest")},
                        "dependency_snapshot": _cmd(sys.executable, "-m", "pip", "freeze") if metadata.packages_distributions().get("pip") else _cmd("uv", "pip", "freeze", "--python", sys.executable)},
        "archives": checks,
    }
    _save(output / "environment.json", result)
    return result


def baseline_replay(output: Path) -> dict[str, Any]:
    results: dict[str, Any] = {}
    for name, relative in {"precontact": SOURCE, **CONTACTS}.items():
        original = PROJECT_ROOT / relative
        staged = output / "replay_staging" / name
        staged.mkdir(parents=True, exist_ok=True)
        for filename in ("metrics.json", "trace.npz"):
            shutil.copy2(original / filename, staged / filename)
        try:
            validation = validate_capture_output(staged) if name == "precontact" else validate_contact_output(staged, overwrite=True)
            results[name] = {"status": "PASS" if validation["passed"] else "FAIL", "checks": validation["checks"],
                             "replay": validation.get("replay", validation.get("fresh_torque_replay")),
                             "validation_copy": str(staged / "validation.json")}
        except Exception as exc:
            results[name] = {"status": "BLOCKED", "error": f"{type(exc).__name__}: {exc}"}
    result = {"method": "archived metrics/trace copied byte-for-byte; existing independent validators write only to replay_staging", "results": results}
    _save(output / "baseline_replay_summary.json", result)
    return result


def _site_twist(model: mujoco.MjModel, data: mujoco.MjData, site: int) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    linear = np.zeros((3, model.nv))
    angular = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, linear, angular, site)
    return linear @ data.qvel, angular @ data.qvel, linear, angular


def point_twist(origin_position: np.ndarray, point_position: np.ndarray, origin_linear: np.ndarray, angular: np.ndarray) -> np.ndarray:
    """World-frame rigid-point velocity, including omega cross r."""
    return origin_linear + np.cross(angular, point_position - origin_position)


def velocity_map_residual(full_linear: np.ndarray, full_angular: np.ndarray, qvel: np.ndarray,
                          base_dofs: np.ndarray, arm_dofs: np.ndarray, base_map: np.ndarray) -> dict[str, np.ndarray]:
    """Compare full site velocity with homogeneous Jg qdot and base-bias repair."""
    joint = qvel[arm_dofs]
    bias = qvel[base_dofs] - base_map @ joint
    homogeneous_linear = (full_linear[:, base_dofs] @ base_map + full_linear[:, arm_dofs]) @ joint
    homogeneous_angular = (full_angular[:, base_dofs] @ base_map + full_angular[:, arm_dofs]) @ joint
    actual_linear = full_linear @ qvel
    actual_angular = full_angular @ qvel
    return {"base_bias": bias, "linear_error": actual_linear - homogeneous_linear,
            "angular_error": actual_angular - homogeneous_angular,
            "linear_repaired_error": actual_linear - homogeneous_linear - full_linear[:, base_dofs] @ bias,
            "angular_repaired_error": actual_angular - homogeneous_angular - full_angular[:, base_dofs] @ bias}


def _fresh_state(model: mujoco.MjModel, qpos: np.ndarray, qvel: np.ndarray, torque: np.ndarray, time_s: float) -> mujoco.MjData:
    data = mujoco.MjData(model)
    data.qpos[:] = qpos
    data.qvel[:] = qvel
    data.ctrl[:] = torque
    data.time = time_s
    mujoco.mj_forward(model, data)
    return data


def _summary(values: list[float]) -> dict[str, float | int]:
    a = np.asarray(values, dtype=float)
    return {"count": len(a), "max": float(a.max()) if len(a) else 0.0,
            "rms": float(np.sqrt(np.mean(a*a))) if len(a) else 0.0,
            "max_step": int(np.argmax(a)) if len(a) else None}


def _phase(counts: np.ndarray, index: int) -> str:
    active = counts > 0
    if not active[index]:
        return "contact_loss" if np.any(active[:index]) else "pre_contact"
    if index == 0 or not active[index-1]:
        return "first_contact" if not np.any(active[:index]) else "recontact"
    return "sustained_contact"


def _handoff(output: Path, model: mujoco.MjModel, spec: Any) -> dict[str, Any]:
    source_path = PROJECT_ROOT / SOURCE / "trace.npz"
    contact_path = PROJECT_ROOT / CONTACTS["admittance"] / "trace.npz"
    with np.load(source_path, allow_pickle=False) as source, np.load(contact_path, allow_pickle=False) as contact:
        data = _fresh_state(model, contact["initial_qpos"], contact["initial_qvel"], np.zeros(model.nu), 0.0)
        flange, grasp = site_id(model, "flange_site"), site_id(model, "target_grasp_site")
        fp, gp = data.site_xpos[flange].copy(), data.site_xpos[grasp].copy()
        fr = data.site_xmat[flange].reshape(3, 3).copy()
        fv, fw, _, _ = _site_twist(model, data, flange)
        gv, gw, _, _ = _site_twist(model, data, grasp)
        recess = float(load_contact_config()["interface"]["tool_pad_face_recess_m"])
        tool_point = fp - fr[:, 2] * recess
        tool_v = point_twist(fp, tool_point, fv, fw)
        base_pos, base_dof = spec.base_slices(model)
        joint_pos, joint_dof = spec.joint_addresses(model)
        target_pos, target_dof = spec.target_slices(model)
        source_spec = default_model_spec()
        source_model = source_spec.compile_model()
        source_base_pos, source_base_dof = source_spec.base_slices(source_model)
        source_joint_pos, source_joint_dof = source_spec.joint_addresses(source_model)
        result = {"coordinates": {"positions_and_site_twists": "world", "base_qvel": "MuJoCo free-joint generalized velocity: translational world components, rotational local components", "tool_point": "flange_site minus flange local +Z times face recess"},
                  "source_terminal_time_s": float(source["time"][-1]),
                  "source_terminal_flange_position_world_m": _array(source["flange_position"][-1]),
                  "source_terminal_reference_dq_rad_s": _array(source["reference_dq"][-1]),
                  "source_terminal_reference_ddq_rad_s2": _array(source["reference_ddq"][-1]),
                  "source_terminal_task_history_dq_rad_s": _array(source["task_joint_velocity_rad_s"][-1]),
                  "source_terminal_task_history_ddq_rad_s2": _array(source["task_joint_acceleration_rad_s2"][-1]),
                  "contact_first_logged_reference_q_rad": _array(contact["reference_q"][0]),
                  "contact_first_logged_reference_dq_rad_s": _array(contact["reference_dq"][0]),
                  "contact_first_logged_reference_ddq_rad_s2": _array(contact["reference_ddq"][0]),
                  "reference_q_boundary_jump_rad": _norm(contact["reference_q"][0] - source["reference_q"][-1]),
                  "reference_dq_boundary_jump_rad_s": _norm(contact["reference_dq"][0] - source["reference_dq"][-1]),
                  "reference_ddq_boundary_jump_rad_s2": _norm(contact["reference_ddq"][0] - source["reference_ddq"][-1]),
                  "controller_history_vs_reference_velocity_rad_s": _norm(source["task_joint_velocity_rad_s"][-1] - source["reference_dq"][-1]),
                  "controller_history_vs_reference_acceleration_rad_s2": _norm(source["task_joint_acceleration_rad_s2"][-1] - source["reference_ddq"][-1]),
                  "base_qpos": _array(data.qpos[base_pos]), "base_qvel": _array(data.qvel[base_dof]),
                  "joint_qpos_rad": _array(data.qpos[joint_pos]), "joint_qvel_rad_s": _array(data.qvel[joint_dof]),
                  "target_qpos": _array(data.qpos[target_pos]), "target_qvel": _array(data.qvel[target_dof]),
                  "flange_position_world_m": _array(fp), "flange_linear_velocity_world_m_s": _array(fv),
                  "flange_angular_velocity_world_rad_s": _array(fw),
                  "tool_point_position_world_m": _array(tool_point), "tool_point_linear_velocity_world_m_s": _array(tool_v),
                  "grasp_position_world_m": _array(gp), "grasp_linear_velocity_world_m_s": _array(gv),
                  "grasp_angular_velocity_world_rad_s": _array(gw),
                  "flange_to_grasp_linear_speed_m_s": _norm(fv-gv),
                  "tool_point_to_grasp_linear_speed_m_s": _norm(tool_v-gv),
                  "relative_angular_speed_rad_s": _norm(fw-gw),
                  "source_to_contact_base_qpos_max_abs": float(np.max(np.abs(source["qpos"][-1][source_base_pos] - data.qpos[base_pos]))),
                  "source_to_contact_base_qvel_max_abs": float(np.max(np.abs(source["qvel"][-1][source_base_dof] - data.qvel[base_dof]))),
                  "source_to_contact_joint_qpos_max_abs": float(np.max(np.abs(source["qpos"][-1][source_joint_pos] - data.qpos[joint_pos]))),
                  "source_to_contact_joint_qvel_max_abs": float(np.max(np.abs(source["qvel"][-1][source_joint_dof] - data.qvel[joint_dof])))}
    _save(output / "handoff_audit.json", result)
    return result


def _sampling_and_velocity(output: Path, model: mujoco.MjModel, spec: Any) -> tuple[dict[str, Any], dict[str, Any]]:
    pad, plate = geom_id(model, "gripper_contact_pad"), geom_id(model, "target_contact_plate")
    flange, grasp = site_id(model, "flange_site"), site_id(model, "target_grasp_site")
    base_slice = spec.base_slices(model)[1]
    arm_ids = np.asarray(spec.joint_addresses(model)[1], dtype=int)
    base_ids = np.arange(base_slice.start, base_slice.stop)
    sampling: dict[str, Any] = {}
    velocity: dict[str, Any] = {}
    for name, relative in CONTACTS.items():
        with np.load(PROJECT_ROOT / relative / "trace.npz", allow_pickle=False) as t:
            times = np.asarray(t["time"])
            counts = np.asarray(t["contact_count"])
            sampling_errors = {key: [] for key in ("flange_position_m", "grasp_position_m", "force_n", "penetration_m", "contact_count")}
            velocity_errors = {key: [] for key in ("linear_m_s", "angular_rad_s", "linear_repaired_m_s", "angular_repaired_rad_s", "base_bias_norm")}
            examples = []
            phases: dict[str, dict[str, list[float]]] = {}
            data = mujoco.MjData(model)
            data.qpos[:] = t["initial_qpos"]
            data.qvel[:] = t["initial_qvel"]
            mujoco.mj_forward(model, data)
            replay_qpos_error, replay_qvel_error = 0.0, 0.0
            fresh_forces, fresh_penetrations, fresh_counts = [], [], []
            for i in range(len(times)):
                data.ctrl[:] = t["torque"][i]
                mujoco.mj_step(model, data)
                replay_qpos_error = max(replay_qpos_error, float(np.max(np.abs(data.qpos-t["qpos"][i]))))
                replay_qvel_error = max(replay_qvel_error, float(np.max(np.abs(data.qvel-t["qvel"][i]))))
                fresh = copy.copy(data)
                mujoco.mj_forward(model, fresh)
                position = np.asarray(fresh.site_xpos[flange]).copy()
                normal = np.asarray(fresh.site_xmat[grasp]).reshape(3, 3)[:, 2]
                wrench = aggregate_contact_wrench(model, fresh, selected_geom_ids=[pad], counterpart_geom_ids=[plate], reference_point_world_m=position)
                force = max(0.0, -float(wrench.force_world_n @ normal))
                penetration = maximum_interface_penetration(model, fresh, selected_geom_ids=[pad], counterpart_geom_ids=[plate])
                fresh_forces.append(force)
                fresh_penetrations.append(penetration)
                fresh_counts.append(wrench.contact_count)
                errors = {"flange_position_m": _norm(position-t["flange_position"][i]),
                          "grasp_position_m": _norm(fresh.site_xpos[grasp]-t["target_grasp_position"][i]),
                          "force_n": abs(force-float(t["measured_normal_force_n"][i])),
                          "penetration_m": abs(penetration-float(t["penetration_m"][i])),
                          "contact_count": abs(wrench.contact_count-int(counts[i]))}
                for key, value in errors.items():
                    sampling_errors[key].append(value)
                if any(errors[key] > threshold for key, threshold in (("flange_position_m", 1e-9), ("force_n", 1e-5), ("contact_count", 0))):
                    if len(examples) < 5:
                        examples.append({"step": i, "time_s": float(times[i]), "logged_count": int(counts[i]), "fresh_count": wrench.contact_count,
                                         "logged_force_n": float(t["measured_normal_force_n"][i]), "fresh_force_n": force, "errors": errors})
                mass = np.zeros((model.nv, model.nv))
                mujoco.mj_fullM(model, mass, fresh.qM)
                base_map = -np.linalg.solve(mass[np.ix_(base_ids, base_ids)], mass[np.ix_(base_ids, arm_ids)])
                _, _, linear_jac, angular_jac = _site_twist(model, fresh, flange)
                r = velocity_map_residual(linear_jac, angular_jac, np.asarray(fresh.qvel), base_ids, arm_ids, base_map)
                scalar = {"linear_m_s": _norm(r["linear_error"]), "angular_rad_s": _norm(r["angular_error"]),
                          "linear_repaired_m_s": _norm(r["linear_repaired_error"]),
                          "angular_repaired_rad_s": _norm(r["angular_repaired_error"]),
                          "base_bias_norm": _norm(r["base_bias"])}
                phase = _phase(counts, i)
                phases.setdefault(phase, {key: [] for key in scalar})
                for key, value in scalar.items():
                    velocity_errors[key].append(value)
                    phases[phase][key].append(value)
            sampling[name] = {"time_first_s": float(times[0]), "time_last_s": float(times[-1]),
                              "time_step_max_error_s": float(np.max(np.abs(np.diff(times)-spec.timestep_s))),
                              "replay_qpos_max_abs": replay_qpos_error, "replay_qvel_max_abs": replay_qvel_error,
                              "fresh_poststep_vs_logged": {k: _summary(v) for k, v in sampling_errors.items()},
                              "fresh_recomputed_metrics_descriptive_only": {
                                  "peak_normal_force_n": float(np.max(fresh_forces)),
                                  "normal_force_impulse_ns": float(np.sum(fresh_forces)*spec.timestep_s),
                                  "steady_force_rmse_n": float(np.sqrt(np.mean((np.asarray(fresh_forces)[-100:]-t["desired_normal_force_n"][-100:])**2))),
                                  "maximum_penetration_m": float(np.max(fresh_penetrations)),
                                  "contact_count_disagree_steps": int(np.count_nonzero(np.asarray(fresh_counts) != counts))},
                              "examples": examples,
                              "status": "PASS" if max(sampling_errors["flange_position_m"]) <= 1e-9 and max(sampling_errors["force_n"]) <= 1e-5 and max(sampling_errors["contact_count"]) == 0 else "FAIL"}
            velocity[name] = {"overall": {k: _summary(v) for k, v in velocity_errors.items()},
                              "phases": {phase: {k: _summary(v) for k, v in values.items()} for phase, values in phases.items()}}
    sampling_result = {"method": "independently replay stored torques; copy complete post-mj_step MjData and call mj_forward on copy; original integration history untouched", "variants": sampling}
    velocity_result = {"method": "world site Jacobian times full generalized qvel versus homogeneous Jg qdot; base bias is generalized free-base qvel minus A qdot", "variants": velocity}
    _save(output / "sampling_audit.json", sampling_result)
    _save(output / "velocity_map_audit.json", velocity_result)
    return sampling_result, velocity_result


def _body_angular_momentum_world(model: mujoco.MjModel, data: mujoco.MjData, body: int) -> np.ndarray:
    linear = np.zeros((3, model.nv))
    angular = np.zeros((3, model.nv))
    mujoco.mj_jacBody(model, data, linear, angular, body)
    rotation = np.asarray(data.ximat[body]).reshape(3, 3)
    inertia_world = rotation @ np.diag(model.body_inertia[body]) @ rotation.T
    return inertia_world @ (angular @ data.qvel)


def _admittance(output: Path, config: dict[str, Any], timestep: float, model: mujoco.MjModel) -> dict[str, Any]:
    maximum_offset = float(config["force_control"]["maximum_offset_m"])
    maximum_velocity = float(config["force_control"]["maximum_velocity_m_s"])
    window = int(round(float(config["force_control"]["steady_evaluation_window_s"])/timestep))
    result: dict[str, Any] = {"evaluation_window_s": float(config["force_control"]["steady_evaluation_window_s"]), "variants": {}}
    base_body = body_id(model, "base_link_0")
    for name, relative in CONTACTS.items():
        with np.load(PROJECT_ROOT / relative / "trace.npz", allow_pickle=False) as t:
            force = np.asarray(t["measured_normal_force_n"], dtype=float)
            desired = np.asarray(t["desired_normal_force_n"], dtype=float)
            detected = np.asarray(t["contact_detected"], dtype=bool)
            offset = np.asarray(t["command_normal_offset_m"], dtype=float)
            speed = np.asarray(t["command_normal_offset_velocity_m_s"], dtype=float)
            resets = np.flatnonzero(detected[:-1] & ~detected[1:]) + 1
            predicted_detected = False
            detection_mismatches = []
            for i in range(len(force)):
                lagged_force = force[i-1] if i else 0.0
                predicted_detected = (
                    lagged_force > float(config["interface"]["contact_release_force_n"])
                    if predicted_detected else
                    lagged_force >= float(config["interface"]["contact_detection_force_n"])
                )
                if predicted_detected != detected[i]:
                    detection_mismatches.append(i)
            initial_data = _fresh_state(model, t["initial_qpos"], t["initial_qvel"], np.zeros(model.nu), 0.0)
            final_data = _fresh_state(model, t["qpos"][-1], t["qvel"][-1], t["torque"][-1], float(t["time"][-1]))
            base_angular_momentum_change = _body_angular_momentum_world(model, final_data, base_body) - _body_angular_momentum_world(model, initial_data, base_body)
            reconstructed = NormalAdmittance(normal_admittance_config(config, timestep_s=timestep))
            reconstruction_errors = []
            for i in range(len(force)):
                if i in resets:
                    reconstructed.reset()
                if name != "rigid":
                    state = reconstructed.step(desired_force_n=desired[i], measured_force_n=force[i-1] if i else 0.0)
                    reconstruction_errors.append(max(abs(state.offset_m-offset[i]), abs(state.velocity_m_s-speed[i])))
            row = {"desired_force_final_n": float(desired[-1]), "measured_force_min_n": float(force.min()),
                   "measured_force_max_n": float(force.max()), "mean_force_error_n": float(np.mean(desired-force)),
                   "steady_force_rmse_all_samples_n": float(np.sqrt(np.mean((desired[-window:]-force[-window:])**2))),
                   "steady_window_samples": window, "steady_window_contact_absent_samples": int(np.count_nonzero(~detected[-window:])),
                   "detection_uses_previous_poststep_force": True,
                   "detection_reconstruction_mismatch_steps": detection_mismatches,
                   "contact_loss_onset_indices": resets.tolist(),
                   "admittance_reset_times_s": np.asarray(t["time"])[resets].tolist() if name != "rigid" else [],
                   "admittance_reset_command_jumps_m": (offset[resets]-offset[resets-1]).tolist() if name != "rigid" else [],
                   "admittance_reset_speed_jumps_m_s": (speed[resets]-speed[resets-1]).tolist() if name != "rigid" else [],
                   "offset_limit_occupancy_fraction": float(np.mean(np.abs(offset) >= maximum_offset-1e-12)) if name != "rigid" else None,
                   "velocity_limit_occupancy_fraction": float(np.mean(np.abs(speed) >= maximum_velocity-1e-12)) if name != "rigid" else None,
                   "maximum_reconstructed_state_error": max(reconstruction_errors) if reconstruction_errors else None,
                   "robot_contact_angular_impulse_proxy_nms": _norm(t["contact_angular_impulse_nms"][-1]),
                   "robot_subtree_angular_momentum_change_norm": _norm(t["robot_momentum"][-1, 3:]-t["initial_robot_momentum"][3:]),
                   "base_link_angular_momentum_change_norm_kg_m2_s": _norm(base_angular_momentum_change)}
            result["variants"][name] = row
    _save(output / "admittance_state_audit.json", result)
    return result


def audit(output: Path) -> dict[str, Any]:
    spec = default_contact_model_spec()
    model = spec.compile_model()
    model.geom_pos[geom_id(model, "workspace_obstacle_0")] = [10.0, 10.0, 10.0]
    handoff = _handoff(output, model, spec)
    sampling, velocity = _sampling_and_velocity(output, model, spec)
    admittance = _admittance(output, load_contact_config(), spec.timestep_s, model)
    return {"handoff": handoff, "sampling": sampling, "velocity": velocity, "admittance": admittance}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=PROJECT_ROOT / "output/fpmfc/n110/audit")
    parser.add_argument("--mode", choices=("preflight", "replay", "audit", "all"), default="all")
    args = parser.parse_args()
    output = args.output_dir.resolve()
    if args.mode in ("preflight", "all"):
        preflight(output)
    if args.mode in ("replay", "all"):
        baseline_replay(output)
    if args.mode in ("audit", "all"):
        audit(output)
    print(output)


if __name__ == "__main__":
    main()

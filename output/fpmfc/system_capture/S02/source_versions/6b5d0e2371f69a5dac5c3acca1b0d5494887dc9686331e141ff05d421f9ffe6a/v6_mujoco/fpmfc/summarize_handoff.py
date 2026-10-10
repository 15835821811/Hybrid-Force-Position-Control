"""Read-only paired audit of the frozen N110C results."""

from __future__ import annotations

import json
from pathlib import Path

import mujoco
import numpy as np

from .config import load_fpmfc_config
from .contact_model import default_contact_model_spec
from .handoff_contract import DEFAULT_OUTPUT_ROOT, repo_path, sha256
from .run_contact_consistent import initialize_contact_state


CONDITIONS = ("C0", "C1")
VARIANTS = ("rigid", "admittance", "admittance-no-shape")
INITIAL_FIELDS = (
    "initial_qpos", "initial_qvel", "initial_ctrl", "initial_qacc_warmstart",
    "initial_mocap_pos", "initial_mocap_quat", "initial_robot_momentum",
    "initial_total_momentum_world_origin",
)


def _read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _check_artifacts(directory: Path) -> bool:
    manifest = _read(directory / "artifact_manifest.json")
    return all(sha256(directory / name) == digest for name, digest in manifest["artifacts"].items())


def _max_abs(array: np.ndarray) -> float:
    return float(np.max(np.abs(array))) if array.size else 0.0


def summarize(root: Path = DEFAULT_OUTPUT_ROOT) -> dict:
    """Recompute paired and force-window checks without running a controller."""
    root = Path(root).resolve()
    contract = _read(root / "pairing_manifest.json")
    contract_hash = sha256(root / "pairing_manifest.json")
    sources = {}
    contact = {}
    initial_equal = {}
    all_checks = {}
    for condition in CONDITIONS:
        source_dir = root / "precontact" / condition
        source_metrics = _read(source_dir / "metrics.json")
        source_validation = _read(source_dir / "validation.json")
        preflight = _read(root / "contact" / condition / "preflight.json")
        with np.load(source_dir / "trace.npz", allow_pickle=False) as trace:
            source_initial = {name: trace[name].copy() for name in ("initial_qpos", "initial_qvel")}
            source_final = {name: trace[name][-1].copy() for name in (
                "qpos", "qvel", "momentum", "base_twist", "reference_q", "reference_dq", "reference_ddq",
            )}
            source_history = {
                "reference_q": trace["reference_q"][-1].copy(),
                "reference_dq": trace["reference_dq"][-1].copy(),
                "reference_ddq": trace["reference_ddq"][-1].copy(),
                "controller_velocity": trace["task_joint_velocity_rad_s"][-1].copy(),
                "controller_acceleration": trace["task_joint_acceleration_rad_s2"][-1].copy(),
            }
            source_step_count = len(trace["time"])
        contact_spec = default_contact_model_spec()
        contact_model = contact_spec.compile_model()
        transfer = initialize_contact_state(
            source_trace_path=source_dir / "trace.npz",
            precontact_config=load_fpmfc_config(repo_path(contract["precontact_config_path"])),
            contact_spec=contact_spec,
            contact_model=contact_model,
            contact_data=mujoco.MjData(contact_model),
        )
        history_checks = {
            name: np.array_equal(source_history[name], transfer[name])
            for name in source_history
        }
        sources[condition] = {
            "metrics": source_metrics["metrics"],
            "acceptance": source_metrics["acceptance"],
            "replay_passed": source_validation["replay_passed"],
            "physics_steps": source_step_count,
            "trace_sha256": sha256(source_dir / "trace.npz"),
            "artifact_hashes_valid": _check_artifacts(source_dir),
            "source_controller_history_transfer_checks": history_checks,
            "initial_state": {key: value.tolist() for key, value in source_initial.items()},
            "terminal_state": {key: value.tolist() for key, value in source_final.items()},
            "contact_preflight": {
                "passed": preflight["passed"],
                "state_transfer_checks": {key: value for key, value in preflight["checks"].items()
                                          if key.endswith("preserved")},
                "initial_contact_count": preflight["initial_contact_count"],
                "initial_contact_force_n": preflight["initial_contact_force_n"],
                "initial_penetration_m": preflight["initial_penetration_m"],
                "target_absolute_time_s": preflight["target_absolute_time_s"],
                "contact_local_time_s": preflight["contact_local_time_s"],
            },
        }
        contact[condition] = {}
        initial_by_variant = {}
        for variant in VARIANTS:
            directory = root / "contact" / condition / variant
            metrics = _read(directory / "metrics.json")
            validation = _read(directory / "validation.json")
            with np.load(directory / "trace.npz", allow_pickle=False) as trace:
                initial_by_variant[variant] = {name: trace[name].copy() for name in INITIAL_FIELDS}
                force = np.asarray(trace["measured_normal_force_n"], dtype=float)
                desired = np.asarray(trace["desired_normal_force_n"], dtype=float)
                detected = np.asarray(trace["contact_detected"], dtype=bool)
                times = np.asarray(trace["time"], dtype=float)
                window_rmse = float(np.sqrt(np.mean((force[-100:] - desired[-100:]) ** 2)))
                first_detected = float(times[np.flatnonzero(detected)[0]]) if detected.any() else None
                trace_checks = {
                    "fixed_500_sample_grid": len(force) == 500 and len(trace["task_time"]) == 50,
                    "force_rmse_all_last_100_samples": abs(window_rmse - metrics["metrics"]["steady_force_rmse_n"]) <= 1e-12,
                    "normal_impulse_all_500_samples": abs(float(np.sum(force) * 0.002) - metrics["metrics"]["normal_force_impulse_ns"]) <= 1e-12,
                    "first_detected_contact_matches": first_detected == metrics["metrics"]["first_contact_time_local_s"],
                    "contact_detection_matches": bool(detected.any()) == metrics["metrics"]["ever_contact"],
                    "no_robot_state_overwrite": metrics["qpos_write_count_after_initialization"] == 0
                    and metrics["qvel_write_count_after_initialization"] == 0,
                }
            contact[condition][variant] = {
                "metrics": metrics["metrics"],
                "acceptance": metrics["acceptance"],
                "handoff_command_jumps": metrics["handoff_command_jumps"],
                "independent_torque_replay_passed": validation["passed"],
                "replay_maximum_errors": validation["maximum_errors"],
                "trace_checks": trace_checks,
                "artifact_hashes_valid": _check_artifacts(directory),
                "trace_sha256": sha256(directory / "trace.npz"),
            }
            all_checks[f"{condition}_{variant}_trace"] = all(trace_checks.values())
            all_checks[f"{condition}_{variant}_replay"] = validation["passed"]
            all_checks[f"{condition}_{variant}_artifact_hashes"] = contact[condition][variant]["artifact_hashes_valid"]
        reference = initial_by_variant["rigid"]
        initial_equal[condition] = {
            variant: {name: _max_abs(values[name] - reference[name]) for name in INITIAL_FIELDS}
            for variant, values in initial_by_variant.items()
        }
        all_checks[f"{condition}_same_initial_state_across_variants"] = all(
            difference == 0.0 for fields in initial_equal[condition].values() for difference in fields.values()
        )
        all_checks[f"{condition}_precontact"] = (
            source_metrics["acceptance"]["handoff_passed"] and source_validation["replay_passed"]
            and sources[condition]["artifact_hashes_valid"] and preflight["passed"]
            and all(history_checks.values())
        )
    a = sources["C0"]["terminal_state"]
    b = sources["C1"]["terminal_state"]
    qpos_diff = np.asarray(b["qpos"]) - np.asarray(a["qpos"])
    qvel_diff = np.asarray(b["qvel"]) - np.asarray(a["qvel"])
    initial_difference = {
        key: _max_abs(np.asarray(sources["C1"]["initial_state"][key])
                      - np.asarray(sources["C0"]["initial_state"][key]))
        for key in ("initial_qpos", "initial_qvel")
    }
    all_checks["common_precontact_initial_state"] = all(value == 0.0 for value in initial_difference.values())
    all_checks["C1_relative_gate_passes"] = (
        sources["C1"]["metrics"]["terminal_tool_face_relative_linear_speed_m_s"]
        <= contract["handoff_gates"]["C1_tool_face_relative_linear_speed_m_s"]
        and sources["C1"]["metrics"]["terminal_relative_angular_speed_rad_s"]
        <= contract["handoff_gates"]["C1_relative_angular_speed_rad_s"]
    )
    all_checks["C1_exceeds_old_absolute_zero_speed_gate"] = (
        sources["C1"]["metrics"]["terminal_absolute_linear_speed_m_s"]
        > contract["handoff_gates"]["C0_absolute_linear_speed_m_s"]
        and sources["C1"]["metrics"]["terminal_absolute_angular_speed_rad_s"]
        > contract["handoff_gates"]["C0_absolute_angular_speed_rad_s"]
    )
    deltas = {
        variant: {key: contact["C1"][variant]["metrics"][key] - contact["C0"][variant]["metrics"][key]
                  for key in ("peak_normal_force_n", "normal_force_impulse_ns", "steady_force_rmse_n",
                              "maximum_penetration_m", "contact_loss_events", "contact_detected_fraction")}
        for variant in VARIANTS
    }
    result = {
        "schema_version": "n110c_paired_summary_v1",
        "pairing_manifest_sha256": contract_hash,
        "runtime_identity_sha256": contract["n110c_implementation_identity"]["composite_sha256"],
        "run_count": {"precontact_dynamics": 2, "contact_dynamics": 6},
        "conditions": sources,
        "contact": contact,
        "same_condition_initial_state_max_abs_differences": initial_equal,
        "between_condition_initial_state_max_abs_differences": initial_difference,
        "between_condition_terminal_state": {
            "base_position_max_abs_m": _max_abs(qpos_diff[:3]),
            "base_orientation_quaternion_max_abs": _max_abs(qpos_diff[3:7]),
            "joint_position_max_abs_rad": _max_abs(qpos_diff[7:]),
            "base_linear_velocity_max_abs_m_s": _max_abs(qvel_diff[:3]),
            "base_angular_velocity_max_abs_rad_s": _max_abs(qvel_diff[3:6]),
            "joint_velocity_max_abs_rad_s": _max_abs(qvel_diff[6:]),
            "robot_momentum_vector_difference_norm": float(np.linalg.norm(
                np.asarray(b["momentum"]) - np.asarray(a["momentum"]))),
        },
        "C1_minus_C0_contact_metric_deltas": deltas,
        "checks": all_checks,
        "engineering_audit_passed": all(all_checks.values()),
    }
    return result


def main() -> None:
    result = summarize()
    path = DEFAULT_OUTPUT_ROOT / "paired_summary.json"
    path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"engineering_audit_passed": result["engineering_audit_passed"],
                      "checks": result["checks"]}, indent=2))
    if not result["engineering_audit_passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

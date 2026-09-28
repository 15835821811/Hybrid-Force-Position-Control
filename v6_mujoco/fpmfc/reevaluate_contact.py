"""R group: replay archived torques and re-evaluate synchronized observations.

No controller runs here. Original archives remain immutable, and endpoint
rectangle force integration is kept identical to the historical metric rule.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from ..model import PROJECT_ROOT, body_id, geom_id
from .audit_contact import CONTACTS, _npz_is_real, _save, _sha256
from .contact_config import load_contact_config
from .contact_consistency_metrics import endpoint_force_metrics, hysteresis_loss
from .contact_consistency_provenance import n110_implementation_identity, n110_verification_identity
from .contact_model import default_contact_model_spec
from .contact_observation import ContactObserver, observation_arrays
from .run_contact import _quintic_ramp


def reevaluate_archive(variant: str, output_dir: Path | str, *, overwrite: bool = False) -> dict[str, Any]:
    if variant not in CONTACTS:
        raise ValueError(f"unknown variant: {variant}")
    archive_dir = PROJECT_ROOT / CONTACTS[variant]
    trace_path = archive_dir / "trace.npz"
    if not _npz_is_real(trace_path):
        raise ValueError(f"not a hydrated NPZ: {trace_path}")
    archive_metrics = json.loads((archive_dir / "metrics.json").read_text(encoding="utf-8"))
    if _sha256(trace_path) != archive_metrics["trace_sha256"]:
        raise ValueError("archived trace hash mismatch")
    output = Path(output_dir).resolve()
    if output.exists() and not overwrite:
        raise FileExistsError(output)
    output.mkdir(parents=True, exist_ok=overwrite)

    config = load_contact_config()
    spec = default_contact_model_spec()
    model = spec.compile_model()
    model.geom_pos[geom_id(model, "workspace_obstacle_0")] = [10.0, 10.0, 10.0]
    observer = ContactObserver(model, tool_face_recess_m=float(config["interface"]["tool_pad_face_recess_m"]))
    data = mujoco.MjData(model)
    base_body = body_id(model, "base_link_0")
    records: dict[str, list[Any]] = {}
    angular_impulse = np.zeros(3)
    linear_impulse = np.zeros(3)
    max_qpos_error = max_qvel_error = 0.0

    with np.load(trace_path, allow_pickle=False) as archived:
        initial_qpos = np.asarray(archived["initial_qpos"]).copy()
        initial_qvel = np.asarray(archived["initial_qvel"]).copy()
        data.qpos[:] = initial_qpos
        data.qvel[:] = initial_qvel
        data.ctrl[:] = 0.0
        mujoco.mj_forward(model, data)
        initial_robot_com = np.asarray(data.subtree_com[base_body]).copy()
        historical_force = np.asarray(archived["measured_normal_force_n"]).copy()
        historical_desired = np.asarray(archived["desired_normal_force_n"]).copy()
        historical_time = np.asarray(archived["time"]).copy()
        torques = np.asarray(archived["torque"]).copy()
        for index, torque in enumerate(torques):
            data.ctrl[:] = torque
            mujoco.mj_step(model, data)
            max_qpos_error = max(max_qpos_error, float(np.max(np.abs(data.qpos-archived["qpos"][index]))))
            max_qvel_error = max(max_qvel_error, float(np.max(np.abs(data.qvel-archived["qvel"][index]))))
            observed = observer.observe(data)
            for key, value in observation_arrays(observed).items():
                records.setdefault(key, []).append(np.asarray(value).copy())
            torque_initial_com = observed.contact_torque_at_flange_world_nm + np.cross(
                observed.flange_position_world_m - initial_robot_com,
                observed.contact_force_world_n,
            )
            angular_impulse += torque_initial_com * spec.timestep_s
            linear_impulse += observed.contact_force_world_n * spec.timestep_s

    arrays = {key: np.asarray(value) for key, value in records.items()}
    times = np.asarray(arrays["time"], dtype=float)
    force = np.asarray(arrays["measured_normal_force_n"], dtype=float)
    desired = np.asarray([
        float(config["force_control"]["desired_normal_force_n"])
        * _quintic_ramp(float(t), float(config["force_control"]["force_ramp_duration_s"]))[0]
        for t in times
    ])
    window = int(round(float(config["force_control"]["steady_evaluation_window_s"]) / spec.timestep_s))
    detected, loss_events, maximum_loss_s = hysteresis_loss(
        force,
        float(config["interface"]["contact_detection_force_n"]),
        float(config["interface"]["contact_release_force_n"]),
        spec.timestep_s,
    )
    arrays.update(
        torque=torques, initial_qpos=initial_qpos, initial_qvel=initial_qvel,
        desired_normal_force_n=desired,
        historical_desired_normal_force_n=historical_desired,
        historical_measured_normal_force_n=historical_force,
        detected_at_sample=detected,
        robot_contact_linear_impulse_world_ns=np.cumsum(arrays["contact_force_world_n"], axis=0) * spec.timestep_s,
    )
    np.savez_compressed(output / "trace.npz", **arrays)
    new_metric = endpoint_force_metrics(force, desired, spec.timestep_s, window)
    old_metric = {key: float(archive_metrics["metrics"][key]) for key in new_metric}
    same_desired_metric = endpoint_force_metrics(force, historical_desired, spec.timestep_s, window)
    metrics = {
        "schema_version": "n110_R_synchronized_archive_reevaluation_v1",
        "group": "R", "variant": variant,
        "source_trace_path": str(trace_path), "source_trace_sha256": _sha256(trace_path),
        "source_metrics_sha256": _sha256(archive_dir / "metrics.json"),
        "source_implementation_identity": archive_metrics["implementation_identity"],
        "n110_implementation_identity": n110_implementation_identity(),
        "n110_verification_identity": n110_verification_identity(),
        "model_identity": spec.identity(),
        "observation_contract": observer.contract(),
        "integration_rule": "right-endpoint rectangle, sum(F(t_i) * 0.002 s); this is not RK4 internal impulse",
        "force_frame": "world contact wrench projected onto synchronized grasp +Z; negative sign gives compressive positive",
        "angular_impulse_reference_point": "initial robot center of mass, world frame, from synchronized contact wrench",
        "force_evaluation_window": {"last_samples": window, "duration_s": window * spec.timestep_s,
                                     "all_samples_included": True, "contact_absent_samples": int(np.count_nonzero(~detected[-window:]))},
        "historical_metrics": old_metric,
        "synchronized_force_with_historical_desired": same_desired_metric,
        "synchronized_metrics": {
            **new_metric,
            "maximum_penetration_m": float(np.max(arrays["penetration_m"])),
            "contact_loss_events": loss_events,
            "maximum_sustained_contact_loss_s": maximum_loss_s,
            "relative_linear_speed_max_m_s": float(np.max(arrays["relative_linear_speed_m_s"])),
            "relative_angular_speed_max_rad_s": float(np.max(arrays["relative_angular_speed_rad_s"])),
            "robot_contact_linear_impulse_norm_ns": float(np.linalg.norm(linear_impulse)),
            "robot_contact_angular_impulse_about_initial_com_norm_nms": float(np.linalg.norm(angular_impulse)),
        },
        "measurement_change": {key: new_metric[key]-old_metric[key] for key in new_metric},
        "trace_sha256": _sha256(output / "trace.npz"),
        "replay_qpos_max_abs": max_qpos_error,
        "replay_qvel_max_abs": max_qvel_error,
        "legacy_replay_tolerance_preserved": {"qpos": 1e-8, "qvel": 1e-8},
        "replay_passed": max_qpos_error <= 1e-8 and max_qvel_error <= 1e-8,
    }
    _save(output / "metrics.json", metrics)
    _save(output / "validation.json", {
        "schema_version": "n110_R_replay_validation_v1",
        "archive_trace_sha256_match": True,
        "archive_qpos_replay_error": max_qpos_error,
        "archive_qvel_replay_error": max_qvel_error,
        "original_tolerances": {"qpos": 1e-8, "qvel": 1e-8},
        "passed": metrics["replay_passed"],
    })
    return metrics


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--variant", choices=CONTACTS, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    result = reevaluate_archive(args.variant, args.output_dir, overwrite=args.overwrite)
    print(json.dumps({"group": result["group"], "variant": result["variant"],
                      "replay_passed": result["replay_passed"],
                      "historical": result["historical_metrics"],
                      "synchronized": result["synchronized_metrics"]}, indent=2))


if __name__ == "__main__":
    main()

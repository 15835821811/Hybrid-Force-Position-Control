"""Run the migrated free-floating Flexiv environment and write replayable traces."""

from __future__ import annotations

import argparse
import hashlib
import json
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from .collision import build_collision_pairs, minimum_signed_distance
from .hierarchical_qp import HierarchicalVelocityQP, QPConfig, rotation_error_vector_world
from .irregular_waypoints import build_target
from .model import PROJECT_ROOT, body_id, default_model_spec, geom_id, site_id


@dataclass(frozen=True)
class RunConfig:
    duration_s: float = 25.5
    scenario_count: int = 5
    seed: int = 20260801
    seed_stride: int = 104729
    reference_tracking_band_rad: float = 0.012
    servo_natural_frequency_rad_s: float = 34.0
    servo_acceleration_limit_rad_s2: float = 70.0


def _jsonable(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    raise TypeError(type(value).__name__)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def robot_momentum(model: mujoco.MjModel, data: mujoco.MjData, base_body: int) -> np.ndarray:
    mujoco.mj_subtreeVel(model, data)
    linear = float(model.body_subtreemass[base_body]) * np.asarray(data.subtree_linvel[base_body])
    angular = np.asarray(data.subtree_angmom[base_body])
    return np.concatenate((linear, angular)).copy()


def servo_torque(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    dof_ids: np.ndarray,
    base_dof_slice: slice,
    qpos_ids: np.ndarray,
    reference_q: np.ndarray,
    reference_dq: np.ndarray,
    feedforward_ddq: np.ndarray,
    torque_limits: np.ndarray,
    natural_frequency: float,
    acceleration_limit: float,
    full_mass: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Eliminate six unactuated base accelerations, then apply one torque law."""

    mujoco.mj_forward(model, data)
    mujoco.mj_fullM(model, full_mass, data.qM)
    desired = (
        feedforward_ddq
        + natural_frequency**2 * (reference_q - np.asarray(data.qpos[qpos_ids]))
        + 2.0 * natural_frequency * (reference_dq - np.asarray(data.qvel[dof_ids]))
    )
    desired = np.clip(desired, -acceleration_limit, acceleration_limit)
    base = np.arange(base_dof_slice.start, base_dof_slice.stop)
    mass_bb = full_mass[np.ix_(base, base)]
    mass_ba = full_mass[np.ix_(base, dof_ids)]
    base_acceleration = -np.linalg.solve(
        mass_bb,
        mass_ba @ desired + np.asarray(data.qfrc_bias[base]) - np.asarray(data.qfrc_passive[base]),
    )
    required = (
        full_mass[np.ix_(dof_ids, base)] @ base_acceleration
        + full_mass[np.ix_(dof_ids, dof_ids)] @ desired
        + np.asarray(data.qfrc_bias[dof_ids])
        - np.asarray(data.qfrc_passive[dof_ids])
    )
    return np.clip(required, -torque_limits, torque_limits), desired


def _quaternion_angle(reference: np.ndarray, values: np.ndarray) -> np.ndarray:
    dots = np.abs(np.sum(values * reference[None, :], axis=1))
    return 2.0 * np.arccos(np.clip(dots, -1.0, 1.0))


def _scenario_obstacle_position(seed: int, index: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    side = -1.0 if index % 2 else 1.0
    return np.asarray([
        0.630 + rng.uniform(-0.015, 0.015),
        -0.110 + side * (0.400 + rng.uniform(-0.012, 0.012)),
        0.520 + rng.uniform(-0.030, 0.030),
    ])


def run_scenario(config: RunConfig, qp_config: QPConfig, scenario_index: int, trace_dir: Path) -> dict[str, Any]:
    spec = default_model_spec()
    model = spec.compile_model()
    scenario_seed = config.seed + config.seed_stride * scenario_index
    obstacle_position = _scenario_obstacle_position(scenario_seed, scenario_index)
    obstacle_geom = geom_id(model, "workspace_obstacle_0")
    model.geom_pos[obstacle_geom] = obstacle_position
    # Contacts are disabled: safety must come from the signed-distance constraints.
    model.geom_contype[:] = 0
    model.geom_conaffinity[:] = 0
    data = mujoco.MjData(model)
    spec.reset_home(model, data)
    pairs = build_collision_pairs(model)
    qp = HierarchicalVelocityQP(spec, model, pairs, qp_config)
    qpos_ids, dof_ids = spec.joint_addresses(model)
    base_qpos_slice, base_dof_slice = spec.base_slices(model)
    flange = site_id(model, "flange_site")
    base_body = body_id(model, "base_link_0")
    target = build_target(data.site_xpos[flange], data.site_xmat[flange].reshape(3, 3), scenario_seed)
    initial_qpos = np.asarray(data.qpos).copy()
    initial_qvel = np.asarray(data.qvel).copy()
    initial_momentum = robot_momentum(model, data, base_body)
    task_stride = int(round(spec.task_period_s / spec.timestep_s))
    steps = int(round(config.duration_s / spec.timestep_s))
    reference_q = spec.home_joint_position.copy()
    segment_start_velocity = np.zeros(7)
    command_velocity = np.zeros(7)
    segment_step = 0
    full_mass = np.zeros((model.nv, model.nv))
    log: dict[str, list[Any]] = {key: [] for key in (
        "time", "qpos", "qvel", "torque", "reference_q", "reference_dq",
        "flange_position", "flange_rotation", "target_position", "position_error",
        "orientation_error_rad", "base_qpos", "momentum", "minimum_clearance",
    )}
    task_log: dict[str, list[Any]] = {key: [] for key in (
        "task_time", "task_success", "task_iterations", "task_full_latency",
        "task_solver_latency", "task_active_clearance", "task_binding_clearance",
        "task_minimum_queried_clearance", "task_minimum_slack", "task_reaction_residual",
    )}

    for step in range(steps):
        current_time = float(data.time)
        if step % task_stride == 0:
            target_position, target_velocity = target.sample(current_time)
            result = qp.solve(
                data,
                target_position=target_position,
                target_velocity=target_velocity,
                target_rotation=target.target_rotation_world,
                target_angular_velocity=np.zeros(3),
            )
            segment_start_velocity = command_velocity.copy()
            command_velocity = result.joint_velocity.copy()
            segment_step = 0
            data.mocap_pos[0] = target_position
            task_log["task_time"].append(current_time)
            task_log["task_success"].append(result.success)
            task_log["task_iterations"].append(result.iterations)
            task_log["task_full_latency"].append(result.full_latency_s)
            task_log["task_solver_latency"].append(result.solver_latency_s)
            task_log["task_active_clearance"].append(result.active_clearance_constraints)
            task_log["task_binding_clearance"].append(result.binding_clearance_constraints)
            task_log["task_minimum_queried_clearance"].append(result.minimum_queried_clearance_m)
            task_log["task_minimum_slack"].append(result.minimum_constraint_slack)
            task_log["task_reaction_residual"].append(result.reaction_momentum_residual_norm)

        interpolation = float(segment_step + 1) / task_stride
        reference_dq = segment_start_velocity + interpolation * (command_velocity - segment_start_velocity)
        feedforward_ddq = (command_velocity - segment_start_velocity) / spec.task_period_s
        reference_q = np.clip(reference_q + reference_dq * spec.timestep_s, qp.joint_lower, qp.joint_upper)
        measured_q = np.asarray(data.qpos[qpos_ids])
        reference_q = np.clip(reference_q, measured_q - config.reference_tracking_band_rad, measured_q + config.reference_tracking_band_rad)
        torque, _desired = servo_torque(
            model, data, dof_ids, base_dof_slice, qpos_ids,
            reference_q, reference_dq, feedforward_ddq, spec.torque_limits_nm,
            config.servo_natural_frequency_rad_s, config.servo_acceleration_limit_rad_s2,
            full_mass,
        )
        data.ctrl[:] = torque
        mujoco.mj_step(model, data)
        segment_step += 1
        target_position, _ = target.sample(float(data.time))
        flange_position = np.asarray(data.site_xpos[flange]).copy()
        flange_rotation = np.asarray(data.site_xmat[flange]).reshape(3, 3).copy()
        clearance, _pair_name = minimum_signed_distance(model, data, pairs)
        log["time"].append(float(data.time))
        log["qpos"].append(np.asarray(data.qpos).copy())
        log["qvel"].append(np.asarray(data.qvel).copy())
        log["torque"].append(torque.copy())
        log["reference_q"].append(reference_q.copy())
        log["reference_dq"].append(reference_dq.copy())
        log["flange_position"].append(flange_position)
        log["flange_rotation"].append(flange_rotation)
        log["target_position"].append(target_position)
        log["position_error"].append(float(np.linalg.norm(target_position - flange_position)))
        log["orientation_error_rad"].append(float(np.linalg.norm(rotation_error_vector_world(target.target_rotation_world, flange_rotation))))
        log["base_qpos"].append(np.asarray(data.qpos[base_qpos_slice]).copy())
        log["momentum"].append(robot_momentum(model, data, base_body))
        log["minimum_clearance"].append(clearance)

    arrays = {key: np.asarray(value) for key, value in {**log, **task_log}.items()}
    arrays["initial_qpos"] = initial_qpos
    arrays["initial_qvel"] = initial_qvel
    arrays["obstacle_position"] = obstacle_position
    arrays["scenario_seed"] = np.asarray(scenario_seed, dtype=np.int64)
    trace_dir.mkdir(parents=True, exist_ok=True)
    trace_path = (trace_dir / f"flexiv_mujoco_scenario_{scenario_index:02d}.npz").resolve()
    np.savez_compressed(trace_path, **arrays)

    position_errors = arrays["position_error"]
    base_poses = arrays["base_qpos"]
    momentum_delta = arrays["momentum"] - initial_momentum[None, :]
    result = {
        "scenario_id": f"flexiv_mujoco_scenario_{scenario_index:02d}",
        "seed": scenario_seed,
        "obstacle_position_m": obstacle_position,
        "target_contract": target.to_dict(),
        "trace": str(trace_path.relative_to(PROJECT_ROOT.resolve())),
        "trace_sha256": _sha256(trace_path),
        "physics_steps": steps,
        "task_ticks": len(arrays["task_time"]),
        "qp_success_rate": float(np.mean(arrays["task_success"])),
        "tracking_rms_m": float(np.sqrt(np.mean(position_errors**2))),
        "tracking_max_m": float(np.max(position_errors)),
        "orientation_max_deg": float(np.rad2deg(np.max(arrays["orientation_error_rad"]))),
        "minimum_signed_clearance_m": float(np.min(arrays["minimum_clearance"])),
        "base_translation_drift_m": float(np.max(np.linalg.norm(base_poses[:, :3] - initial_qpos[None, :3], axis=1))),
        "base_orientation_drift_deg": float(np.rad2deg(np.max(_quaternion_angle(initial_qpos[3:7], base_poses[:, 3:7])))),
        "momentum_residual_max": float(np.max(np.linalg.norm(momentum_delta, axis=1))),
        "qp_latency_p99_ms": float(1000.0 * np.percentile(arrays["task_full_latency"], 99.0)),
        "torque_peak_nm": np.max(np.abs(arrays["torque"]), axis=0),
        "qpos_write_count_after_initialization": 0,
        "qvel_write_count_after_initialization": 0,
    }
    return result


def run_suite(config: RunConfig, output_dir: Path) -> dict[str, Any]:
    if config.scenario_count < 1 or config.duration_s <= 0.0:
        raise ValueError("scenario count and duration must be positive")
    output_dir = Path(output_dir).resolve()
    trace_dir = output_dir / "traces"
    qp_config = QPConfig()
    started = time.perf_counter()
    scenarios = [run_scenario(config, qp_config, index, trace_dir) for index in range(config.scenario_count)]
    spec = default_model_spec()
    report = {
        "migration": "Simscape Flexiv Rizon 4s to MuJoCo, V6-lite logical architecture",
        "scope": "single_7dof_arm_on_free_floating_satellite",
        "model_identity": spec.identity(),
        "run_config": asdict(config),
        "qp_config": asdict(qp_config),
        "elapsed_wall_s": time.perf_counter() - started,
        "scenarios": scenarios,
        "aggregate": {
            "scenario_count": len(scenarios),
            "qp_success_rate_min": min(item["qp_success_rate"] for item in scenarios),
            "tracking_rms_m_max": max(item["tracking_rms_m"] for item in scenarios),
            "tracking_max_m_max": max(item["tracking_max_m"] for item in scenarios),
            "minimum_signed_clearance_m": min(item["minimum_signed_clearance_m"] for item in scenarios),
            "momentum_residual_max": max(item["momentum_residual_max"] for item in scenarios),
            "qp_latency_p99_ms_max": max(item["qp_latency_p99_ms"] for item in scenarios),
        },
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    metrics_path = output_dir / "migration_metrics.json"
    metrics_path.write_text(json.dumps(report, ensure_ascii=False, indent=2, default=_jsonable), encoding="utf-8")
    manifest = {
        "metrics": str(metrics_path.resolve().relative_to(PROJECT_ROOT.resolve())),
        "metrics_sha256": _sha256(metrics_path),
        "traces": [{"path": item["trace"], "sha256": item["trace_sha256"]} for item in scenarios],
        "model_identity": spec.identity(),
    }
    manifest_path = output_dir / "artifact_manifest.json"
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=25.5)
    parser.add_argument("--scenario-count", type=int, default=5)
    parser.add_argument("--seed", type=int, default=20260801)
    parser.add_argument("--output-dir", type=Path, default=PROJECT_ROOT / "output")
    args = parser.parse_args()
    report = run_suite(RunConfig(duration_s=args.duration, scenario_count=args.scenario_count, seed=args.seed), args.output_dir)
    print(json.dumps(report["aggregate"], indent=2))


if __name__ == "__main__":
    main()

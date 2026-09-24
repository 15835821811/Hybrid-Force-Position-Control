"""Independently audit and torque-replay one saved contact experiment."""

from __future__ import annotations

import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from ..model import PROJECT_ROOT, geom_id, site_id
from .contact import aggregate_contact_wrench, maximum_interface_penetration
from .contact_config import load_contact_config
from .contact_model import default_contact_model_spec
from .contact_provenance import contact_implementation_identity


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _all_numeric_finite(trace: np.lib.npyio.NpzFile) -> bool:
    return all(
        np.all(np.isfinite(trace[key]))
        for key in trace.files
        if np.issubdtype(np.asarray(trace[key]).dtype, np.number)
    )


def validate_contact_output(
    output_dir: Path | str, *, overwrite: bool = False
) -> dict[str, Any]:
    output = Path(output_dir).resolve()
    metrics_path = output / "metrics.json"
    trace_path = output / "trace.npz"
    validation_path = output / "validation.json"
    manifest_path = output / "artifact_manifest.json"
    if not metrics_path.is_file() or not trace_path.is_file():
        raise FileNotFoundError(f"contact metrics or trace missing under {output}")
    if not overwrite and (validation_path.exists() or manifest_path.exists()):
        raise FileExistsError(f"contact validation already exists: {output}")

    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    contact_config_path = (PROJECT_ROOT / metrics["contact_config_path"]).resolve()
    contact_config = load_contact_config(contact_config_path)
    spec = default_contact_model_spec()
    if contact_config_path != Path(spec.contact_config_path).resolve():
        spec = type(spec)(contact_config_path=contact_config_path)
    model = spec.compile_model()
    model.geom_pos[geom_id(model, "workspace_obstacle_0")] = [10.0, 10.0, 10.0]
    data = mujoco.MjData(model)
    flange = site_id(model, "flange_site")
    grasp = site_id(model, "target_grasp_site")
    pad = geom_id(model, "gripper_contact_pad")
    target = geom_id(model, "target_contact_plate")

    with np.load(trace_path, allow_pickle=False) as trace:
        required = {
            "initial_qpos",
            "initial_qvel",
            "qpos",
            "qvel",
            "torque",
            "flange_position",
            "target_grasp_position",
            "target_grasp_rotation",
            "measured_normal_force_n",
            "desired_normal_force_n",
            "penetration_m",
            "contact_count",
            "source_trace_sha256",
        }
        missing = sorted(required - set(trace.files))
        if missing:
            raise ValueError(f"contact trace is missing arrays: {missing}")
        steps = len(trace["time"])
        if any(len(trace[key]) != steps for key in required if key not in {
            "initial_qpos", "initial_qvel", "source_trace_sha256"
        }):
            raise ValueError("contact trace physics arrays have inconsistent lengths")
        data.qpos[:] = np.asarray(trace["initial_qpos"], dtype=np.float64)
        data.qvel[:] = np.asarray(trace["initial_qvel"], dtype=np.float64)
        data.ctrl[:] = 0.0
        data.time = 0.0
        mujoco.mj_forward(model, data)

        replay_qpos: list[np.ndarray] = []
        replay_qvel: list[np.ndarray] = []
        replay_flange: list[np.ndarray] = []
        replay_grasp: list[np.ndarray] = []
        replay_force: list[float] = []
        replay_penetration: list[float] = []
        replay_contact_count: list[int] = []
        for torque in np.asarray(trace["torque"], dtype=np.float64):
            data.ctrl[:] = torque
            mujoco.mj_step(model, data)
            flange_position = np.asarray(data.site_xpos[flange]).copy()
            grasp_rotation = np.asarray(data.site_xmat[grasp]).reshape(3, 3).copy()
            normal = grasp_rotation[:, 2]
            wrench = aggregate_contact_wrench(
                model,
                data,
                selected_geom_ids=[pad],
                counterpart_geom_ids=[target],
                reference_point_world_m=flange_position,
            )
            penetration = maximum_interface_penetration(
                model,
                data,
                selected_geom_ids=[pad],
                counterpart_geom_ids=[target],
            )
            replay_qpos.append(np.asarray(data.qpos).copy())
            replay_qvel.append(np.asarray(data.qvel).copy())
            replay_flange.append(flange_position)
            replay_grasp.append(np.asarray(data.site_xpos[grasp]).copy())
            replay_force.append(max(0.0, -float(wrench.force_world_n @ normal)))
            replay_penetration.append(penetration)
            replay_contact_count.append(wrench.contact_count)

        replay_qpos_array = np.asarray(replay_qpos)
        replay_qvel_array = np.asarray(replay_qvel)
        replay_flange_array = np.asarray(replay_flange)
        replay_grasp_array = np.asarray(replay_grasp)
        replay_force_array = np.asarray(replay_force)
        replay_penetration_array = np.asarray(replay_penetration)
        replay_contact_count_array = np.asarray(replay_contact_count)
        qpos_error = float(np.max(np.abs(replay_qpos_array - trace["qpos"])))
        qvel_error = float(np.max(np.abs(replay_qvel_array - trace["qvel"])))
        flange_error = float(
            np.max(np.linalg.norm(replay_flange_array - trace["flange_position"], axis=1))
        )
        grasp_error = float(
            np.max(
                np.linalg.norm(
                    replay_grasp_array - trace["target_grasp_position"], axis=1
                )
            )
        )
        force_error = float(
            np.max(np.abs(replay_force_array - trace["measured_normal_force_n"]))
        )
        penetration_error = float(
            np.max(np.abs(replay_penetration_array - trace["penetration_m"]))
        )
        contact_count_exact = bool(
            np.array_equal(replay_contact_count_array, trace["contact_count"])
        )
        trace_finite = _all_numeric_finite(trace)
        source_trace_sha256 = str(np.asarray(trace["source_trace_sha256"]).item())
        desired_force = np.asarray(trace["desired_normal_force_n"], dtype=np.float64)
        stored_force = np.asarray(trace["measured_normal_force_n"], dtype=np.float64)
        steady_steps = max(
            1,
            int(
                round(
                    float(
                        contact_config["force_control"][
                            "steady_evaluation_window_s"
                        ]
                    )
                    / spec.timestep_s
                )
            ),
        )
        recomputed = {
            "peak_normal_force_n": float(np.max(stored_force)),
            "normal_force_impulse_ns": float(
                np.sum(stored_force) * spec.timestep_s
            ),
            "steady_force_rmse_n": float(
                np.sqrt(
                    np.mean(
                        (
                            stored_force[-steady_steps:]
                            - desired_force[-steady_steps:]
                        )
                        ** 2
                    )
                )
            ),
            "maximum_penetration_m": float(np.max(trace["penetration_m"])),
        }

    stored_source_path = Path(
        metrics["source_handoff"]["source_trace_path"]
    ).resolve()
    metric_errors = {
        key: abs(float(metrics["metrics"][key]) - value)
        for key, value in recomputed.items()
    }
    current_implementation = contact_implementation_identity()
    checks = {
        "trace_sha256": _sha256(trace_path) == metrics["trace_sha256"],
        "trace_numeric_finite": trace_finite,
        "physics_step_count": steps == int(metrics["physics_steps"]),
        "contact_config_sha256": _sha256(contact_config_path)
        == metrics["contact_config_sha256"],
        "model_identity": spec.identity() == metrics["model_identity"],
        "implementation_identity": current_implementation
        == metrics["implementation_identity"],
        "source_trace_exists": stored_source_path.is_file(),
        "source_trace_sha256": stored_source_path.is_file()
        and _sha256(stored_source_path) == source_trace_sha256
        and source_trace_sha256
        == metrics["source_handoff"]["source_trace_sha256"],
        "qpos_no_post_initialization_writes": int(
            metrics["qpos_write_count_after_initialization"]
        )
        == 0,
        "qvel_no_post_initialization_writes": int(
            metrics["qvel_write_count_after_initialization"]
        )
        == 0,
        # Constraint warm-start state is intentionally not persisted.  Contact
        # replay is therefore checked against tight physical tolerances instead
        # of requiring bitwise identity of the iterative solver trajectory.
        "replay_qpos": qpos_error <= 1e-8,
        "replay_qvel": qvel_error <= 1e-8,
        "replay_flange_position": flange_error <= 1e-9,
        "replay_target_grasp_position": grasp_error <= 1e-9,
        "replay_normal_force": force_error <= 1e-5,
        "replay_penetration": penetration_error <= 1e-8,
        "replay_contact_count": contact_count_exact,
        "metrics_recomputed": max(metric_errors.values()) <= 1e-12,
    }
    passed = bool(all(checks.values()))
    validation = {
        "material_passport": {
            "origin_skill": "experiment-plan",
            "origin_mode": "run",
            "origin_date": datetime.now(timezone.utc).isoformat(),
            "verification_status": "VERIFIED" if passed else "FAILED",
            "version_label": "contact_validation_v1",
        },
        "variant": metrics["variant"],
        "checks": checks,
        "passed": passed,
        "fresh_torque_replay": {
            "maximum_qpos_absolute_error": qpos_error,
            "maximum_qvel_absolute_error": qvel_error,
            "maximum_flange_position_error_m": flange_error,
            "maximum_target_grasp_position_error_m": grasp_error,
            "maximum_normal_force_error_n": force_error,
            "maximum_penetration_error_m": penetration_error,
            "contact_count_exact": contact_count_exact,
        },
        "metric_recomputation_absolute_errors": metric_errors,
        "model_identity": spec.identity(),
        "implementation_identity": current_implementation,
    }
    validation_path.write_text(
        json.dumps(validation, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    manifest = {
        "schema_version": "fpmfc_contact_artifact_manifest_v1",
        "variant": metrics["variant"],
        "verification_status": "VERIFIED" if passed else "FAILED",
        "artifacts": {
            "metrics.json": _sha256(metrics_path),
            "trace.npz": _sha256(trace_path),
            "validation.json": _sha256(validation_path),
        },
        "source_trace": {
            "path": str(stored_source_path),
            "sha256": source_trace_sha256,
        },
    }
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return validation


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    validation = validate_contact_output(
        args.output_dir, overwrite=args.overwrite
    )
    print(json.dumps(validation, indent=2))


if __name__ == "__main__":
    main()

"""Freeze and verify the N110C C0/C1 contract before any contact result."""

from __future__ import annotations

import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import yaml

from ..model import PROJECT_ROOT, default_model_spec, site_id
from .config import load_fpmfc_config
from .contact_config import load_contact_config
from .contact_model import default_contact_model_spec
from .handoff_provenance import n110c_implementation_identity
from .handoff_trajectory import contact_flange_boundary
from .shape import ArmShapeKinematics
from .target import sync_mujoco_target, target_from_config


DEFAULT_HANDOFF_CONFIG = PROJECT_ROOT / "configs/fpmfc_n110_handoff.yaml"
DEFAULT_OUTPUT_ROOT = PROJECT_ROOT / "output/fpmfc/n110/live_twist_handoff"


def sha256(path: Path | str) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024*1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def repo_path(relative: str) -> Path:
    path = (PROJECT_ROOT/relative).resolve()
    if not path.is_relative_to(PROJECT_ROOT.resolve()):
        raise ValueError(f"path outside repository: {relative}")
    return path


def load_handoff_config(path: Path | str = DEFAULT_HANDOFF_CONFIG) -> dict[str, Any]:
    source = Path(path).resolve()
    values = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(values, dict) or values.get("schema_version") != "fpmfc_n110_live_twist_handoff_v1":
        raise ValueError("unexpected N110C handoff config schema")
    if values["contact_mapping_mode"] != "homogeneous":
        raise ValueError("N110C contact comparisons must use S homogeneous mapping")
    if set(values["paired_conditions"]) != {"C0", "C1"}:
        raise ValueError("C0 and C1 are both required")
    if values["run_budget"]["precontact_dynamics_trajectories"] != 2 or values["run_budget"]["contact_dynamics_trajectories_if_qualified"] != 6:
        raise ValueError("unexpected N110C run budget")
    return values


def source_candidate(config: dict[str, Any]) -> dict[str, Any]:
    trace_path = repo_path(config["source_trace"])
    metrics_path = repo_path(config["source_metrics"])
    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    trace_hash = sha256(trace_path)
    if trace_hash != metrics["trace"]["sha256"]:
        raise ValueError("N073 archived trace hash mismatch")
    with np.load(trace_path, allow_pickle=False) as source:
        candidate = {
            "capture_time_s": float(source["capture_time_s"]),
            "terminal_arm_angle_rad": float(source["terminal_arm_angle_rad"]),
            "initial_qpos": source["initial_qpos"].tolist(),
            "initial_qvel": source["initial_qvel"].tolist(),
        }
    if candidate["capture_time_s"] != float(metrics["candidate"]["executed_capture_time_s"]):
        raise ValueError("source trace/metrics capture time mismatch")
    if candidate["terminal_arm_angle_rad"] != float(metrics["candidate"]["terminal_arm_angle_rad"]):
        raise ValueError("source trace/metrics arm angle mismatch")
    candidate.update({
        "source_trace_path": config["source_trace"],
        "source_trace_sha256": trace_hash,
        "source_metrics_path": config["source_metrics"],
        "source_metrics_sha256": sha256(metrics_path),
        "source_implementation_identity": metrics["implementation_identity"],
    })
    return candidate


def _initial_reference(precontact: dict[str, Any]) -> dict[str, Any]:
    spec = default_model_spec()
    model = spec.compile_model()
    data = mujoco.MjData(model)
    spec.reset_home(model, data)
    target = target_from_config(precontact)
    sync_mujoco_target(model, data, target, target.sample(0.0))
    mujoco.mj_forward(model, data)
    flange = site_id(model, "flange_site")
    shape_values = precontact["shape"]
    shape = ArmShapeKinematics(
        spec, model, data,
        shoulder_joint=shape_values["shoulder_joint"],
        elbow_joint=shape_values["elbow_joint"],
        wrist_joint=shape_values["wrist_joint"],
        singularity_margin=float(shape_values["singularity_margin"]),
    )
    return {
        "flange_position_world_m": np.asarray(data.site_xpos[flange]).tolist(),
        "flange_rotation_world": np.asarray(data.site_xmat[flange]).reshape(3, 3).tolist(),
        "arm_angle_rad": float(shape.sample(data).angle_rad),
        "initial_qpos": np.asarray(data.qpos).tolist(),
        "initial_qvel": np.asarray(data.qvel).tolist(),
        "initial_linear_velocity_world_m_s": [0.0, 0.0, 0.0],
        "initial_angular_velocity_world_rad_s": [0.0, 0.0, 0.0],
        "initial_linear_acceleration_world_m_s2": [0.0, 0.0, 0.0],
        "initial_angular_acceleration_world_rad_s2": [0.0, 0.0, 0.0],
    }


def prepare_manifest(config_path: Path | str = DEFAULT_HANDOFF_CONFIG,
                     output_root: Path | str = DEFAULT_OUTPUT_ROOT) -> dict[str, Any]:
    config_path = Path(config_path).resolve()
    config = load_handoff_config(config_path)
    output = Path(output_root).resolve()
    path = output/"pairing_manifest.json"
    if path.exists():
        raise FileExistsError(f"frozen pairing manifest already exists: {path}")
    candidate = source_candidate(config)
    precontact_path = repo_path(config["precontact_config"])
    contact_path = repo_path(config["contact_config"])
    precontact = load_fpmfc_config(precontact_path)
    contact = load_contact_config(contact_path)
    target = target_from_config(precontact)
    boundary = contact_flange_boundary(
        target, candidate["capture_time_s"],
        tool_face_recess_m=float(contact["interface"]["tool_pad_face_recess_m"]),
    )
    initial = _initial_reference(precontact)
    np.testing.assert_array_equal(initial["initial_qpos"], candidate["initial_qpos"])
    np.testing.assert_array_equal(initial["initial_qvel"], candidate["initial_qvel"])
    manifest = {
        "schema_version": "n110c_frozen_pairing_contract_v1",
        "frozen_at_utc": datetime.now(timezone.utc).isoformat(),
        "baseline_commit": config["baseline_commit"],
        "handoff_config_path": str(config_path.relative_to(PROJECT_ROOT)),
        "handoff_config_sha256": sha256(config_path),
        "precontact_config_path": config["precontact_config"],
        "precontact_config_sha256": sha256(precontact_path),
        "contact_config_path": config["contact_config"],
        "contact_config_sha256": sha256(contact_path),
        "source_candidate": candidate,
        "precontact_model_identity": default_model_spec().identity(),
        "contact_model_identity": default_contact_model_spec().identity(),
        "n110c_implementation_identity": n110c_implementation_identity(),
        "initial_reference": initial,
        "terminal_reference": {
            "flange_position_world_m": boundary.position_world_m.tolist(),
            "flange_rotation_world": boundary.rotation_world.tolist(),
            "target_grasp_position_world_m": boundary.target_grasp_position_world_m.tolist(),
            "target_grasp_linear_velocity_world_m_s": boundary.target_grasp_linear_velocity_world_m_s.tolist(),
            "contact_normal_world": boundary.contact_normal_world.tolist(),
            "tool_face_recess_m": boundary.tool_face_recess_m,
            "C0_terminal_linear_velocity_world_m_s": [0.0, 0.0, 0.0],
            "C0_terminal_angular_velocity_world_rad_s": [0.0, 0.0, 0.0],
            "C0_terminal_linear_acceleration_world_m_s2": [0.0, 0.0, 0.0],
            "C0_terminal_angular_acceleration_world_rad_s2": [0.0, 0.0, 0.0],
            "C1_terminal_linear_velocity_world_m_s": boundary.linear_velocity_world_m_s.tolist(),
            "C1_terminal_angular_velocity_world_rad_s": boundary.angular_velocity_world_rad_s.tolist(),
            "C1_terminal_linear_acceleration_world_m_s2": boundary.linear_acceleration_world_m_s2.tolist(),
            "C1_terminal_angular_acceleration_world_rad_s2": boundary.angular_acceleration_world_rad_s2.tolist(),
        },
        "paired_conditions": config["paired_conditions"],
        "terminal_reference_contract": config["terminal_reference"],
        "reference_numerical_tolerances": config["reference_numerical_tolerances"],
        "finite_difference_tolerances": config["finite_difference_tolerances"],
        "replay_tolerances": config["replay_tolerances"],
        "handoff_gates": config["handoff_gates"],
        "run_budget": config["run_budget"],
        "fixed_contact_mapping_mode": "homogeneous",
        "contact_force_window_rule": "last 0.2 s, all 100 samples including zero-force loss, fixed tau=0 start",
    }
    output.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    return manifest


def load_manifest(path: Path | str = DEFAULT_OUTPUT_ROOT/"pairing_manifest.json") -> dict[str, Any]:
    manifest_path = Path(path).resolve()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("schema_version") != "n110c_frozen_pairing_contract_v1":
        raise ValueError("unexpected pairing manifest schema")
    config_path = repo_path(manifest["handoff_config_path"])
    config = load_handoff_config(config_path)
    if sha256(config_path) != manifest["handoff_config_sha256"]:
        raise ValueError("handoff config changed after manifest freeze")
    if source_candidate(config) != manifest["source_candidate"]:
        raise ValueError("source candidate changed after manifest freeze")
    if n110c_implementation_identity() != manifest["n110c_implementation_identity"]:
        raise ValueError("N110C execution source changed after manifest freeze")
    if default_model_spec().identity() != manifest["precontact_model_identity"]:
        raise ValueError("precontact model changed after manifest freeze")
    if default_contact_model_spec().identity() != manifest["contact_model_identity"]:
        raise ValueError("contact model changed after manifest freeze")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_HANDOFF_CONFIG)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    args = parser.parse_args()
    result = prepare_manifest(args.config, args.output_root)
    print(json.dumps({"manifest": str((args.output_root/"pairing_manifest.json").resolve()),
                      "candidate": result["source_candidate"],
                      "terminal_reference": result["terminal_reference"]}, indent=2))


if __name__ == "__main__":
    main()

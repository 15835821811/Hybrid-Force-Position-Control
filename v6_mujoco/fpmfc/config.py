"""Configuration and provenance checks for the adaptive FPMFC reproduction."""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from ..model import PROJECT_ROOT


DEFAULT_CONFIG_PATH = PROJECT_ROOT / "configs" / "fpmfc_paper.yaml"
ALLOWED_PROVENANCE = frozenset({"paper", "user", "paper+user", "calibrated"})
NON_PARAMETER_KEYS = frozenset({"schema_version", "experiment_id", "provenance"})


def _parameter_leaves(value: Any, prefix: str = "") -> list[str]:
    if isinstance(value, Mapping):
        leaves: list[str] = []
        for key, child in value.items():
            if not prefix and key in NON_PARAMETER_KEYS:
                continue
            child_prefix = f"{prefix}.{key}" if prefix else str(key)
            leaves.extend(_parameter_leaves(child, child_prefix))
        return leaves
    return [prefix]


def validate_fpmfc_config(config: Mapping[str, Any]) -> None:
    required_sections = {
        "model",
        "target",
        "trajectory",
        "shape",
        "optimization",
        "controller",
        "acceptance",
        "contact_extension",
        "provenance",
    }
    missing_sections = sorted(required_sections - set(config))
    if missing_sections:
        raise ValueError(f"missing FPMFC configuration sections: {missing_sections}")

    provenance = config["provenance"]
    if not isinstance(provenance, Mapping):
        raise TypeError("provenance must be a mapping from dotted parameter path to source")
    leaves = set(_parameter_leaves(config))
    missing_sources = sorted(leaves - set(provenance))
    extra_sources = sorted(set(provenance) - leaves)
    invalid_sources = sorted(
        f"{key}={value}" for key, value in provenance.items() if value not in ALLOWED_PROVENANCE
    )
    if missing_sources or extra_sources or invalid_sources:
        raise ValueError(
            "invalid parameter provenance: "
            f"missing={missing_sources}, extra={extra_sources}, invalid={invalid_sources}"
        )

    model = config["model"]
    if float(model["base_mass_kg"]) <= 0.0:
        raise ValueError("base mass must be positive")
    if len(model["base_diagonal_inertia_kg_m2"]) != 3:
        raise ValueError("base inertia must have three diagonal entries")
    if len(model["home_joint_position_deg"]) != 7:
        raise ValueError("Flexiv home configuration must contain seven joints")

    target = config["target"]
    if target.get("geometry") != "cube":
        raise ValueError("target.geometry must be 'cube'")
    side_length = float(target["side_length_m"])
    if not np.isfinite(side_length) or side_length <= 0.0:
        raise ValueError("target.side_length_m must be finite and positive")
    for key in (
        "center_position_world_m",
        "center_linear_velocity_world_m_s",
        "initial_rpy_xyz_deg",
        "angular_velocity_body_deg_s",
        "grasp_offset_body_m",
        "grasp_frame_rpy_xyz_deg",
    ):
        if len(target[key]) != 3:
            raise ValueError(f"target.{key} must contain three values")
    grasp_offset = np.asarray(target["grasp_offset_body_m"], dtype=np.float64)
    if not np.all(np.isfinite(grasp_offset)):
        raise ValueError("target.grasp_offset_body_m must be finite")
    half_length = 0.5 * side_length
    if not np.isclose(np.max(np.abs(grasp_offset)), half_length, atol=1e-12):
        raise ValueError("target grasp point must lie on a cube face")
    if np.count_nonzero(np.isclose(np.abs(grasp_offset), half_length, atol=1e-12)) != 1:
        raise ValueError("target grasp point must lie at one cube-face center")
    if np.count_nonzero(np.abs(grasp_offset) > 1e-12) != 1:
        raise ValueError("target grasp offset must select one cube-face center")
    grasp_rotation = Rotation.from_euler(
        "xyz", target["grasp_frame_rpy_xyz_deg"], degrees=True
    ).as_matrix()
    target_outward_normal = grasp_offset / np.linalg.norm(grasp_offset)
    flange_outward_normal = grasp_rotation[:, 2]
    if not np.allclose(
        flange_outward_normal, -target_outward_normal, rtol=0.0, atol=1e-10
    ):
        raise ValueError(
            "target grasp frame must make flange +Z oppose the cube-face outward normal"
        )

    time_bounds = config["trajectory"]["capture_time_bounds_s"]
    if len(time_bounds) != 2 or not 0.0 < float(time_bounds[0]) < float(time_bounds[1]):
        raise ValueError("capture time bounds must be positive and increasing")
    optimizer = config["optimization"]
    if int(optimizer["pso_population"]) < 2 or int(optimizer["pso_generations"]) < 1:
        raise ValueError("PSO population and generations are invalid")
    if not 0.0 < float(optimizer["pso_velocity_fraction"]) <= 1.0:
        raise ValueError("PSO velocity fraction must be in (0, 1]")
    acceptance = config["acceptance"]
    controller = config["controller"]
    planning_clearance = float(controller["minimum_clearance_m"])
    if planning_clearance <= 0.0:
        raise ValueError("controller minimum clearance must be positive")
    if int(controller["qp_max_iterations"]) < 1:
        raise ValueError("controller QP iteration limit must be positive")
    if float(controller["qp_tolerance"]) <= 0.0 or float(
        controller["feasibility_tolerance"]
    ) <= 0.0:
        raise ValueError("controller QP tolerances must be positive")
    if not 0.0 < float(controller["angular_saturation_transition_ratio"]) < 1.0:
        raise ValueError("controller angular saturation transition ratio must be in (0, 1)")
    if float(controller["joint_jerk_limit_rad_s3"]) <= 0.0:
        raise ValueError("controller joint jerk limit must be positive")
    positive_acceptance = (
        "terminal_position_error_m",
        "terminal_orientation_error_rad",
        "terminal_arm_angle_error_rad",
        "terminal_linear_velocity_m_s",
        "terminal_angular_velocity_rad_s",
        "maximum_torque_saturation_fraction",
        "maximum_momentum_delta",
    )
    if any(float(acceptance[key]) <= 0.0 for key in positive_acceptance):
        raise ValueError("acceptance thresholds must be positive")
    if not 0.0 < float(acceptance["minimum_task_success_rate"]) <= 1.0:
        raise ValueError("minimum task success rate must be in (0, 1]")
    if "minimum_clearance_m" in acceptance:
        acceptance_clearance = float(acceptance["minimum_clearance_m"])
        if acceptance_clearance <= 0.0:
            raise ValueError("acceptance minimum clearance must be positive")
        if planning_clearance < acceptance_clearance:
            raise ValueError(
                "controller planning clearance cannot be below the acceptance clearance"
            )


def apply_runtime_overrides(
    config: Mapping[str, Any],
    *,
    objective_weights: Sequence[float] | None = None,
    planning_clearance_m: float | None = None,
) -> dict[str, Any]:
    """Create a provenance-complete effective config without editing source YAML.

    A planning-clearance override preserves the source controller clearance as
    the independent dynamic acceptance threshold.  This creates an explicit
    planning/acceptance margin while keeping the source configuration immutable.
    """

    effective = copy.deepcopy(dict(config))
    if objective_weights is not None:
        weights = [float(value) for value in objective_weights]
        if len(weights) != 2:
            raise ValueError("objective weights must contain exactly two values")
        if any(value < 0.0 for value in weights) or not any(
            value > 0.0 for value in weights
        ):
            raise ValueError("objective weights must be nonnegative and not both zero")
        effective["optimization"]["objective_weights"] = weights
        effective["provenance"]["optimization.objective_weights"] = "calibrated"

    if planning_clearance_m is not None:
        planning_clearance = float(planning_clearance_m)
        if planning_clearance <= 0.0:
            raise ValueError("planning clearance must be positive")
        original_clearance = float(effective["controller"]["minimum_clearance_m"])
        acceptance = effective["acceptance"]
        if "minimum_clearance_m" not in acceptance:
            acceptance["minimum_clearance_m"] = original_clearance
            effective["provenance"]["acceptance.minimum_clearance_m"] = effective[
                "provenance"
            ].get("controller.minimum_clearance_m", "calibrated")
        effective["controller"]["minimum_clearance_m"] = planning_clearance
        effective["provenance"]["controller.minimum_clearance_m"] = "calibrated"

    validate_fpmfc_config(effective)
    return effective


def load_fpmfc_config(path: Path | str = DEFAULT_CONFIG_PATH) -> dict[str, Any]:
    config_path = Path(path)
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise TypeError(f"FPMFC configuration at {config_path} is not a mapping")
    validate_fpmfc_config(config)
    return config

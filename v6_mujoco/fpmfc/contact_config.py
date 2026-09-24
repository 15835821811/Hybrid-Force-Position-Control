"""Configuration contract for the user-scenario contact extension."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml

from ..model import PROJECT_ROOT
from .contact import NormalAdmittanceConfig


DEFAULT_CONTACT_CONFIG_PATH = PROJECT_ROOT / "configs" / "fpmfc_contact.yaml"
ALLOWED_PROVENANCE = frozenset({"paper", "user", "paper+user", "calibrated"})


def _parameter_leaves(value: Any, prefix: str = "") -> list[str]:
    if isinstance(value, Mapping):
        leaves: list[str] = []
        for key, child in value.items():
            if not prefix and key in {"schema_version", "experiment_id", "provenance"}:
                continue
            child_prefix = f"{prefix}.{key}" if prefix else str(key)
            leaves.extend(_parameter_leaves(child, child_prefix))
        return leaves
    return [prefix]


def validate_contact_config(config: Mapping[str, Any]) -> None:
    required = {
        "schema_version",
        "experiment_id",
        "precontact_config",
        "physical_target",
        "interface",
        "force_control",
        "acceptance",
        "provenance",
    }
    missing = sorted(required - set(config))
    if missing:
        raise ValueError(f"missing contact configuration sections: {missing}")
    provenance = config["provenance"]
    leaves = set(_parameter_leaves(config))
    missing_sources = sorted(leaves - set(provenance))
    extra_sources = sorted(set(provenance) - leaves)
    invalid_sources = sorted(
        f"{key}={value}"
        for key, value in provenance.items()
        if value not in ALLOWED_PROVENANCE
    )
    if missing_sources or extra_sources or invalid_sources:
        raise ValueError(
            "invalid contact parameter provenance: "
            f"missing={missing_sources}, extra={extra_sources}, invalid={invalid_sources}"
        )

    target = config["physical_target"]
    mass = float(target["mass_kg"])
    inertia = np.asarray(target["diagonal_inertia_kg_m2"], dtype=np.float64)
    if not np.isfinite(mass) or mass <= 0.0:
        raise ValueError("physical_target.mass_kg must be finite and positive")
    if inertia.shape != (3,) or not np.all(np.isfinite(inertia)) or np.any(inertia <= 0.0):
        raise ValueError("physical_target.diagonal_inertia_kg_m2 must be positive")
    if np.any(2.0 * inertia > np.sum(inertia) + 1e-12):
        raise ValueError("physical target diagonal inertia violates triangle inequalities")
    sensitivity = float(target["parameter_sensitivity_fraction"])
    if not 0.0 < sensitivity < 1.0:
        raise ValueError("target sensitivity fraction must lie in (0, 1)")

    interface = config["interface"]
    for key in (
        "tool_pad_radius_m",
        "tool_pad_half_thickness_m",
        "tool_pad_face_recess_m",
    ):
        if float(interface[key]) <= 0.0:
            raise ValueError(f"interface.{key} must be positive")
    plate = np.asarray(interface["target_plate_half_size_m"], dtype=np.float64)
    friction = np.asarray(interface["friction"], dtype=np.float64)
    if plate.shape != (3,) or not np.all(np.isfinite(plate)) or np.any(plate <= 0.0):
        raise ValueError("target plate half size must contain three positive values")
    if (
        friction.shape != (3,)
        or not np.all(np.isfinite(friction))
        or np.any(friction < 0.0)
    ):
        raise ValueError("interface friction must contain three nonnegative values")
    contact_margin = float(interface["contact_margin_m"])
    if not np.isfinite(contact_margin) or contact_margin < 0.0:
        raise ValueError("interface contact margin must be finite and nonnegative")
    solver_time_constant = float(interface["solver_time_constant_s"])
    solver_damping_ratio = float(interface["solver_damping_ratio"])
    if not np.isfinite(solver_time_constant) or solver_time_constant < 2.0 * 0.002:
        raise ValueError("contact solver time constant must be at least two physics steps")
    if not np.isfinite(solver_damping_ratio) or solver_damping_ratio <= 0.0:
        raise ValueError("contact solver damping ratio must be finite and positive")
    detection = float(interface["contact_detection_force_n"])
    release = float(interface["contact_release_force_n"])
    if not 0.0 <= release < detection:
        raise ValueError("contact release force must be below detection force")

    force = config["force_control"]
    desired = float(force["desired_normal_force_n"])
    if not 2.0 <= desired <= 5.0:
        raise ValueError("desired normal force must remain in the calibrated 2--5 N range")
    if float(force["force_ramp_duration_s"]) <= 0.0:
        raise ValueError("force ramp duration must be positive")
    duration = float(force["contact_stage_duration_s"])
    steady_window = float(force["steady_evaluation_window_s"])
    if not np.isfinite(duration) or duration <= 0.0:
        raise ValueError("contact stage duration must be finite and positive")
    if not np.isfinite(steady_window) or not 0.0 < steady_window <= duration:
        raise ValueError("steady evaluation window must lie in (0, duration]")
    rigid_offset = float(force["rigid_normal_offset_m"])
    if not np.isfinite(rigid_offset) or not 0.0 < rigid_offset <= float(
        force["maximum_offset_m"]
    ):
        raise ValueError("rigid normal offset must lie in (0, maximum offset]")
    if not 0.0 < float(force["damping_ratio"]):
        raise ValueError("admittance damping ratio must be positive")
    normal_admittance_config(config, timestep_s=0.002)

    acceptance = config["acceptance"]
    for key, value in acceptance.items():
        if float(value) <= 0.0:
            raise ValueError(f"acceptance.{key} must be positive")


def normal_admittance_config(
    config: Mapping[str, Any], *, timestep_s: float
) -> NormalAdmittanceConfig:
    force = config["force_control"]
    mass = float(force["virtual_mass_kg"])
    stiffness = float(force["stiffness_n_m"])
    damping_ratio = float(force["damping_ratio"])
    damping = 2.0 * damping_ratio * np.sqrt(mass * stiffness)
    return NormalAdmittanceConfig(
        virtual_mass_kg=mass,
        damping_n_s_m=float(damping),
        stiffness_n_m=stiffness,
        timestep_s=float(timestep_s),
        maximum_offset_m=float(force["maximum_offset_m"]),
        maximum_velocity_m_s=float(force["maximum_velocity_m_s"]),
    )


def load_contact_config(
    path: Path | str = DEFAULT_CONTACT_CONFIG_PATH,
) -> dict[str, Any]:
    config_path = Path(path)
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise TypeError(f"contact configuration at {config_path} is not a mapping")
    validate_contact_config(config)
    return config

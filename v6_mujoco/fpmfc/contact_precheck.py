"""Run the N100 contact-wrench and N101 admittance prechecks."""

from __future__ import annotations

import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from ..model import PROJECT_ROOT
from .contact import NormalAdmittance, aggregate_contact_wrench
from .contact_config import (
    DEFAULT_CONTACT_CONFIG_PATH,
    load_contact_config,
    normal_admittance_config,
)
from .contact_provenance import contact_implementation_identity


CONTACT_XML = """
<mujoco>
  <option timestep="0.001" gravity="0 0 -9.81"/>
  <worldbody>
    <geom name="floor" type="plane" size="1 1 0.1"/>
    <body name="box" pos="0 0 0.049">
      <freejoint/>
      <geom name="box_geom" type="box" size="0.05 0.05 0.05" mass="2"/>
    </body>
  </worldbody>
</mujoco>
"""


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _json_default(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(type(value).__name__)


def run_contact_precheck(
    *, config_path: Path, output_path: Path
) -> dict[str, Any]:
    config_file = Path(config_path).resolve()
    config = load_contact_config(config_file)

    model = mujoco.MjModel.from_xml_string(CONTACT_XML)
    data = mujoco.MjData(model)
    for _ in range(2000):
        mujoco.mj_step(model, data)
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    box = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "box_geom")
    reference_a = np.array([0.0, 0.0, float(data.qpos[2])])
    reference_b = reference_a + np.array([0.1, -0.2, 0.05])
    box_a = aggregate_contact_wrench(
        model,
        data,
        selected_geom_ids=[box],
        counterpart_geom_ids=[floor],
        reference_point_world_m=reference_a,
    )
    box_b = aggregate_contact_wrench(
        model,
        data,
        selected_geom_ids=[box],
        counterpart_geom_ids=[floor],
        reference_point_world_m=reference_b,
    )
    floor_a = aggregate_contact_wrench(
        model,
        data,
        selected_geom_ids=[floor],
        counterpart_geom_ids=[box],
        reference_point_world_m=reference_a,
    )
    expected_support = 2.0 * 9.81
    support_error = abs(box_a.force_world_n[2] - expected_support)
    action_reaction_error = float(
        np.linalg.norm(box_a.force_world_n + floor_a.force_world_n)
    )
    transported_torque = box_a.torque_world_nm + np.cross(
        reference_a - reference_b, box_a.force_world_n
    )
    transport_error = float(
        np.linalg.norm(box_b.torque_world_nm - transported_torque)
    )

    admittance_config = normal_admittance_config(config, timestep_s=0.002)
    admittance = NormalAdmittance(admittance_config)
    desired_force = float(config["force_control"]["desired_normal_force_n"])
    offsets = []
    velocities = []
    times = []
    for index in range(2500):
        state = admittance.step(
            desired_force_n=desired_force, measured_force_n=0.0
        )
        offsets.append(state.offset_m)
        velocities.append(state.velocity_m_s)
        times.append((index + 1) * admittance_config.timestep_s)
    offsets_array = np.asarray(offsets)
    times_array = np.asarray(times)
    steady_offset = desired_force / admittance_config.stiffness_n_m
    natural_frequency = np.sqrt(
        admittance_config.stiffness_n_m / admittance_config.virtual_mass_kg
    )
    exact = steady_offset * (
        1.0
        - (1.0 + natural_frequency * times_array)
        * np.exp(-natural_frequency * times_array)
    )
    exact_error = float(np.max(np.abs(offsets_array - exact)))
    overshoot = float(max(0.0, np.max(offsets_array) - steady_offset))
    steady_error = float(abs(offsets_array[-1] - steady_offset))
    settling_candidates = np.flatnonzero(
        np.abs(offsets_array - steady_offset) <= 0.02 * abs(steady_offset)
    )
    settling_time = None
    for index in settling_candidates:
        if np.all(
            np.abs(offsets_array[index:] - steady_offset)
            <= 0.02 * abs(steady_offset)
        ):
            settling_time = float(times_array[index])
            break

    checks = {
        "n100_four_contacts_detected": box_a.contact_count == 4,
        "n100_support_force_matches_weight": support_error <= 1e-8,
        "n100_action_reaction_sign": action_reaction_error <= 1e-10,
        "n100_reference_point_transport": transport_error <= 1e-10,
        "n101_critical_damping_from_config": np.isclose(
            admittance_config.damping_n_s_m,
            2.0
            * np.sqrt(
                admittance_config.virtual_mass_kg
                * admittance_config.stiffness_n_m
            ),
            atol=1e-12,
        ),
        "n101_discrete_matches_continuous": exact_error <= 3e-5,
        "n101_no_overshoot": overshoot <= 1e-9,
        "n101_steady_state_error": steady_error <= 1e-7,
        "n101_limits_respected": bool(
            np.max(np.abs(offsets_array)) <= admittance_config.maximum_offset_m
            and np.max(np.abs(velocities))
            <= admittance_config.maximum_velocity_m_s
        ),
    }
    report = {
        "material_passport": {
            "origin_skill": "experiment-plan",
            "origin_mode": "run",
            "origin_date": datetime.now(timezone.utc).isoformat(),
            "verification_status": "VERIFIED" if all(checks.values()) else "FAILED_GATE",
            "version_label": "n100_n101_contact_precheck_v1",
        },
        "experiment_ids": ["N100", "N101"],
        "config_path": str(config_file),
        "config_sha256": _sha256(config_file),
        "contact_implementation_identity": contact_implementation_identity(),
        "checks": checks,
        "passed": bool(all(checks.values())),
        "n100_contact_wrench": {
            "contact_count": box_a.contact_count,
            "box_force_world_n": box_a.force_world_n,
            "floor_force_world_n": floor_a.force_world_n,
            "box_torque_at_reference_a_nm": box_a.torque_world_nm,
            "box_torque_at_reference_b_nm": box_b.torque_world_nm,
            "support_force_error_n": support_error,
            "action_reaction_force_error_n": action_reaction_error,
            "reference_transport_error_nm": transport_error,
        },
        "n101_normal_admittance": {
            "parameters": {
                "virtual_mass_kg": admittance_config.virtual_mass_kg,
                "damping_n_s_m": admittance_config.damping_n_s_m,
                "stiffness_n_m": admittance_config.stiffness_n_m,
                "damping_ratio": float(config["force_control"]["damping_ratio"]),
                "timestep_s": admittance_config.timestep_s,
                "maximum_offset_m": admittance_config.maximum_offset_m,
                "maximum_velocity_m_s": admittance_config.maximum_velocity_m_s,
            },
            "desired_force_n": desired_force,
            "analytic_steady_offset_m": steady_offset,
            "discrete_final_offset_m": float(offsets_array[-1]),
            "maximum_continuous_solution_error_m": exact_error,
            "overshoot_m": overshoot,
            "steady_state_error_m": steady_error,
            "two_percent_settling_time_s": settling_time,
            "maximum_velocity_m_s": float(np.max(np.abs(velocities))),
        },
    }
    output = Path(output_path).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(report, indent=2, ensure_ascii=False, default=_json_default) + "\n",
        encoding="utf-8",
    )
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONTACT_CONFIG_PATH)
    parser.add_argument(
        "--output",
        type=Path,
        default=(
            PROJECT_ROOT
            / "output"
            / "fpmfc"
            / "contact"
            / "n100_n101_precheck.json"
        ),
    )
    args = parser.parse_args()
    report = run_contact_precheck(config_path=args.config, output_path=args.output)
    print(json.dumps(report, indent=2, ensure_ascii=False, default=_json_default))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

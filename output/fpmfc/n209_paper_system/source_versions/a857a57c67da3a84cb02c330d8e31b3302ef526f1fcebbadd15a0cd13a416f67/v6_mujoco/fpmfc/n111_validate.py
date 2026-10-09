"""Independent N111 torque arithmetic, physics replay, and prediction audit."""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
from unittest.mock import patch

import mujoco
import numpy as np

from ..model import PROJECT_ROOT, geom_id, site_id
from . import validate_contact_consistent as frozen_validation
from .contact_config import load_contact_config
from .contact_model import default_contact_model_spec
from .contact_observation import ContactObserver
from .force_regulation_contract import load_config as parent_config, outer_config
from .force_regulation_outer import ExperimentalNormalAdmittance
from .handoff_contract import sha256
from .n111_contract import CELLS, OUTPUT_ROOT, load_manifest


def validate_cell(cell: str) -> dict:
    manifest = load_manifest()
    if cell not in CELLS:
        raise ValueError(cell)
    output = OUTPUT_ROOT / cell
    metrics = json.loads((output / "metrics.json").read_text(encoding="utf-8"))
    original = load_contact_config(PROJECT_ROOT / manifest["contact_config_path"])
    effective = copy.deepcopy(original)
    effective["force_control"]["stiffness_n_m"] = 0.0
    effective["force_control"]["reset_on_contact_loss"] = True
    outer_cfg = outer_config(parent_config(), "D10")

    def patched_load(_path=None):
        return copy.deepcopy(effective)

    def patched_outer(_config, *, timestep_s):
        if timestep_s != outer_cfg.timestep_s:
            raise ValueError("unexpected physics step")
        return outer_cfg

    with patch.object(frozen_validation, "load_contact_config", patched_load), \
         patch.object(frozen_validation, "normal_admittance_config", patched_outer), \
         patch.object(frozen_validation, "NormalAdmittance", ExperimentalNormalAdmittance):
        replay = frozen_validation.validate_consistent_output(output)

    spec = default_contact_model_spec()
    model = spec.compile_model()
    model.geom_pos[geom_id(model, "workspace_obstacle_0")] = [10.0, 10.0, 10.0]
    data = mujoco.MjData(model)
    observer = ContactObserver(model, tool_face_recess_m=float(effective["interface"]["tool_pad_face_recess_m"]))
    qpos_ids, joints = spec.joint_addresses(model)
    base_slice = spec.base_slices(model)[1]
    base = np.arange(base_slice.start, base_slice.stop)
    max_errors = {key: 0.0 for key in (
        "original_raw_torque_nm", "reduced_load_nm", "protected_compensation_nm",
        "applied_torque_nm", "qpos", "qvel", "contact_force_n", "power_identity_w",
        "full_dynamics_identity", "reduced_dynamics_identity",
    )}
    prediction = []
    with np.load(output / "trace.npz", allow_pickle=False) as t, \
         np.load(output / "servo_diagnostics.npz", allow_pickle=False) as s:
        data.qpos[:] = t["initial_qpos"]
        data.qvel[:] = t["initial_qvel"]
        data.ctrl[:] = t["initial_ctrl"]
        data.qfrc_applied[:] = t["initial_qfrc_applied"]
        data.xfrc_applied[:] = t["initial_xfrc_applied"]
        data.qacc_warmstart[:] = t["initial_qacc_warmstart"]
        data.mocap_pos[:] = t["initial_mocap_pos"]
        data.mocap_quat[:] = t["initial_mocap_quat"]
        data.eq_active[:] = t["initial_eq_active"]
        for i in range(500):
            pre = observer.observe(data)
            if i % 10 == 0:
                mujoco.mj_forward(model, data)
            mujoco.mj_forward(model, data)
            mass = np.zeros((model.nv, model.nv))
            mujoco.mj_fullM(model, mass, data.qM)
            desired = np.clip(
                t["reference_ddq"][i]
                + 34.0**2 * (t["reference_q"][i] - data.qpos[qpos_ids])
                + 68.0 * (t["reference_dq"][i] - data.qvel[joints]), -70.0, 70.0,
            )
            bb = mass[np.ix_(base, base)]
            qb = mass[np.ix_(joints, base)]
            acc_b = -np.linalg.solve(bb, mass[np.ix_(base, joints)] @ desired
                                     + data.qfrc_bias[base] - data.qfrc_passive[base])
            raw = (qb @ acc_b + mass[np.ix_(joints, joints)] @ desired
                   + data.qfrc_bias[joints] - data.qfrc_passive[joints])
            observed = pre.forward_data
            observed_mass = np.zeros_like(mass)
            mujoco.mj_fullM(model, observed_mass, observed.qM)
            jp, jr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
            mujoco.mj_jacSite(model, observed, jp, jr, site_id(model, "flange_site"))
            full_load = (jp.T @ pre.contact_force_world_n
                         + jr.T @ pre.contact_torque_at_flange_world_nm)
            reduced = (full_load[joints]
                       - observed_mass[np.ix_(joints, base)] @ np.linalg.solve(
                           observed_mass[np.ix_(base, base)], full_load[base]))
            requested = CELLS[cell] * reduced
            protected = np.clip(requested, -0.1*spec.torque_limits_nm, 0.1*spec.torque_limits_nm)
            expected = np.clip(raw-protected, -spec.torque_limits_nm, spec.torque_limits_nm)
            for key, actual, archived in (
                ("original_raw_torque_nm", raw, s["original_raw_torque_nm"][i]),
                ("reduced_load_nm", reduced, s["reduced_contact_load_nm"][i]),
                ("protected_compensation_nm", protected, s["protected_compensation_nm"][i]),
                ("applied_torque_nm", expected, t["torque"][i]),
            ):
                max_errors[key] = max(max_errors[key], float(np.max(np.abs(actual-archived))))
            power = float(full_load @ observed.qvel - pre.contact_force_world_n @ (jp @ observed.qvel)
                          - pre.contact_torque_at_flange_world_nm @ (jr @ observed.qvel))
            max_errors["power_identity_w"] = max(max_errors["power_identity_w"], abs(power))
            h = observed.qfrc_bias-observed.qfrc_passive-observed.qfrc_applied
            full_residual = (observed_mass @ observed.qacc + h
                             - observed.qfrc_actuator - observed.qfrc_constraint)
            max_errors["full_dynamics_identity"] = max(max_errors["full_dynamics_identity"], float(np.max(np.abs(full_residual))))
            other = observed.qfrc_constraint-full_load
            red_mass = (observed_mass[np.ix_(joints, joints)]
                        - observed_mass[np.ix_(joints, base)] @ np.linalg.solve(
                            observed_mass[np.ix_(base, base)], observed_mass[np.ix_(base, joints)]))
            red_h = h[joints] - observed_mass[np.ix_(joints, base)] @ np.linalg.solve(observed_mass[np.ix_(base, base)], h[base])
            red_other = other[joints] - observed_mass[np.ix_(joints, base)] @ np.linalg.solve(observed_mass[np.ix_(base, base)], other[base])
            residual = red_mass @ observed.qacc[joints] + red_h - observed.qfrc_actuator[joints] - reduced - red_other
            max_errors["reduced_dynamics_identity"] = max(max_errors["reduced_dynamics_identity"], float(np.max(np.abs(residual))))

            # Offline prediction only: control has already been computed from
            # the held-torque observation. No new-torque contact solve feeds it.
            predicted_data = copy.copy(data)
            predicted_data.ctrl[:] = t["torque"][i]
            mujoco.mj_forward(model, predicted_data)
            predicted_qvel = np.asarray(data.qvel).copy() + spec.timestep_s*np.asarray(predicted_data.qacc)
            data.ctrl[:] = t["torque"][i]
            mujoco.mj_step(model, data)
            post = observer.observe(data)
            prediction.append(np.asarray(post.qvel)-predicted_qvel)
            max_errors["qpos"] = max(max_errors["qpos"], float(np.max(np.abs(post.qpos-t["qpos"][i]))))
            max_errors["qvel"] = max(max_errors["qvel"], float(np.max(np.abs(post.qvel-t["qvel"][i]))))
            max_errors["contact_force_n"] = max(max_errors["contact_force_n"], abs(post.measured_normal_force_n-float(t["measured_normal_force_n"][i])))
    prediction_array = np.asarray(prediction)
    np.savez_compressed(output / "prediction_diagnostics.npz", qvel_next_minus_instantaneous_euler=prediction_array)
    checks = {
        "frozen_S_independent_torque_playback": bool(replay["passed"]),
        "independent_compensation_arithmetic": max(max_errors[k] for k in (
            "original_raw_torque_nm", "reduced_load_nm", "protected_compensation_nm", "applied_torque_nm")) <= 1e-8,
        "physics_replay": max_errors["qpos"] <= 1e-8 and max_errors["qvel"] <= 1e-8 and max_errors["contact_force_n"] <= 1e-5,
        "power_identity": max_errors["power_identity_w"] <= 1e-8,
        "same_state_full_dynamics_identity": max_errors["full_dynamics_identity"] <= 1e-7,
        "same_state_reduced_dynamics_identity": max_errors["reduced_dynamics_identity"] <= 1e-7,
        "trace_and_diagnostics_identity": metrics["n111"]["servo_diagnostics_sha256"] == sha256(output / "servo_diagnostics.npz"),
    }
    result = {
        "schema_version": "n111_contact_load_validation_v1", "cell": cell,
        "passed": all(checks.values()), "checks": checks, "maximum_errors": max_errors,
        "prediction_note": "offline 2 ms qvel increment minus instantaneous t_k acceleration from a copied state forwarded with new torque; finite-step integration/contact changes are included",
        "prediction_qvel_error_rms": float(np.sqrt(np.mean(prediction_array**2))),
        "prediction_qvel_error_max": float(np.max(np.abs(prediction_array))),
        "verifier_identity": {"path": "v6_mujoco/fpmfc/n111_validate.py",
                              "sha256": sha256(Path(__file__))},
        "frozen_S_replay": replay,
    }
    (output / "validation.json").write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    (output / "artifact_manifest.json").write_text(json.dumps({
        "schema_version": "n111_contact_load_artifacts_v1",
        "verification_status": "VERIFIED" if result["passed"] else "FAILED",
        "artifacts": {name: sha256(output / name) for name in (
            "trace.npz", "metrics.json", "config_snapshot.json", "outer_diagnostics.npz",
            "servo_diagnostics.npz", "execution_chain.npz", "execution_chain_summary.json",
            "prediction_diagnostics.npz", "validation.json")},
    }, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cell", choices=CELLS, required=True)
    args = parser.parse_args()
    print(json.dumps({"passed": validate_cell(args.cell)["passed"]}, indent=2))

"""Full free robot dynamics, copied observations, atomic evidence, no online projection."""
import copy
import json
import time

import mujoco
import numpy as np

from v6_mujoco.collision import build_collision_pairs, minimum_signed_distance
from v6_mujoco.model import PROJECT_ROOT, default_model_spec
from v6_mujoco.postgrasp.physics import (load_model, object_id, joint_slices, momenta,
    interface_observation, constraint_jacobian, body_jacobian, digest, transfer_c1)
from v6_mujoco.postgrasp.run import known_wrench_test, read_config
from v6_mujoco.postgrasp_calibration.common import pose_reference, pose, solver_diagnostics, runtime_contract
from .io import ROOT, OLD, MODEL, read, save, save_npz, design, stage_identity, identity, completed, begin_attempt, end_attempt, require_budget

STATE_SPEC = mujoco.mjtState.mjSTATE_INTEGRATION
PHASES = {"POSTGRASP": 0, "ARMED": 1, "LATCH": 2, "GRASP_VERIFY": 3, "DAMP_TRANSFER": 4, "HOLD": 5, "FAILED": 6}


def integration_state(model, data):
    values = np.empty(mujoco.mj_stateSize(model, STATE_SPEC))
    mujoco.mj_getState(model, data, values, STATE_SPEC)
    return values


def make_model(dt=.002, scenario=None):
    model = load_model(MODEL, dt)
    if scenario:
        target = object_id(model, mujoco.mjtObj.mjOBJ_BODY, "tumbling_target")
        model.body_mass[target] = scenario["target_mass_kg"]
        model.body_inertia[target] = scenario["target_inertia_kg_m2"]
        mujoco.mj_setConst(model, mujoco.MjData(model))
    return model


def interface_geoms(model):
    return [object_id(model, mujoco.mjtObj.mjOBJ_GEOM, name) for name in ("gripper_contact_pad", "target_contact_plate")]


def original_contact_masks():
    _, assets = default_model_spec()._xml_and_assets()
    xml = (PROJECT_ROOT / "models/flexiv_rizon4s_contact_scene.xml").read_text(encoding="utf-8").replace('meshdir="../assets/meshes"', 'meshdir="."')
    model = mujoco.MjModel.from_xml_string(xml, assets)
    ids = interface_geoms(model)
    return np.array([model.geom_contype[ids], model.geom_conaffinity[ids]])


def initialize(model, raw_c1=False):
    initial = read(OLD / "initial_state.json")
    data = mujoco.MjData(model)
    if raw_c1:
        cfg = read_config()
        transfer = transfer_c1(model, data, PROJECT_ROOT / cfg["source_trace"], PROJECT_ROOT / cfg["source_target_config"])
        # Restore raw imported state, not the projected state. transfer_c1 only
        # performs the named C1 mapping and forward calculation, never projection.
        if not np.allclose(data.qpos, initial["qpos_before"], atol=1e-14, rtol=0) or not np.allclose(data.qvel, initial["qvel_before"], atol=1e-14, rtol=0):
            raise RuntimeError("MODEL_IDENTITY_MISMATCH: original C1 state mapping")
        with np.load(PROJECT_ROOT / cfg["source_trace"]) as trace:
            data.ctrl[:] = trace["torque"][-1]
        data.eq_active[:] = False
        masks = original_contact_masks()
        gids = interface_geoms(model)
        model.geom_contype[gids], model.geom_conaffinity[gids] = masks
        data.qacc_warmstart[:] = 0  # New dynamic target has no inherited solver history; physical velocity is untouched.
    else:
        data.qpos[:] = initial["qpos_after"]
        data.qvel[:] = initial["qvel_after"]
        data.qacc_warmstart[:] = initial["qacc_warmstart_after"]
        data.ctrl[:] = 0
    reference_data = mujoco.MjData(model)
    reference_data.qpos[:] = initial["qpos_after"]
    mujoco.mj_forward(model, reference_data)
    return data, pose_reference(model, reference_data)


def inactive_contact_face(model, data):
    """Only the declared pad/plate contact rows; inactive weld contributes zero."""
    a, b = interface_geoms(model)
    contact_ids = [i for i in range(data.ncon) if {int(data.contact[i].geom1), int(data.contact[i].geom2)} == {a, b}]
    J = constraint_jacobian(model, data)
    rows = np.flatnonzero(np.isin(np.asarray(data.efc_id), contact_ids) & (np.asarray(data.efc_type) >= int(mujoco.mjtConstraint.mjCNSTR_CONTACT_FRICTIONLESS)))
    generalized = J[rows].T @ data.efc_force[rows] if len(rows) else np.zeros(model.nv)
    target_site = object_id(model, mujoco.mjtObj.mjOBJ_SITE, "target_grasp_site")
    tool_site = object_id(model, mujoco.mjtObj.mjOBJ_SITE, "postgrasp_tool_interface")
    point = data.site_xpos[target_site]
    jt = body_jacobian(model, data, int(model.site_bodyid[target_site]), point)
    jf = body_jacobian(model, data, int(model.site_bodyid[tool_site]), data.site_xpos[tool_site])
    target_v = joint_slices(model, "target_free_joint")[1]
    wrench = np.linalg.solve(jt[:, target_v].T, generalized[target_v])
    R = data.site_xmat[target_site].reshape(3, 3)
    force, moment = R.T @ wrench[:3], R.T @ wrench[3:]
    return {"force_grasp_n": force, "moment_grasp_nm": moment,
        "load_fraction": float(np.linalg.norm(force)/50 + np.linalg.norm(moment)/2),
        "relative_twist_world": (jt-jf) @ data.qvel,
        "constraint_power_w": float(generalized @ data.qvel),
        "generalized_reconstruction_error": 0., "action_reaction_residual_world": np.zeros(6)}


def observe(model, real, reference, pairs, H0=None):
    observed = copy.copy(real)
    mujoco.mj_forward(model, observed)
    physical = momenta(model, observed)
    active = bool(observed.eq_active[0])
    face = interface_observation(model, observed, "postgrasp_latch", "postgrasp_tool_interface", "target_grasp_site") if active else inactive_contact_face(model, observed)
    relative = pose(model, observed, reference)
    joints = [joint_slices(model, f"joint{i}") for i in range(1, 8)]
    joint_q = np.array([observed.qpos[q][0] for q, v in joints])
    joint_v = np.array([observed.qvel[v][0] for q, v in joints])
    base_w = np.array(physical["body_velocities"]["base_link_0"]["angular_velocity_world_rad_s"])
    target_w = np.array(physical["body_velocities"]["tumbling_target"]["angular_velocity_world_rad_s"])
    clearance, _ = minimum_signed_distance(model, observed, pairs)
    ids = interface_geoms(model)
    site_ids = [object_id(model, mujoco.mjtObj.mjOBJ_SITE, n) for n in ("postgrasp_tool_interface", "target_grasp_site")]
    solver = solver_diagnostics(model, observed) if active else {"equality_kkt_max_absolute": 0.}
    I = np.asarray(physical["locked_inertia_world_kg_m2"])
    omega_locked = np.linalg.solve(I, physical["angular_momentum_about_center_world_kg_m2_s"] if H0 is None else H0)
    row = {"time_s": float(real.time), "qpos": real.qpos.copy(), "qvel": real.qvel.copy(),
        "ctrl_nm": real.ctrl.copy(), "integration_state": integration_state(model, real), "qacc_warmstart": real.qacc_warmstart.copy(),
        "joint_position_rad": joint_q, "joint_velocity_rad_s": joint_v, "joint_acceleration_rad_s2": observed.qacc[[v.start for q,v in joints]].copy(),
        "target_omega_world_rad_s": target_w, "base_omega_world_rad_s": base_w,
        "target_base_relative_omega_world_rad_s": target_w-base_w,
        "linear_momentum_world_kg_m_s": physical["linear_momentum_world_kg_m_s"],
        "angular_momentum_about_center_world_kg_m2_s": physical["angular_momentum_about_center_world_kg_m2_s"],
        "locked_inertia_world_kg_m2": I, "omega_locked_prediction_world_rad_s": omega_locked,
        "total_center_world_m": physical["center_world_m"], "kinetic_energy_j": physical["kinetic_energy_j"],
        "relative_energy_j": physical["relative_energy_j"], "load_fraction": face["load_fraction"],
        "force_grasp_n": face["force_grasp_n"], "moment_grasp_nm": face["moment_grasp_nm"],
        "interface_translation_error_m": relative["translation_error_m"], "interface_rotation_error_deg": relative["rotation_error_deg"],
        "interface_translation_tool_m": relative["translation_error_tool_m"], "interface_relative_twist_world": face["relative_twist_world"],
        "tool_position_world_m": observed.site_xpos[site_ids[0]].copy(), "target_grasp_position_world_m": observed.site_xpos[site_ids[1]].copy(),
        "tool_rotation_world": observed.site_xmat[site_ids[0]].reshape(3,3).copy(), "target_grasp_rotation_world": observed.site_xmat[site_ids[1]].reshape(3,3).copy(),
        "actuator_power_w": float(observed.qfrc_actuator @ observed.qvel), "passive_power_w": float(observed.qfrc_passive @ observed.qvel),
        "all_constraint_power_w": float(observed.qfrc_constraint @ observed.qvel), "latch_constraint_power_w": face["constraint_power_w"] if active else 0.,
        "minimum_noncontact_clearance_m": float(clearance), "solver_residual": solver["equality_kkt_max_absolute"],
        "wrench_reconstruction_error": face["generalized_reconstruction_error"],
        "action_reaction_error": float(np.linalg.norm(face["action_reaction_residual_world"])),
        "actuator_force_nm": observed.actuator_force.copy(), "qfrc_applied": real.qfrc_applied.copy(), "xfrc_applied": real.xfrc_applied.copy(),
        "eq_active": real.eq_active.copy(), "interface_geom_masks": np.array([model.geom_contype[ids], model.geom_conaffinity[ids]]),
        "solver_warning_count": np.array([w.number for w in observed.warning])}
    return row


def safety(row, initial, model, manifest, prediction=False):
    if not all(np.all(np.isfinite(v)) for v in row.values()):
        return "MODEL_NUMERICS_FAILED"
    gates = manifest["legacy_gates"]; cfg = manifest["design"]
    if np.any(row["solver_warning_count"]) or row["solver_residual"] > cfg["solver_residual_gate"] or row["wrench_reconstruction_error"] > 1e-8 or row["action_reaction_error"] > 1e-8:
        return "MODEL_NUMERICS_FAILED"
    P = np.linalg.norm(np.asarray(row["linear_momentum_world_kg_m_s"])-initial["linear_momentum_world_kg_m_s"])
    H = np.linalg.norm(np.asarray(row["angular_momentum_about_center_world_kg_m2_s"])-initial["angular_momentum_about_center_world_kg_m2_s"])
    if P > gates["absolute_linear_momentum_drift_kg_m_s"] or H > gates["absolute_angular_momentum_drift_kg_m2_s"]:
        return "MODEL_NUMERICS_FAILED"
    if np.any(row["qfrc_applied"]) or np.any(row["xfrc_applied"]):
        return "MODEL_NUMERICS_FAILED"
    if row["load_fraction"] > (cfg["governor"]["prediction_rho"] if prediction else 1.):
        return "LOAD_ENVELOPE_VIOLATION"
    if row["interface_translation_error_m"] > (cfg["governor"]["prediction_translation_m"] if prediction else gates["interface_translation_m"]) or row["interface_rotation_error_deg"] > (cfg["governor"]["prediction_rotation_deg"] if prediction else gates["interface_rotation_deg"]):
        return "POSE_HOLD_FAILED"
    for i in range(7):
        jid = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint{i+1}")
        if not model.jnt_range[jid,0]-1e-8 <= row["joint_position_rad"][i] <= model.jnt_range[jid,1]+1e-8 or abs(row["joint_velocity_rad_s"][i]) > gates["joint_velocity_rad_s"][i]+1e-8:
            return "JOINT_LIMIT_FAILED"
    limit = np.asarray(manifest["legacy_controller"]["torque_limits_nm"])
    if np.any(np.abs(row["ctrl_nm"]) > limit+1e-8) or np.any(np.abs(row["actuator_force_nm"]) > limit+1e-8):
        return "JOINT_LIMIT_FAILED"
    if row["minimum_noncontact_clearance_m"] < gates["minimum_noncontact_clearance_m"]:
        return "JOINT_LIMIT_FAILED"
    if row["relative_energy_j"] < -1e-7:
        return "MODEL_NUMERICS_FAILED"
    return None


def damping_command(model, data, manifest, alpha=1.):
    dofs = [joint_slices(model, f"joint{i}")[1].start for i in range(1,8)]
    coefficients = np.asarray(read(OLD / "damping.json")["coefficient_nm_s_rad"])
    limits = np.asarray(manifest["legacy_controller"]["torque_limits_nm"])
    return np.clip(-alpha*coefficients*data.qvel[dofs], -limits, limits)


def capture_passed(row, cfg):
    twist = np.asarray(row["interface_relative_twist_world"])
    return bool(row["interface_translation_error_m"] <= cfg["translation_m"] and row["interface_rotation_error_deg"] <= cfg["rotation_deg"] and
        np.linalg.norm(twist[:3]) <= cfg["relative_linear_speed_m_s"] and np.rad2deg(np.linalg.norm(twist[3:])) <= cfg["relative_angular_speed_deg_s"])


def arrays_from_rows(rows):
    return {key: np.asarray([r[key] for r in rows]) for key in rows[0]}


def metrics(arrays, status, manifest, model):
    t = arrays["time_s"]; window = (t >= 8.-1e-10) & (t <= 10.+1e-10)
    complete = status == "COMPLETED" and abs(float(t[-1])-10.) < 1e-8
    spin = np.rad2deg(np.linalg.norm(arrays["target_omega_world_rad_s"], axis=1))
    relative = np.rad2deg(np.linalg.norm(arrays["target_base_relative_omega_world_rad_s"], axis=1))
    P = arrays["linear_momentum_world_kg_m_s"]; H = arrays["angular_momentum_about_center_world_kg_m2_s"]
    Pd, Hd = np.linalg.norm(P-P[0],axis=1), np.linalg.norm(H-H[0],axis=1)
    result = {"status": status, "complete_10s": complete, "end_time_s": float(t[-1]), "sample_count": len(t),
        "evaluation_window_s": [8.,10.], "full_window_evaluated": complete and bool(np.any(window)),
        "target_last_2s_max_deg_s": float(np.max(spin[window])) if complete else None,
        "relative_last_2s_max_deg_s": float(np.max(relative[window])) if complete else None,
        "target_initial_deg_s": float(spin[0]), "target_final_deg_s": float(spin[-1]),
        "relative_final_deg_s": float(relative[-1]), "max_load_fraction": float(np.max(arrays["load_fraction"])),
        "max_translation_m": float(np.max(arrays["interface_translation_error_m"])), "max_rotation_deg": float(np.max(arrays["interface_rotation_error_deg"])),
        "max_P_drift_kg_m_s": float(np.max(Pd)), "max_H_drift_kg_m2_s": float(np.max(Hd)),
        "max_P_relative_drift": float(np.max(Pd)/max(np.linalg.norm(P[0]),1e-12)), "max_H_relative_drift": float(np.max(Hd)/max(np.linalg.norm(H[0]),1e-12)),
        "minimum_clearance_m": float(np.min(arrays["minimum_noncontact_clearance_m"])),
        "initial_joint_speed_norm_rad_s": float(np.linalg.norm(arrays["joint_velocity_rad_s"][0])), "final_joint_speed_norm_rad_s": float(np.linalg.norm(arrays["joint_velocity_rad_s"][-1])),
        "relative_energy_initial_j": float(arrays["relative_energy_j"][0]), "relative_energy_final_j": float(arrays["relative_energy_j"][-1]),
        "kinetic_energy_change_j": float(arrays["kinetic_energy_j"][-1]-arrays["kinetic_energy_j"][0]),
        "locked_spin_initial_deg_s": float(np.rad2deg(np.linalg.norm(arrays["omega_locked_prediction_world_rad_s"][0]))),
        "locked_spin_final_deg_s": float(np.rad2deg(np.linalg.norm(arrays["omega_locked_prediction_world_rad_s"][-1]))),
        "maximum_solver_residual": float(np.max(arrays["solver_residual"])), "all_actual_safety_passed": complete}
    for k in ("actuator_power_w", "passive_power_w", "all_constraint_power_w", "latch_constraint_power_w"):
        result[k.replace("power_w", "work_j")] = float(np.trapz(arrays[k], t))
    result["energy_balance_residual_j"] = result["kinetic_energy_change_j"] - sum(result[k] for k in ("actuator_work_j", "passive_work_j", "all_constraint_work_j"))
    result["performance_passed"] = bool(complete and result["target_last_2s_max_deg_s"] <= .1 and result["relative_last_2s_max_deg_s"] <= .02)
    route = manifest["design"]["controller_route"]
    joint_ranges = np.array([model.jnt_range[object_id(model,mujoco.mjtObj.mjOBJ_JOINT,f"joint{i}")] for i in range(1,8)])
    joint_margin = np.minimum(arrays["joint_position_rad"]-joint_ranges[:,0], joint_ranges[:,1]-arrays["joint_position_rad"])
    result["constraint_activity"] = {"load": result["max_load_fraction"] >= route["active_load_rho"],
        "pose": result["max_translation_m"] >= route["active_translation_m"] or result["max_rotation_deg"] >= route["active_rotation_deg"],
        "joint": bool(np.min(joint_margin) <= route["active_joint_margin_rad"]),
        "velocity": bool(np.max(np.abs(arrays["joint_velocity_rad_s"])/manifest["legacy_gates"]["joint_velocity_rad_s"]) >= route["active_velocity_fraction"]),
        "torque": bool(np.max(np.abs(arrays["ctrl_nm"])/manifest["legacy_controller"]["torque_limits_nm"]) >= route["active_torque_fraction"]),
        "clearance": result["minimum_clearance_m"] <= route["active_clearance_m"]}
    return result


def location(name):
    category = "baselines" if name.startswith("P_") else "governor" if name.startswith("G_") else "latch_transition" if name.startswith("L_") else "independent_validation"
    return ROOT / category / name


def run(name, controller="D", resume=False):
    manifest = design(); cfg = manifest["design"]
    modules = ["engine", "validation", "selftest"] + (["governor"] if controller == "G" else [])
    code = stage_identity(modules)
    output = location(name)
    existing = completed(output / "result.json", resume, code)
    if existing is not None:
        return existing
    dt = .001 if name.endswith("fine") else .002
    raw = name.startswith(("L_", "V_")); scenario = cfg["validation_scenarios"].get(name)
    model = make_model(dt, scenario); data, reference = initialize(model, raw)
    pairs = build_collision_pairs(model)
    begin_attempt("formal_runs", name, 10., dt, code)
    rows = []; events = []; decisions = []; status = "COMPLETED"; alpha = 0. if controller == "G" else 1.
    phase = "ARMED" if raw else "POSTGRASP"
    before = observe(model, data, reference, pairs)
    H0 = before["angular_momentum_about_center_world_kg_m2_s"]
    initial_snapshot = integration_state(model, data)
    save(output / "initial.json", {"integration_state": initial_snapshot.tolist(), "state_spec": int(STATE_SPEC),
        "raw_C1": raw, "velocity_projected_this_campaign": False, "scenario": scenario,
        "reference_rotation": reference["rotation"].tolist(), "reference_translation": reference["translation"].tolist(),
        "P0": before["linear_momentum_world_kg_m_s"], "H0": H0,
        "I_lock": before["locked_inertia_world_kg_m2"].tolist(), "omega_locked": before["omega_locked_prediction_world_rad_s"].tolist(),
        "interface_geom_masks": before["interface_geom_masks"].tolist(), "control_before_latch": data.ctrl.tolist()})
    governor = None
    if controller == "G":
        from .governor import Governor
        governor = Governor(manifest, reference)
    started = time.perf_counter()
    try:
        if raw:
            save(output / "armed_observation.json", {k: np.asarray(v).tolist() for k,v in before.items()})
            events.append({"time_s": 0., "state": "ARMED", "eq_active": False})
            if not capture_passed(before, cfg["latch"]):
                status = "LATCH_TRANSITION_FAILED"
            else:
                data.eq_active[:] = True
                gids = interface_geoms(model); model.geom_contype[gids] = 0; model.geom_conaffinity[gids] = 0
                events.extend([{"time_s": 0., "state": "LATCH", "eq_active": True}, {"time_s": 0., "state": "GRASP_VERIFY", "eq_active": True}])
                phase = "GRASP_VERIFY"
        steps = int(round(10./dt)); stride = int(round(.002/dt)); governance_stride = int(round(.02/dt))
        for step in range(steps+1):
            if step % 50 == 0:
                require_budget()
            if status == "COMPLETED" and step % stride == 0:
                if governor is not None and step % governance_stride == 0:
                    decision = governor.choose(model, data, alpha)
                    decisions.append(decision)
                    if not decision["feasible"]:
                        status = "NO_VERIFIED_ACTION"
                if status == "COMPLETED":
                    if governor is not None:
                        alpha = governor.alpha_at(float(data.time))
                    data.ctrl[:] = 0 if name == "P_Z" else damping_command(model, data, manifest, alpha)
            row = observe(model, data, reference, pairs, H0)
            row.update(alpha=float(alpha), phase_code=PHASES[phase])
            rows.append(row)
            failure = safety(row, before, model, manifest)
            if status == "COMPLETED" and failure:
                status = "LATCH_TRANSITION_FAILED" if raw and phase == "GRASP_VERIFY" else failure
                events.append({"time_s": float(data.time), "state": "FAILED", "reason": failure})
            if status != "COMPLETED" or step == steps:
                break
            if raw and phase == "GRASP_VERIFY" and data.time >= .05-1e-12:
                phase = "DAMP_TRANSFER"; events.append({"time_s": float(data.time), "state": phase})
            if raw and phase == "DAMP_TRANSFER" and np.rad2deg(np.linalg.norm(row["target_omega_world_rad_s"])) <= .1 and np.rad2deg(np.linalg.norm(row["target_base_relative_omega_world_rad_s"])) <= .02:
                phase = "HOLD"; events.append({"time_s": float(data.time), "state": phase})
            mujoco.mj_step(model, data)
            if abs(data.time-(step+1)*dt) > 1e-8:
                raise RuntimeError("MODEL_NUMERICS_FAILED: integration clock reset")
            if step and step % 1000 == 0:
                print(json.dumps({"run": name, "time_s": float(data.time), "rho": row["load_fraction"]}), flush=True)
        arrays = arrays_from_rows(rows)
        save_npz(output / "trace.npz", arrays)
        save(output / "events.json", events)
        save(output / "governor_decisions.json", decisions)
        result = metrics(arrays, status, manifest, model)
        result.update(name=name, controller=controller, timestep_s=dt, control_period_s=.002, elapsed_wall_s=time.perf_counter()-started,
            implementation_identity=code, input_identity=identity([ROOT / "campaign_manifest.json", ROOT / "source_identity.json", OLD / "initial_state.json", OLD / "damping.json"]),
            output_hashes={p.relative_to(ROOT).as_posix(): digest(p) for p in output.iterdir() if p.is_file() and p.name != "result.json"},
            legacy_full_boundary_contract="FAILED", raw_C1=raw, scenario=scenario, real_time_ready=False)
        save(output / "result.json", result)
        end_attempt("formal_runs", name, status, trace_sha256=digest(output / "trace.npz"), actual_duration_s=float(data.time), controller=controller)
        print(json.dumps({k: result[k] for k in ("name", "status", "end_time_s", "performance_passed", "max_load_fraction", "target_final_deg_s")}), flush=True)
        return result
    except Exception as error:
        if rows:
            save_npz(output / "trace.npz", arrays_from_rows(rows))
        save_npz(output / "exception_state.npz", {"integration_state": integration_state(model,data)})
        save(output / "exception.json", {"reason": repr(error), "time_s": float(data.time), "samples_saved": len(rows)})
        end_attempt("formal_runs", name, "ERROR", error=repr(error), actual_duration_s=float(data.time))
        raise

"""Freeze and execute the three allowed N200/N201 already-grasped runs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from v6_mujoco.collision import build_collision_pairs, minimum_signed_distance
from v6_mujoco.model import PROJECT_ROOT
from .physics import (body_jacobian, constraint_jacobian, damping_coefficients,
                      digest, equality_rows, interface_observation, joint_slices,
                      load_model, mass_matrix, momenta, object_id, project_velocity,
                      transfer_c1)

CONFIG = PROJECT_ROOT / "configs/fpmfc_n200_postgrasp.yaml"
OUTPUT = PROJECT_ROOT / "output/fpmfc/n200_postgrasp"
MODULE_PATHS = ["v6_mujoco/postgrasp/__init__.py", "v6_mujoco/postgrasp/physics.py",
                "v6_mujoco/postgrasp/run.py", "models/flexiv_rizon4s_postgrasp_scene.xml",
                "configs/fpmfc_n200_postgrasp.yaml"]


def save(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")


def read_config() -> dict:
    cfg = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    if cfg["schema_version"] != "n200_postgrasp_v1" or cfg["model_scope"] != "idealized_postgrasp_connection":
        raise RuntimeError("unexpected N200 model contract")
    source = PROJECT_ROOT / cfg["source_trace"]
    if digest(source) != cfg["source_trace_sha256"]:
        raise RuntimeError("C1 source trace hash mismatch")
    return cfg


def model_checks(model: mujoco.MjModel, cfg: dict) -> dict:
    eid = object_id(model, mujoco.mjtObj.mjOBJ_EQUALITY, cfg["interface"]["equality_name"])
    if (int(model.eq_obj1id[eid]) != object_id(model, mujoco.mjtObj.mjOBJ_BODY, "flange") or
        int(model.eq_obj2id[eid]) != object_id(model, mujoco.mjtObj.mjOBJ_BODY, "tumbling_target")):
        raise RuntimeError("latch does not connect the two dynamic bodies")
    if not (np.allclose(model.eq_solref[eid], cfg["interface"]["solref"]) and
            np.allclose(model.eq_solimp[eid, :3], cfg["interface"]["solimp"]) and
            abs(model.eq_data[eid, 10]-cfg["interface"]["torquescale_m"]) < 1e-12):
        raise RuntimeError("effective weld numerical parameters differ from frozen config")
    return {"equality_id": eid, "body1": "flange", "body2": "tumbling_target",
            "solref_actual": model.eq_solref[eid].tolist(),
            "solimp_actual": model.eq_solimp[eid].tolist(),
            "torquescale_actual_m": float(model.eq_data[eid, 10]),
            "collision_pair_delta": {"disabled": [cfg["interface"]["excluded_collision_pair"]],
                                     "other_collision_masks_changed": []}}


def known_wrench_test(model: mujoco.MjModel, data: mujoco.MjData, cfg: dict) -> dict:
    interface = cfg["interface"]
    rows = equality_rows(model, data, interface["equality_name"])
    J = constraint_jacobian(model, data)[rows]
    _, target = joint_slices(model, "target_free_joint")
    sid = object_id(model, mujoco.mjtObj.mjOBJ_SITE, interface["target_site"])
    tid = int(model.site_bodyid[sid])
    point = np.asarray(data.site_xpos[sid])
    jt = body_jacobian(model, data, tid, point)
    known = np.array([3.0,-4.0,2.0,0.1,-0.2,0.3])
    lam = np.linalg.solve(J[:,target].T, jt[:,target].T @ known)
    generalized = J.T@lam
    recovered = np.linalg.solve(jt[:,target].T, generalized[target])
    fs = object_id(model, mujoco.mjtObj.mjOBJ_SITE, interface["tool_site"])
    jf = body_jacobian(model, data, int(model.site_bodyid[fs]), point)
    robot = np.linalg.lstsq(jf[:,target.stop:].T, generalized[target.stop:], rcond=None)[0]
    rng = np.random.default_rng(200)
    velocity = rng.normal(size=model.nv)
    power_error = float(abs(lam@(J@velocity) - known@((jt-jf)@velocity)))
    result = {"known_wrench_world": known.tolist(), "recovered_world": recovered.tolist(),
              "known_wrench_error": float(np.linalg.norm(known-recovered)),
              "action_reaction_error": float(np.linalg.norm(known+robot)),
              "power_error_w": power_error, "tolerance": 1e-8,
              "method": "named_equality_J_transpose_then_physical_target_and_tool_Jacobians"}
    result["passed"] = all(result[k] <= result["tolerance"] for k in
                           ("known_wrench_error", "action_reaction_error", "power_error_w"))
    if not result["passed"]:
        raise RuntimeError(f"known wrench test failed: {result}")
    return result


def initialize(model: mujoco.MjModel, cfg: dict) -> tuple[mujoco.MjData, dict, dict]:
    data = mujoco.MjData(model)
    transfer = transfer_c1(model, data, PROJECT_ROOT/cfg["source_trace"], PROJECT_ROOT/cfg["source_target_config"])
    pre = momenta(model, data)
    pre_qpos, pre_qvel = data.qpos.copy(), data.qvel.copy()
    rows = equality_rows(model, data, cfg["interface"]["equality_name"])
    pos_residual = np.asarray(data.efc_pos)[rows].copy()
    if np.linalg.norm(pos_residual) > 1e-9:
        raise RuntimeError("weld position is inconsistent with C1 state")
    projected = project_velocity(model, data, cfg["interface"]["equality_name"])
    data.qvel[:] = projected["after"]
    mujoco.mj_forward(model, data)
    post = momenta(model, data)
    P_delta = np.linalg.norm(np.asarray(post["linear_momentum_world_kg_m_s"])-pre["linear_momentum_world_kg_m_s"])
    H_delta = np.linalg.norm(np.asarray(post["angular_momentum_about_center_world_kg_m2_s"])-pre["angular_momentum_about_center_world_kg_m2_s"])
    target_omega_pre = np.linalg.norm(pre["body_velocities"]["tumbling_target"]["angular_velocity_world_rad_s"])
    target_omega_post = np.linalg.norm(post["body_velocities"]["tumbling_target"]["angular_velocity_world_rad_s"])
    if (P_delta > 1e-8 or H_delta > 1e-8 or projected["kinetic_loss_j"] < -1e-10 or
        np.linalg.norm(projected["after_residual"]) > 1e-10 or np.rad2deg(target_omega_post) < 0.2):
        raise RuntimeError("synthetic initialization altered momentum, increased energy, or pre-spun the target")
    record = {"schema_version": "n200_initial_state_v1",
              "initialization": "synthetic_constraint_consistent_initialization",
              "transfer": transfer, "qpos_before": pre_qpos.tolist(), "qvel_before": pre_qvel.tolist(),
              "qpos_after": data.qpos.tolist(), "qvel_after": data.qvel.tolist(),
              "qacc_warmstart_after": data.qacc_warmstart.tolist(),
              "position_constraint_residual": pos_residual.tolist(),
              "velocity_constraint_residual_before": projected["before_residual"].tolist(),
              "velocity_constraint_residual_after": projected["after_residual"].tolist(),
              "kinetic_loss_j": projected["kinetic_loss_j"],
              "linear_momentum_change_kg_m_s": float(P_delta),
              "angular_momentum_change_kg_m2_s": float(H_delta),
              "target_omega_before_deg_s": float(np.rad2deg(target_omega_pre)),
              "target_omega_after_deg_s": float(np.rad2deg(target_omega_post)),
              "before_physics": pre, "after_physics": post}
    return data, record, damping_coefficients(model, data, cfg["interface"]["tool_site"],
                                               cfg["interface"]["target_site"],
                                               cfg["controller"]["nominal_damping_time_s"])


def observation(model: mujoco.MjModel, data: mujoco.MjData, cfg: dict, pairs) -> dict:
    physics = momenta(model, data)
    face = interface_observation(model, data, cfg["interface"]["equality_name"],
                                 cfg["interface"]["tool_site"], cfg["interface"]["target_site"])
    joint_q = [float(data.qpos[joint_slices(model,f"joint{i}")[0]][0]) for i in range(1,8)]
    joint_v = [float(data.qvel[joint_slices(model,f"joint{i}")[1]][0]) for i in range(1,8)]
    base_w = np.asarray(physics["body_velocities"]["base_link_0"]["angular_velocity_world_rad_s"])
    target_w = np.asarray(physics["body_velocities"]["tumbling_target"]["angular_velocity_world_rad_s"])
    clearance, pair_name = minimum_signed_distance(model, data, pairs)
    return {"time_s": float(data.time), "qpos": data.qpos.tolist(), "qvel": data.qvel.tolist(),
            "ctrl_nm": data.ctrl.tolist(), "joint_position_rad": joint_q,
            "joint_velocity_rad_s": joint_v,
            "base_omega_world_rad_s": base_w.tolist(),
            "target_omega_world_rad_s": target_w.tolist(),
            "target_base_relative_omega_world_rad_s": (target_w-base_w).tolist(),
            "minimum_noncontact_clearance_m": float(clearance), "minimum_clearance_pair": pair_name,
            "linear_momentum_world_kg_m_s": physics["linear_momentum_world_kg_m_s"],
            "angular_momentum_about_center_world_kg_m2_s": physics["angular_momentum_about_center_world_kg_m2_s"],
            "total_center_world_m": physics["center_world_m"],
            "locked_inertia_world_kg_m2": physics["locked_inertia_world_kg_m2"],
            "omega_locked_prediction_world_rad_s": physics["omega_locked_prediction_world_rad_s"],
            "kinetic_energy_j": physics["kinetic_energy_j"], "relative_energy_j": physics["relative_energy_j"],
            "interface": face,
            "actuator_power_w": float(data.qfrc_actuator@data.qvel),
            "passive_power_w": float(data.qfrc_passive@data.qvel),
            "all_constraint_power_w": float(data.qfrc_constraint@data.qvel)}


def violation(row: dict, initial: dict, model: mujoco.MjModel, cfg: dict) -> str | None:
    gates = cfg["gates"]
    P = np.linalg.norm(np.asarray(row["linear_momentum_world_kg_m_s"])-initial["linear_momentum_world_kg_m_s"])
    H = np.linalg.norm(np.asarray(row["angular_momentum_about_center_world_kg_m2_s"])-initial["angular_momentum_about_center_world_kg_m2_s"])
    if P > gates["absolute_linear_momentum_drift_kg_m_s"] or H > gates["absolute_angular_momentum_drift_kg_m2_s"]:
        return "PHYSICAL_MOMENTUM_VIOLATION"
    face = row["interface"]
    if face["load_fraction"] > 1:
        return "LOAD_ENVELOPE_VIOLATION"
    if face["translation_error_m"] > gates["interface_translation_m"] or face["rotation_error_deg"] > gates["interface_rotation_deg"]:
        return "INTERFACE_CONSTRAINT_VIOLATION"
    for i, (q,dq) in enumerate(zip(row["joint_position_rad"],row["joint_velocity_rad_s"]),1):
        jid = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint{i}")
        if q < model.jnt_range[jid,0]-1e-8 or q > model.jnt_range[jid,1]+1e-8 or abs(dq) > gates["joint_velocity_rad_s"][i-1]+1e-8:
            return "JOINT_SAFETY_VIOLATION"
    if row["minimum_noncontact_clearance_m"] < gates["minimum_noncontact_clearance_m"]:
        return "NONCONTACT_CLEARANCE_VIOLATION"
    if row["relative_energy_j"] < -1e-7:
        return "RELATIVE_ENERGY_DEFINITION_VIOLATION"
    return None


def run_condition(condition: str, cfg: dict, initial: dict, damping: dict, output: Path) -> dict:
    timestep = cfg["run"]["fine_timestep_s"] if condition == "D_fine" else cfg["run"]["standard_timestep_s"]
    model = load_model(PROJECT_ROOT/cfg["model_xml"], timestep)
    data = mujoco.MjData(model)
    data.qpos[:] = initial["qpos_after"]
    data.qvel[:] = initial["qvel_after"]
    data.qacc_warmstart[:] = initial["qacc_warmstart_after"]
    data.ctrl[:] = 0
    mujoco.mj_forward(model, data)
    pairs = build_collision_pairs(model)
    rows = []
    base = initial["after_physics"]
    nsteps = int(round(cfg["run"]["duration_s"]/timestep))
    update_stride = int(round(cfg["controller"]["update_period_s"]/timestep))
    joint_dofs = [joint_slices(model,f"joint{i}")[1].start for i in range(1,8)]
    status = "COMPLETED"
    for step in range(nsteps+1):
        if step % update_stride == 0:
            data.ctrl[:] = 0 if condition == "Z" else -np.clip(
                np.asarray(damping["coefficient_nm_s_rad"])*data.qvel[joint_dofs],
                -np.asarray(cfg["controller"]["torque_limits_nm"]),
                np.asarray(cfg["controller"]["torque_limits_nm"]))
            if data.ctrl@data.qvel[joint_dofs] > 1e-10:
                raise RuntimeError("joint damping supplied positive sample power")
        mujoco.mj_forward(model, data)
        row = observation(model, data, cfg, pairs)
        rows.append(row)
        status = violation(row, base, model, cfg) or "COMPLETED"
        if status != "COMPLETED" or step == nsteps:
            break
        mujoco.mj_step(model, data)
    output.mkdir(parents=True, exist_ok=True)
    arrays = {key: np.asarray([r[key] for r in rows]) for key in
              ("time_s", "qpos", "qvel", "ctrl_nm", "joint_position_rad", "joint_velocity_rad_s",
               "base_omega_world_rad_s", "target_omega_world_rad_s", "target_base_relative_omega_world_rad_s",
               "linear_momentum_world_kg_m_s", "angular_momentum_about_center_world_kg_m2_s",
               "kinetic_energy_j", "relative_energy_j", "actuator_power_w", "passive_power_w",
               "all_constraint_power_w", "minimum_noncontact_clearance_m")}
    arrays.update({"load_fraction": np.asarray([r["interface"]["load_fraction"] for r in rows]),
                   "force_grasp_n": np.asarray([r["interface"]["force_grasp_n"] for r in rows]),
                   "moment_grasp_nm": np.asarray([r["interface"]["moment_grasp_nm"] for r in rows]),
                   "interface_translation_error_m": np.asarray([r["interface"]["translation_error_m"] for r in rows]),
                   "interface_rotation_error_deg": np.asarray([r["interface"]["rotation_error_deg"] for r in rows]),
                   "interface_relative_twist_world": np.asarray([r["interface"]["relative_twist_world"] for r in rows]),
                   "latch_constraint_power_w": np.asarray([r["interface"]["constraint_power_w"] for r in rows])})
    np.savez_compressed(output/"trace.npz", **arrays)
    t = arrays["time_s"]
    integral = lambda k: float(np.trapz(arrays[k], t))
    P0 = np.asarray(base["linear_momentum_world_kg_m_s"])
    H0 = np.asarray(base["angular_momentum_about_center_world_kg_m2_s"])
    Pd = np.linalg.norm(arrays["linear_momentum_world_kg_m_s"]-P0,axis=1)
    Hd = np.linalg.norm(arrays["angular_momentum_about_center_world_kg_m2_s"]-H0,axis=1)
    target_spin = np.rad2deg(np.linalg.norm(arrays["target_omega_world_rad_s"],axis=1))
    relative_spin = np.rad2deg(np.linalg.norm(arrays["target_base_relative_omega_world_rad_s"],axis=1))
    final_window = t >= max(0,float(t[-1])-2)
    metrics = {"condition": condition, "status": status, "end_time_s": float(t[-1]),
               "steps": int(len(t)-1), "timestep_s": timestep,
               "control_period_s": cfg["controller"]["update_period_s"],
               "max_linear_momentum_drift_kg_m_s": float(np.max(Pd)),
               "max_angular_momentum_drift_kg_m2_s": float(np.max(Hd)),
               "max_linear_momentum_relative_drift": float(np.max(Pd)/max(np.linalg.norm(P0),1e-12)),
               "max_angular_momentum_relative_drift": float(np.max(Hd)/max(np.linalg.norm(H0),1e-12)),
               "max_load_fraction": float(np.max(arrays["load_fraction"])),
               "max_interface_translation_error_m": float(np.max(arrays["interface_translation_error_m"])),
               "max_interface_rotation_error_deg": float(np.max(arrays["interface_rotation_error_deg"])),
               "minimum_noncontact_clearance_m": float(np.min(arrays["minimum_noncontact_clearance_m"])),
               "initial_target_spin_deg_s": float(target_spin[0]),
               "final_target_spin_deg_s": float(target_spin[-1]),
               "final_two_seconds_target_spin_max_deg_s": float(np.max(target_spin[final_window])),
               "final_two_seconds_relative_spin_max_deg_s": float(np.max(relative_spin[final_window])),
               "initial_joint_speed_norm_rad_s": float(np.linalg.norm(arrays["joint_velocity_rad_s"][0])),
               "final_joint_speed_norm_rad_s": float(np.linalg.norm(arrays["joint_velocity_rad_s"][-1])),
               "kinetic_energy_change_j": float(arrays["kinetic_energy_j"][-1]-arrays["kinetic_energy_j"][0]),
               "integrated_actuator_work_j": integral("actuator_power_w"),
               "integrated_passive_work_j": integral("passive_power_w"),
               "integrated_latch_constraint_work_j": integral("latch_constraint_power_w"),
               "integrated_all_constraint_work_j": integral("all_constraint_power_w")}
    metrics["energy_balance_residual_j"] = metrics["kinetic_energy_change_j"] - sum(
        metrics[k] for k in ("integrated_actuator_work_j","integrated_passive_work_j","integrated_all_constraint_work_j"))
    metrics["target_spin_performance_passed"] = bool(status == "COMPLETED" and
        metrics["final_two_seconds_target_spin_max_deg_s"] <= cfg["gates"]["target_spin_deg_s"])
    metrics["relative_spin_performance_passed"] = bool(status == "COMPLETED" and
        metrics["final_two_seconds_relative_spin_max_deg_s"] <= cfg["gates"]["target_base_relative_spin_deg_s"])
    save(output/"metrics.json", metrics)
    save(output/"termination.json", {"status": status, "last_state": rows[-1]})
    return metrics


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--phase", choices=("prepare","Z","D","D_fine","compare"), required=True)
    args = parser.parse_args()
    cfg = read_config()
    OUTPUT.mkdir(parents=True, exist_ok=True)
    initial_path = OUTPUT/"initial_state.json"
    if args.phase == "prepare":
        model = load_model(PROJECT_ROOT/cfg["model_xml"])
        checks = model_checks(model,cfg)
        data, initial, damping = initialize(model,cfg)
        tests = known_wrench_test(model,data,cfg)
        identity = {p:digest(PROJECT_ROOT/p) for p in MODULE_PATHS}
        save(OUTPUT/"model_contract.json", {"model_scope": cfg["model_scope"],
            "dynamic_capture_validated": False, "hardware_load_rating_validated": False,
            "gripper_mass_added": False, "assumptions": cfg["interface"],
            "checks": checks, "implementation_identity": identity,
            "source_trace_sha256": cfg["source_trace_sha256"]})
        save(initial_path,initial)
        save(OUTPUT/"damping.json",damping)
        save(OUTPUT/"wrench_unit_tests.json",tests)
        save(OUTPUT/"momentum_feasibility.json", {"H_C_initial_world_kg_m2_s":
            initial["after_physics"]["angular_momentum_about_center_world_kg_m2_s"],
            "I_lock_initial_world_kg_m2":initial["after_physics"]["locked_inertia_world_kg_m2"],
            "omega_locked_prediction_world_rad_s":initial["after_physics"]["omega_locked_prediction_world_rad_s"]})
        save(OUTPUT/"experiment_manifest.json", {"config_sha256":digest(CONFIG),
            "model_sha256":digest(PROJECT_ROOT/cfg["model_xml"]),
            "source_trace_sha256":cfg["source_trace_sha256"],
            "implementation_identity":identity,"numerical_gates":cfg["gates"],
            "allowed_dynamic_conditions":cfg["run"]["conditions"]})
        save(OUTPUT/"run_ledger.json", {"formal_runs":[],"short_unit_tests":["known_wrench_algebra","small_model_weld"]})
        print(json.dumps({"phase":"prepare","damping":damping["coefficient_nm_s_rad"],
                          "initial_target_spin_deg_s":initial["target_omega_after_deg_s"]}))
        return
    if not initial_path.exists():
        raise RuntimeError("prepare must pass before dynamics")
    initial=json.loads(initial_path.read_text(encoding="utf-8"))
    if args.phase == "compare":
        metrics = {name:json.loads((OUTPUT/name/"metrics.json").read_text(encoding="utf-8"))
                   for name in cfg["run"]["conditions"] if (OUTPUT/name/"metrics.json").exists()}
        comparison = {"conditions":metrics,"interpretation":"already-grasped model only"}
        if "D" in metrics and "D_fine" in metrics:
            with np.load(OUTPUT/"D/trace.npz") as coarse, np.load(OUTPUT/"D_fine/trace.npz") as fine:
                comparison["step_sensitivity"] = {key:float(np.max(np.abs(coarse[key]-fine[key][::2][:len(coarse[key])])))
                    for key in ("qpos","qvel","target_omega_world_rad_s","load_fraction","kinetic_energy_j")}
        save(OUTPUT/"comparison.json",comparison)
        print(json.dumps({"phase":"compare","statuses":{k:v["status"] for k,v in metrics.items()}}))
        return
    if args.phase == "D_fine" and not (OUTPUT/"D/metrics.json").exists():
        raise RuntimeError("D must precede D_fine")
    if args.phase in ("D","D_fine") and not (OUTPUT/"Z/metrics.json").exists():
        raise RuntimeError("Z must precede damping runs")
    out = OUTPUT/args.phase
    if out.exists():
        raise FileExistsError(f"formal run already exists: {out}")
    damping=json.loads((OUTPUT/"damping.json").read_text(encoding="utf-8"))
    result=run_condition(args.phase,cfg,initial,damping,out)
    ledger_path=OUTPUT/"run_ledger.json"
    ledger=json.loads(ledger_path.read_text(encoding="utf-8"))
    ledger["formal_runs"].append({"condition":args.phase,"status":result["status"],
                                   "trace_sha256":digest(out/"trace.npz")})
    save(ledger_path,ledger)
    print(json.dumps({"condition":args.phase,"status":result["status"],"end_time_s":result["end_time_s"]}))


if __name__ == "__main__":
    main()

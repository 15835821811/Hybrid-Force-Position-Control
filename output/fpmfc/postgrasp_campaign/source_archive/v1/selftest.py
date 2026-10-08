"""Preflight and short physical checks before any full campaign trajectory."""
import copy
import json
import time

import mujoco
import numpy as np
from v6_mujoco.model import PROJECT_ROOT
from v6_mujoco.collision import build_collision_pairs
from v6_mujoco.postgrasp.physics import load_model, constraint_jacobian, equality_rows, digest, momenta
from v6_mujoco.postgrasp.run import known_wrench_test, read_config
from v6_mujoco.postgrasp_calibration.common import runtime_contract
from .io import ROOT, OLD, MODEL, read, save, save_npz, design, identity
from .engine import (make_model, initialize, observe, integration_state, damping_command,
    arrays_from_rows, metrics, safety, STATE_SPEC)
from .validation import replay_arrays


def run():
    manifest = design(); model = make_model(); data, reference = initialize(model)
    pairs = build_collision_pairs(model)
    prior = integration_state(model, data).copy()
    first = observe(model, data, reference, pairs)
    if not np.array_equal(prior, integration_state(model,data)):
        raise RuntimeError("observer changed live integration state")
    old = load_model(PROJECT_ROOT / "models/flexiv_rizon4s_n201_interface_candidate.xml")
    fields = ("body_mass", "body_inertia", "body_ipos", "body_iquat", "jnt_type", "jnt_qposadr", "jnt_dofadr", "jnt_range", "dof_damping", "dof_armature", "actuator_gear", "actuator_ctrlrange", "actuator_forcerange", "geom_contype", "geom_conaffinity", "site_pos", "site_quat", "eq_type", "eq_objtype", "eq_obj1id", "eq_obj2id", "eq_data", "eq_solref", "eq_solimp")
    assert all(np.array_equal(getattr(model,k),getattr(old,k)) for k in fields)
    synced = copy.copy(data); mujoco.mj_forward(model,synced)
    known = known_wrench_test(model,synced,read_config())
    J = constraint_jacobian(model,synced)[equality_rows(model,synced,"postgrasp_latch")]
    contracts = {}
    for dt in (.002,.001):
        m = make_model(dt); d, _ = initialize(m); mujoco.mj_forward(m,d)
        contracts[str(dt)] = runtime_contract(m,d)
        assert contracts[str(dt)]["refsafe"] and contracts[str(dt)]["timeconst_effective_s"] == .004
    if np.linalg.norm(synced.efc_pos) > 1e-9 or np.linalg.norm(J@data.qvel) > 1e-10 or np.linalg.matrix_rank(J) != 6:
        raise RuntimeError("full model initial state not constraint consistent")
    if safety(first,first,model,manifest):
        raise RuntimeError("MODEL_NUMERICS_FAILED: full-model initial safety preflight")
    # Three steps, separately counted as a short validator/observer unit fixture.
    initial = {"integration_state": prior.tolist(), "H0": first["angular_momentum_about_center_world_kg_m2_s"]}
    rows=[]
    for i in range(4):
        data.ctrl[:] = damping_command(model,data,manifest)
        before = integration_state(model,data)
        row = observe(model,data,reference,pairs,initial["H0"])
        assert np.array_equal(before,integration_state(model,data))
        row.update(alpha=1.,phase_code=0); rows.append(row)
        if i<3: mujoco.mj_step(model,data)
    arrays = arrays_from_rows(rows)
    replay = replay_arrays(make_model(),arrays,initial,reference,manifest)
    assert replay["passed"]
    # An early stop cannot acquire a shifted two-second evaluation window.
    partial = metrics(arrays,"COMPLETED",manifest,model)
    assert not partial["complete_10s"] and partial["target_last_2s_max_deg_s"] is None and not partial["performance_passed"]
    violated = copy.deepcopy(first); violated["load_fraction"] = 1.0000001
    assert safety(violated,first,model,manifest) == "LOAD_ENVELOPE_VIOLATION"
    report = {"passed": True, "known_wrench": known, "observer_nonintrusive": True,
        "unit_duration_s": .006, "unit_physics_steps": 3, "unit_replay_steps": 3,
        "short_recorded_input_replay": replay, "fixed_8_to_10s_window_test": True, "strict_actual_load_gate_test": True,
        "initial_constraint_position_norm": float(np.linalg.norm(synced.efc_pos)), "initial_constraint_velocity_norm": float(np.linalg.norm(J@data.qvel)),
        "constraint_rank": int(np.linalg.matrix_rank(J)), "protected_arrays_equal": list(fields),
        "runtime_contracts": contracts, "initial_observation": {k: np.asarray(v).tolist() for k,v in first.items()},
        "verification_identity": identity([__file__, PROJECT_ROOT / "v6_mujoco/postgrasp_campaign/engine.py", PROJECT_ROOT / "v6_mujoco/postgrasp_campaign/validation.py"])}
    # The initial velocity residual uses the frozen state, not the unit endpoint.
    report["initial_constraint_velocity_norm"] = float(np.linalg.norm(J@np.asarray(read(OLD/"initial_state.json")["qvel_after"])))
    save_npz(ROOT / "full_model_preflight/unit_trace.npz",arrays)
    save(ROOT / "full_model_preflight/result.json",report)
    ledger = read(ROOT / "run_ledger.json")
    ledger["unit_tests"].append({"kind": "copied observation, initial constraints, known wrench, recorded-input minimal replay, fixed performance window, capacity gate", "physics_steps":3,"replay_steps":3,"passed":True,"executed_epoch":time.time()})
    save(ROOT / "run_ledger.json",ledger)
    print(json.dumps({"full_model_preflight": "PASSED", "initial_rho": first["load_fraction"], "initial_locked_spin_deg_s": float(np.rad2deg(np.linalg.norm(first["omega_locked_prediction_world_rad_s"])))}))
    return report


if __name__ == "__main__":
    run()

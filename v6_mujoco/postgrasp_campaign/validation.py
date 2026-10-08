"""Independent recorded-input replay; never recomputes controller decisions."""
import json
import time

import mujoco
import numpy as np
from v6_mujoco.collision import build_collision_pairs
from v6_mujoco.postgrasp.physics import digest
from .io import ROOT, read, save, design, identity
from .engine import (make_model, observe, interface_geoms, STATE_SPEC, location, metrics, safety)


def replay_arrays(model, arrays, initial, reference, manifest):
    data = mujoco.MjData(model)
    mujoco.mj_setState(model, data, np.asarray(initial["integration_state"]), STATE_SPEC)
    pairs = build_collision_pairs(model); gids = interface_geoms(model)
    keys = ["qpos", "qvel", "force_grasp_n", "moment_grasp_nm", "load_fraction",
        "interface_translation_error_m", "interface_rotation_error_deg", "linear_momentum_world_kg_m_s", "angular_momentum_about_center_world_kg_m2_s"]
    errors = {k: 0. for k in keys}; reconstructed = []; first = None; first_violation = None
    for index, timestamp in enumerate(arrays["time_s"]):
        data.ctrl[:] = arrays["ctrl_nm"][index]
        data.eq_active[:] = arrays["eq_active"][index]
        model.geom_contype[gids], model.geom_conaffinity[gids] = arrays["interface_geom_masks"][index]
        row = observe(model, data, reference, pairs, initial["H0"])
        if abs(float(data.time)-float(timestamp)) > 1e-10:
            raise RuntimeError("VALIDATION_FAILED: replay timestamps")
        if first is None:
            first = row
        failure = safety(row, first, model, manifest)
        if failure and first_violation is None:
            first_violation = {"index": index, "time_s": float(data.time), "reason": failure}
        for key in keys:
            errors[key] = max(errors[key], float(np.max(np.abs(np.asarray(row[key])-arrays[key][index]))))
        reconstructed.append(row)
        if index < len(arrays["time_s"])-1:
            mujoco.mj_step(model, data)
    tolerance = manifest["design"]["replay_tolerances"]
    limits = {k: tolerance.get(k, tolerance["momentum"]) for k in keys}
    passed = all(errors[k] <= limits[k] for k in keys)
    return {"passed": passed, "maximum_absolute_differences": errors, "strict_tolerances": limits,
        "first_reconstructed_safety_failure": first_violation, "physics_steps": max(0,len(reconstructed)-1),
        "method": "recorded actuator ctrl and equality/contact-mask switch events; no controller/governor invocation"}


def validate_run(name):
    manifest = design(); output = location(name); result = read(output / "result.json")
    initial = read(output / "initial.json")
    reference = {"rotation": np.array(initial["reference_rotation"]), "translation": np.array(initial["reference_translation"])}
    with np.load(output / "trace.npz") as archive:
        arrays = {key: archive[key] for key in archive.files}
    model = make_model(result["timestep_s"], result["scenario"])
    replay = replay_arrays(model, arrays, initial, reference, manifest)
    recalculated = metrics(arrays, result["status"], manifest, model)
    differences = {k: {"saved": result[k], "recomputed": v} for k,v in recalculated.items() if result[k] != v}
    replay["metrics_match"] = not differences
    replay["metric_differences"] = differences
    failure = replay["first_reconstructed_safety_failure"]
    replay["no_unstopped_safety_failure"] = failure is None or failure["index"] == len(arrays["time_s"])-1
    replay["passed"] &= not differences and replay["no_unstopped_safety_failure"]
    replay.update(name=name, trace_sha256=digest(output / "trace.npz"),
        verification_identity=identity([__file__]), execution_identity=result["implementation_identity"])
    # Each replay invocation is independently accounted, including validate-only.
    ledger = read(ROOT / "run_ledger.json")
    ledger["replays"].append({"name": name, "kind": "same_step_recorded_input", "physics_steps": replay["physics_steps"], "passed": replay["passed"], "executed_epoch": time.time(), "verification_identity": replay["verification_identity"]})
    save(ROOT / "run_ledger.json", ledger)
    save(output / "validation.json", replay)
    if not replay["passed"]:
        raise RuntimeError(f"VALIDATION_FAILED: {name}")
    print(json.dumps({"replay": name, "passed": True, "max_qpos_error": replay["maximum_absolute_differences"]["qpos"]}), flush=True)
    return replay


def sensitivity(coarse_name, fine_name):
    m = design(); a_path = location(coarse_name); b_path = location(fine_name)
    a_result, b_result = read(a_path / "result.json"), read(b_path / "result.json")
    with np.load(a_path / "trace.npz") as a, np.load(b_path / "trace.npz") as b:
        keys = {"interface_translation_m": "interface_translation_error_m", "interface_rotation_deg": "interface_rotation_error_deg", "load_fraction": "load_fraction"}
        common_length = min(len(a["time_s"]), len(b["time_s"][::2]))
        differences = {label: float(np.max(np.abs(a[key][:common_length]-b[key][::2][:common_length]))) for label,key in keys.items()}
        for label,key in (("target_spin_deg_s", "target_omega_world_rad_s"), ("relative_spin_deg_s", "target_base_relative_omega_world_rad_s")):
            differences[label] = float(np.max(np.abs(np.rad2deg(np.linalg.norm(a[key][:common_length],axis=1))-np.rad2deg(np.linalg.norm(b[key][::2][:common_length],axis=1)))))
        all_curve_differences = {key: float(np.max(np.abs(a[key][:common_length]-b[key][::2][:common_length]))) for key in ("qpos", "qvel", "force_grasp_n", "moment_grasp_nm", "kinetic_energy_j", "linear_momentum_world_kg_m_s", "angular_momentum_about_center_world_kg_m2_s")}
    passed = a_result["complete_10s"] and b_result["complete_10s"] and a_result["performance_passed"] and b_result["performance_passed"] and all(differences[k] <= limit for k,limit in m["design"]["step_sensitivity_gates"].items())
    result = {"passed": bool(passed), "coarse": coarse_name, "fine": fine_name, "full_curve_differences": differences,
        "other_full_curve_differences": all_curve_differences, "gates": m["design"]["step_sensitivity_gates"], "control_period_both_s": .002,
        "common_samples": common_length, "verification_identity": identity([__file__])}
    save(b_path / "step_sensitivity.json", result)
    return result


def validate_all():
    ledger = read(ROOT / "run_ledger.json")
    for attempt in ledger["formal_runs"]:
        if (location(attempt["name"]) / "result.json").exists():
            validate_run(attempt["name"])

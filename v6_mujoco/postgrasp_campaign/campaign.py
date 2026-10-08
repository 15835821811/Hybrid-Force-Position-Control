"""Automatic preregistered stage routing; result gates control every transition."""
import json
from pathlib import Path

from v6_mujoco.postgrasp.physics import digest
from .io import ROOT, OLD, read, save, design, check_identity, identity
from . import engine, restricted, validation, selftest


def verified_run(name, controller, resume):
    result = engine.run(name, controller, resume)
    path = engine.location(name) / "validation.json"
    if resume and path.exists():
        verification = read(path)
        check_identity(verification["verification_identity"])
        if not verification["passed"] or verification["trace_sha256"] != digest(engine.location(name)/"trace.npz") or verification["execution_identity"] != result["implementation_identity"]:
            raise RuntimeError(f"VALIDATION_FAILED: resume {name}")
    else:
        validation.validate_run(name)
    return result


def skip(names, reason):
    ledger = read(ROOT / "run_ledger.json")
    for name in names:
        if not any(row["name"] == name for row in ledger["formal_runs"]):
            ledger["skipped"][name] = reason
    save(ROOT / "run_ledger.json", ledger)
    save(ROOT / "execution_status.json", {"reason": reason, "skipped": ledger["skipped"], "formal_run_count": len(ledger["formal_runs"])})
    print(json.dumps({"campaign_stop_or_skip": reason, "conditions": names}), flush=True)


def select(controller, coarse, fine):
    path = ROOT / "selected_controller.json"
    payload = {"controller": controller, "coarse_evidence": coarse, "fine_evidence": fine,
        "damping_sha256": digest(OLD / "damping.json"), "parameter_adaptation_after_selection": False,
        "evidence_identity": identity([engine.location(n)/"result.json" for n in (coarse,fine)]+[engine.location(n)/"validation.json" for n in (coarse,fine)]+[engine.location(fine)/"step_sensitivity.json"])}
    if path.exists() and read(path) != payload:
        raise RuntimeError("MODEL_IDENTITY_MISMATCH: frozen controller selection")
    save(path,payload)


def execute(stage="all", resume=False):
    manifest = design()
    if restricted.run(resume=True if stage != "S" else resume)["status"] != "VERIFIED_IN_TESTED_CASES":
        skip(manifest["formal_conditions"], "RESTRICTED_DOMAIN_FAILED"); return
    preflight = ROOT / "full_model_preflight/result.json"
    if preflight.exists():
        check_identity(read(preflight)["verification_identity"])
        if not read(preflight)["passed"]:
            skip(manifest["formal_conditions"], "MODEL_NUMERICS_FAILED"); return
    else:
        selftest.run()
    z = verified_run("P_Z", "Z", resume)
    if z["status"] == "MODEL_NUMERICS_FAILED":
        skip(manifest["formal_conditions"][1:], "MODEL_NUMERICS_FAILED in P_Z"); return
    d = verified_run("P_D", "D", resume)
    if d["performance_passed"]:
        fine = verified_run("P_D_fine", "D", resume)
        if not validation.sensitivity("P_D", "P_D_fine")["passed"]:
            skip(["G_nominal","G_fine","L_nominal","L_fine","V_light","V_heavy"], "VALIDATION_FAILED: P_D fine"); return
        select("D", "P_D", "P_D_fine")
        skip(["G_nominal","G_fine"], "NOT_NEEDED: unchanged D passed")
        controller = "D"
    else:
        if d["status"] == "MODEL_NUMERICS_FAILED":
            skip(["P_D_fine","G_nominal","G_fine","L_nominal","L_fine","V_light","V_heavy"], "MODEL_NUMERICS_FAILED in P_D"); return
        if d["status"] == "POSE_HOLD_FAILED" and d["max_load_fraction"] < manifest["design"]["controller_route"]["low_load_pose_failure_rho"]:
            skip(["P_D_fine","G_nominal","G_fine","L_nominal","L_fine","V_light","V_heavy"], "MODEL_NUMERICS_FAILED: full model compliance at low load; governor cannot mask it"); return
        need_governor = d["status"] in ("LOAD_ENVELOPE_VIOLATION","POSE_HOLD_FAILED","JOINT_LIMIT_FAILED") or (d["complete_10s"] and any(d["constraint_activity"].values()))
        if not need_governor:
            skip(["P_D_fine","G_nominal","G_fine","L_nominal","L_fine","V_light","V_heavy"], "PERFORMANCE_NOT_REACHED without active constraint evidence"); return
        skip(["P_D_fine"], "P_D failed; route to preregistered G, no failed-baseline fine retry")
        if not (Path(__file__).parent / "governor.py").exists():
            save(ROOT / "execution_status.json", {"reason":"G_IMPLEMENTATION_REQUIRED", "cause":d["status"], "formal_run_count":len(read(ROOT/"run_ledger.json")["formal_runs"])})
            print("G_IMPLEMENTATION_REQUIRED",flush=True); return
        g = verified_run("G_nominal", "G", resume)
        if not g["performance_passed"]:
            skip(["G_fine","L_nominal","L_fine","V_light","V_heavy"], g["status"] if g["status"] != "COMPLETED" else "PERFORMANCE_NOT_REACHED"); return
        fine = verified_run("G_fine", "G", resume)
        if not validation.sensitivity("G_nominal","G_fine")["passed"]:
            skip(["L_nominal","L_fine","V_light","V_heavy"], "VALIDATION_FAILED: G fine"); return
        select("G", "G_nominal", "G_fine"); controller = "G"
    if stage in ("P","G"):
        return
    nominal = verified_run("L_nominal", controller, resume)
    if not nominal["performance_passed"]:
        skip(["L_fine","V_light","V_heavy"], nominal["status"] if nominal["status"] != "COMPLETED" else "PERFORMANCE_NOT_REACHED"); return
    fine = verified_run("L_fine", controller, resume)
    if not validation.sensitivity("L_nominal","L_fine")["passed"]:
        skip(["V_light","V_heavy"], "VALIDATION_FAILED: L fine"); return
    if stage == "L":
        return
    for name in ("V_light","V_heavy"):
        verified_run(name,controller,resume)
    save(ROOT / "execution_status.json", {"reason":"ALL_ADMITTED_STAGES_EXECUTED", "formal_run_count":len(read(ROOT/"run_ledger.json")["formal_runs"])})

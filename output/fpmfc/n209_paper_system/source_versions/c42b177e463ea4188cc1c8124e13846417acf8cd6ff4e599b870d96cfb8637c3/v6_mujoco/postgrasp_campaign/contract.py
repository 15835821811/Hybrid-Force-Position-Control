"""Freeze all conditional branches before the first campaign run."""
import platform
import time
from datetime import datetime, timezone

import mujoco
import numpy as np
import scipy
import yaml

from v6_mujoco.model import PROJECT_ROOT
from v6_mujoco.postgrasp.physics import digest
from .io import ROOT, OLD, CAL, CONFIG, MODEL, read, save, identity, check_identity


def prepare():
    if (ROOT / "campaign_manifest.json").exists():
        from .io import design
        design()
        return read(ROOT / "campaign_manifest.json")
    cfg = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    old = read(OLD / "experiment_manifest.json")
    cal = read(CAL / "experiment_manifest.json")
    check_identity(old["implementation_identity"])
    check_identity(cal["implementation_identity"])
    paths = [OLD / name for name in cal["historical_artifact_hashes"]]
    for name, expected in cal["historical_artifact_hashes"].items():
        if digest(OLD / name) != expected:
            raise RuntimeError(f"MODEL_IDENTITY_MISMATCH: historical {name}")
    for name, expected in cal["output_hashes"].items():
        if digest(CAL / name) != expected:
            raise RuntimeError(f"MODEL_IDENTITY_MISMATCH: calibration {name}")
    original = yaml.safe_load((PROJECT_ROOT / "configs/fpmfc_n200_postgrasp.yaml").read_text(encoding="utf-8"))
    source = PROJECT_ROOT / original["source_trace"]
    if source.read_bytes()[:2] != b"PK" or digest(source) != old["source_trace_sha256"]:
        raise RuntimeError("MISSING_SOURCE: C1 must be hydrated original NPZ")
    candidate = PROJECT_ROOT / "models/flexiv_rizon4s_n201_interface_candidate.xml"
    parameters = read(CAL / "calibrated_connection.json")["parameters"]
    assert parameters["solref"] == [.004, 1] and parameters["torquescale_m"] == .15
    assert read(CAL / "calibrated_connection.json")["status"] == "CONNECTION_MODEL_NOT_QUALIFIED"
    MODEL.write_bytes(candidate.read_bytes())
    paths += [source, candidate, MODEL, PROJECT_ROOT / "configs/fpmfc_n200_postgrasp.yaml",
        PROJECT_ROOT / original["source_target_config"], CAL / "calibrated_connection.json",
        CAL / "load_test_manifest.json", CAL / "load_tests/candidate_2_balanced/results.json",
        PROJECT_ROOT / "paper/N200_N201_POSTGRASP_BASELINE_REPORT.md",
        PROJECT_ROOT / "paper/N201_INTERFACE_CALIBRATION_REPORT.md"]
    files = dict(old["implementation_identity"]); files.update(cal["implementation_identity"]); files.update(identity(paths))
    legacy_cases = read(CAL / "load_test_manifest.json")["cases"]
    results = read(CAL / "load_tests/candidate_2_balanced/results.json")["results"]
    worst_load = max(results, key=lambda r: r["max_interface_load_fraction"])
    worst_position = max((r for r in results if r["case"] != worst_load["case"]), key=lambda r: r["maximum_translation_m"])
    worst_mixed = max((r for r in results if r["case"].startswith("mixed") and r["case"] not in [worst_load["case"], worst_position["case"]]), key=lambda r: r["max_interface_load_fraction"])
    selected = [worst_load["case"], worst_position["case"], worst_mixed["case"]]
    new_cases = [{"name": c["name"], "wrench_grasp": (np.asarray(c["wrench_grasp"]) * cfg["fixture_scale"]).tolist()} for c in legacy_cases]
    load_contract = read(CAL / "load_test_manifest.json")
    load_contract.update(cases=new_cases, fine_cases=selected, input_scale=.8,
        peak_force_n=40., peak_moment_nm=1.6,
        maximum_profile_derivative_per_s=1.875 / .2,
        maximum_pure_force_rate_n_s=375., maximum_pure_moment_rate_nm_s=15.,
        fine_case_selection={"worst_actual_rho": worst_load, "largest_translation_excluding_first": worst_position, "worst_mixed": worst_mixed})
    save(ROOT / "restricted_fixture/input_contract.json", load_contract)
    save(ROOT / "source_identity.json", {"files": files, "legacy_full_boundary_contract": "FAILED",
        "path_mapping": {"historical_repository_root": str(PROJECT_ROOT), "policy": "resolve repository-relative paths externally; never rewrite historical documents"},
        "C1_npz_hydrated": True})
    routes = {
        "S": "19 fixed scaled fixture tests; any failure blocks P/G/L/V",
        "P_Z": "baseline; load/joint/pose safety stop permits P_D only if model observation/numerics valid",
        "P_D_success": "P_D_fine; then freeze D; then L_nominal/L_fine; then both V cases; skip G",
        "P_D_constraint_failure": "G permitted for actual load/joint/velocity limits or pose with rho>=0.4, if model numerics valid",
        "P_D_safe_performance_failure": "stop upgrade unless preregistered active constraint evidence exists",
        "P_D_low_load_pose_failure": "full model compliance investigation; block G/L/V rather than conceal it",
        "G": "only when routed; G_nominal then G_fine; failure blocks L/V; no tuning",
        "L": "raw C1 without velocity projection; immediate capture gate; safety from latch sample; nominal then fine",
        "V": "only after both L pass; both density scenarios independently; fixed nominal controller model",
        "any_identity_validation_numerics_error": "stop affected dependencies and retain atomic traces",
        "budget": "4 hours from campaign task start; checkpoint, no background promise"}
    manifest = {"schema_version": "postgrasp_campaign_v1", "frozen_utc": datetime.now(timezone.utc).isoformat(),
        "baseline_commit": cfg["baseline_commit"], "branch": "codex/n201s-to-n204-grasp-detumbling",
        "config_sha256": digest(CONFIG), "source_identity_sha256": digest(ROOT / "source_identity.json"),
        "resource_started_epoch": 1791457602, "resource_budget_s": cfg["resource_budget_s"],
        "design": cfg, "legacy_gates": original["gates"], "legacy_controller": original["controller"],
        "connection_parameters": parameters, "model_sha256": digest(MODEL),
        "restricted_fixture_contract_sha256": digest(ROOT / "restricted_fixture/input_contract.json"),
        "routes": routes, "formal_conditions": ["P_Z", "P_D", "P_D_fine", "G_nominal", "G_fine", "L_nominal", "L_fine", "V_light", "V_heavy"],
        "environment": {"python": platform.python_version(), "mujoco": mujoco.__version__, "numpy": np.__version__, "scipy": scipy.__version__},
        "observer": "full MjData copy; mj_forward only on observer; real mj_step once per physics step; held ctrl between control ticks",
        "legacy_full_boundary_contract": "FAILED", "hardware_load_rating_validated": False,
        "physical_gripper_capture_validated": False, "stage_runtime_policy": "exact transitive stage implementation/verification identities frozen before each stage; later optional modules do not replace completed runtime"}
    save(ROOT / "campaign_manifest.json", manifest)
    save(ROOT / "run_ledger.json", {"fixture_runs": [], "formal_runs": [], "replays": [], "unit_tests": [], "predictions": [], "skipped": {}})
    return manifest

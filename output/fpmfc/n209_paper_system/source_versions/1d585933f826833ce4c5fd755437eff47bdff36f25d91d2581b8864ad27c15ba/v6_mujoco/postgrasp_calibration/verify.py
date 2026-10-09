"""Static completion audit and manifest sealing; never starts a trajectory."""

import argparse
import json

import mujoco
import numpy as np

from v6_mujoco.model import PROJECT_ROOT
from v6_mujoco.postgrasp.physics import digest, load_model
from v6_mujoco.postgrasp.run import known_wrench_test, read_config
from .common import ROOT, OLD, read, save


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def inspect():
    manifest = read(ROOT / "experiment_manifest.json")
    for relative, expected in manifest["implementation_identity"].items():
        require(digest(PROJECT_ROOT / relative) == expected, f"implementation changed: {relative}")
    old_manifest = read(OLD / "experiment_manifest.json")
    for relative, expected in old_manifest["implementation_identity"].items():
        require(digest(PROJECT_ROOT / relative) == expected, f"historical implementation changed: {relative}")
    for relative, expected in manifest["historical_artifact_hashes"].items():
        require(digest(OLD / relative) == expected, f"historical output changed: {relative}")
    cfg = read_config()  # Includes the original C1 trace byte identity check.
    result = read(ROOT / "calibrated_connection.json")
    require(result["status"] == "CONNECTION_MODEL_NOT_QUALIFIED", "wrong final status")
    require(result["holding_precision_passed"] and not result["load_envelope_passed"], "qualification gates conflated")
    require(result["initial_state_sha256"] == digest(OLD / "initial_state.json"), "initial state changed")
    require(result["damping_sha256"] == digest(OLD / "damping.json"), "damping changed")
    require(not result["initial_state_reprojected"], "unnecessary reprojection")
    provisional = read(ROOT / "candidate_2.json")["parameters"]
    corrected = read(ROOT / "candidate_2_reference_correction.json")["parameters"]
    require(provisional == corrected, "reference correction introduced another physical parameter set")
    require(corrected == result["parameters"], "selected configuration differs")

    ledger = read(ROOT / "run_ledger.json")
    require(len(ledger["interface_tests"]) == 8, "missing attempted suite")
    require(sum(s["cases"] for s in ledger["interface_tests"]) == 128, "missing fixture tests")
    require(sum(s["cases"] for s in ledger["independent_replays"]) == 16, "missing separate replays")
    require(not ledger["formal_runs"] and not ledger["complete_robot_dynamics_attempts"], "unexpected formal trajectory")
    for suite in ledger["interface_tests"]:
        require(digest(PROJECT_ROOT / suite["runtime_path"]) == suite["runtime_sha256"], "attempt runtime identity mismatch")
        directory = ROOT / "load_tests" / suite["suite"]
        require(len(list(directory.glob("*.npz"))) == suite["cases"], "missing attempted raw trace")
    for condition in ("Z_new", "D_new", "D_new_fine"):
        require(not (ROOT / condition / "trace.npz").exists(), "forbidden new robot run")
        require(read(ROOT / condition / "skipped.json")["formal_trajectory_count"] == 0, "skip record mismatch")

    spec = read(ROOT / "load_test_manifest.json")
    require(spec["gates"]["translation_m"] == .0005 and spec["gates"]["rotation_deg"] == .1, "holding gates changed")
    require(spec["load_envelope"] == {"force_n": 50, "moment_nm": 2, "provenance": "simulation_assumption"}, "load envelope changed")
    require(len(spec["cases"]) == 16 and spec["candidate_budget"] == 2, "test budget changed")
    for case in spec["cases"]:
        w = np.asarray(case["wrench_grasp"])
        require(np.linalg.norm(w[:3]) / 50 + np.linalg.norm(w[3:]) / 2 <= 1, "test input exceeds envelope")

    stats = {}; events = []; sensitivity = {}
    for label in ("original_balanced", "candidate_1_balanced", "candidate_2_balanced", "candidate_2_balanced_fine"):
        suite = read(ROOT / "load_tests" / label / "results.json")
        rows = suite["results"]
        stats[label] = {key: max(r[key] for r in rows) for key in (
            "maximum_translation_m", "maximum_rotation_deg", "max_interface_load_fraction",
            "linear_momentum_drift", "angular_momentum_drift", "maximum_net_force_moment", "maximum_solver_residual")}
        if label.startswith("candidate_2"):
            for case in spec["cases"]:
                with np.load(ROOT / "load_tests" / label / (case["name"] + ".npz")) as trace:
                    require(all(np.all(np.isfinite(trace[k])) for k in trace.files), "non-finite qualified fixture trace")
                    fraction = np.linalg.norm(trace["interface_wrench_grasp"][:, :3], axis=1) / 50 + np.linalg.norm(trace["interface_wrench_grasp"][:, 3:], axis=1) / 2
                    require(np.max(np.abs(fraction - trace["interface_load_fraction"])) < 1e-12, "wrench envelope reconstruction mismatch")
                    index = int(np.argmax(fraction))
                    events.append({"suite": label, "case": case["name"], "time_s": float(trace["time_s"][index]),
                        "fraction": float(fraction[index]), "wrench_grasp": trace["interface_wrench_grasp"][index].tolist(),
                        "commanded_wrench_grasp": trace["commanded_wrench_grasp"][index].tolist()})
    peak = max(e["fraction"] for e in events)
    require(peak == result["maximum_measured_load_fraction"] and peak > 1, "actual load failure not reproduced")
    for case in spec["cases"]:
        filename = case["name"] + ".npz"
        with np.load(ROOT / "load_tests/candidate_2_balanced" / filename) as a, np.load(ROOT / "load_tests/candidate_2_balanced_fine" / filename) as b:
            delta = a["interface_wrench_grasp"] - b["interface_wrench_grasp"][::2]
            sensitivity[case["name"]] = {"force_component_n": float(np.max(np.abs(delta[:, :3]))),
                                        "moment_component_nm": float(np.max(np.abs(delta[:, 3:])))}

    model = load_model(PROJECT_ROOT / "models/flexiv_rizon4s_n201_interface_candidate.xml")
    data = mujoco.MjData(model)
    initial = read(OLD / "initial_state.json")
    data.qpos[:] = initial["qpos_after"]
    data.qvel[:] = initial["qvel_after"]
    data.qacc_warmstart[:] = initial["qacc_warmstart_after"]
    mujoco.mj_forward(model, data)
    wrench = known_wrench_test(model, data, cfg)
    require(wrench["passed"], "site-based known wrench test failed")
    for dt in ("0.002", "0.001"):
        contract = read(ROOT / "candidate_runtime_contract.json")["contracts"][dt]
        require(contract["definition"] == "site_based" and contract["active"] and contract["refsafe"], "new equality contract mismatch")
        require(contract["timeconst_effective_s"] == .004 and contract["iterations"] == 50, "effective numerical settings changed")
    report = PROJECT_ROOT / "paper/N201_INTERFACE_CALIBRATION_REPORT.md"
    require(report.exists() and (ROOT / "commands.md").exists() and (ROOT / "tests.log").exists(), "required documentation missing")
    return {"schema_version": "n201r_completion_audit_v1", "passed": True,
        "verification_scope": "static byte identities, archived complete curves, current site-based known-wrench forward test; zero new dynamics",
        "candidate_known_wrench_test": wrench, "valid_suite_metrics": stats,
        "maximum_actual_load_event": max(events, key=lambda e: e["fraction"]),
        "step_sensitivity_wrench_units_separated": sensitivity,
        "historical_identity_preserved": True, "candidate_parameter_count": 2,
        "attempted_fixture_cases": 128, "separate_fixture_replay_cases": 16, "new_formal_trajectories": 0,
        "requirements": {
            "observation_and_initialization": "audited without rerunning historical trajectories",
            "original_and_two_candidates": "all tests preserved; four invalid suites explicitly excluded",
            "holding_and_actual_load": "holding passed, actual load failed, overall connection not qualified",
            "effective_refsafety_and_step": "2 ms and 1 ms RK4, refsafe on, identical effective 4 ms time constant",
            "protected_model_and_controller": "original assets and state/gains retained; protected compiled arrays compared",
            "conditional_formal_runs": "skipped because independent connection qualification failed",
            "performance": "NOT_EVALUATED; no 10 s new run and no detumbling success claim",
            "delivery": "report, raw tests, ledger, commands, identities, and verification saved; local commit gate remains Git status/diff check"}}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seal", action="store_true", help="save static verification and include final documentation identities")
    args = parser.parse_args()
    result = inspect()
    if args.seal:
        save(ROOT / "verification.json", result)
        manifest = read(ROOT / "experiment_manifest.json")
        sources = list((PROJECT_ROOT / "v6_mujoco/postgrasp_calibration").glob("*.py"))
        sources += [PROJECT_ROOT / "tests/test_n201_interface_calibration.py"]
        for path in sources:
            manifest["implementation_identity"][path.relative_to(PROJECT_ROOT).as_posix()] = digest(path)
        manifest["verification_identity"] = {
            path.relative_to(PROJECT_ROOT).as_posix(): digest(path)
            for path in (PROJECT_ROOT / "v6_mujoco/postgrasp_calibration/verify.py", PROJECT_ROOT / "tests/test_n201_interface_calibration.py")}
        manifest["delivery_identity"] = {
            path.relative_to(PROJECT_ROOT).as_posix(): digest(path)
            for path in (PROJECT_ROOT / "paper/N201_INTERFACE_CALIBRATION_REPORT.md", PROJECT_ROOT / ".gitattributes", PROJECT_ROOT / "requirements.txt")}
        manifest["output_hashes"] = {
            path.relative_to(ROOT).as_posix(): digest(path)
            for path in ROOT.rglob("*") if path.is_file() and path.name != "experiment_manifest.json"}
        manifest["completion_audit_passed"] = True
        save(ROOT / "experiment_manifest.json", manifest)
    else:
        manifest = read(ROOT / "experiment_manifest.json")
        for relative, expected in manifest["output_hashes"].items():
            require(digest(ROOT / relative) == expected, f"output changed since seal: {relative}")
        for relative, expected in manifest.get("delivery_identity", {}).items():
            require(digest(PROJECT_ROOT / relative) == expected, f"delivery changed since seal: {relative}")
    print(json.dumps({"completion_audit_passed": True, "known_wrench_passed": True,
        "actual_load_peak": result["maximum_actual_load_event"]["fraction"], "new_formal_trajectories": 0,
        "dynamics_started_by_this_verifier": 0, "manifest_sealed": args.seal}))


if __name__ == "__main__":
    main()

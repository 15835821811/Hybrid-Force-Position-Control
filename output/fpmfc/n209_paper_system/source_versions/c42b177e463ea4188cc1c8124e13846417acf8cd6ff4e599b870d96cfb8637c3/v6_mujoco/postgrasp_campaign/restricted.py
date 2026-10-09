"""Exactly nineteen new independent tests; immutable candidate and scaled input only."""
import json
import time

import numpy as np
from v6_mujoco.postgrasp.physics import digest
from v6_mujoco.postgrasp_calibration.load_tests import test_case
from .io import ROOT, CAL, read, save, save_npz, design, stage_identity, completed, begin_attempt, end_attempt, identity


def run(resume=False):
    manifest = design()
    code = stage_identity(["restricted", "contract"])
    output = ROOT / "restricted_fixture"
    summary_path = output / "result.json"
    previous = completed(summary_path, resume, code)
    if previous is not None:
        return previous
    spec = read(output / "input_contract.json")
    cases = {c["name"]: c for c in spec["cases"]}
    schedule = [(c, .002) for c in cases] + [(c, .001) for c in spec["fine_cases"]]
    results = []; passed = True
    for name, dt in schedule:
        label = name + ("_fine" if dt == .001 else "_coarse")
        begin_attempt("fixture_runs", label, .9, dt, code)
        started = time.perf_counter()
        try:
            row, arrays, geometry, xml = test_case(manifest["connection_parameters"], cases[name], dt)
            # Commit raw evidence before any metric or summary operation.
            save_npz(output / (label + ".npz"), arrays)
            (output / ("fixture_fine.xml" if dt == .001 else "fixture_coarse.xml")).write_bytes(xml.encode("utf-8"))
            row["actual_capacity_passed"] = row["max_interface_load_fraction"] <= 1.
            row["passed"] = bool(row["pose_holding_passed"] and row["numerics_passed"] and row["actual_capacity_passed"])
            row["label"] = label; row["geometry"] = geometry
            if dt == .001:
                with np.load(output / (name + "_coarse.npz")) as coarse:
                    differences = {k: float(np.max(np.abs(coarse[k] - arrays[k][::2]))) for k in ("translation_error_m", "rotation_error_deg", "interface_load_fraction")}
                row["full_curve_step_differences"] = differences
                row["passed"] &= differences["translation_error_m"] <= .00005 and differences["rotation_error_deg"] <= .01 and differences["interface_load_fraction"] <= .02
            save(output / (label + ".json"), row)
            results.append(row)
            end_attempt("fixture_runs", label, "PASSED" if row["passed"] else "RESTRICTED_DOMAIN_FAILED", elapsed_s=time.perf_counter()-started, trace_sha256=digest(output / (label + ".npz")))
            print(json.dumps({"fixture": label, "passed": row["passed"], "rho": row["max_interface_load_fraction"]}), flush=True)
            if not row["passed"]:
                passed = False; break
        except Exception as error:
            end_attempt("fixture_runs", label, "ERROR", error=repr(error))
            raise
    output_hashes = {p.relative_to(ROOT).as_posix(): digest(p) for p in output.iterdir() if p.is_file() and p.name != "result.json"}
    result = {"status": "VERIFIED_IN_TESTED_CASES" if passed and len(results) == 19 else "FAILED", "legacy_full_boundary_contract": "FAILED",
        "hardware_load_rating_validated": False, "results": results, "implementation_identity": code,
        "input_identity": identity([ROOT / "campaign_manifest.json", ROOT / "source_identity.json", output / "input_contract.json"]),
        "output_hashes": output_hashes, "scope": "only these 16 directions and 3 selected fine cases, fixed smooth profile, actual distal and target fixture inertias; not arbitrary robot sequences"}
    save(summary_path, result)
    return result

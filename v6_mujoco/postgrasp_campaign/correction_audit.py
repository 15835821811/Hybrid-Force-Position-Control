"""Read-only proof of the P_Z diagnostic false positive; no dynamics rerun."""
import numpy as np
import mujoco
from pathlib import Path
from v6_mujoco.model import PROJECT_ROOT
from v6_mujoco.collision import build_collision_pairs
from v6_mujoco.postgrasp.physics import digest
from .io import ROOT, read, save, design, identity
from .engine import make_model, observe, safety, STATE_SPEC
from .validation import validate_run


def main():
    manifest=design(); directory=ROOT/"baselines/P_Z"
    original=read(directory/"result.json")
    archives={}
    for relative,expected in original["implementation_identity"].items():
        current=PROJECT_ROOT/relative
        if digest(current)==expected:
            archives[relative]={"path":relative,"sha256":expected}
        else:
            archived=ROOT/"source_archive/v1"/current.name
            if digest(archived)!=expected: raise RuntimeError("original execution source unavailable")
            archives[relative]={"path":archived.relative_to(PROJECT_ROOT).as_posix(),"sha256":expected}
    initial=read(directory/"initial.json")
    reference={"rotation":np.array(initial["reference_rotation"]),"translation":np.array(initial["reference_translation"])}
    m=make_model(); d=mujoco.MjData(m); pairs=build_collision_pairs(m)
    rows=[]; first=None
    with np.load(directory/"trace.npz") as trace:
        for state in trace["integration_state"]:
            mujoco.mj_setState(m,d,state,STATE_SPEC)
            row=observe(m,d,reference,pairs,initial["H0"])
            if first is None:first=row
            rows.append({"time_s":float(d.time),"raw_net_moment_world_nm":row["net_interface_moment_world_nm"].tolist(),
                "geometric_couple_world_nm":row["soft_site_geometric_couple_world_nm"].tolist(),
                "force_error_n":row["action_reaction_force_error_n"],"moment_geometry_error_nm":row["action_reaction_moment_geometry_error_nm"],
                "corrected_safety_failure":safety(row,first,m,manifest)})
    assert all(r["corrected_safety_failure"] is None for r in rows)
    assert max(r["moment_geometry_error_nm"] for r in rows)<1e-12
    replay=validate_run("P_Z")
    save(directory/"verifier_correction.json",{
        "original_stop_is_validator_false_positive":True,"original_status_retained":original["status"],
        "new_interpretation":"six millisecond valid saved prefix; incomplete Z due to overly strict coincident-point diagnostic; no new Z trajectory",
        "reason":"raw common-point net moment equals separated-site geometric couple delta_p cross F, not a wrench reconstruction error",
        "original_run_still_counts":True,"source_archive_mapping":archives,"trace_sha256":digest(directory/"trace.npz"),
        "rows":rows,"read_only_replay_passed":replay["passed"],"new_fields_tolerance":{
            "force_error_n":1e-8,"moment_geometry_error_nm":1e-8,"calibration_method":"known coincident wrench and all four frozen-state separated-site identities; no physical threshold change"},
        "actual_load_pose_and_PH_gates_unchanged":True,"verification_identity":identity([__file__,Path(__file__).parent/"engine.py",Path(__file__).parent/"validation.py"]),
        "documentation":"https://mujoco.readthedocs.io/en/3.3.2/computation/index.html#equality"})
    print("P_Z saved prefix re-evaluated; diagnostic false positive proven; original results retained; no new Z")


if __name__=="__main__":main()

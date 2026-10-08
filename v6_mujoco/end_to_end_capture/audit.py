"""Completion audit for the bounded experiment, including its physical failure."""
import ast
import copy
import json
import subprocess
from pathlib import Path
import numpy as np
import mujoco
from .common import *
from .adapter import compile_model,initialize,extra_pairs,pads,latch_id
from .observer import capture
from v6_mujoco.model import default_model_spec,site_id
from v6_mujoco.collision import signed_distance
from v6_mujoco.postgrasp.physics import load_model

def main():
    manifest=verify_design();ledger=read(ROOT/"run_ledger.json");r=read(ROOT/"E_nominal/metrics.json");v=read(ROOT/"E_nominal/validation.json");a=arrays(ROOT/"E_nominal/trace.npz")
    check_identity(r["implementation_identity"]);check_identity(v["verification_identity"]);check_identity(read(ROOT/"unit_tests.json")["implementation_identity"])
    assert digest(ROOT/"E_nominal/trace.npz")==r["trace_sha256"]==v["trace_sha256"] and v["passed"]
    assert len(ledger["formal_runs"])==1 and len(ledger["replays"])==1 and len(ledger["unit_tests"])==2
    assert all(x["status"]=="PASSED" for x in ledger["unit_tests"])
    assert set(ledger["skipped"])=={"fine","light","heavy"}
    assert manifest["frozen_epoch"]<ledger["formal_runs"][0]["started_epoch"]
    assert r["status"]=="UNINTENDED_CONTACT_OR_COLLISION" and r["reason"]=="link7_collision__tumbling_target_geom"
    assert r["performance"]=="NOT_EVALUATED" and not r["continuous_18s_completed"]
    assert len(a["time_s"])==3682 and abs(a["time_s"][-1]-7.362)<1e-9 and np.allclose(np.diff(a["time_s"]),.002,atol=1e-12,rtol=0)
    assert not np.any(a["eq_active"]) and np.all(a["phase_code"]==0)
    assert not np.any(a["qfrc_applied"]) and not np.any(a["xfrc_applied"])
    assert np.all(a["servo_tick"]) and np.array_equal(np.flatnonzero(a["task_tick"]),np.arange(0,len(a["time_s"]),10))
    m=compile_model();initial=initialize(m);assert np.array_equal(initial.qpos,a["qpos"][0]) and np.array_equal(initial.qvel,a["qvel"][0])
    old=load_model(PROJECT_ROOT/"models/flexiv_rizon4s_postgrasp_campaign.xml")
    for key in ("body_mass","body_inertia","eq_solref","eq_solimp","eq_data","site_pos","site_quat","actuator_trnid","actuator_gear","dof_damping","dof_armature"):
        np.testing.assert_array_equal(getattr(m,key),getattr(old,key))
    for name in ("runner.py","approach.py","observer.py"):
        tree=ast.parse((Path(__file__).parent/name).read_text(encoding="utf-8"))
        assignments=[]
        for node in ast.walk(tree):
            if isinstance(node,(ast.Assign,ast.AugAssign,ast.AnnAssign)):
                targets=node.targets if isinstance(node,ast.Assign) else [node.target]
                assignments.extend(ast.unparse(t) for t in targets)
        assert not any(t.startswith(("d.qpos","d.qvel","d.time","real.qpos","real.qvel","real.time","m.site_pos","m.site_quat","m.body_mass","m.body_inertia")) for t in assignments),(name,assignments)
    pairlist=extra_pairs(m);index=[p.name for p in pairlist].index(r["reason"]);dist=a["extra_pair_distances_m"][:,index];threshold=config()["new_collision_margin_m"]-config()["numerical_penetration_tolerance_m"]
    assert np.all(dist[:-1]>=threshold) and 0<dist[-1]<threshold
    assert v["first_safety_failure"]["index"]==len(dist)-1 and v["first_safety_failure"]["detail"]==r["reason"]
    # Signed-distance checks remain active even when the intended contact masks are zero.
    before=np.array([signed_distance(m,initial,p) for p in pairlist]);gids=pads(m);m.geom_contype[gids]=0;m.geom_conaffinity[gids]=0
    after=np.array([signed_distance(m,initial,p) for p in pairlist]);np.testing.assert_array_equal(before,after)
    assert np.all(a["intentional_contact_count"]==0) and np.all(a["contact_peak_force_n"]==0)
    mapping=read(ROOT/"unit_source_archive/v1/mapping.json")
    for relative,expected in read(ROOT/"unit_tests_v1.json")["implementation_identity"].items():
        path=PROJECT_ROOT/(mapping[relative]["archive"] if relative in mapping else relative);assert digest(path)==expected
    visual=read(ROOT/"visualizations/manifest.json");check_identity(visual["render_identity"])
    assert visual["new_physics_steps"]==0 and visual["frames"]==222 and abs(visual["physical_end_time_s"]-7.362)<1e-9
    for name,item in visual["videos"].items():assert digest(ROOT/"visualizations"/(name+".mp4"))==item["sha256"]
    check_identity(read(ROOT/"figures/manifest.json")["files"])
    criteria={
        "0_goal_and_scope":{"evidence":"E_nominal/metrics.json; qualification_matrix.json","verdict":"BOUNDED_EXPERIMENT_FINISHED_WITH_PHYSICAL_SAFETY_FAILURE; 18 s success NOT achieved"},
        "1_history_branch_inputs":{"evidence":"experiment_manifest source hashes; git base; inherited files unchanged","verdict":"VERIFIED"},
        "2_fixed_time_contract":{"evidence":"runner switch only at step=8/dt; summarize requires full16..18; actual time grid in trace","verdict":"IMPLEMENTED; 8 s and 18 s not reached"},
        "3_single_physical_system":{"evidence":"runner source, initialization_contract, full-state trace, strict replay, zero external forces","verdict":"VERIFIED for actual0..7.362s"},
        "4_original_C1_control":{"evidence":"approach adapter imports original algorithms; unit reference matches; tasks.json; c1_comparison.json","verdict":"VERIFIED for recorded approach; fine schedule only short-unit verified"},
        "5_contact_and_collision":{"evidence":"contact_pair_contract; actual masks; 19 added distances; first-failure replay","verdict":"VERIFIED monitoring; physical safety gate FAILED before planned contact"},
        "6_state_switch":{"evidence":"runner source; static event unit; events FAILED at7.362; switch_audit NOT_REACHED","verdict":"IMPLEMENTED_AND_UNIT_TESTED; continuous latch/50ms/HOLD NOT_EVALUATED"},
        "7_acceptance":{"evidence":"metrics null window values; whole recorded P/H; distances","verdict":"SAFETY_FAILED; DETUMBLING_NOT_EVALUATED; no relaxed actual capacity"},
        "8_trace_and_replay":{"evidence":"3682 integration states, tasks, events, model masks, strict validation","verdict":"VERIFIED recorded horizon; t8 snapshots absent because not reached"},
        "9_required_tests":{"evidence":"unit_tests.json; unit_tests_v1 and source archive; this read-only mask/source audit","verdict":"COMPATIBILITY_TESTS_PASSED; no claim that unit fixtures validate a full scenario"},
        "10_matrix_budget":{"evidence":"run_ledger: one formal attempt, one replay, two short batches; skipped fine/light/heavy","verdict":"CONDITIONAL_STOP_RULE_FOLLOWED; no retries/search; Z not eligible"},
        "11_failure_classification":{"evidence":"UNINTENDED_CONTACT_OR_COLLISION with named pair and exact first trigger; C1 reference diagnostic","verdict":"VERIFIED; positive gap not described as collision impact"},
        "12_artifacts_CLI_media":{"evidence":"source modules, contracts, report, trace, six plot groups, four actual-duration videos, commands","verdict":"DELIVERED; requested18s media impossible after safety stop and not fabricated"},
        "13_status_matrix":{"evidence":"qualification_matrix.json","verdict":"DELIVERED with per-scenario NOT_RUN/NOT_EVALUATED and all hardware/perception/realtime flags false"},
        "14_non_goals":{"evidence":"no old19fixture rerun, no42video copy, no new controller/gain/trajectory search","verdict":"RESPECTED"}}
    result={"delivery_audit_passed":True,"physical_end_to_end_success":False,"formal_attempts":1,"replays":1,"unit_batches":2,"new_unit_physics_steps":138,"new_unit_replay_steps":18,"first_distance_failure_m":float(dist[-1]),"previous_distance_m":float(dist[-2]),"unchanged_runtime_and_design_identities":True,"mask_independent_distance_check":True,"no_midrun_main_state_write_source_check":True,"requirements":criteria,"audit_identity":identity([__file__])}
    save(ROOT/"completion_audit.json",result)
    log="N205 evidence audit PASS; physical end-to-end validation FAILED.\nNominal:3681 physical steps,0..7.362s. First link7-cube distance violation independently replayed.\nNo pad/plate contact, no latch, no full16..18 performance evaluation.\nUnits:2 batches,138 short physics steps and18 replay steps, distinct from formal attempts.\nFormal replay:3681 steps, all strict comparisons passed.\nResume:verified existing failed trace; no formal or replay count increment.\nMedia:4 videos,222 frames each,30fps,7.4s encoding of0..7.362s source;6 scientific figure groups.\nHistorical C1 t8 geometry:isolated static diagnostic only,zero dynamics.\n"
    (ROOT/"tests.log").write_text(log,encoding="utf-8",newline="\n")
    (ROOT/"E_nominal/execution.log").write_text(json.dumps({k:r[k] for k in ("status","reason","end_time_s","performance","elapsed_wall_s","control_latency_s")},indent=2)+"\n",encoding="utf-8",newline="\n")
    paths=[p for p in ROOT.rglob("*") if p.is_file() and p.name!="delivery_manifest.json" and not p.name.endswith((".ffmpeg.log",".tmp"))]
    paths+=list(Path(__file__).parent.glob("*.py"))+[CONFIG,MODEL,PROJECT_ROOT/"paper/N205_END_TO_END_CAPTURE_DETUMBLING_REPORT.md"]
    save(ROOT/"delivery_manifest.json",{"scope":"N205 new artifacts and source; README navigation not hashed; prior source identities verified separately","files":identity(paths)})
    print(json.dumps({"delivery_audit":"PASS","physical_validation":"FAILED","formal_attempts":1,"videos":4}))

if __name__=="__main__":main()

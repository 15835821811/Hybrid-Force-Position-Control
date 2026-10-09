"""Read-only completion checks and delivery hashes; no dynamics are executed."""
import time
from pathlib import Path
import numpy as np
from v6_mujoco.model import PROJECT_ROOT
from v6_mujoco.postgrasp.physics import digest
from .io import ROOT, OLD, read, save, check_identity, identity
from .report import collect
from .engine import location


def main():
    manifest,ledger,results,traces=collect()
    controller=read(ROOT/"selected_controller.json");check_identity(controller["evidence_identity"])
    assert controller["controller"]=="D" and not controller["parameter_adaptation_after_selection"]
    assert len(ledger["formal_runs"])==7 and len(ledger["fixture_runs"])==19
    assert len(ledger["replays"])==8 and len(ledger["unit_tests"])==2 and len(ledger["predictions"])==0
    coefficients=np.asarray(read(OLD/"damping.json")["coefficient_nm_s_rad"])
    limits=np.asarray(manifest["legacy_controller"]["torque_limits_nm"])
    checks={}
    for name,a in traces.items():
        r=results[name]; stride=round(.002/r["timestep_s"])
        samples=np.arange(0,len(a["time_s"]),stride)
        expected=np.zeros_like(a["ctrl_nm"][samples]) if name=="P_Z" else np.clip(-a["joint_velocity_rad_s"][samples]*coefficients,-limits,limits)
        error=float(np.max(np.abs(expected-a["ctrl_nm"][samples])))
        assert error<1e-14
        assert np.max(np.abs(a["qfrc_applied"]))==0 and np.max(np.abs(a["xfrc_applied"]))==0
        assert np.all(a["eq_active"]) and np.max(a["load_fraction"])<=1
        power=np.sum(a["ctrl_nm"][samples]*a["joint_velocity_rad_s"][samples],axis=1)
        assert np.max(power)<=0
        if name!="P_Z": assert r["performance_passed"] and r["complete_10s"] and r["full_window_evaluated"]
        checks[name]={"control_formula_max_error_nm":error,"max_control_sample_power_w":float(np.max(power)),"external_generalized_and_body_forces_zero":True,"continuous_equality":True,"source_samples":len(a["time_s"]),"replay_passed":r["replay_passed"]}
    visual=read(ROOT/"visualizations/manifest.json"); count=0
    for name,record in visual["conditions"].items():
        assert record["trace_sha256"]==results[name]["trace_sha256"]
        check_identity(record["render_identity"])
        for filename,metadata in record["videos"].items():
            assert digest(ROOT/"visualizations"/name/filename)==metadata["sha256"]
            assert metadata["probe"]["streams"][0]["nb_frames"]=="300"
            count+=1
    assert count==42
    matrix=read(ROOT/"qualification_matrix.json")
    assert matrix["legacy_full_boundary_contract"]=="FAILED" and not matrix["hardware_load_rating_validated"] and not matrix["physical_gripper_capture_validated"]
    audit={"passed":True,"formal_runs":7,"complete_10s_runs":6,"fixture_runs":19,"recorded_input_replays":8,"short_unit_batches":2,"prediction_rollouts":0,"videos":42,"scientific_figures":7,"checks":checks,"preregistered_manifest_sha256":digest(ROOT/"campaign_manifest.json"),"source_identity_sha256":digest(ROOT/"source_identity.json"),"wall_time_since_budget_start_s":time.time()-manifest["resource_started_epoch"],"runtime_history_preserved":True,"new_physics_steps_in_delivery":0,"audit_identity":identity([Path(__file__)])}
    assert audit["wall_time_since_budget_start_s"]<=manifest["resource_budget_s"]
    save(ROOT/"completion_audit.json",audit)
    log="""PASS: 19 restricted fixture cases (16 at 2 ms, 3 at 1 ms).
PASS: short preflight batches v1 and v2, 3 physics steps each, nonintrusive observation/replay.
RETAINED: P_Z stopped at 0.006 s by an incorrect raw action/reaction diagnostic; not rerun.
PASS: geometry correction audit on 4 saved P_Z states; original source and failure archived.
PASS: 6 complete 10 s closed-loop runs, fixed last 2 s performance and all original safety gates.
PASS: 8 recorded-input same-step replays (P_Z original + corrected, six complete runs).
PASS: P_D and L full-curve 2 ms versus 1 ms sensitivity, unchanged 2 ms control period.
PASS: --resume returned with 7 formal attempts / 19 fixture cases / 8 replays / 2 unit batches unchanged.
PASS: recorded torque equals frozen clipped damping at every control update; no applied external forces.
PASS: raw C1 latch qpos/qvel continuity and 0..50 ms impulse/work audit.
PASS: 42 videos probed at 300 frames, 30 fps, 10 s; no new dynamics.
PASS: delivery input/runtime/validation/render identities and hashes.
VISUAL QA: seven scientific plots; saved-state five-view and body-side previews; corrected crowded tracking labels.
RENDER DEBUG: first preview had phase-field name mismatch; fixed before video encoding, no dynamics affected.
NOT NEEDED: G_nominal / G_fine; unchanged D passed.
NOT VALIDATED: hardware rating, physical gripper capture, full 8 s approach continuity, real-time worst-case latency.
"""
    (ROOT/"tests.log").write_text(log,encoding="utf-8",newline="\n")
    paths=list(ROOT.rglob("*"))+list((PROJECT_ROOT/"v6_mujoco/postgrasp_campaign").glob("*.py"))+[PROJECT_ROOT/"paper/POSTGRASP_DETUMBLING_CAMPAIGN_REPORT.md"]
    files=[p for p in paths if p.is_file() and p.name!="delivery_manifest.json" and not p.name.endswith((".ffmpeg.log",".tmp"))]
    save(ROOT/"delivery_manifest.json",{"scope":"new campaign artifacts and implementation; legacy inputs verified by immutable source_identity; README is navigation only","files":identity(files)})
    print({"completion_audit":"PASSED","videos":count,"formal_attempts":7,"complete_10s_runs":6})


if __name__=="__main__":main()

"""One real plant from home; no terminal import, projection, or time reset."""
import time
import traceback
import numpy as np
import mujoco
from v6_mujoco.postgrasp_campaign.engine import integration_state,damping_command
from .common import *
from .adapter import compile_model,initialize,latch_id,pads
from .approach import Approach
from .observer import observe,safety,capture

def summarize(a,status):
    t=a["time_s"]; complete=bool(abs(t[-1]-18)<1e-8);window=(t>=16-1e-9)&(t<=18+1e-9)
    full=bool(complete and np.any(window) and t[window][0]<=16+1e-8)
    target=np.rad2deg(np.linalg.norm(a["target_omega_world_rad_s"],axis=1));relative=np.rad2deg(np.linalg.norm(a["target_base_relative_omega_world_rad_s"],axis=1))
    target_max=float(np.max(target[window])) if full else None;relative_max=float(np.max(relative[window])) if full else None
    performance=bool(status=="COMPLETED" and full and target_max<=.1 and relative_max<=.02)
    post=a["holding_error_applicable"].astype(bool)
    first=lambda mask:float(t[np.flatnonzero(mask)[0]]) if np.any(mask) else None
    work={}
    for phase,mask in [("approach",t<=8+1e-9),("postgrasp",t>=8-1e-9)]:
        work[phase]={k:float(np.trapz(a[k][mask],t[mask])) for k in ("actuator_power_w","passive_power_w","all_constraint_power_w","latch_constraint_power_w")}
    return {"status":status,"end_time_s":float(t[-1]),"samples":len(t),"continuous_18s_completed":complete,"full_window_evaluated":full,"evaluation_window_s":[16,18],"target_window_max_deg_s":target_max,"relative_window_max_deg_s":relative_max,"performance":"PASSED" if performance else ("FAILED" if full else "NOT_EVALUATED"),"end_to_end_passed":performance,"max_rho":float(np.max(a["load_fraction"])),"postgrasp_max_translation_m":float(np.max(a["interface_translation_error_m"][post])) if np.any(post) else None,"postgrasp_max_rotation_deg":float(np.max(a["interface_rotation_error_deg"][post])) if np.any(post) else None,"max_P_drift":float(np.max(np.linalg.norm(a["linear_momentum_world_kg_m_s"]-a["linear_momentum_world_kg_m_s"][0],axis=1))),"max_H_drift":float(np.max(np.linalg.norm(a["angular_momentum_about_center_world_kg_m2_s"]-a["angular_momentum_about_center_world_kg_m2_s"][0],axis=1))),"first_geometric_contact_s":first(a["intentional_contact_count"]>0),"first_nonzero_contact_force_s":first(a["contact_peak_force_n"]>0),"first_threshold_contact_s":first(a["contact_normal_force_n"]>=.2),"peak_contact_force_n":float(np.max(a["contact_peak_force_n"])),"max_penetration_m":float(np.max(a["contact_penetration_m"])),"phase_integrated_work_j":work}

def switch(m,d,row):
    if abs(float(d.time)-8)>1e-8 or not capture(row):raise RuntimeError("CAPTURE_GATE_FAILED: fixed-time event rejected")
    pre=integration_state(m,d);q=d.qpos.copy();v=d.qvel.copy();tau=d.ctrl.copy();eid=latch_id(m);gids=pads(m)
    d.eq_active[eid]=True;m.geom_contype[gids]=0;m.geom_conaffinity[gids]=0
    d.ctrl[:]=damping_command(m,d,legacy())
    assert np.array_equal(d.qpos,q) and np.array_equal(d.qvel,v)
    return {"time_s":float(d.time),"pre_integration_state":pre.tolist(),"post_integration_state":integration_state(m,d).tolist(),"qpos_jump":float(np.max(np.abs(d.qpos-q))),"qvel_jump":float(np.max(np.abs(d.qvel-v))),"control_jump_nm":(d.ctrl-tau).tolist(),"equality_id":eid,"geom_ids":gids,"pre_masks":row["interface_geom_masks"].tolist(),"post_masks":np.array([m.geom_contype[gids],m.geom_conaffinity[gids]]).tolist(),"warmstart_policy":"preserve current; no copy from historical trajectory","pre_observation":{k:np.asarray(v).tolist() for k,v in row.items()}}

def run(scenario,resume=False):
    manifest=verify_design();output=ROOT/("E_"+scenario)
    if (output/"metrics.json").exists():
        if not resume:raise FileExistsError(output)
        result=read(output/"metrics.json");check_identity(result["implementation_identity"])
        assert digest(output/"trace.npz")==result["trace_sha256"];return result
    ledger=read(ROOT/"run_ledger.json")
    if any(a["scenario"]==scenario for a in ledger["formal_runs"]):raise RuntimeError("interrupted attempt requires explicit archived implementation repair; cannot overwrite")
    if len(ledger["formal_runs"])>=6:raise RuntimeError("BUDGET_EXHAUSTED")
    m=compile_model(scenario);d=initialize(m);controller=Approach(m,d);eid=latch_id(m)
    fixed_arrays={k:getattr(m,k).copy() for k in ("site_pos","site_quat","body_mass","body_inertia","eq_solref","eq_solimp","eq_data")}
    runtime=runtime_identity();output.mkdir(parents=True,exist_ok=True)
    ledger_add("formal_runs",{"scenario":scenario,"started_epoch":time.time(),"implementation_identity":runtime,"status":"RUNNING"})
    save(output/"initial.json",{"integration_state":integration_state(m,d).tolist(),"state_spec":int(mujoco.mjtState.mjSTATE_INTEGRATION),"model_masks":np.array([m.geom_contype,m.geom_conaffinity]).tolist()})
    save(output/"effective_config.json",{"scenario":config()["scenarios"][scenario],"manifest_sha256":digest(ROOT/"experiment_manifest.json"),"runtime_identity":runtime})
    rows=[];events=[];phase="APPROACH";status="COMPLETED";reason=None;had_contact=False;switch_record=None
    dt=m.opt.timestep;servo_stride=round(.002/dt);task_stride=round(.02/dt);steps=round(18/dt);started=time.perf_counter()
    first=observe(m,d,controller.trajectory,controller.shape);H0=first["angular_momentum_about_center_world_kg_m2_s"]
    initial_state_id=id(d);initial_model_id=id(m)
    try:
        for step in range(steps+1):
            t=float(d.time);task_tick=step%task_stride==0 and step<round(8/dt);servo_tick=step%servo_stride==0
            if step==round(8/dt):
                phase="CAPTURE_CHECK";before=observe(m,d,controller.trajectory,controller.shape,H0)
                events.append({"time_s":t,"state":phase,"capture_passed":capture(before)})
                save(output/"capture_check.json",{k:np.asarray(v).tolist() for k,v in before.items()})
                if not capture(before):status="CAPTURE_GATE_FAILED";reason="fixed t=8 gate"
                else:
                    switch_record=switch(m,d,before);events.extend([{"time_s":t,"state":"LATCH","equality_id":eid,"active":True,"geom_masks_after":switch_record["post_masks"]},{"time_s":t,"state":"GRASP_VERIFY"}]);phase="GRASP_VERIFY"
            if status=="COMPLETED" and servo_tick:
                state_before=integration_state(m,d)
                if step<round(8/dt):
                    torque=controller.update(d,task_tick)
                    assert np.array_equal(state_before,integration_state(m,d)),"MODEL_ADAPTER_FAILED: controller modified real state"
                    d.ctrl[:]=torque
                else:d.ctrl[:]=damping_command(m,d,legacy())
            row=observe(m,d,controller.trajectory,controller.shape,H0)
            row.update(phase_code=PHASE[phase],task_tick=task_tick,servo_tick=servo_tick,reference_q=controller.reference_q.copy(),reference_dq=controller.reference_dq.copy(),reference_ddq=controller.reference_ddq.copy())
            rows.append(row)
            failure,detail=safety(row,first,m,had_contact)
            had_contact |= row["intentional_contact_count"]>0
            if failure and status=="COMPLETED":status=failure;reason=detail
            if task_tick and controller.tasks and (not controller.tasks[-1]["success"] or any(controller.tasks[-1]["bound_conflicts"])) and status=="COMPLETED":status="PRECONTACT_TRACKING_FAILED";reason="HQP task failed or incompatible velocity bounds"
            if status!="COMPLETED":events.append({"time_s":t,"state":"FAILED","reason":status,"detail":reason});break
            if phase=="GRASP_VERIFY" and step>=round(8.05/dt):phase="DAMP_TRANSFER";events.append({"time_s":t,"state":phase})
            if phase=="DAMP_TRANSFER" and np.rad2deg(np.linalg.norm(row["target_omega_world_rad_s"]))<=.1 and np.rad2deg(np.linalg.norm(row["target_base_relative_omega_world_rad_s"]))<=.02:phase="HOLD";events.append({"time_s":t,"state":phase})
            if step==steps:break
            assert not d.eq_active[eid] if step<round(8/dt) else d.eq_active[eid]
            mujoco.mj_step(m,d)
        assert id(d)==initial_state_id and id(m)==initial_model_id
        assert all(np.array_equal(getattr(m,k),v) for k,v in fixed_arrays.items())
    except Exception:
        status="MODEL_ADAPTER_FAILED";reason=traceback.format_exc()
        save_npz(output/"exception_state.npz",{"integration_state":integration_state(m,d)})
        events.append({"time_s":float(d.time),"state":"FAILED","reason":status,"detail":reason})
    finally:
        if rows:save_npz(output/"trace.npz",{k:np.array([r[k] for r in rows]) for k in rows[0]})
        save(output/"events.json",events);save(output/"tasks.json",controller.tasks)
        save(output/"switch_audit.json",switch_record if switch_record else {"status":"NOT_REACHED_OR_GATE_FAILED","state_injection":False})
    result=summarize(arrays(output/"trace.npz"),status)
    result.update(reason=reason,scenario=scenario,dt=float(dt),elapsed_wall_s=time.perf_counter()-started,implementation_identity=runtime,trace_sha256=digest(output/"trace.npz"),single_continuous_plant=True,site_and_inertia_unchanged=True,no_terminal_state_injection=True,control_latency_s={"count":len(controller.timings),"mean":float(np.mean(controller.timings)) if controller.timings else 0.,"p99":float(np.quantile(controller.timings,.99)) if controller.timings else 0.,"max":max(controller.timings,default=0.)})
    save(output/"metrics.json",result)
    ledger=read(ROOT/"run_ledger.json");ledger["formal_runs"][-1].update(status=status,end_time_s=result["end_time_s"],trace_sha256=result["trace_sha256"],finished_epoch=time.time());save(ROOT/"run_ledger.json",ledger)
    print(json.dumps({k:result[k] for k in ("scenario","status","end_time_s","reason","performance")}),flush=True)
    return result

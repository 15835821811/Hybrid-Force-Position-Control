"""Replay from this run's t=0 state, using recorded controls and named events."""
import copy
import time
import mujoco
import numpy as np
from v6_mujoco.postgrasp_campaign.engine import STATE_SPEC,integration_state
from v6_mujoco.model import default_model_spec
from .common import *
from .adapter import compile_model,initialize,pads,latch_id
from .approach import Approach
from .observer import observe,safety
from .runner import summarize

def replay(m,initial,a):
    d=mujoco.MjData(m);mujoco.mj_setState(m,d,np.array(initial["integration_state"]),STATE_SPEC)
    m.geom_contype[:],m.geom_conaffinity[:]=initial["model_masks"]
    # Constructor establishes reference geometry only; update/solve are never called.
    context=Approach(m,copy.copy(d));gids=pads(m);eid=latch_id(m)
    limits={"qpos":1e-11,"qvel":1e-10,"force_grasp_n":1e-8,"moment_grasp_nm":1e-8,"load_fraction":1e-8,"interface_translation_error_m":1e-10,"interface_rotation_error_deg":1e-8,"linear_momentum_world_kg_m_s":1e-10,"angular_momentum_about_center_world_kg_m2_s":1e-10,"contact_peak_force_n":1e-8,"extra_pair_distances_m":1e-10}
    errors={k:0. for k in limits};first=None;failure=None;had_contact=False;changes=[]
    for i,t in enumerate(a["time_s"]):
        assert abs(float(d.time)-float(t))<1e-10
        if bool(d.eq_active[eid])!=bool(a["eq_active"][i,eid]):
            assert abs(t-8)<1e-8 and bool(a["eq_active"][i,eid]);changes.append(float(t));d.eq_active[eid]=True
        expected_masks=a["interface_geom_masks"][i]
        if not np.array_equal(np.array([m.geom_contype[gids],m.geom_conaffinity[gids]]),expected_masks):
            assert abs(t-8)<1e-8 and np.all(expected_masks==0)
            m.geom_contype[gids],m.geom_conaffinity[gids]=expected_masks
        d.ctrl[:]=a["ctrl_nm"][i]
        row=observe(m,d,context.trajectory,context.shape,a["angular_momentum_about_center_world_kg_m2_s"][0]);first=row if first is None else first
        violation,detail=safety(row,first,m,had_contact);had_contact|=row["intentional_contact_count"]>0
        if violation and failure is None:failure={"index":i,"time_s":float(t),"status":violation,"detail":detail}
        for k in limits:errors[k]=max(errors[k],float(np.max(np.abs(np.asarray(row[k])-a[k][i]))))
        if i<len(a["time_s"])-1:mujoco.mj_step(m,d)
    return {"passed":bool(all(errors[k]<=limits[k] for k in limits) and (failure is None or failure["index"]==len(a["time_s"])-1)),"max_errors":errors,"strict_limits":limits,"first_safety_failure":failure,"events_replayed_s":changes,"steps":len(a["time_s"])-1,"controller_update_calls":0,"state_restore_count":1,"method":"initial t=0 integration state once; archived controls; only named equality and pad mask event; mj_step thereafter"}

def compare_c1(a,m):
    old=arrays(C1/"precontact/C1/trace.npz");oldmodel=default_model_spec().compile_model()
    from .adapter import addresses
    newids=addresses(m);oldids={n:{"qpos":list(range(*__import__('v6_mujoco.postgrasp.physics',fromlist=['joint_slices']).joint_slices(oldmodel,n)[0].indices(oldmodel.nq))),"qvel":list(range(*__import__('v6_mujoco.postgrasp.physics',fromlist=['joint_slices']).joint_slices(oldmodel,n)[1].indices(oldmodel.nv)))} for n in newids if n!="target_free_joint"}
    indices=np.flatnonzero((a["time_s"]>1e-9)&(a["time_s"]<=8+1e-9));indices=indices[np.abs(a["time_s"][indices]/.002-np.rint(a["time_s"][indices]/.002))<1e-7]
    errors=[];first=None;contact_time=None
    if np.any(a["intentional_contact_count"]):contact_time=float(a["time_s"][np.flatnonzero(a["intentional_contact_count"])[0]])
    for i in indices:
        j=round(a["time_s"][i]/.002)-1
        e={key:max(float(np.max(np.abs(a[key][i,newids[n][key]]-old[key][j,oldids[n][key]]))) for n in oldids) for key in ("qpos","qvel")}
        errors.append((float(a["time_s"][i]),e))
        if first is None and (e["qpos"]>1e-8 or e["qvel"]>1e-7):first={"time_s":float(a["time_s"][i]),**e,"physical_contact_already_detected":bool(contact_time is not None and a["time_s"][i]>=contact_time)}
    before=[e for t,e in errors if contact_time is None or t<contact_time]
    return {"comparison":"named robot coordinates; only common 2 ms samples","pre_first_contact_common_samples":len(before),"pre_first_contact_max_errors":{k:max((e[k] for e in before),default=0.) for k in ("qpos","qvel")},"first_difference_above_diagnostic_threshold":first,"thresholds":{"qpos":1e-8,"qvel":1e-7},"not_a_requirement_for_bitwise_equality":True}

def validate(scenario):
    verify_design();out=ROOT/("E_"+scenario);r=read(out/"metrics.json");check_identity(r["implementation_identity"])
    assert digest(out/"trace.npz")==r["trace_sha256"]
    a=arrays(out/"trace.npz");m=compile_model(scenario);v=replay(m,read(out/"initial.json"),a)
    recalculated=summarize(a,r["status"]);v["metrics_match"]=all(r[k]==value for k,value in recalculated.items());v["passed"] &=v["metrics_match"]
    v.update(trace_sha256=r["trace_sha256"],verification_identity=identity([__file__]),execution_identity=r["implementation_identity"])
    save(out/"validation.json",v);save(out/"c1_comparison.json",compare_c1(a,m))
    ledger_add("replays",{"scenario":scenario,"passed":v["passed"],"steps":v["steps"],"executed_epoch":time.time(),"verification_identity":v["verification_identity"]})
    if not v["passed"]:raise RuntimeError("REPLAY_FAILED")
    print(json.dumps({"replay":scenario,"passed":v["passed"],"qpos_error":v["max_errors"]["qpos"]}),flush=True)
    return v

def sensitivity():
    a=arrays(ROOT/"E_nominal/trace.npz");b=arrays(ROOT/"E_fine/trace.npz");assert len(a["time_s"])==len(b["time_s"][::2])
    diffs={k:float(np.max(np.abs(a[k]-b[k][::2]))) for k in ("qpos","qvel","force_grasp_n","moment_grasp_nm","interface_translation_error_m","interface_rotation_error_deg","load_fraction")}
    result={"full_curve_differences":diffs,"same_control_period_s":.002,"same_task_period_s":.02,"samples":len(a["time_s"]),"both_complete_and_passed":all(read(ROOT/("E_"+s)/"metrics.json")["end_to_end_passed"] for s in ("nominal","fine")),"postgrasp_gates_inherited":legacy()["design"]["step_sensitivity_gates"]}
    mask=a["time_s"]>=8-1e-9
    actual={"interface_translation_m":float(np.max(np.abs(a["interface_translation_error_m"][mask]-b["interface_translation_error_m"][::2][mask]))),"interface_rotation_deg":float(np.max(np.abs(a["interface_rotation_error_deg"][mask]-b["interface_rotation_error_deg"][::2][mask]))),"load_fraction":float(np.max(np.abs(a["load_fraction"][mask]-b["load_fraction"][::2][mask])))}
    for k,field in [("target_spin_deg_s","target_omega_world_rad_s"),("relative_spin_deg_s","target_base_relative_omega_world_rad_s")]:actual[k]=float(np.max(np.abs(np.rad2deg(np.linalg.norm(a[field],axis=1))-np.rad2deg(np.linalg.norm(b[field][::2],axis=1)))))
    result["gate_differences"]=actual;result["passed"]=result["both_complete_and_passed"] and all(actual[k]<=v for k,v in result["postgrasp_gates_inherited"].items())
    save(ROOT/"step_sensitivity.json",result);return result

"""Finite compatibility tests; short dynamics are separately accounted."""
import copy
import time
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from v6_mujoco.model import default_model_spec,body_id
from v6_mujoco.postgrasp.physics import body_jacobian,joint_slices
from v6_mujoco.postgrasp_campaign.engine import integration_state
from v6_mujoco.fpmfc.target import target_from_config
from v6_mujoco.fpmfc.run_handoff_precontact import _trajectory
from .common import *
from .adapter import compile_model,initialize,latch_id,pads,contract
from .approach import Approach
from .observer import observe,safety,capture
from .runner import switch,summarize
from .validator import replay

def run():
    verify_design();out=ROOT/"unit_tests.json"
    if out.exists():
        saved=read(out);check_identity(saved["implementation_identity"]);return saved
    ledger_add("unit_tests",{"status":"RUNNING","started_epoch":time.time(),"scope":"four 20ms target propagation probes, two 6ms approach tick/replay probes, static switch fixture; no formal trajectory"})
    findings={};prop=[];steps=0
    for scenario in ("nominal","fine"):
        for scale in (1.,1.2):
            m=compile_model(scenario);d=initialize(m,scale);bid=body_id(m,"tumbling_target");count=round(.02/m.opt.timestep)
            jac=body_jacobian(m,d,bid,d.xipos[bid]);velocity=jac@d.qvel
            target=target_from_config(preconfig());np.testing.assert_allclose(velocity[3:],target.sample(0).angular_velocity_world_rad_s*scale,atol=1e-14)
            for _ in range(count):mujoco.mj_step(m,d)
            mujoco.mj_forward(m,d);R=d.xmat[bid].reshape(3,3);pred=target.initial_rotation_world@Rotation.from_rotvec(target.angular_velocity_body_rad_s*scale*.02).as_matrix()
            error=float(Rotation.from_matrix(pred.T@R).magnitude());assert error<1e-10
            prop.append({"scenario":scenario,"spin_scale":scale,"duration_s":float(d.time),"angle_error_rad":error,"actual_rotation":R.tolist()});steps+=count
    findings["target_free_propagation_and_changed_velocity_response"]=prop
    ticktests=[]
    for scenario in ("nominal","fine"):
        m=compile_model(scenario);d=initialize(m);c=Approach(m,d);dt=m.opt.timestep;rows=[]
        initial={"integration_state":integration_state(m,d).tolist(),"model_masks":np.array([m.geom_contype,m.geom_conaffinity]).tolist()}
        first=observe(m,d,c.trajectory,c.shape);assert safety(first,first,m)==(None,None)
        for i in range(round(.006/dt)+1):
            before=integration_state(m,d)
            if i%round(.002/dt)==0:
                tau=c.update(d,i==0);assert np.array_equal(before,integration_state(m,d));d.ctrl[:]=tau
            row=observe(m,d,c.trajectory,c.shape);rows.append(row)
            if i<round(.006/dt):mujoco.mj_step(m,d);steps+=1
        a={k:np.array([r[k] for r in rows]) for k in rows[0]}
        replay_result=replay(compile_model(scenario),initial,a);assert replay_result["passed"]
        metric=summarize(a,"COMPLETED");assert metric["performance"]=="NOT_EVALUATED"
        assert len(c.timings)==4 and len(c.tasks)==1 and not np.any(a["eq_active"])
        map_,_=c.hqp.reaction_velocity_map(copy.copy(d));_,tv=joint_slices(m,"target_free_joint");assert np.all(map_[tv]==0) and map_.shape==(m.nv,7)
        ticktests.append({"scenario":scenario,"control_updates":len(c.timings),"task_updates":len(c.tasks),"replay":replay_result,"short_performance":metric["performance"]})
    findings["isolated_control_observation_schedules_and_replay"]=ticktests
    m=compile_model();d=initialize(m);c=Approach(m,d);row=observe(m,d,c.trajectory,c.shape)
    assert not capture(row)
    try:switch(m,d,row)
    except RuntimeError:pass
    else:raise AssertionError("premature/capture failure did not reject")
    assert not d.eq_active[latch_id(m)]
    # Static event-only fixture, zero physics steps. This history is NEVER used by runner.initialize.
    snapshot=read(PROJECT_ROOT/"output/fpmfc/n200_postgrasp/initial_state.json")
    fixture=copy.copy(d);fixture.qpos[:]=snapshot["qpos_before"];fixture.qvel[:]=snapshot["qvel_before"];fixture.time=8
    row=observe(m,fixture,c.trajectory,c.shape);assert capture(row)
    before={k:getattr(m,k).copy() for k in ("site_pos","site_quat","body_mass","body_inertia")}
    event=switch(m,fixture,row);assert event["qpos_jump"]==event["qvel_jump"]==0
    assert all(np.array_equal(v,getattr(m,k)) for k,v in before.items())
    findings["static_switch_fixture"]={"origin":"existing raw C1 numerical fixture, isolated unit only, zero dynamics; no end-to-end evidence","qpos_jump":event["qpos_jump"],"qvel_jump":event["qvel_jump"],"frames_inertia_unchanged":True,"premature_or_failed_capture_rejected":True}
    trajectory=_trajectory(read(C1/"pairing_manifest.json"),"C1");source=arrays(C1/"precontact/C1/trace.npz")
    reference_errors=[]
    for i in (0,1999,3999):
        r=trajectory.sample(float(source["time"][i]));reference_errors.append(float(np.max(np.abs(r.position_world_m-source["desired_position"][i]))))
    assert max(reference_errors)<1e-14
    findings["C1_reference_exact_archived_samples"]=reference_errors
    models={s:compile_model(s) for s in ("nominal","light","heavy")}
    for s,model in models.items():
        data=initialize(model);assert not data.eq_active[latch_id(model)]
        np.testing.assert_array_equal(data.qvel,initialize(models["nominal"]).qvel)
    findings["all_density_models_initialized_at_home_same_velocities"]=True
    findings["pair_contract"]=contract()["contact_pair_contract"]
    result={"passed":True,"physical_steps":steps,"recorded_input_replay_steps":sum(t["replay"]["steps"] for t in ticktests),"findings":findings,"implementation_identity":runtime_identity()}
    save(out,result);ledger=read(ROOT/"run_ledger.json");ledger["unit_tests"][-1].update(status="PASSED",physical_steps=steps,replay_steps=result["recorded_input_replay_steps"],finished_epoch=time.time());save(ROOT/"run_ledger.json",ledger)
    print(json.dumps({"unit_tests":"PASSED","short_physics_steps":steps}),flush=True);return result

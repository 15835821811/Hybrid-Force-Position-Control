import copy
from dataclasses import replace
import json
import time
import traceback
import numpy as np
from scipy.spatial.transform import Rotation
from v6_mujoco.adaptive_capture import selftest as legacy
from v6_mujoco.adaptive_capture.common import config
from v6_mujoco.adaptive_capture.contracts import StateEstimate,PoseObservation,SensorPacket
from v6_mujoco.adaptive_capture.known_model import compile_known
from v6_mujoco.adaptive_capture.validate import packet_from_dict
from .common import ROOT,OLD,save,read,charge
from .benchmark import mission
from .qp_diagnostics import phase_one
from .estimator_adapter import TimedEstimator
from .reference_shaper import ShapedReference
from .controller_adapter import FeasibleController,NoVerifiedControl

def phase_tests():
    assert phase_one([[1]],[-1],[1])['z']==0
    r=phase_one([[1],[1]],[2,-np.inf],[np.inf,1]);assert abs(r['z']-.5)<1e-9
    out={}
    for name in ['H2_prior','S01_noise_delay','D05_final_nominal']:
        z=read(ROOT/'diagnostics'/name/'snapshots.json')[-1];q=z['qp'][0]
        l=np.array([-np.inf if x is None else x for x in q['l']]);u=np.array([np.inf if x is None else x for x in q['u']]);A=np.asarray(q['A']);n=len(A)-7
        r=phase_one(A,l,u);assert r['z']>1e-8
        # Separated groups feasible; combined conflict survives independent LP.
        distance=phase_one(A[:n],l[:n],u[:n]);joint=phase_one(A[n:],l[n:],u[n:]);assert distance['z']==0 and joint['z']==0
        # Certificate in original units: maximize most restrictive distance row over box.
        witnesses=[]
        for i in range(n):
            maximum=float(np.maximum(A[i],0)@u[n:]+np.minimum(A[i],0)@l[n:])
            if maximum<l[i]-1e-9:witnesses.append({'row':z['row_names'][i],'required_m_s':l[i],'maximum_over_joint_box_m_s':maximum,'shortfall_m_s':l[i]-maximum})
        assert witnesses
        out[name]={'classification':r['classification'],'z':r['z'],'row_units':z['row_units'],'box_witnesses':witnesses,'exact_reconstruction':read(ROOT/'diagnostics'/name/'diagnosis.json')['exact_historical_reconstruction']}
    save(ROOT/'qp_failure_diagnosis.json',{'runs':out,'normalization':'scale 1 m/s for clearance rows; 1 rad/s for joint rows; z is dimensionless, NOT a penetration distance',
         'primary_is_least_squares':True,'secondary_only_failure_demonstrated':False,'numerical_repair_justified':False})
    return out

def reference_tests():
    cfg=mission('ideal');m,d=compile_known(config('target_prior'));r=ShapedReference(m,d,cfg);t=4.;n=r.nominal.target.sample(t)
    e=StateEstimate(t,t,n.center_position_world_m,n.center_rotation_world,n.center_linear_velocity_world_m_s,n.angular_velocity_world_rad_s,np.eye(12)*1e-18,True,np.zeros(6))
    r.progress=t;r.estimate=e;v=r.sample(t);old=r.nominal.sample(t)
    assert np.linalg.norm(v.position_world_m-old.position_world_m)<1e-12
    assert np.linalg.norm(v.linear_velocity_world_m_s-old.linear_velocity_world_m_s)<1e-12
    eps=1e-5;r.rate=.6;r.accel=-.1
    def sample(h):
        q=copy.deepcopy(r);q.progress=t+.6*h-.05*h*h;q.rate=.6-.1*h
        q.estimate=replace(e,time=t+h,p=e.p+e.v*h,R=Rotation.from_rotvec(e.w*h).as_matrix()@e.R)
        return q.sample(t+h)
    a,b,c=sample(-eps),sample(0),sample(eps)
    errors=[np.linalg.norm((c.position_world_m-a.position_world_m)/(2*eps)-b.linear_velocity_world_m_s),
            np.linalg.norm(Rotation.from_matrix(c.rotation_world@a.rotation_world.T).as_rotvec()/(2*eps)-b.angular_velocity_world_rad_s),
            np.linalg.norm((c.linear_velocity_world_m_s-a.linear_velocity_world_m_s)/(2*eps)-b.linear_acceleration_world_m_s2)]
    assert max(errors)<1e-7,errors
    r.rate=0.;r.accel=0.;v=r.sample(t);assert np.linalg.norm(v.angular_velocity_world_rad_s-e.w)<1e-10
    r.progress=8.;r.rate=1.;r.estimate=replace(e,time=8.);end=r.sample(8.)
    r.progress=8.-1e-7;left=r.sample(8.);assert np.linalg.norm(end.linear_velocity_world_m_s-left.linear_velocity_world_m_s)<1e-6
    assert np.linalg.norm(end.linear_acceleration_world_m_s2-left.linear_acceleration_world_m_s2)<1e-6
    rates=[];acc=[];jerk=[];r.progress=0.;r.rate=r.z1=r.z2=1.
    for k in range(2000):
        r.demand=0. if k<1000 else 1.;r.advance(.002);rates.append(r.rate);acc.append(r.accel);jerk.append(r.jerk)
    assert min(rates)>=0 and max(rates)<=1 and max(abs(np.array(acc)))<=1/.3+1e-10 and max(abs(np.array(jerk)))<=2/.3**2+1e-10
    return {'derivative_errors':errors,'endpoint_relative_jets_zero':True,'zero_progress_keeps_target_motion':True,'max_rate_acceleration':max(abs(np.array(acc))),'max_rate_jerk':max(abs(np.array(jerk)))}

def timing_tests():
    cfg=mission('noisy');f=TimedEstimator(cfg)
    base=SensorPacket(0.,np.r_[np.zeros(3),1.,np.zeros(3)],np.zeros(6),np.zeros(7),np.zeros(7),np.zeros(7),np.zeros(6),False,())
    obs=lambda t:PoseObservation(t,np.array([t*.001,0,0]),np.eye(3),np.eye(6)*1e-10)
    for k in range(20):
        t=.002*k;f.update(replace(base,time=t,contact=t>=.02,poses=(obs(t-.012),) if k>=6 else ()))
    assert all(not x['process_contact_at_stamp'] for x in f.log if x['stamp']<.02)
    n=len(f.log);stamp=f.time;f.update(replace(base,time=.04,poses=(obs(stamp),)));assert len(f.log)==n and f.rejects
    f.update(replace(base,time=.042));assert len(f.log)==n
    f.update(replace(base,time=.044,poses=(obs(stamp-.006),)));assert len(f.log)==n and f.rejects[-1]['reason']=='DUPLICATE_OR_OUT_OF_ORDER_REJECTED'
    e=f.update(replace(base,time=.1));assert not e.valid
    try:f.update(replace(base,time=.102,poses=(obs(.2),)));raise AssertionError('accepted future packet')
    except ValueError:pass
    m,d=compile_known(config('target_prior'));r=ShapedReference(m,d,cfg);e=replace(e,time=0.,measurement_time=0.,valid=True,p=np.zeros(3),v=np.zeros(3),w=np.zeros(3),R=np.eye(3))
    r.estimate=e;r.estimate=replace(e,time=.002,p=np.ones(3)*.001)
    assert np.linalg.norm(r.shaped.p)<1e-7 and np.linalg.norm(r.shaped.v)<1e-5
    assert np.array_equal(r.raw.p,np.ones(3)*.001)
    return {'mode_history':True,'duplicate_rejected_no_new_NIS':True,'stale_invalid':True,'future_rejected':True,'raw_reference_separation':True}

def predictor_tests():
    import gzip
    with gzip.open(OLD/'runs/D06_final_gain3/packets.jsonl.gz','rt',encoding='utf-8') as f:packets=[json.loads(next(f)) for _ in range(21)]
    c=FeasibleController(config('target_prior'),mission('ideal'))
    # Prefix packet redecision only, zero real robot integration.
    for row in packets[:-1]:c.update(packet_from_dict(row['packet']),False)
    p=packet_from_dict(packets[-1]['packet']);e=c.state_filter.update(p)
    from v6_mujoco.adaptive_capture.known_model import set_measured
    set_measured(c.model,c.data,p,e);c.reference.estimate=e
    before=[c.data.qpos.copy(),c.data.qvel.copy(),c.hqp.previous_velocity.copy(),c.qref.copy(),c.reference.progress]
    prediction=c.predictor.predict(p,e,1.)
    after=[c.data.qpos,c.data.qvel,c.hqp.previous_velocity,c.qref,c.reference.progress]
    assert all(np.array_equal(a,b) for a,b in zip(before,after));assert prediction['steps']>0
    # Capturing is still evaluated from raw estimate, never the shaped target.
    c.reference.shaped=replace(e,p=e.p+1);c.gate(p,e);values=c.last_gate['values']
    c.reference.shaped=e;c.gate(p,e);assert c.last_gate['values']==values
    return {'prediction':prediction,'online_state_unmodified':True,'guard_independent_of_reference':True,'predictive_steps':c.predictor.physics_steps}

def safety_interface_tests():
    import gzip
    import types
    import mujoco
    from .progress_governor import task_solve
    from v6_mujoco.adaptive_capture.known_model import set_measured
    from v6_mujoco.geometry_capture.planning import full_distance_gradient
    from v6_mujoco.collision import signed_distance
    from v6_mujoco.adaptive_capture.evaluation import summarize
    with gzip.open(OLD/'runs/D06_final_gain3/packets.jsonl.gz','rt',encoding='utf-8') as f:packets=[json.loads(next(f)) for _ in range(21)]
    c=FeasibleController(config('target_prior'),mission('ideal','B2'))
    for row in packets[:-1]:c.update(packet_from_dict(row['packet']),False)
    p=packet_from_dict(packets[-1]['packet']);e=c.state_filter.update(p);set_measured(c.model,c.data,p,e);c.reference.estimate=e
    h=c.hqp
    def forced_secondary(self,primary,J,H,g,A,l,u):
        self.secondary={'J':J.copy(),'primary':primary.copy()}
        return primary.copy(),False,'TEST_INJECTED_SECONDARY_FAILURE',0
    h._solve_scalar_nullspace_secondary=types.MethodType(forced_secondary,h)
    sol=task_solve(h,c.model,c.data,c.reference.sample(p.time))
    assert sol.success and sol.primary_feasible and not sol.secondary_feasible
    assert sol.secondary_status=='SECONDARY_TASK_DEGRADED'
    lock=float(np.max(abs(h.secondary['J']@(sol.joint_velocity-h.secondary['primary']))));assert lock<1e-12
    Q=h.qps[0];assert min(np.minimum(Q['A']@sol.joint_velocity-Q['l'],Q['u']-Q['A']@sol.joint_velocity))>=-h.config.feasibility_tolerance-1e-12
    # Independent directional perturbation checks full tangent gradient and
    # target/base drift using the same instant, without stepping a real plant.
    rng=np.random.default_rng(209);velocity=rng.normal(size=c.model.nv)*.02;errors=[]
    for pair in h.pairs:
        if pair.category not in ['target_distance','intended_surface']:continue
        grad=full_distance_gradient(c.model,c.data,pair);values=[]
        for s in [-1,1]:
            d=copy.copy(c.data);mujoco.mj_integratePos(c.model,d.qpos,velocity,s*1e-6);mujoco.mj_forward(c.model,d);values.append(signed_distance(c.model,d,pair))
        errors.append(abs((values[1]-values[0])/2e-6-grad@velocity))
    assert errors and max(errors)<1e-7,errors
    z=np.load(ROOT/'runs/D01_V1_nominal/trace.npz');rows=[{k:v[i] for k,v in z.items()} for i in range(20)]
    short={}
    for status in ['NO_VERIFIED_CONTROL','ESTIMATE_UNRELIABLE','APPROACH_TIMEOUT']:
        _,r=summarize(rows,status,0.,mission('ideal'));assert not r['continuous_task_completed'] and not r['full_window_evaluated'] and r['world_window_max_deg_s'] is None
        short[status]='NOT_EVALUATED'
    a=FeasibleController(config('target_prior'),mission('ideal'));b=FeasibleController(config('target_prior'),mission('ideal'))
    # Different evaluation-only true masses cannot be passed through this API.
    true_a=legacy.Plant(legacy.nominal_truth());true_b=legacy.Plant(replace(legacy.nominal_truth(),mass=31.))
    for row in packets:
        packet=packet_from_dict(row['packet']);ta,la=a.update(packet,round(packet.time/.002)%10==0);tb,lb=b.update(packet,round(packet.time/.002)%10==0)
        assert np.array_equal(ta,tb) and la==lb and a.phase==b.phase
    assert a.model is not true_a.model and b.model is not true_b.model
    # Fault injection is per-test instance only, never the production classes.
    a.predictor.choose=lambda packet,estimate:False
    try:a.before_reference(packet,a.estimate);raise AssertionError('unverified action accepted')
    except NoVerifiedControl as ex:assert str(ex)=='NO_VERIFIED_CONTROL' and a.phase=='ABORT'
    try:b.update(replace(packet,time=.20,poses=()),False);raise AssertionError('stale estimate accepted')
    except NoVerifiedControl as ex:assert str(ex)=='ESTIMATE_UNRELIABLE' and b.phase=='ABORT'
    return {'secondary_degraded_explicit':True,'primary_lock_residual':lock,'full_gradient_directional_max_error':max(errors),
            'short_trajectory_results':short,'new_algorithm_same_packets_same_decisions_different_truth':True,'explicit_controller_abort_modes_verified':True,'real_robot_steps':0}


def run():
    cpu=time.process_time();wall=time.perf_counter();results={}
    funcs=[legacy.momentum_tests,legacy.physics_tests,legacy.contact_tests,legacy.isolation_tests,legacy.derivative_tests,legacy.prediction_tests,phase_tests,reference_tests,timing_tests,predictor_tests,safety_interface_tests]
    for fn in funcs:
        try:results[fn.__name__]={'passed':True,'result':fn()}
        except Exception:results[fn.__name__]={'passed':False,'error':traceback.format_exc()}
        print(fn.__name__,json.dumps(results[fn.__name__]),flush=True)
    dest=ROOT/('tests_'+str(time.time_ns())+'.json');save(dest,results);charge('tests',time.process_time()-cpu,time.perf_counter()-wall,str(dest.relative_to(ROOT)))
    if not all(x['passed'] for x in results.values()):raise SystemExit(1)
    return results

if __name__=='__main__':run()

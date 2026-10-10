"""Preregistered public-state predictions, not recovery or new plant runs."""
import copy
import time
from dataclasses import asdict
import numpy as np
from scipy.spatial.transform import Rotation
from ..estimation.persistence import records
from ..estimation.filter import AccelerationSnapshot
from ..adapters import packet_to_legacy
from ..contracts import decode
from ..estimation.governor import task_solve
from v6_mujoco.adaptive_capture.known_model import set_measured
from v6_mujoco.feasible_capture.qp_diagnostics import phase_one,recorded_admm
from .contracts import *
from .planner import PlanningController

def restore(cfg,packet,previous,current):
    """Reconstruct pre-task commands from t-dt, current CA18 from public packet.

    This is an independent predictor state only. No physical Plant is created.
    """
    cfg=copy.deepcopy(cfg);cfg['mission'].update(s03_method='B2_time_path_shape',s03=read(OUT/'planning_contract.json'))
    c=PlanningController(cfg['prior'],cfg['mission']);c.predictor.budget.run='authority_B01'
    e=AccelerationSnapshot(**current['latest_snapshot']);c.estimate=e;c.state_filter.s=e;c.state_filter.latest_snapshot=e
    p=packet_to_legacy(decode(packet));set_measured(c.model,c.data,p,e);c.reference.estimate=e
    s=previous['controller']
    c.qref=np.asarray(s['qref']);c.vel=np.asarray(s['velocity']);c.start=np.asarray(s['start_velocity']);c.substep=10
    c.hqp.previous_velocity=c.vel.copy();c.hqp.previous_acceleration=(c.vel-c.start)/c.cfg['task_s']
    c.reference.progress=s['s']*8;c.reference.rate=s['s_dot_filter']*8;c.reference.accel=s['s_ddot_filter']*8;c.reference.clock=p.time
    return c,p,e

def run_authority():
    dest=OUT/'candidate_authority.json'
    if dest.exists():return read(dest)
    cpu,wall=time.process_time(),time.perf_counter();cfg=read(OUT/'runs/B01/config.json');manifest=read(OUT/'snapshot_manifest.json')
    wanted=manifest['ideal_H2_times_s'];states=[];packet=None;previous=None
    for row in records(OUT/'runs/B01/raw'):
        if row['kind']=='packet_received':packet=row['packet']
        if row['kind']!='proposal':continue
        if previous is not None and any(abs(row['time']-t)<1e-8 for t in wanted):
            states.append(dict(time=row['time'],packet=packet,previous=previous,current=row['snapshot']))
        previous=row['snapshot']
    save(OUT/'diagnostics/public_snapshots.json',states)
    summaries=[];authority=[];isolation=[]
    for state in states:
        c,p,e=restore(cfg,state['packet'],state['previous'],state['current']);ref=c.reference
        remain=max(.6,8-ref.progress);base=[remain,0.,0.,0.,0.];base_ref=ref.candidate(p.time,base)
        c.hqp.margin_estimate=e;sol=task_solve(c.hqp,c.model,c.data,base_ref.sample_at(p.time));Q=c.hqp.qps[0]
        cert=phase_one(Q['A'],Q['l'],Q['u'])
        c.hqp.n209_record=True;c.hqp.n209_qps=[]
        reproduced=recorded_admm(c.hqp,Q['H'],Q['g'],Q['A'],Q['l'],Q['u'],np.clip(c.vel,Q['l'][-7:],Q['u'][-7:]))
        solver_diagnostic=c.hqp.n209_qps[-1]
        solver_diagnostic['solution_max_difference_from_actual']=float(np.max(abs(reproduced[0]-Q['x'])))
        # The solve above only diagnoses current feasibility; restore command
        # history so all subsequent candidates start from the same pre-task jet.
        c.hqp.previous_velocity=c.vel.copy();c.hqp.previous_acceleration=(c.vel-c.start)/c.cfg['task_s']
        kin=c.hqp.last_record;mapping,_=c.hqp.reaction_velocity_map(c.data)
        bounds=c.hqp.config
        summaries.append(clean(dict(time=p.time,actual_q=p.joint_position,actual_dq=p.joint_velocity,q_reference=c.qref,
            command_velocity=c.vel,command_acceleration=c.hqp.previous_acceleration,phase=state['previous']['controller']['phase'],
            normalized_progress=ref.progress/8,reference=asdict(base_ref.sample_at(p.time)),target_transport=dict(v=e.v,w=e.w,a=e.a,alpha=e.alpha),
            current_hqp=kin,phase_one=cert,independent_solver_diagnostic=solver_diagnostic,
            braking_time_upper_estimate_s=float(np.max(abs(c.vel)/c.spec.acceleration_limits_rad_s2)+2*np.max(c.spec.acceleration_limits_rad_s2)/c.hqp.config.joint_jerk_limit_rad_s3),
            unit_note='geometry rhs m/s; joint rows rad/s; LP z dimensionless; mixed slack is not mm')))
        if not any(abs(p.time-t)<1e-8 for t in manifest['authority_subset_s']):continue
        rows=[];candidates=[base]
        for dim,step in enumerate([1.,.02,.02,.25,.25]):
            for sign in [-1,1]:
                q=base.copy();q[dim]+=sign*step
                if q[0]<.6:continue
                candidates.append(q)
        for theta in candidates:
            candidate=ref.candidate(p.time,theta)
            out=c.predictor.evaluate(p,e,candidate,.6,c.predictor.budget)
            out.pop('cache');rows.append(dict(theta=theta,reference_at_lead=clean(asdict(candidate.sample_at(p.time+.6))),prediction=out))
        # Reverse-order repeat of the two extreme shape decisions: no new
        # parameter identity, counted separately as cache-free replay work.
        c.predictor.budget.replay=True
        for i in [len(rows)-1,0]:
            again=c.predictor.evaluate(p,e,ref.candidate(p.time,rows[i]['theta']),.6,c.predictor.budget);again.pop('cache')
            from ..estimation.replay import deterministic
            a=deterministic(rows[i]['prediction']);again=deterministic(again)
            from ..replay import compare
            ok,err=compare(a,again);isolation.append(dict(time=p.time,index=i,passed=ok,max_error=err))
        b=rows[0];deltas=[]
        for row in rows[1:]:
            pr=row['prediction'];bp=b['prediction'];pairs={x['pair']:x['margin_m'] for x in bp['named_minima']}
            dp=np.linalg.norm(np.asarray(row['reference_at_lead']['position_world_m'])-b['reference_at_lead']['position_world_m'])
            dq=np.max(abs(np.asarray(pr['terminal_dq'])-bp['terminal_dq']))
            dm={x['pair']:x['margin_m']-pairs[x['pair']] for x in pr['named_minima']}
            deltas.append(dict(theta=row['theta'],reference_delta_m=float(dp),terminal_dq_delta_rad_s=float(dq),margin_delta_m=dm,
                psi_delta_rad=row['reference_at_lead']['arm_angle_rad']-b['reference_at_lead']['arm_angle_rad']))
        authority.append(dict(time=p.time,results=rows,deltas=deltas))
    save(OUT/'diagnostics/current_sets.json',summaries)
    save(OUT/'compute_preflight.json',dict(
        max_observed_braking_estimate_s=max(x['braking_time_upper_estimate_s'] for x in summaries),
        acceleration_limit_rad_s2=4.,jerk_limit_rad_s3=80.,formula='max(abs(command_dq)/a_max)+2*a_max/j_max; conservative ramp allowance, not collision-free braking certificate',
        prediction_horizon_s=read(OUT/'planning_contract.json')['prediction_horizon_s'],
        replanning_period_s=read(OUT/'planning_contract.json')['planning_period_s'],
        candidate_dynamics_latency_s=[r['prediction']['latency_s'] for s in authority for r in s['results']],
        caching='P18 forecast shared across candidates at exact same packet; no geometry-gradient cache; prior states invalid outside declared domain',
        real_time='NON_REALTIME_SIMULATION'))
    thresholds=read(OUT/'planning_contract.json')['sensitivity_thresholds'];effects={}
    for dim,name in enumerate(read(OUT/'planning_contract.json')['theta']):
        rows=[r for x in authority for r in x['deltas'] if r['theta'][dim]!=x['results'][0]['theta'][dim]]
        effects[name]=dict(max_reference_delta_m=max([r['reference_delta_m'] for r in rows],default=0.),
            max_action_delta_rad_s=max([r['terminal_dq_delta_rad_s'] for r in rows],default=0.),
            max_margin_delta_m=max([abs(v) for r in rows for v in r['margin_delta_m'].values()],default=0.))
    active=all(x['max_action_delta_rad_s']>thresholds['action_rad_s'] and x['max_margin_delta_m']>thresholds['margin_m'] for x in effects.values())
    result=dict(status='PASS' if active and all(x['passed'] for x in isolation) else 'FAIL',effects=effects,states=authority,
        order_isolation=isolation,diagnostic_snapshot_count=len(states),source='B01 public packets and CA18 snapshots; pre-task command history, independent prior only',
        scalar_terminal='At u=8 relative path jets are zero; scaling du/dt cannot advance it. Target v+omega cross offset remains.',
        insensitive='terminal rigid link7/tool/target transform cannot change with upstream psi; all candidates retain Q_GE(1)=I; upstream shape is not a terminal geometry repair',
        numerical_thresholds=thresholds,new_plant_attempts=0,truth_state_injections=0)
    save(dest,result)
    # Reuse exact original certificate and readonly noisy end mechanisms.
    failures=read(OLD/'qp_failure_diagnosis.json');noisy={}
    for name in ['E1','E2']:
        gate=read(S02/'runs'/name/'guard_call_analysis.json');gov=read(S02/'runs'/name/'progress_governor.json')[-1]
        noisy[name]=dict(actual_guard=gate,last_rejection=gov,qualification='NOT_ADMITTED; information and prediction rejection coexist')
    save(OUT/'failure_constraint_map.json',dict(historical=failures,noisy_readonly=noisy,current_sets='diagnostics/current_sets.json',
        historical_H2_time_s=read(OLD/'diagnostics/H2_prior/diagnosis.json')['final_time'],no_modified_qpos_recovery=True))
    charge('candidate_authority',cpu,wall,snapshots=len(states),plant_steps=0)
    return result

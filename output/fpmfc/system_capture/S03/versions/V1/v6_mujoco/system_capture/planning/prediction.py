"""Independent prior rollout, actual servo ramp and frozen contact constraints."""
import copy
import time
from dataclasses import replace,asdict
import numpy as np
import mujoco
from v6_mujoco.system_capture.estimation.governor import SnapshotProgressGovernor,task_solve
from v6_mujoco.adaptive_capture.known_model import set_measured
from v6_mujoco.geometry_capture.design import planning_minimum
from v6_mujoco.collision import signed_distance
from v6_mujoco.model import body_id
from v6_mujoco.postgrasp.physics import body_jacobian
from v6_mujoco.run import servo_torque
from .contracts import OUT,read,save,LIMITS,clean

class Budget:
    def __init__(self,replay=False):self.replay=replay;self.run='diagnostic';self.local_evaluations=0
    def register(self,method,t,theta,model):
        b=read(OUT/'candidate_budget.json')
        if not self.replay:
            if b['evaluations']>=LIMITS['candidates']:raise RuntimeError('CANDIDATE_BUDGET_EXHAUSTED')
            b['evaluations']+=1;b['entries'].append(dict(id=b['evaluations'],run=self.run,method=method,time_s=t,theta=theta,model=model))
            save(OUT/'candidate_budget.json',b)
        self.local_evaluations+=1
    def reserve(self,n):
        b=read(OUT/'candidate_budget.json')
        total=b['predictive_steps']+b['replay_predictive_steps']+b.get('reserved_steps',0)
        if total+n>LIMITS['predictive_steps']:raise RuntimeError('PREDICTION_STEP_BUDGET_EXHAUSTED')
        b['reserved_steps']=b.get('reserved_steps',0)+n;save(OUT/'candidate_budget.json',b)
    def finish(self,reserved,steps,kinematic=0):
        b=read(OUT/'candidate_budget.json');b['reserved_steps']=b.get('reserved_steps',0)-reserved
        b['replay_predictive_steps' if self.replay else 'predictive_steps']+=steps;b['kinematic_steps']+=kinematic
        save(OUT/'candidate_budget.json',b)

class PriorRollout(SnapshotProgressGovernor):
    def evaluate(self,packet,e,reference,horizon,budget,forecasts=None):
        c=self.owner;m=self.m;cfg=c.cfg;dt=cfg['servo_s'];n=round(horizon/dt)
        budget.register(cfg['s03_method'],packet.time,reference.theta,'independent_frozen_prior_dynamics')
        budget.reserve(n);started=time.perf_counter();steps=0
        try:
            d=mujoco.MjData(m);set_measured(m,d,packet,e)
            h=self.qp;h.previous_velocity=c.hqp.previous_velocity.copy();h.previous_acceleration=c.hqp.previous_acceleration.copy()
            ref=copy.deepcopy(reference);vel=c.vel.copy();start=c.start.copy();qref=c.qref.copy();substep=0
            mass=np.zeros((m.nv,m.nv));qj,vj=c.spec.joint_addresses(m);_,bv=c.spec.base_slices(m);bid=body_id(m,'tumbling_target')
            maxerr=maxrho=maxlin=maxang=0.;minslack=1e9;reason=None;pair_min=np.full(len(h.pairs),1e9);base_peak=0.;cache=[];last_qp=None
            stride=round(cfg['task_s']/dt);forecast=e
            for k in range(n+1):
                mujoco.mj_forward(m,d)
                if k:
                    forecast=forecasts[k] if forecasts is not None else c.state_filter.predict_snapshot(forecast,packet.time+k*dt)
                    twist=body_jacobian(m,d,bid,d.xpos[bid])@d.qvel
                    pe=replace(forecast,p=d.xpos[bid].copy(),R=d.xmat[bid].reshape(3,3).copy(),v=twist[:3],w=twist[3:])
                    ref.estimate=pe
                else:pe=e
                if k%stride==0:
                    h.margin_estimate=pe;sol=task_solve(h,m,d,ref.sample(d.time));self.task_solves+=1
                    last_qp=clean(h.last_record)
                    maxerr=max(maxerr,sol.position_error_m);maxlin=max(maxlin,sol.primary_linear_velocity_residual_m_s);maxang=max(maxang,sol.primary_angular_velocity_residual_rad_s)
                    minslack=min(minslack,sol.minimum_constraint_slack)
                    pair_min=np.minimum(pair_min,[signed_distance(m,d,p)-planning_minimum(p) for p in h.pairs])
                    base_peak=max(base_peak,float(np.linalg.norm(d.qvel[bv][3:])))
                    cache.append(dict(time=float(d.time),q=d.qpos[qj].copy(),dq=d.qvel[vj].copy(),target_p=pe.p.copy(),target_R=pe.R.copy()))
                    if not sol.primary_feasible or not sol.success or any(h.last_bound_conflicts):reason='PREDICTED_HQP_OR_BOUND_FAILURE';break
                    if maxerr>cfg['n209']['tracking_error_limit_m']:reason='PREDICTED_TRACKING_ERROR';break
                    if d.time>=cfg['n209']['task_residual_startup_s']-1e-12 and (sol.primary_linear_velocity_residual_m_s>cfg['n209']['linear_residual_limit_m_s'] or sol.primary_angular_velocity_residual_rad_s>cfg['n209']['angular_residual_limit_rad_s']):reason='PREDICTED_TASK_RESIDUAL';break
                    if min(pair_min)<-1e-4:reason='PREDICTED_GEOMETRY';break
                    start=vel.copy();vel=sol.joint_velocity.copy();substep=0
                if np.any(abs(d.qvel[vj])>c.spec.velocity_limits_rad_s) or np.any(d.qpos[qj]<h.joint_lower) or np.any(d.qpos[qj]>h.joint_upper):reason='PREDICTED_JOINT_LIMIT';break
                peak=0.
                for j in range(d.ncon):
                    force=np.zeros(6);mujoco.mj_contactForce(m,d,j,force);peak=max(peak,float(np.linalg.norm(force[:3])));maxrho=max(maxrho,float(np.linalg.norm(force[:3])/50+np.linalg.norm(force[3:])/2))
                if peak>20 or maxrho>.8:reason='PREDICTED_CONTACT_LOAD';break
                if k==n:break
                ratio=min((substep+1)*dt/cfg['task_s'],1.);dq=start+ratio*(vel-start);ddq=(vel-start)/cfg['task_s']
                qref=np.clip(qref+dq*dt,h.joint_lower,h.joint_upper);qref=np.clip(qref,d.qpos[qj]-c.run.reference_tracking_band_rad,d.qpos[qj]+c.run.reference_tracking_band_rad)
                torque,_=servo_torque(m,d,vj,bv,qj,qref,dq,ddq,c.spec.torque_limits_nm,c.run.servo_natural_frequency_rad_s,c.run.servo_acceleration_limit_rad_s2,mass)
                d.ctrl[:]=torque;mujoco.mj_step(m,d);steps+=1;self.physics_steps+=1;substep+=1;ref.advance(dt,True)
            return clean(dict(verified=reason is None,reason=reason,steps=steps,horizon_reached_s=steps*dt,
                max_tracking_error_m=maxerr,max_linear_residual_m_s=maxlin,max_angular_residual_rad_s=maxang,
                min_linear_constraint_slack_mixed_units=minslack,max_rho=maxrho,peak_base_rad_s=base_peak,
                named_minima=[dict(pair=p.name,margin_m=v,threshold_m=planning_minimum(p)) for p,v in zip(h.pairs,pair_min)],
                terminal_q=d.qpos[qj],terminal_dq=d.qvel[vj],terminal_reference=asdict(ref.sample(d.time)),
                failed_hqp=last_qp if reason else None,cache=cache,latency_s=time.perf_counter()-started))
        finally:budget.finish(n,steps)

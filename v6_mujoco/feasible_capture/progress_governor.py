"""Finite sampled prior prediction with actual 2 ms servo ramp, no truth input."""
import copy
import time
from dataclasses import replace
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from v6_mujoco.adaptive_capture.known_model import compile_known,set_measured
from v6_mujoco.adaptive_capture.risk import impact_proxy
from v6_mujoco.geometry_capture.reference import shape_model
from v6_mujoco.geometry_capture.design import planning_minimum
from v6_mujoco.model import site_id,body_id
from v6_mujoco.collision import signed_distance
from v6_mujoco.postgrasp.physics import body_jacobian,joint_slices
from v6_mujoco.end_to_end_capture.adapter import pads
from v6_mujoco.run import servo_torque
from .controller_adapter import DiagnosticHQP

def task_solve(hqp,m,d,ref):
    mapping,_=hqp.reaction_velocity_map(d);drift=d.qvel-mapping@d.qvel[hqp.dof_ids]
    sid=site_id(m,'flange_site');J=body_jacobian(m,d,int(m.site_bodyid[sid]),d.site_xpos[sid]);v=J@drift
    return hqp.solve_fpmfc(copy.copy(d),target_position=ref.position_world_m,target_velocity=ref.linear_velocity_world_m_s-v[:3],
        target_rotation=ref.rotation_world,target_angular_velocity=ref.angular_velocity_world_rad_s-v[3:],
        target_arm_angle_rad=ref.arm_angle_rad,target_arm_angle_velocity_rad_s=ref.arm_angle_velocity_rad_s)

class ProgressGovernor:
    def __init__(self,owner):
        self.owner=owner;self.m,self.initial=compile_known(owner.prior,owner.cfg['solver_tolerance'])
        # Restore only original pad/plate contact on this independently built prior.
        m=self.m;ids=pads(m);m.geom_contype[ids]=[2,4];m.geom_conaffinity[ids]=[4,2]
        self.qp=DiagnosticHQP(owner.spec,m,owner.hqp.pairs,shape_model(m,self.initial),
            controller_config=owner.hqp.fpmfc_config,constraint_config=owner.hqp.config)
        self.qp.use_measured_drift=True;self.qp.prediction=True;self.qp.use_margins=True
        self.log=[];self.physics_steps=0;self.task_solves=0
    def choose(self,packet,e):
        c=self.owner;cfg=c.cfg['n209'];started=time.perf_counter();results=[];chosen=None
        # Current set cannot be rescued by changing the future progress demand.
        for fraction in cfg['candidate_fractions']:
            result=self.predict(packet,e,float(fraction));results.append(result)
            if result['verified']:
                chosen=float(fraction);break
        self.log.append({'time':packet.time,'candidates':results,'chosen_fraction':chosen,'latency_s':time.perf_counter()-started,
                         'scope':'finite prior, covariance approximation; no recursive-feasibility or hardware guarantee'})
        if chosen is not None:c.reference.demand=chosen*c.reference.max_rate
        return chosen is not None
    def predict(self,packet,e,fraction):
        c=self.owner;m=self.m;cfg=c.cfg;dt=cfg['servo_s'];d=mujoco.MjData(m);set_measured(m,d,packet,e)
        h=self.qp;h.previous_velocity=c.hqp.previous_velocity.copy();h.previous_acceleration=c.hqp.previous_acceleration.copy()
        ref=copy.deepcopy(c.reference);ref.demand=fraction*ref.max_rate
        vel=c.vel.copy();start=c.start.copy();qref=c.qref.copy();substep=0;mass=np.zeros((m.nv,m.nv))
        qj,vj=c.spec.joint_addresses(m);_,bv=c.spec.base_slices(m);bid=body_id(m,'tumbling_target')
        count=round(cfg['n209']['horizon_s']/dt);stride=round(cfg['task_s']/dt)
        maxerr=0.;maxrho=0.;maxlin=0.;maxang=0.;minslack=1e10;reason=None;steps=0
        for k in range(count+1):
            mujoco.mj_forward(m,d)
            if k:
                tw=body_jacobian(m,d,bid,d.xpos[bid])@d.qvel
                _,_,_,_,P=c.state_filter.propagate(e.p,e.R,e.v,e.w,e.covariance,k*dt)
                pe=replace(e,time=packet.time+k*dt,p=d.xpos[bid].copy(),R=d.xmat[bid].reshape(3,3).copy(),v=tw[:3],w=tw[3:],covariance=P)
                ref.estimate=pe
            else:pe=e
            if k%stride==0:
                h.margin_estimate=pe;r=ref.sample(d.time);sol=task_solve(h,m,d,r);self.task_solves+=1
                maxerr=max(maxerr,sol.position_error_m);maxlin=max(maxlin,sol.primary_linear_velocity_residual_m_s);maxang=max(maxang,sol.primary_angular_velocity_residual_rad_s)
                minslack=min(minslack,sol.minimum_constraint_slack)
                if not sol.success or any(h.last_bound_conflicts):reason='PREDICTED_HQP_OR_BOUND_FAILURE';break
                if maxerr>cfg['n209']['tracking_error_limit_m']:reason='PREDICTED_TRACKING_ERROR';break
                # Task residual is a soft performance test. Permit the declared
                # initial servo transient; all hard checks still run from t=0.
                settled=d.time>=cfg['n209'].get('task_residual_startup_s',0.)-1e-12
                if settled and (sol.primary_linear_velocity_residual_m_s>cfg['n209']['linear_residual_limit_m_s'] or sol.primary_angular_velocity_residual_rad_s>cfg['n209']['angular_residual_limit_rad_s']):reason='PREDICTED_TASK_RESIDUAL';break
                if any(signed_distance(m,d,p)<planning_minimum(p)-1e-4 for p in h.pairs):reason='PREDICTED_GEOMETRY';break
                start=vel.copy();vel=sol.joint_velocity.copy();substep=0
            if np.any(abs(d.qvel[vj])>c.spec.velocity_limits_rad_s) or np.any(d.qpos[qj]<h.joint_lower) or np.any(d.qpos[qj]>h.joint_upper):reason='PREDICTED_JOINT_LIMIT';break
            # Predict actual pad contact wrench without subtracting numerical terms.
            peak=0.
            for i in range(d.ncon):
                force=np.zeros(6);mujoco.mj_contactForce(m,d,i,force);peak=max(peak,float(np.linalg.norm(force[:3])))
                maxrho=max(maxrho,float(np.linalg.norm(force[:3])/50+np.linalg.norm(force[3:])/2))
            if peak>20 or maxrho>.8:reason='PREDICTED_CONTACT_LOAD';break
            if k==count:break
            ratio=min((substep+1)*dt/cfg['task_s'],1.);dq=start+ratio*(vel-start);ddq=(vel-start)/cfg['task_s']
            qref=np.clip(qref+dq*dt,h.joint_lower,h.joint_upper)
            qref=np.clip(qref,d.qpos[qj]-c.run.reference_tracking_band_rad,d.qpos[qj]+c.run.reference_tracking_band_rad)
            torque,_=servo_torque(m,d,vj,bv,qj,qref,dq,ddq,c.spec.torque_limits_nm,c.run.servo_natural_frequency_rad_s,c.run.servo_acceleration_limit_rad_s2,mass)
            d.ctrl[:]=torque;mujoco.mj_step(m,d);self.physics_steps+=1;steps+=1;substep+=1
            energy_ok=ref.progress<6.7 or impact_proxy(m,d).get('upper_energy_j',0.)<cfg['approach_energy_limit_j']
            uncertainty_ok=max(np.linalg.eigvalsh(pe.covariance[:3,:3]))<.005**2 and max(np.linalg.eigvalsh(pe.covariance[3:6,3:6]))<.02**2
            ref.advance(dt,peak<10 and energy_ok and uncertainty_ok)
        return {'fraction':fraction,'verified':reason is None,'reason':reason,'steps':steps,'horizon_reached_s':steps*dt,
                'max_tracking_error_m':maxerr,'max_linear_residual_m_s':maxlin,'max_angular_residual_rad_s':maxang,
                'min_linear_constraint_slack_mixed_units':minslack,'max_rho':maxrho,'terminal_progress':ref.progress}

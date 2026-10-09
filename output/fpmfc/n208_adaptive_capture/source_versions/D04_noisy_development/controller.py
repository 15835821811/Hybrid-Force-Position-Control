"""Controller owns only independent known model + public packets and prior."""
from dataclasses import asdict
import copy
import time
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from v6_mujoco.model import default_model_spec,site_id
from v6_mujoco.postgrasp.physics import body_jacobian,joint_slices
from v6_mujoco.geometry_capture.planning import GeometryHQP,full_distance_gradient
from v6_mujoco.geometry_capture.design import extra_pairs,planning_minimum
from v6_mujoco.geometry_capture.reference import shape_model
from v6_mujoco.collision import build_collision_pairs,signed_distance
from v6_mujoco.fpmfc.run_capture import DynamicRunConfig,_controller_config,_constraint_config
from v6_mujoco.end_to_end_capture.common import preconfig
from v6_mujoco.run import servo_torque
from .known_model import compile_known,set_measured,robot_momentum
from .state_estimator import StateEstimator
from .inertial_estimator import InertialEstimator
from .relative_reference import RelativeReference
from .risk import impact_proxy
from .information_gate import InformationGate
from .governor import BrakeGovernor

DAMPING=np.array([7.921059990727102,12.414562585072085,2.85674716427026,4.576645795069653,.5089894127730749,1.451546073337886,.31702427575402353])

class MeasuredDriftHQP(GeometryHQP):
    def _clearance_constraints(self,data,mapping):
        drift=data.qvel-mapping@data.qvel[self.dof_ids] if self.use_measured_drift else np.zeros(self.model.nv)
        if not self.use_measured_drift:
            _,tv=joint_slices(self.model,'target_free_joint');drift[tv]=data.qvel[tv]
        rows=[];lower=[];minimum=2.;self.last_geometry=[]
        for pair in self.pairs:
            dist=signed_distance(self.model,data,pair);minimum=min(minimum,dist)
            if dist>self.config.clearance_activation_m:continue
            full=full_distance_gradient(self.model,data,pair);g=full@mapping;dd=float(full@drift)
            rhs=-self.config.clearance_barrier_gain*(dist-planning_minimum(pair))-dd
            rows.append(g);lower.append(rhs);self.last_geometry.append({'pair':pair.name,'distance':dist,'drift':dd,'rhs':rhs})
        return np.vstack(rows) if rows else np.zeros((0,7)),np.array(lower),minimum

class Controller:
    def __init__(self,prior,cfg):
        self.cfg=cfg;self.prior=prior;self.model,self.data=compile_known(prior);m,d=self.model,self.data
        self.spec=default_model_spec();self.qids,self.vids=self.spec.joint_addresses(m);_,self.base=self.spec.base_slices(m)
        self.state_filter=StateEstimator(cfg);self.estimator=InertialEstimator(prior,cfg);self.reference=RelativeReference(m,d,cfg['template_duration_s'])
        c=preconfig();self.shape=shape_model(m,d)
        self.hqp=MeasuredDriftHQP(self.spec,m,build_collision_pairs(m)+extra_pairs(m),self.shape,controller_config=_controller_config(c),constraint_config=_constraint_config(c));self.hqp.use_measured_drift=cfg['measured_drift']
        self.run=DynamicRunConfig();self.mass=np.zeros((m.nv,m.nv));self.qref=d.qpos[self.qids].copy();self.vel=np.zeros(7);self.start=np.zeros(7);self.substep=0
        self.phase='OBSERVE';self.latch_time=None;self.confirm_start=None;self.tasks=[];self.timings=[];self.impulse=np.zeros(6);self.last_wrench=None;self.last_time=None;self.estimate=None;self.last_gate={};self.latest_reference=None
        self.information_gate=InformationGate(prior);self.impact={};self.feedback_used=False
        self.governor=BrakeGovernor(prior,DAMPING) if cfg.get('brake_governor',False) else None;self.abort_reason=None
    def gate(self,packet,e):
        m,d=self.model,self.data;tool=site_id(m,'postgrasp_tool_interface');grasp=site_id(m,'target_grasp_site')
        p=d.site_xpos[tool];g=d.site_xpos[grasp];R=d.site_xmat[tool].reshape(3,3);Rg=d.site_xmat[grasp].reshape(3,3)
        J=body_jacobian(m,d,int(m.site_bodyid[tool]),p);jt=J@d.qvel
        vg=e.v+np.cross(e.w,g-e.p);rel=np.r_[vg-jt[:3],e.w-jt[3:]]
        sig=self.cfg['margin_sigma'];P=e.covariance;offset=np.linalg.norm(g-e.p)
        margins=np.array([sig*(np.sqrt(max(np.linalg.eigvalsh(P[:3,:3])))+offset*np.sqrt(max(np.linalg.eigvalsh(P[3:6,3:6])))),sig*np.sqrt(max(np.linalg.eigvalsh(P[3:6,3:6]))),sig*(np.sqrt(max(np.linalg.eigvalsh(P[6:9,6:9])))+offset*np.sqrt(max(np.linalg.eigvalsh(P[9:12,9:12])))),sig*np.sqrt(max(np.linalg.eigvalsh(P[9:12,9:12])))])
        margins[:2]+=np.array([self.cfg.get('position_bias_bound_m',0.)+offset*self.cfg.get('rotation_bias_bound_rad',0.),self.cfg.get('rotation_bias_bound_rad',0.)])
        vals=np.array([np.linalg.norm(g-p),Rotation.from_matrix(R.T@Rg).magnitude(),np.linalg.norm(rel[:3]),np.linalg.norm(rel[3:])]);c=self.cfg['capture'];limits=np.array([c['translation_m'],np.deg2rad(c['rotation_deg']),c['linear_m_s'],np.deg2rad(c['angular_deg_s'])])
        rho=np.linalg.norm(packet.wrench_target_at_grasp_world[:3])/50+np.linalg.norm(packet.wrench_target_at_grasp_world[3:])/2
        self.last_gate={'values':vals.tolist(),'margins':margins.tolist(),'limits':limits.tolist(),'rho':rho,'contact':packet.contact,'valid':e.valid}
        within=bool(e.valid and packet.contact and rho<.8 and np.all(vals+margins<=limits))
        if within:
            geometry_ok=all(signed_distance(m,d,p)>=(-.002 if p.category=='intended_surface' else .04 if p.category in ['tool_robot','workspace','satellite','self'] else .001) for p in self.hqp.pairs)
            feasible=bool(self.tasks and self.tasks[-1]['success'] and not any(self.tasks[-1]['bound_conflicts']))
            self.last_gate.update(geometry_ok=geometry_ok,reference_feasible=feasible)
            within=geometry_ok and feasible
        return within
    def update(self,packet,task_tick,servo_tick=True):
        started=time.perf_counter();e=self.state_filter.update(packet);self.estimate=e
        if not e.valid:self.timings.append(time.perf_counter()-started);return np.zeros(7),False
        set_measured(self.model,self.data,packet,e);m,d=self.model,self.data;self.reference.estimate=e
        g=d.site_xpos[site_id(m,'target_grasp_site')];W=packet.wrench_target_at_grasp_world.copy();W[3:]+=np.cross(g,W[:3])
        if self.last_time is not None:self.impulse+=(packet.time-self.last_time)*.5*(W+self.last_wrench)
        self.last_time=packet.time;self.last_wrench=W
        self.estimator.add(e,robot_momentum(m,d),self.impulse)
        model_pi,trusted=self.information_gate.update(self.estimator,self.cfg['servo_s'],self.cfg['parameter_feedback'],self.latch_time is not None and packet.time-self.latch_time>=self.cfg['grasp_verify_s'])
        # Parameter integration is deferred unless the independent prediction
        # gate passes; shadow estimates are never labelled feedback use.
        latch=False
        if self.phase=='OBSERVE' and packet.time>=self.cfg['observe_s']:self.phase='RENDEZVOUS'
        if self.latch_time is None:
            if self.reference.progress>=6.:self.phase='RELATIVE_APPROACH'
            if self.reference.progress>=7.8:self.phase='CAPTURE_WINDOW'
            ok=self.gate(packet,e) if self.phase=='CAPTURE_WINDOW' else False
            if ok:
                if self.confirm_start is None:self.confirm_start=packet.time
                if packet.time-self.confirm_start>=self.cfg['capture_confirmation_s']-1e-9:
                    prediction_ok=self.governor is None or self.governor.choose(packet,e,model_pi)
                    self.last_gate['prospective_latch_prediction_safe']=prediction_ok
                    if prediction_ok:self.latch_time=packet.time;self.phase='GRASP_VERIFY';latch=True
            else:self.confirm_start=None
        if self.latch_time is not None:
            if packet.time-self.latch_time>=self.cfg['grasp_verify_s']:self.phase='DAMP_TRANSFER'
            _,bv=self.spec.base_slices(m);Rb=Rotation.from_quat(np.r_[packet.base_pose[4:7],packet.base_pose[3]]).as_matrix();wb=Rb@packet.base_velocity[3:]
            if self.phase=='DAMP_TRANSFER' and np.rad2deg(np.linalg.norm(e.w))<=.1 and np.rad2deg(np.linalg.norm(e.w-wb))<=.02:self.phase='HOLD'
            if self.governor is not None and (task_tick or latch):
                if not self.governor.choose(packet,e,model_pi):self.phase='ABORT';self.abort_reason='no feasible brake candidate in sampled prior prediction'
                self.feedback_used|=self.information_gate.used
            alpha=self.governor.alpha if self.governor is not None else 1.
            torque=np.clip(-alpha*DAMPING*packet.joint_velocity,-self.spec.torque_limits_nm,self.spec.torque_limits_nm)
        elif self.phase=='OBSERVE':torque=np.zeros(7)
        else:
            if task_tick:
                r=self.reference.sample(packet.time);self.latest_reference=r
                self.impact=impact_proxy(m,d)
                mapping,_=self.hqp.reaction_velocity_map(d);drift=d.qvel-mapping@d.qvel[self.vids]
                sid=site_id(m,'flange_site');J=body_jacobian(m,d,int(m.site_bodyid[sid]),d.site_xpos[sid]);twist_drift=J@drift if self.cfg['measured_drift'] else np.zeros(6)
                result=self.hqp.solve_fpmfc(copy.copy(d),target_position=r.position_world_m,target_velocity=r.linear_velocity_world_m_s-twist_drift[:3],target_rotation=r.rotation_world,target_angular_velocity=r.angular_velocity_world_rad_s-twist_drift[3:],target_arm_angle_rad=r.arm_angle_rad,target_arm_angle_velocity_rad_s=r.arm_angle_velocity_rad_s)
                self.start=self.vel.copy();self.vel=result.joint_velocity.copy();self.substep=0
                self.tasks.append({'time':packet.time,'success':result.success,'bound_conflicts':self.hqp.last_bound_conflicts.tolist(),'position_error_m':result.position_error_m,'rotation_error_rad':result.orientation_error_rad,'drift':twist_drift.tolist(),'minimum_slack':result.minimum_constraint_slack})
            ratio=min((self.substep+1)*self.cfg['servo_s']/self.cfg['task_s'],1.);dq=self.start+ratio*(self.vel-self.start);ddq=(self.vel-self.start)/self.cfg['task_s']
            self.qref=np.clip(self.qref+dq*self.cfg['servo_s'],self.hqp.joint_lower,self.hqp.joint_upper)
            self.qref=np.clip(self.qref,packet.joint_position-self.run.reference_tracking_band_rad,packet.joint_position+self.run.reference_tracking_band_rad)
            torque,_=servo_torque(m,d,self.vids,self.base,self.qids,self.qref,dq,ddq,self.spec.torque_limits_nm,self.run.servo_natural_frequency_rad_s,self.run.servo_acceleration_limit_rad_s2,self.mass)
            self.substep+=1
            energy_ok=self.reference.progress<6.7 or self.impact.get('upper_energy_j',0.)<self.cfg.get('approach_energy_limit_j',.002)
            uncertainty_ok=max(np.linalg.eigvalsh(e.covariance[:3,:3]))<.005**2 and max(np.linalg.eigvalsh(e.covariance[3:6,3:6]))<.02**2
            self.reference.advance(self.cfg['servo_s'],np.linalg.norm(W[:3])<10. and energy_ok and uncertainty_ok)
        self.timings.append(time.perf_counter()-started)
        return torque,latch

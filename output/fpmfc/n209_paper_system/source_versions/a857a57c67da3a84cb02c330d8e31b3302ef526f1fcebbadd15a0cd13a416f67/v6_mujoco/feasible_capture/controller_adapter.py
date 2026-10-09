"""N208 control with explicit approach extension; fixed post-grasp structure."""
import copy
from dataclasses import asdict,replace
import numpy as np
import mujoco
from v6_mujoco.adaptive_capture.controller import Controller,MeasuredDriftHQP
from v6_mujoco.geometry_capture.planning import full_distance_gradient
from v6_mujoco.geometry_capture.design import planning_minimum
from v6_mujoco.collision import signed_distance
from v6_mujoco.postgrasp.physics import joint_slices
from .estimator_adapter import TimedEstimator
from .reference_shaper import ShapedReference

class NoVerifiedControl(RuntimeError):pass

class DiagnosticHQP(MeasuredDriftHQP):
    def __init__(self,*a,**kw):
        super().__init__(*a,**kw);self.records=[];self.qps=[];self.secondary=None;self.margin_estimate=None;self.use_margins=False;self.prediction=False
    def _solve_admm(self,H,g,A,l,u,x):
        result=super()._solve_admm(H,g,A,l,u,x)
        self.qps.append({'H':H.copy(),'g':g.copy(),'A':A.copy(),'l':l.copy(),'u':u.copy(),'x':result[0].copy(),'status':result[2],'iterations':result[3]})
        return result
    def _solve_scalar_nullspace_secondary(self,primary,J,H,g,A,l,u):
        result=super()._solve_scalar_nullspace_secondary(primary,J,H,g,A,l,u)
        self.secondary={'H':H.copy(),'g':g.copy(),'J':J.copy(),'primary':primary.copy(),'x':result[0].copy(),'status':result[2]}
        return result
    def _clearance_constraints(self,d,mapping):
        drift=d.qvel-mapping@d.qvel[self.dof_ids];rows=[];lower=[];minimum=2.;self.last_geometry=[]
        _,tv=joint_slices(self.model,'target_free_joint')
        for pair in self.pairs:
            dist=signed_distance(self.model,d,pair);minimum=min(minimum,dist)
            if dist>self.config.clearance_activation_m:continue
            full=full_distance_gradient(self.model,d,pair);g=full@mapping;dd=float(full@drift);margin=0.
            if self.use_margins and self.margin_estimate is not None:
                e=self.margin_estimate;J=np.zeros(12);J[:3]=full[tv][:3];J[3:6]=full[tv][3:]@e.R.T
                J[6:9]=.02*J[:3];J[9:]=.02*J[3:6]
                margin=3*np.sqrt(max(0.,float(J@e.covariance@J)))+5e-6*np.linalg.norm(J[:3])+6e-6*np.linalg.norm(J[3:6])
            rhs=-self.config.clearance_barrier_gain*(dist-planning_minimum(pair)-margin)-dd
            rows.append(g);lower.append(rhs)
            self.last_geometry.append({'pair':pair.name,'distance':dist,'drift':dd,'rhs':rhs,'margin_m':margin,'full_gradient':full.copy(),'gradient':g.copy(),'d_min_m':planning_minimum(pair)})
        return np.vstack(rows) if rows else np.zeros((0,7)),np.array(lower),minimum
    def solve_fpmfc(self,d,**kw):
        self.qps=[];self.secondary=None;v=self.previous_velocity.copy();a=self.previous_acceleration.copy()
        result=super().solve_fpmfc(d,**kw)
        # Explicit safe primary degradation only. Never override shape singularity.
        if result.primary_feasible and not result.secondary_feasible and not self.shape.sample(d).singular and not any(self.last_bound_conflicts):
            Q=self.qps[0];x=Q['x'];slack=np.minimum(Q['A']@x-Q['l'],Q['u']-Q['A']@x)
            if min(slack)>=-self.config.feasibility_tolerance-1e-12:
                self.previous_velocity=v;self.previous_acceleration=a;self._commit_velocity(x)
                result=replace(result,joint_velocity=x,success=True,secondary_status='SECONDARY_TASK_DEGRADED',minimum_constraint_slack=float(min(slack)))
        self.last_result=result
        self.last_record={'time':float(d.time),'result':asdict(result),'qps':self.qps,'secondary':self.secondary,'geometry':self.last_geometry,
                          'previous_velocity':v,'previous_acceleration':a,'raw_lower':self.last_raw_velocity_lower.copy(),'raw_upper':self.last_raw_velocity_upper.copy()}
        if not self.prediction:self.records.append(self.last_record)
        return result

class FeasibleController(Controller):
    def __init__(self,prior,cfg):
        super().__init__(prior,cfg);self.method=cfg.get('method','B1');self.predictor=None;self.progress_log=[]
        old=self.hqp
        self.hqp=DiagnosticHQP(self.spec,self.model,old.pairs,self.shape,controller_config=old.fpmfc_config,constraint_config=old.config)
        self.hqp.use_measured_drift=cfg['measured_drift']
        if self.method!='B0':
            self.state_filter=TimedEstimator(cfg);self.reference=ShapedReference(self.model,self.data,cfg)
            self.hqp.use_margins=True
        if self.method=='B1':
            from .progress_governor import ProgressGovernor
            self.predictor=ProgressGovernor(self)
    def before_reference(self,packet,e):
        self.hqp.margin_estimate=e
        if self.predictor is not None:
            if not self.predictor.choose(packet,e):
                self.abort_reason='NO_VERIFIED_CONTROL';self.phase='ABORT';raise NoVerifiedControl(self.abort_reason)
    def update(self,packet,task_tick,servo_tick=True):
        result=super().update(packet,task_tick,servo_tick)
        if not self.estimate.valid and packet.time>self.cfg['observe_s']+.02:
            self.abort_reason='ESTIMATE_UNRELIABLE';self.phase='ABORT';raise NoVerifiedControl(self.abort_reason)
        return result

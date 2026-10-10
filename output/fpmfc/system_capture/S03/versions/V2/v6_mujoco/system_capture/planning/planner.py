"""Deterministic low frequency planning with explicit validity and no recovery claim."""
import copy
import time
from dataclasses import replace
import numpy as np
from scipy.spatial.transform import Rotation
from ..estimation.controller import AccelerationController
from ..adapters import ControllerAdapter
from v6_mujoco.feasible_capture.controller_adapter import NoVerifiedControl
from v6_mujoco.feasible_capture.qp_diagnostics import phase_one
from .parameterization import PlannedReference
from .screening import screen
from .prediction import PriorRollout,Budget
from .contracts import clean

class Planner(PriorRollout):
    def __init__(self,owner,replay=False):
        super().__init__(owner);self.budget=Budget(replay);self.failure=None;self.next_plan=-1.;self.cache=[];self.selection=None
        self.calls=0;self.kinematic_steps=0;self.latencies=[];self.cache_errors=[]

    def family(self,t):
        r=self.owner.reference;p=self.owner.cfg['s03'];remain=max(p['minimum_remaining_s'],r.arrival-t)
        remain=min(remain,19.9-t);shape=self.owner.cfg['s03_method']=='B2_time_path_shape'
        # Same five members and budgets. B1 is exactly the shape-fixed domain.
        if r.segment is None:base=[8.,0.,0.,0.,0.]
        else:base=[remain,0.,0.,0.,float(r.theta[4]) if shape else 0.]
        vals=[base,[max(.6,remain-1.),0.,0.,0.,0.],[min(19.9-t,remain+1.),0.,0.,0.,0.],
            [remain,p['offset_m'],0.,p['shape_rad'] if shape else 0.,p['shape_rad'] if shape else 0.],
            [remain,0.,-p['offset_m'],-p['shape_rad'] if shape else 0.,-p['shape_rad'] if shape else 0.]]
        return vals

    def admissible(self,r):
        p=self.owner.cfg['s03'];T=r.segment['u'].duration
        for t in np.linspace(r.clock,r.arrival,41):
            u,xy,psi=r.jets(t)
            if u[0]<-1e-8 or u[0]>8+1e-8 or u[1]<-1e-8 or u[1]>p['max_virtual_rate'] or abs(u[2])>p['max_virtual_accel']:return False,'PROGRESS_JET_BOUNDS'
            if np.max(abs(xy[:,0]))>p['max_offset_m']+1e-9:return False,'PATH_OFFSET_BOUNDS'
            if abs(psi[0])>p['max_shape_correction_rad'] or abs(psi[1])>p['max_shape_velocity_rad_s'] or abs(psi[2])>p['max_shape_accel_rad_s2']:return False,'SHAPE_JET_BOUNDS'
        return True,None

    def cache_valid(self,packet,e):
        if not self.cache:return False
        k=min(self.cache,key=lambda x:abs(x['time']-packet.time))
        q=float(np.max(abs(packet.joint_position-k['q'])));dq=float(np.max(abs(packet.joint_velocity-k['dq'])))
        p=float(np.linalg.norm(e.p-k['target_p']));angle=Rotation.from_matrix(e.R@np.asarray(k['target_R']).T).magnitude()
        row=dict(time=packet.time,q_error_rad=q,dq_error_rad_s=dq,p_error_m=p,R_error_rad=angle)
        self.cache_errors.append(row);limits=self.owner.cfg['s03']['cache_domain']
        return abs(k['time']-packet.time)<=.010001 and q<=limits['q_rad'] and dq<=limits['dq_rad_s'] and p<=limits['p_m'] and angle<=limits['R_rad']

    def choose(self,packet,e):
        c=self.owner;t=packet.time;p=c.cfg['s03'];r=c.reference
        if not e.valid:self.failure='ESTIMATE_NOT_READY';return False
        if r.blocked:self.failure='PREDICTION_REJECTED';return False
        if t<self.next_plan-1e-9 and t<=r.valid_until+1e-9 and self.cache_valid(packet,e):return True
        started=time.perf_counter();self.calls+=1;records=[];options=[]
        try:
            for i,theta in enumerate(self.family(t)):
                # Member 0 continues the executed segment, preserving all jets.
                ref=copy.deepcopy(r) if i==0 and r.segment is not None else r.candidate(t,theta)
                ref.clock=t;ref._sync();ref.theta=theta
                self.budget.register(c.cfg['s03_method'],t,theta,'CA_kinematic_screen')
                valid,why=self.admissible(ref)
                row=dict(index=i,theta=theta,reference=clean(ref.record()),screen=None,jet_valid=valid,reason=why)
                if valid:
                    s=screen(c,packet,e,ref,p['screen_horizon_s']);row['screen']=clean(s);self.kinematic_steps+=s['steps'];self.budget.finish(0,0,s['steps'])
                    if s['valid']:
                        # Hard violation proxy first; reference-change penalty
                        # precedes disturbance only when both are otherwise close.
                        change=(0. if i==0 else .05)+abs(theta[1])/.04*.05+abs(theta[2])/.04*.05
                        score=s['score'][0]*10+s['score'][1]*2+s['score'][2]+s['score'][3]*.05+change
                        options.append((score,i,ref))
                records.append(row)
            chosen=None;selected=None;dynamic=[]
            # Cache full covariance forecasts only within this exact packet.
            horizon=p['prediction_horizon_s'];n=round(horizon/c.cfg['servo_s']);forecasts=[e]
            for k in range(n):forecasts.append(c.state_filter.predict_snapshot(forecasts[-1],t+(k+1)*c.cfg['servo_s']))
            for score,i,ref in sorted(options,key=lambda x:(x[0],x[1]))[:p['dynamic_survivors']]:
                h=min(horizon,max(.2,ref.arrival-t+.06));result=self.evaluate(packet,e,ref,h,self.budget,forecasts)
                cache=result.pop('cache');records[i]['prediction']=result;dynamic.append((i,result))
                if result['verified']:
                    chosen=ref;selected=i;self.cache=cache;chosen.valid_until=t+h;break
                if result['steps']==0 and result.get('failed_hqp'):
                    Q=result['failed_hqp']['qps'][0];lo=np.array([(-np.inf if x is None else x) for x in Q['l']]);hi=np.array([(np.inf if x is None else x) for x in Q['u']])
                    certificate=phase_one(Q['A'],lo,hi);records[i]['current_phase_one']=certificate
                    if certificate['classification']=='LINEAR_HARD_SET_INFEASIBLE':self.failure='CURRENT_HARD_SET_EMPTY';break
            if chosen is not None:
                c.reference=chosen;self.selection=selected;self.next_plan=t+min(p['planning_period_s'],chosen.valid_until-t-.02);self.failure=None
            else:
                self.failure=self.failure or ('NO_FEASIBLE_PATH_IN_SEARCHED_FAMILY' if not options else 'PREDICTION_REJECTED')
                # Current state has already triggered a fresh unsuccessful check;
                # do not relabel an old validation as a new feasible candidate.
            self.log.append(clean(dict(time=t,candidates=records,selected=selected,failure=self.failure,
                valid_until=c.reference.valid_until,latency_s=time.perf_counter()-started,scope='finite family; prior-only; SIM_ABORT is not recovery')))
            self.latencies.append(time.perf_counter()-started)
            return chosen is not None
        except RuntimeError as ex:
            if 'BUDGET' not in str(ex):raise
            self.failure='PLAN_EXPIRED';self.log.append(dict(time=t,failure=self.failure,reason=str(ex),selected=None,candidates=records));return False

class PlanningController(AccelerationController):
    def __init__(self,prior,cfg,replay=False):
        super().__init__(prior,cfg)
        self.reference=PlannedReference(self.model,self.data,cfg);self.predictor=Planner(self,replay)
    def before_reference(self,packet,e):
        self.hqp.margin_estimate=e
        if not self.predictor.choose(packet,e):
            self.abort_reason=self.predictor.failure;self.phase='ABORT';raise NoVerifiedControl('NO_VERIFIED_CONTROL')
    def update(self,packet,task_tick,servo_tick=True):
        if self.latch_time is None and self.reference.segment is not None:
            if packet.time>self.reference.valid_until+1e-9:
                self.predictor.failure='PLAN_EXPIRED';self.phase='ABORT';raise NoVerifiedControl('NO_VERIFIED_CONTROL')
            if self.reference.blocked:
                self.predictor.failure='PREDICTION_REJECTED';self.phase='ABORT';raise NoVerifiedControl('NO_VERIFIED_CONTROL')
        return super().update(packet,task_tick,servo_tick)
    def planning_snapshot(self):return dict(reference=self.reference.record(),failure=self.predictor.failure,selected=self.predictor.selection,
        next_plan=self.predictor.next_plan,evaluations=self.predictor.budget.local_evaluations)

class PlanningAdapter(ControllerAdapter):
    def update(self,*args,**kwargs):
        result=super().update(*args,**kwargs)
        if not result.actuation_valid:result=replace(result,decision=self.legacy.predictor.failure or 'ESTIMATE_NOT_READY')
        if self.reference is not None:self.reference=replace(self.reference,valid_until=self.legacy.reference.valid_until)
        return result

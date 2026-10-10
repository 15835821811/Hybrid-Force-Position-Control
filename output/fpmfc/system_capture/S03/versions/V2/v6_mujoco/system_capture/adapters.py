"""Lossless packet boundaries and composition around unmodified legacy objects."""
import numpy as np
from .contracts import (FrameConvention,PoseObservation,RobotObservation,SensorPacket,StateEstimate,
                        ParameterEstimate,MotionReference,ControlProposal,NamedConstraint,freeze,plain)
from .common import clean

def packet_from_legacy(p):
    robot=RobotObservation(p.base_pose,p.base_velocity[:3],p.base_velocity[3:],p.joint_position,p.joint_velocity,p.actuator_torque)
    poses=tuple(PoseObservation(z.stamp,p.time,z.position,z.rotation,z.covariance,FrameConvention('W','target_geometry')) for z in p.poses)
    return SensorPacket(p.time,p.time,p.time,robot,poses,p.wrench_target_at_grasp_world,p.contact)

def packet_to_legacy(p):
    from v6_mujoco.adaptive_capture.contracts import SensorPacket as LegacyPacket,PoseObservation as LegacyPose
    r=p.robot
    poses=tuple(LegacyPose(z.t_sample,z.p.copy(),z.R.copy(),z.P.copy()) for z in p.poses)
    return LegacyPacket(p.t_control,r.base_pose_wxyz.copy(),np.r_[r.base_linear_velocity_W,r.base_angular_velocity_B],r.q7.copy(),r.dq7.copy(),r.actuator_tau7.copy(),p.wrench_target_at_grasp_W.copy(),p.contact,poses)

def estimate_from_legacy(e,cfg,process_mode):
    if e is None:return None
    bias=np.zeros(12);bias[:3]=cfg.get('position_bias_bound_m',0.);bias[3:6]=cfg.get('rotation_bias_bound_rad',0.)
    return StateEstimate(e.measurement_time,e.time,e.p,e.R,e.v,e.w,e.covariance,bias,e.valid,process_mode)

class SensorAdapter:
    def __init__(self,frontend):self._frontend=frontend
    def sample(self,model,data):return packet_from_legacy(self._frontend.sample(model,data))

class PlantAdapter:
    """Evaluation-side only. No plant handle is accepted by ControllerAdapter."""
    def __init__(self,plant):self._plant=plant
    def apply(self,proposal):
        if not proposal.actuation_valid:raise RuntimeError('SIM_ABORT: no actuation authorized')
        if proposal.latch_request:self._plant.latch()
        self._plant.step(proposal.tau7)
    def replay_event(self,latch):
        if latch:self._plant.latch()
    def replay_step(self,tau7):self._plant.step(np.array(tau7,copy=True))
    def assert_frozen(self):self._plant.assert_frozen()

class ControllerAdapter:
    def __init__(self,legacy):
        self.legacy=legacy
        self.last_raw=None
        self.last_snapshot=None
        self.reference=None
        self.estimate=None
        self.parameters=None
        self.last_time=-1.

    def update(self,packet,task_tick,servo_tick=True):
        c=self.legacy
        if not isinstance(packet,SensorPacket):raise TypeError('validated SensorPacket required')
        if packet.t_control <= self.last_time:raise ValueError('strictly increasing control timestamps')
        self.last_time=packet.t_control
        before_ref=c.latest_reference
        before_progress=(c.reference.progress,c.reference.rate,c.reference.accel)
        raw=packet_to_legacy(packet);decision='CONTROL';valid=True;latch=False;tau=np.zeros(7)
        try:tau,latch=c.update(raw,task_tick,servo_tick)
        except RuntimeError as ex:
            if str(ex) not in ('NO_VERIFIED_CONTROL','ESTIMATE_UNRELIABLE'):raise
            decision=str(ex);valid=False
        self.last_raw=freeze({'tau7':np.array(tau,copy=True),'latch_request':bool(latch),'phase':c.phase,'decision':decision,'actuation_valid':valid})
        self.last_snapshot=freeze(clean({'estimate':c.estimate,'reference':c.latest_reference,'gate':c.last_gate,'qref':c.qref,'velocity':c.vel,
            'start_velocity':c.start,'phase':c.phase,'s':c.reference.progress/8,'s_dot_filter':c.reference.rate/8,'s_ddot_filter':c.reference.accel/8,
            'pi_est':c.estimator.pi,'pi_ctrl':c.information_gate.model_pi,'raw_target':getattr(c.reference,'raw',c.estimate),'reference_target':c.reference.estimate}))
        mode='CONTACT_PROCESS' if bool(getattr(c.state_filter,'contact',False)) else 'FREE_PROCESS'
        self.estimate=estimate_from_legacy(c.estimate,c.cfg,mode)
        es=c.estimator.snapshot()
        self.parameters=ParameterEstimate(c.estimator.pi,c.estimator.cov,'scaled coordinates; '+es['covariance_scope'],int(c.estimator.rank),bool(es['physical']),
             c.estimator.predict[-1] if c.estimator.predict else {'status':'NOT_EVALUATED'},bool(c.feedback_used))
        if c.latest_reference is not None and c.latest_reference is not before_ref:
            r=c.latest_reference;T=np.eye(4);T[:3,:3]=r.rotation_world;T[:3,3]=r.position_world_m
            self.reference=MotionReference(packet.t_control,T,r.linear_velocity_world_m_s,r.angular_velocity_world_rad_s,
                r.linear_acceleration_world_m_s2,r.angular_acceleration_world_rad_s2,*[v/8 for v in before_progress],r.arm_angle_rad,r.arm_angle_velocity_rad_s,packet.t_control+c.cfg['task_s'])
        if c.latch_time is not None:self.reference=None
        statuses={'primary':'NOT_CALLED','secondary':'NOT_CALLED','controller':decision};constraints=[]
        h=c.hqp
        fresh_task=bool(c.tasks and abs(c.tasks[-1]['time']-packet.t_control)<1e-10)
        if task_tick and fresh_task and hasattr(h,'last_result'):
            r=h.last_result;statuses={'primary':r.primary_status,'secondary':r.secondary_status,'success':bool(r.success)}
            from v6_mujoco.geometry_capture.design import planning_minimum
            limits={pair.name:planning_minimum(pair) for pair in h.pairs}
            for row in getattr(h,'last_geometry',[]):
                constraints.append(NamedConstraint(row['pair'],'algorithm_acceptance','m',.001,'legacy HQP planning distance; distinct physical gate',float(row.get('distance',row.get('distance_m'))),float(limits[row['pair']]), '>='))
        fallback='SIM_ABORT' if not valid else 'LEGACY_OBSERVE_ZERO' if not c.estimate.valid or c.phase=='OBSERVE' else 'NONE'
        return ControlProposal(packet.t_control,tau,c.phase,statuses,tuple(constraints),bool(latch),fallback,decision,valid)

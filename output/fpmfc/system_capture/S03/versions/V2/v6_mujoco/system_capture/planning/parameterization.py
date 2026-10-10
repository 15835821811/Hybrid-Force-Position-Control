"""Five dimensional future reference, frozen terminal transform and target transport."""
import copy
from dataclasses import replace
import numpy as np
from v6_mujoco.feasible_capture.reference_shaper import ShapedReference
from v6_mujoco.geometry_capture.audit import siteT
from v6_mujoco.model import site_id
from .trajectory_segment import JetSegment

class PlannedReference(ShapedReference):
    def __init__(self,m,d,cfg):
        super().__init__(m,d,cfg)
        self.clock=0.;self.segment=None;self.valid_until=-1.;self.blocked=False
        self.T_FE=np.linalg.inv(siteT(m,d,'flange_site'))@siteT(m,d,'postgrasp_tool_interface')
        bid=m.site_bodyid[site_id(m,'target_grasp_site')];T_WO=np.eye(4);T_WO[:3,:3]=d.xmat[bid].reshape(3,3);T_WO[:3,3]=d.xpos[bid]
        self.T_OG=np.linalg.inv(T_WO)@siteT(m,d,'target_grasp_site')
        # Both axes are fixed in the model's grasp frame; never chosen from a failure.
        self.axes=self.T_OG[:3,:3]@np.array([[0.,1.],[0.,0.],[-1.,0.]])

    @property
    def normalized_progress(self):return self.progress/8.
    @property
    def relative_approach(self):return self.normalized_progress>=.75
    @property
    def capture_window(self):return self.normalized_progress>=.975

    def jets(self,t):
        if self.segment is None:
            return np.array([self.progress,self.rate,self.accel]),np.zeros((2,3)),np.zeros(3)
        u=self.segment['u'].jet(t);xy=np.array([s.jet(t) for s in self.segment['xy']]);psi=self.segment['psi'].jet(t)
        if t>=self.arrival-1e-11:u=np.array([8.,0.,0.]);xy[:]=0;psi[1:]=0
        return u,xy,psi

    @property
    def arrival(self):return self.segment['u'].start+self.segment['u'].duration if self.segment else self.clock+8-self.progress

    def candidate(self,t,theta):
        result=copy.deepcopy(self);result.clock=t
        T,n,z,pm,pe=map(float,theta)
        u,xy,ps=self.jets(t)
        # End rate 1 preserves exactly the frozen baseline when T=8-u and
        # initial jet=(u,1,0). Relative terminal jets are zero, even at clamping.
        result.segment=dict(u=JetSegment.connect(t,T,u,[8.,1.,0.]),
            xy=[JetSegment.connect(t,T,xy[j],[0.,0.,0.],bump=x) for j,x in enumerate([n,z])],
            psi=JetSegment.connect(t,T,ps,[pe,0.,0.],bump=pm))
        result.theta=list(map(float,theta));result.blocked=False
        result._sync();return result

    def _sync(self):
        u,_,_=self.jets(self.clock);self.progress,self.rate,self.accel=map(float,u)
        self.jerk=0. if self.segment is None or self.clock>=self.arrival else float(np.polynomial.polynomial.polyval((self.clock-self.segment['u'].start)/self.segment['u'].duration,np.polynomial.polynomial.polyder(self.segment['u'].coefficients,3))/self.segment['u'].duration**3)

    def advance(self,dt,allow=True):
        self.clock+=dt;self._sync()
        # A veto invalidates the plan; no discontinuous stop of a moving target.
        self.blocked |= not allow

    def sample(self,t):
        if self.segment is None:return super().sample(t)
        return self.sample_at(self.clock)

    def sample_at(self,t):
        e=self.estimate;u,xy,ps=self.jets(t);q,qd,qdd=u;r=self.nominal.sample(float(np.clip(q,0,8)));n=self.nominal.target.sample(float(np.clip(q,0,8)))
        Rn=n.center_rotation_world;pn=n.center_position_world_m;vn=n.center_linear_velocity_world_m_s;wn=n.angular_velocity_world_rad_s
        local=Rn.T@(r.position_world_m-pn)
        vr=Rn.T@(r.linear_velocity_world_m_s-vn-np.cross(wn,r.position_world_m-pn))
        ar=Rn.T@(r.linear_acceleration_world_m_s2-2*np.cross(wn,r.linear_velocity_world_m_s-vn)+np.cross(wn,np.cross(wn,r.position_world_m-pn)))
        wr=Rn.T@(r.angular_velocity_world_rad_s-wn)
        alphar=Rn.T@(r.angular_acceleration_world_rad_s2-np.cross(wn,r.angular_velocity_world_rad_s-wn))
        b=self.axes@xy[:,0];bd=self.axes@xy[:,1];bdd=self.axes@xy[:,2]
        offset=e.R@(local+b);vrel=e.R@(qd*vr+bd);wrel=e.R@(qd*wr)
        # Raw/shaped target state is independent of the local correction.
        a=self.target_a if self.cfg.get('shape_reference',False) else e.a
        alpha=self.target_alpha if self.cfg.get('shape_reference',False) else e.alpha
        return replace(r,time_s=t,position_world_m=e.p+offset,rotation_world=e.R@Rn.T@r.rotation_world,
            linear_velocity_world_m_s=e.v+np.cross(e.w,offset)+vrel,
            linear_acceleration_world_m_s2=a+np.cross(alpha,offset)+np.cross(e.w,np.cross(e.w,offset))+2*np.cross(e.w,vrel)+e.R@(qdd*vr+qd*qd*ar+bdd),
            angular_velocity_world_rad_s=e.w+wrel,
            angular_acceleration_world_rad_s2=alpha+np.cross(e.w,wrel)+e.R@(qdd*wr+qd*qd*alphar),
            arm_angle_rad=r.arm_angle_rad+ps[0],arm_angle_velocity_rad_s=r.arm_angle_velocity_rad_s*qd+ps[1],
            arm_angle_acceleration_rad_s2=r.arm_angle_acceleration_rad_s2*qd*qd+r.arm_angle_velocity_rad_s*qdd+ps[2])

    def record(self):
        return dict(clock=self.clock,valid_until=self.valid_until,arrival=self.arrival,blocked=self.blocked,theta=getattr(self,'theta',None),
            segment=None if self.segment is None else dict(u=self.segment['u'].record(),xy=[s.record() for s in self.segment['xy']],psi=self.segment['psi'].record()),
            T_FE=self.T_FE,T_OG=self.T_OG,axes=self.axes)

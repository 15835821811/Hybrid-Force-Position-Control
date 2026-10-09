"""Estimated target transform times fixed relative C1 path; one derivative chain."""
from dataclasses import replace
import numpy as np
from .contracts import StateEstimate
from v6_mujoco.geometry_capture.reference import Reference

class RelativeReference:
    def __init__(self,m,d):
        self.nominal=Reference(m,d,'standard_C1');self.progress=0.;self.rate=1.;self.accel=0.;self.estimate=None
    def advance(self,dt,allow=True):
        desired=1. if allow else 0.
        wanted=np.clip((desired-self.rate)*4,-.5,.5)
        self.accel+=np.clip(wanted-self.accel,-dt,dt)
        self.rate=float(np.clip(self.rate+self.accel*dt,0.,1.))
        self.progress=min(8.,self.progress+self.rate*dt)
    def sample(self,t):
        e=self.estimate
        u=min(self.progress,8.);r=self.nominal.sample(u);n=self.nominal.target.sample(u)
        pn=n.center_position_world_m;Rn=n.center_rotation_world;vn=n.center_linear_velocity_world_m_s;wn=n.angular_velocity_world_rad_s
        C=e.R@Rn.T;offset=C@(r.position_world_m-pn)
        rate=self.rate if u<8-1e-10 else 0.;acc=self.accel if u<8-1e-10 else 0.
        vr=C@(r.linear_velocity_world_m_s-vn-np.cross(wn,r.position_world_m-pn))
        wr=C@(r.angular_velocity_world_rad_s-wn)
        ar=C@(r.linear_acceleration_world_m_s2-2*np.cross(wn,r.linear_velocity_world_m_s-vn)+np.cross(wn,np.cross(wn,r.position_world_m-pn)))
        alphar=C@(r.angular_acceleration_world_rad_s2-np.cross(wn,r.angular_velocity_world_rad_s-wn))
        return replace(r,position_world_m=e.p+offset,rotation_world=C@r.rotation_world,
            linear_velocity_world_m_s=e.v+np.cross(e.w,offset)+rate*vr,
            angular_velocity_world_rad_s=e.w+rate*wr,
            linear_acceleration_world_m_s2=np.cross(e.w,np.cross(e.w,offset))+2*rate*np.cross(e.w,vr)+rate**2*ar+acc*vr,
            angular_acceleration_world_rad_s2=rate*np.cross(e.w,wr)+rate**2*alphar+acc*wr,
            arm_angle_velocity_rad_s=r.arm_angle_velocity_rad_s*rate)

"""Estimated target transform times fixed relative C1 path; one derivative chain."""
from dataclasses import replace
import numpy as np
from .contracts import StateEstimate
from v6_mujoco.geometry_capture.reference import Reference

class RelativeReference:
    def __init__(self,m,d,duration=8.):
        self.nominal=Reference(m,d,'standard_C1');self.progress=0.;self.max_rate=8./duration;self.rate=self.max_rate;self.accel=0.;self.estimate=None
        # Three positive first-order filters: rate remains in [0,max_rate].
        # Exact ZOH propagation is C2 in rate across demand changes, without
        # clipping an inconsistent acceleration at either rate boundary.
        self.z1=self.z2=self.rate;self.tau=max(self.max_rate/.5,np.sqrt(2*self.max_rate/1.))
        self.jerk=0.
    def advance(self,dt,allow=True):
        u=self.max_rate if allow else 0.;h=dt/self.tau;decay=np.exp(-h)
        a,b,c=self.z1-u,self.z2-u,self.rate-u
        integral=u*dt+self.tau*(c*(-np.expm1(-h))+b*(1-(1+h)*decay)+a*(1-(1+h+.5*h*h)*decay))
        self.z1=u+a*decay;self.z2=u+(b+a*h)*decay;self.rate=float(u+(c+b*h+.5*a*h*h)*decay)
        self.accel=(self.z2-self.rate)/self.tau;self.jerk=(self.z1-2*self.z2+self.rate)/self.tau**2
        self.progress=min(8.,self.progress+integral)
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

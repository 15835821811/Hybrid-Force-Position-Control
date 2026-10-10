"""Frozen inertial quintic precontact reference; endpoint velocity is zero."""
import numpy as np
from scipy.spatial.transform import Rotation
from ..contracts import MotionReference
from .common import source,assumptions
from .kinematics import rz

def target(t):
    s=source()['initial'];w=np.array([0.,0.,np.deg2rad(s['target_spin_deg_s'])]);R=rz(w[2]*t)
    offset=R@np.array(assumptions()['target']['grasp_offset_target_m'])
    p=np.array(s['target_origin_in_initial_base_m'])+offset
    orientation=R@Rotation.from_euler('xyz',s['grasp_orientation_reported_rad']).as_matrix()
    return p,orientation,np.cross(w,offset),w

def blend(t,T):
    s=np.clip(t/T,0,1);h=10*s**3-15*s**4+6*s**5
    v=(30*s*s-60*s**3+30*s**4)/T;a=(60*s-180*s*s+120*s**3)/T**2
    return float(h),float(v),float(a)

class Reference:
    def __init__(self,initial_terms,initial_shape,T,psi):
        self.T=float(T);self.psi0=float(initial_shape);self.psi=float(psi)
        self.p0=initial_terms.p.copy();self.R0=initial_terms.R.copy();self.pf,self.Rf,_,_=target(T)
        self.dr=Rotation.from_matrix(self.Rf@self.R0.T).as_rotvec();self.dp=self.pf-self.p0
    def sample(self,t):
        h,v,a=blend(t,self.T);R=Rotation.from_rotvec(h*self.dr).as_matrix()@self.R0
        T=np.eye(4);T[:3,:3]=R;T[:3,3]=self.p0+h*self.dp
        return MotionReference(float(t),T,v*self.dp,v*self.dr,a*self.dp,a*self.dr,h,v,a,
            self.psi0+h*(self.psi-self.psi0),v*(self.psi-self.psi0),float(t)+assumptions()['controller']['task_period_s'])

def integrate_configuration(qpos,velocity,dt):
    q=np.array(qpos,copy=True);q[:3]+=dt*velocity[:3]
    R=Rotation.from_quat(q[[4,5,6,3]])*Rotation.from_rotvec(dt*velocity[3:6]);xyzw=R.as_quat();q[3:7]=xyzw[[3,0,1,2]]
    q[7:]+=dt*velocity[6:];return q

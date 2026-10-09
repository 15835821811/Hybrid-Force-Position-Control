import copy
from dataclasses import replace
import numpy as np
from scipy.spatial.transform import Rotation
from v6_mujoco.adaptive_capture.relative_reference import RelativeReference

def clip_norm(x,limit):
    return x*min(1.,limit/max(np.linalg.norm(x),1e-30))

class ShapedReference(RelativeReference):
    """Independent target command state; capture always uses raw estimate.

    Translation has exact constant-jerk integration. Rotation uses a midpoint
    Lie increment, with O(dt^3) local error; this is a numerical approximation.
    """
    def __init__(self,m,d,cfg):
        self.raw=None;self.shaped=None;self.cfg=cfg;self.target_a=np.zeros(3);self.target_alpha=np.zeros(3)
        super().__init__(m,d,cfg['template_duration_s'])
        self.tau=cfg['n209']['progress_tau_s'];self.demand=self.max_rate
    @property
    def estimate(self):return self.shaped
    @estimate.setter
    def estimate(self,e):
        self.raw=e
        if e is None:return
        if self.shaped is None or not self.cfg.get('shape_reference',False):self.shaped=copy.deepcopy(e);return
        dt=e.time-self.shaped.time
        if dt<=0:return
        old=self.shaped;w0=self.cfg['n209']['reference_bandwidth_rad_s']
        a=w0*w0*(e.p-old.p)+2*w0*(e.v-old.v)
        alpha=w0*w0*Rotation.from_matrix(e.R@old.R.T).as_rotvec()+2*w0*(e.w-old.w)
        j=clip_norm((clip_norm(a,.1)-self.target_a)/dt,2.)
        ja=clip_norm((clip_norm(alpha,1.)-self.target_alpha)/dt,20.)
        p=old.p+old.v*dt+.5*self.target_a*dt**2+j*dt**3/6
        v=old.v+self.target_a*dt+.5*j*dt**2
        w=old.w+self.target_alpha*dt+.5*ja*dt**2
        R=Rotation.from_rotvec((old.w+.5*self.target_alpha*dt+ja*dt**2/6)*dt).as_matrix()@old.R
        self.target_a+=j*dt;self.target_alpha+=ja*dt
        self.shaped=replace(e,p=p,R=R,v=v,w=w)
    def advance(self,dt,allow=True):
        u=self.demand if allow else 0.;h=dt/self.tau;decay=np.exp(-h)
        a,b,c=self.z1-u,self.z2-u,self.rate-u
        integral=u*dt+self.tau*(c*(-np.expm1(-h))+b*(1-(1+h)*decay)+a*(1-(1+h+.5*h*h)*decay))
        self.z1=u+a*decay;self.z2=u+(b+a*h)*decay;self.rate=float(u+(c+b*h+.5*a*h*h)*decay)
        self.accel=(self.z2-self.rate)/self.tau;self.jerk=(self.z1-2*self.z2+self.rate)/self.tau**2
        self.progress=min(8.,self.progress+integral)
    def sample(self,t):
        r=super().sample(t)
        if self.cfg.get('shape_reference',False) and self.shaped is not None:
            offset=r.position_world_m-self.shaped.p
            r=replace(r,linear_acceleration_world_m_s2=r.linear_acceleration_world_m_s2+self.target_a+np.cross(self.target_alpha,offset),
                      angular_acceleration_world_rad_s2=r.angular_acceleration_world_rad_s2+self.target_alpha)
        return r

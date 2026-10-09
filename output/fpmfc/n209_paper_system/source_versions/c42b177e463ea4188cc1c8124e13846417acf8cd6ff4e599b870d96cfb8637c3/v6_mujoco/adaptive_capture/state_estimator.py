"""Causal SO(3) left-error pose/twist EKF. Measurement-time state + prediction."""
import numpy as np
from scipy.spatial.transform import Rotation
from .contracts import StateEstimate,skew

class StateEstimator:
    def __init__(self,config):
        self.cfg=config;self.time=None;self.p=np.zeros(3);self.R=np.eye(3);self.v=np.zeros(3);self.w=np.zeros(3)
        self.P=np.diag([1e-4]*6+[.1]*6);self.innovation=np.zeros(6);self.n=0;self.contact=False
    def propagate(self,p,R,v,w,P,dt):
        if dt<=0:return p.copy(),R.copy(),v.copy(),w.copy(),P.copy()
        F=np.eye(12);F[:3,6:9]=np.eye(3)*dt;F[3:6,3:6]+=skew(w)*dt;F[3:6,9:12]=np.eye(3)*dt
        factor=self.cfg['contact_process_multiplier'] if self.contact else 1.
        a=self.cfg['process_linear_accel']*factor;alpha=self.cfg['process_angular_accel']*factor
        Q=np.zeros((12,12))
        # White acceleration density, distinct units for linear/angular blocks.
        for pose,vel,s in [(0,6,a),(3,9,alpha)]:
            Q[pose:pose+3,pose:pose+3]=np.eye(3)*s*s*dt**3/3
            Q[pose:pose+3,vel:vel+3]=Q[vel:vel+3,pose:pose+3]=np.eye(3)*s*s*dt**2/2
            Q[vel:vel+3,vel:vel+3]=np.eye(3)*s*s*dt
        return p+v*dt,Rotation.from_rotvec(w*dt).as_matrix()@R,v.copy(),w.copy(),F@P@F.T+Q
    def update(self,packet):
        self.contact=packet.contact or np.linalg.norm(packet.wrench_target_at_grasp_world[:3])>.02
        for z in packet.poses:
            if self.time is None:
                self.p=z.position.copy();self.R=z.rotation.copy();self.time=z.stamp;self.P[:6,:6]=z.covariance;self.n=1;continue
            if z.stamp<=self.time:raise ValueError('nonmonotone or duplicate sensor timestamp')
            self.p,self.R,self.v,self.w,self.P=self.propagate(self.p,self.R,self.v,self.w,self.P,z.stamp-self.time)
            self.innovation=np.r_[z.position-self.p,Rotation.from_matrix(z.rotation@self.R.T).as_rotvec()]
            H=np.zeros((6,12));H[:6,:6]=np.eye(6);S=H@self.P@H.T+z.covariance
            K=np.linalg.solve(S,H@self.P).T;dx=K@self.innovation
            self.p+=dx[:3];self.R=Rotation.from_rotvec(dx[3:6]).as_matrix()@self.R;self.v+=dx[6:9];self.w+=dx[9:]
            A=np.eye(12)-K@H;self.P=A@self.P@A.T+K@z.covariance@K.T
            G=np.eye(12);G[3:6,3:6]-=.5*skew(dx[3:6]);self.P=G@self.P@G.T
            self.time=z.stamp;self.n+=1
        if self.time is None:return StateEstimate(packet.time,-1.,self.p,self.R,self.v,self.w,self.P,False,self.innovation)
        p,R,v,w,P=self.propagate(self.p,self.R,self.v,self.w,self.P,packet.time-self.time)
        return StateEstimate(packet.time,self.time,p,R,v,w,P,self.n>=3 and packet.time-self.time<=self.cfg['stale_limit_s'],self.innovation.copy())

"""Measurement-time EKF with causal process-mode history and fresh-only NIS."""
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation
from v6_mujoco.adaptive_capture.state_estimator import StateEstimator
from v6_mujoco.adaptive_capture.contracts import StateEstimate,skew

class TimedEstimator(StateEstimator):
    def __init__(self,cfg):
        super().__init__(cfg);self.history=deque();self.log=[];self.last_arrival=-1.;self.rejects=[]
    def between(self,p,R,v,w,P,start,end):
        mode=False;cursor=start
        for t,contact in self.history:
            if t<=start:mode=contact;continue
            if t>=end:break
            self.contact=mode;p,R,v,w,P=self.propagate(p,R,v,w,P,t-cursor);cursor=t;mode=contact
        self.contact=mode
        return self.propagate(p,R,v,w,P,end-cursor)
    def update(self,packet):
        if packet.time<self.last_arrival:raise ValueError('nonmonotone control time')
        self.last_arrival=packet.time
        mode=bool(packet.contact or np.linalg.norm(packet.wrench_target_at_grasp_world[:3])>.1)
        self.history.append((packet.time,mode))
        while len(self.history)>1 and self.history[1][0]<packet.time-.25:self.history.popleft()
        for z in sorted(packet.poses,key=lambda z:z.stamp):
            if z.stamp>packet.time+1e-10:raise ValueError('future measurement forbidden')
            if self.time is not None and z.stamp<=self.time:
                self.rejects.append({'arrival':packet.time,'stamp':z.stamp,'reason':'DUPLICATE_OR_OUT_OF_ORDER_REJECTED'});continue
            if z.stamp<packet.time-.25:
                self.rejects.append({'arrival':packet.time,'stamp':z.stamp,'reason':'OUTSIDE_MODE_BUFFER'});continue
            if self.time is None:
                self.p=z.position.copy();self.R=z.rotation.copy();self.time=z.stamp;self.P[:6,:6]=z.covariance;self.n=1;continue
            self.p,self.R,self.v,self.w,self.P=self.between(self.p,self.R,self.v,self.w,self.P,self.time,z.stamp)
            self.innovation=np.r_[z.position-self.p,Rotation.from_matrix(z.rotation@self.R.T).as_rotvec()]
            H=np.zeros((6,12));H[:6,:6]=np.eye(6);S=H@self.P@H.T+z.covariance
            nis=float(self.innovation@np.linalg.solve(S,self.innovation));K=np.linalg.solve(S,H@self.P).T;dx=K@self.innovation
            self.p+=dx[:3];self.R=Rotation.from_rotvec(dx[3:6]).as_matrix()@self.R;self.v+=dx[6:9];self.w+=dx[9:]
            A=np.eye(12)-K@H;self.P=A@self.P@A.T+K@z.covariance@K.T
            G=np.eye(12);G[3:6,3:6]-=.5*skew(dx[3:6]);self.P=G@self.P@G.T
            self.time=z.stamp;self.n+=1
            self.log.append({'stamp':z.stamp,'arrival':packet.time,'NIS':nis,'innovation':self.innovation.copy(),'correction':dx,'process_contact_at_stamp':self.contact})
        if self.time is None:return StateEstimate(packet.time,-1.,self.p,self.R,self.v,self.w,self.P,False,self.innovation)
        p,R,v,w,P=self.between(self.p,self.R,self.v,self.w,self.P,self.time,packet.time)
        return StateEstimate(packet.time,self.time,p,R,v,w,P,self.n>=3 and packet.time-self.time<=self.cfg['stale_limit_s'],self.innovation.copy())

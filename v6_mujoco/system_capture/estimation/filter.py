"""Causal 18-state local CA model, world velocities and left SO(3) errors.

No target dynamics, COM, force/mass shortcut, future samples or plant handles.
White jerk density units: m/s**(5/2), rad/s**(5/2); Q uses density squared.
"""
from collections import deque
from dataclasses import dataclass, replace
import copy
import time
import numpy as np
from scipy.spatial.transform import Rotation
from v6_mujoco.adaptive_capture.contracts import StateEstimate, skew
from ..contracts import freeze, covariance, rotation

ORDER = 'dp_W,dtheta_W,dv_W,domega_W,da_W,dalpha_W'
TIDX = np.r_[0:3,6:9,12:15]
RIDX = np.r_[3:6,9:12,15:18]

@dataclass(frozen=True)
class AccelerationSnapshot(StateEstimate):
    a: np.ndarray
    alpha: np.ndarray
    full_covariance: np.ndarray
    process_contact: bool
    linear_jerk_density: float
    angular_jerk_density: float
    contact_multiplier: float
    max_step_s: float
    covariance_order: str = ORDER

    def __post_init__(self):
        # Arrays never alias live state. SciPy 1.11 legacy Rotation consumers
        # require a writable R buffer, so R is an owned copy; covariance uses
        # immutable bytes. Every replace/prediction recopies R as well.
        for name in ('p','R','v','w','covariance','innovation','a','alpha','full_covariance'):
            value=np.array(getattr(self,name),dtype=float,copy=True)
            object.__setattr__(self,name,value if name=='R' else freeze(value))
        if self.covariance_order != ORDER or self.full_covariance.shape != (18,18):
            raise ValueError('18-state snapshot required; never reconstruct from P12')
        if not np.allclose(self.covariance,self.full_covariance[:12,:12],rtol=0,atol=1e-14):
            raise ValueError('inconsistent P12 marginal')

def left_jacobian(x):
    t=np.linalg.norm(x); K=skew(x)
    if t<1e-5: return np.eye(3)+(.5-t*t/24)*K+(1/6-t*t/120)*(K@K)
    return np.eye(3)+(1-np.cos(t))/t**2*K+(t-np.sin(t))/t**3*(K@K)

def white_jerk(dt, density):
    h=dt
    return density**2*np.array([[h**5/20,h**4/8,h**3/6],
        [h**4/8,h**3/3,h**2/2],[h**3/6,h**2/2,h]])

def angular_step(R,w,alpha,h,density):
    """RK4 mean and independent variational/covariance ODE, at most 2 ms.

    R is projected to SO(3). General multiaxis acceleration is not replaced by
    a single exact-looking exponential. Finite differences test this Jacobian.
    """
    def A(t):
        a=np.zeros((9,9));a[:3,:3]=skew(w+alpha*t)
        a[:3,3:6]=np.eye(3);a[3:6,6:]=np.eye(3)
        return a
    D=np.zeros((9,9));D[6:,6:]=np.eye(3)*density**2
    def fun(t,r,f):
        a=A(t)
        return skew(w+alpha*t)@r,a@f
    r,f=R,np.eye(9)
    k1=fun(0,r,f)
    k2=fun(h/2,*(x+h/2*k for x,k in zip((r,f),k1)))
    k3=fun(h/2,*(x+h/2*k for x,k in zip((r,f),k2)))
    k4=fun(h,*(x+h*k for x,k in zip((r,f),k3)))
    r,f=(x+h/6*(a+2*b+2*c+d) for x,a,b,c,d in zip((r,f),k1,k2,k3,k4))
    # Integrate Q as positive-weight noise-factor outer products. Direct RK4
    # on Q from zero misses the h^5 white-jerk orientation variance and is not
    # PSD preserving at ideal-noise/contact transitions. Never clip eigenvalues.
    q=np.zeros((9,9));G=np.zeros((9,3));G[6:]=np.eye(3)*density
    for x,weight in ((-np.sqrt(3/5),5/9),(0.,8/9),(np.sqrt(3/5),5/9)):
        u=(x+1)*h/2;d=h-u
        k1=A(u)@G;k2=A(u+d/2)@(G+d/2*k1)
        k3=A(u+d/2)@(G+d/2*k2);k4=A(h)@(G+d*k3)
        B=G+d/6*(k1+2*k2+2*k3+k4)
        q+=h/2*weight*(B@B.T)
    return Rotation.from_matrix(r).as_matrix(),f,q

def propagate(s, dt, contact=None):
    if not np.isfinite(dt) or dt < -1e-12:raise ValueError('invalid propagation interval')
    if dt<=0:return replace(s)
    mode=s.process_contact if contact is None else bool(contact)
    factor=s.contact_multiplier if mode else 1.
    p,R,v,w,a,alpha,P=[np.array(getattr(s,k),copy=True) for k in ('p','R','v','w','a','alpha','full_covariance')]
    count=max(1,int(np.ceil(dt/s.max_step_s-1e-12)));h=dt/count
    f1=np.array([[1,h,h*h/2],[0,1,h],[0,0,1]])
    ft=np.kron(f1,np.eye(3));qt=np.kron(white_jerk(h,s.linear_jerk_density*factor),np.eye(3))
    for _ in range(count):
        R,fr,qr=angular_step(R,w,alpha,h,s.angular_jerk_density*factor)
        F=np.zeros((18,18));Q=np.zeros((18,18))
        F[np.ix_(TIDX,TIDX)]=ft;F[np.ix_(RIDX,RIDX)]=fr
        Q[np.ix_(TIDX,TIDX)]=qt;Q[np.ix_(RIDX,RIDX)]=qr
        P=F@P@F.T+Q
        p+=v*h+.5*a*h*h;v+=a*h;w+=alpha*h
    P=(P+P.T)/2
    return replace(s,time=s.time+dt,p=p,R=R,v=v,w=w,full_covariance=P,
                   covariance=P[:12,:12],process_contact=mode)

def predict_snapshot(snapshot,target_time,assumed_mode_schedule=None):
    """Read-only forecast using full P18. Default: hold latest observed mode.

    Future schedules are assumptions supplied by the caller, not contact truth.
    No measurement corrections are invented in the blind forecast.
    """
    if not isinstance(snapshot,AccelerationSnapshot):raise TypeError('full snapshot required')
    if not np.isfinite(target_time) or target_time < snapshot.time-1e-12:
        raise ValueError('prediction target precedes snapshot')
    s=replace(snapshot);mode=s.process_contact;cursor=s.time
    for t,contact in assumed_mode_schedule or ():
        if not cursor <= t <= target_time:raise ValueError('invalid assumed mode schedule')
        s=propagate(s,t-cursor,mode);cursor=t;mode=bool(contact)
    return propagate(s,target_time-cursor,mode)

class AccelerationEstimator:
    def __init__(self,cfg):
        self.cfg=copy.deepcopy(cfg);self.model=copy.deepcopy(cfg['s02_estimator'])
        self.time=None;self.last_arrival=-1.;self.n=0;self.history=deque();self.log=[];self.rejects=[]
        self.contact=False;self.innovation=np.zeros(6);self.update_times=[];self.predict_times=[]
        m=self.model
        for k in ('linear_jerk_density','angular_jerk_density','contact_multiplier','max_step_s'):
            if not np.isfinite(m[k]) or m[k]<=0:raise ValueError('positive finite '+k)
        P=np.diag([1e-4]*6+[m['initial_velocity_variance']]*3+[m['initial_omega_variance']]*3+
                  [m['initial_accel_variance']]*3+[m['initial_alpha_variance']]*3)
        self.s=AccelerationSnapshot(0.,-1.,np.zeros(3),np.eye(3),np.zeros(3),np.zeros(3),P[:12,:12],False,np.zeros(6),
            np.zeros(3),np.zeros(3),P,False,m['linear_jerk_density'],m['angular_jerk_density'],m['contact_multiplier'],m['max_step_s'])
        self.latest_snapshot=self.s

    @property
    def P(self):return self.s.full_covariance

    def between(self,s,start,end):
        mode=False;cursor=start
        for t,contact in self.history:
            if t<=start:mode=contact;continue
            if t>=end:break
            s=propagate(s,t-cursor,mode);cursor=t;mode=contact
        return propagate(s,end-cursor,mode)

    def predict_snapshot(self,snapshot,target_time,assumed_mode_schedule=None):
        started=time.perf_counter()
        result=predict_snapshot(snapshot,target_time,assumed_mode_schedule)
        self.predict_times.append(time.perf_counter()-started)
        return result

    def update(self,packet):
        started=time.perf_counter()
        if not np.isfinite(packet.time) or packet.time<self.last_arrival:raise ValueError('nonmonotone control time')
        if not np.isfinite(packet.wrench_target_at_grasp_world).all():raise ValueError('nonfinite wrench')
        # Validate the complete batch before changing the online posterior.
        for z in packet.poses:
            if not np.isfinite(z.stamp) or z.stamp>packet.time+1e-10:raise ValueError('future/nonfinite measurement forbidden')
            if not np.isfinite(z.position).all():raise ValueError('nonfinite pose')
            rotation(z.rotation);covariance(z.covariance,6)
        self.last_arrival=packet.time
        mode=bool(packet.contact or np.linalg.norm(packet.wrench_target_at_grasp_world[:3])>.1)
        self.history.append((packet.time,mode))
        while len(self.history)>1 and self.history[1][0]<packet.time-.25:self.history.popleft()
        for z in sorted(packet.poses,key=lambda z:z.stamp):
            reason='DUPLICATE_OR_OUT_OF_ORDER_REJECTED' if self.time is not None and z.stamp<=self.time else 'OUTSIDE_MODE_BUFFER' if z.stamp<packet.time-.25 else None
            if reason:
                self.rejects.append(dict(arrival=packet.time,stamp=z.stamp,reason=reason));continue
            if self.time is None:
                P=self.P.copy();P[:6,:6]=z.covariance
                self.s=replace(self.s,time=z.stamp,measurement_time=z.stamp,p=z.position,R=z.rotation,
                               full_covariance=P,covariance=P[:12,:12]);self.time=z.stamp;self.n=1;continue
            s=self.between(self.s,self.time,z.stamp);P=s.full_covariance
            innovation=np.r_[z.position-s.p,Rotation.from_matrix(z.rotation@s.R.T).as_rotvec()]
            S=P[:6,:6]+z.covariance;K=np.linalg.solve(S,P[:6,:]).T;dx=K@innovation
            A=np.eye(18);A[:,:6]-=K
            P=A@P@A.T+K@z.covariance@K.T
            G=np.eye(18);G[3:6,3:6]=left_jacobian(dx[3:6])
            P=G@P@G.T;P=(P+P.T)/2
            self.n+=1;self.time=z.stamp;self.innovation=innovation
            self.s=replace(s,measurement_time=z.stamp,p=s.p+dx[:3],R=Rotation.from_rotvec(dx[3:6]).as_matrix()@s.R,
                v=s.v+dx[6:9],w=s.w+dx[9:12],a=s.a+dx[12:15],alpha=s.alpha+dx[15:18],
                full_covariance=P,covariance=P[:12,:12],innovation=innovation)
            self.log.append(dict(stamp=z.stamp,arrival=packet.time,innovation=innovation.copy(),
                NIS=float(innovation@np.linalg.solve(S,innovation)),innovation_covariance=S.copy(),
                correction=dx.copy(),process_contact_at_stamp=s.process_contact))
        if self.time is None:
            out=replace(self.s,time=packet.time,valid=False,process_contact=mode)
        else:
            out=self.between(self.s,self.time,packet.time)
            out=replace(out,valid=self.n>=self.model['minimum_samples'] and packet.time-self.time<=self.cfg['stale_limit_s'],process_contact=mode)
        self.contact=mode;self.latest_snapshot=out
        self.update_times.append(time.perf_counter()-started)
        return out

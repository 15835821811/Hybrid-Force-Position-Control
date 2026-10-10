"""Independent standard-DH and rigid-body momentum calculations (no engine calls)."""
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from .common import source,assumptions,skew,wrap

def rz(t):
    c,s=np.cos(t),np.sin(t);return np.array([[c,-s,0],[s,c,0],[0,0,1.]])
def rx(t):
    c,s=np.cos(t),np.sin(t);return np.array([[1.,0,0],[0,c,-s],[0,s,c]])
class IndependentSRS:
    def __init__(self):
        s=source();self.mass=np.array(s['inertia']['mass_kg']);self.inertias=np.array(s['inertia']['diagonal_kg_m2'])
        self.d=np.array(s['dh']['d_m']);self.alpha=np.deg2rad(s['dh']['alpha_deg']);self.offset=np.deg2rad(s['dh']['theta_table_deg']);self.mount=rx(assumptions()['model']['mount_rotation_x_rad'])
    def fk(self,q):
        p=np.zeros(3);R=self.mount.copy();origins=[];axes=[];coms=[np.zeros(3)];rotations=[np.eye(3)];ends=[]
        for i in range(7):
            origins.append(p.copy());axes.append(R[:,2].copy());R=R@rz(q[i]+self.offset[i])
            coms.append(p+R@np.array([0,0,self.d[i]/2]));rotations.append(R.copy())
            p=p+R@np.array([0,0,self.d[i]]);R=R@rx(self.alpha[i]);ends.append(p.copy())
        return SimpleNamespace(p=p,R=R,origins=np.array(origins),axes=np.array(axes),coms=np.array(coms),rotations=np.array(rotations),ends=np.array(ends))
    def jac_point(self,point,last,f):
        J=np.zeros((3,7));J[:,:last]=np.cross(f.axes[:last],point-f.origins[:last]).T;return J
    def terms(self,qpos):
        qpos=np.asarray(qpos);b=qpos[:3];Rb=Rotation.from_quat(qpos[[4,5,6,3]]).as_matrix();f=self.fk(qpos[7:])
        M=np.zeros((13,13));mom=np.zeros((6,13));body_terms=[]
        for i in range(8):
            p=b+Rb@f.coms[i];R=Rb@f.rotations[i]
            Jv=np.zeros((3,13));Jw=np.zeros((3,13));Jv[:,:3]=np.eye(3);Jv[:,3:6]=-skew(p-b)@Rb;Jw[:,3:6]=Rb
            if i:Jv[:,6:]=Rb@self.jac_point(f.coms[i],i,f);Jw[:,6:6+i]=Rb@f.axes[:i].T
            I=R@np.diag(self.inertias[i])@R.T
            M+=self.mass[i]*Jv.T@Jv+Jw.T@I@Jw
            mom[:3]+=self.mass[i]*Jv;mom[3:]+=I@Jw+skew(p)@ (self.mass[i]*Jv)
            body_terms.append((p,R,Jv,Jw,I))
        A=-np.linalg.solve(M[:6,:6],M[:6,6:]);projection=np.vstack((A,np.eye(7)))
        p=b+Rb@f.p;R=Rb@f.R;J=np.zeros((6,13));J[:3,:3]=np.eye(3);J[:3,3:6]=-skew(p-b)@Rb;J[3:,3:6]=Rb
        J[:3,6:]=Rb@self.jac_point(f.p,7,f);J[3:,6:]=Rb@f.axes.T
        return SimpleNamespace(p=p,R=R,J=J,Jg=J@projection,A=A,P=projection,M=M,momentum_matrix=mom,bodies=body_terms,base_rotation=Rb,fk=f)
    def shape(self,q):
        f=self.fk(q);S=f.origins[1];E=f.origins[3];W=f.origins[5]
        JS=self.jac_point(S,1,f);JE=self.jac_point(E,3,f);JW=self.jac_point(W,5,f)
        return shape_from_points(S,E,W,JE-JS,JW-JS,self.mount[:,2])

def shape_from_points(S,E,W,De,Dw,V=None):
    V=np.array([0.,0.,1.]) if V is None else np.asarray(V);e=E-S;w=W-S;L=np.linalg.norm(w)
    if L<1e-8:raise ValueError('SHAPE_SHOULDER_WRIST_SINGULAR')
    u=w/L;projection=e-u*(u@e);x=V@projection;y=u@np.cross(V,projection)
    if x*x+y*y<1e-14:raise ValueError('SHAPE_REFERENCE_OR_ELBOW_SINGULAR')
    Du=(np.eye(3)-np.outer(u,u))@Dw/L
    Dp=De-Du*(u@e)-np.outer(u,e@Du+u@De)
    Dx=V@Dp;Dy=np.cross(V,projection)@Du+u@skew(V)@Dp
    jac=(x*Dy-y*Dx)/(x*x+y*y)
    # l is not explicitly defined in the source text. Retain both readings.
    lnormal=np.cross(w,V);l=np.cross(lnormal,u);pn=np.linalg.norm(projection);ln=np.linalg.norm(l)
    first=np.cross(u,projection/pn)
    printed=first@De/pn+((V@u)/ln*np.cross(u,l/ln)-(u@e)/(L*pn)*first)@Dw if ln>1e-10 else np.full(7,np.nan)
    normal=first@De/pn+((V@u)/ln*np.cross(u,lnormal/ln)-(u@e)/(L*pn)*first)@Dw if ln>1e-10 else np.full(7,np.nan)
    return SimpleNamespace(angle=float(np.arctan2(y,x)),J=jac,printed_eq23=printed,eq23_normal_reading=normal,S=S,E=E,W=W,
                           length=L,projection_norm=pn,reference_projection_norm=ln/L)

"""Momentum about fixed world O. pi order is explicit and tested independently."""
import numpy as np

def tensor(pi):
    return np.array([[pi[4],pi[7],pi[8]],[pi[7],pi[5],pi[9]],[pi[8],pi[9],pi[6]]])
def parameters(m,c,Ic):
    c=np.asarray(c);I=np.asarray(Ic)+m*((c@c)*np.eye(3)-np.outer(c,c))
    return np.r_[m,m*c,I[0,0],I[1,1],I[2,2],I[0,1],I[0,2],I[1,2]]
def pseudoinertia(pi):
    I=tensor(pi);P=np.zeros((4,4));P[:3,:3]=.5*np.trace(I)*np.eye(3)-I;P[:3,3]=P[3,:3]=pi[1:4];P[3,3]=pi[0]
    return P
def regressor(p,R,v,w,O=np.zeros(3)):
    Y=np.zeros((6,10))
    for k in range(10):
        pi=np.eye(10)[k];rh=R@pi[1:4];P=pi[0]*v+np.cross(w,rh)
        Y[:,k]=np.r_[P,np.cross(p-O,P)+np.cross(rh,v)+R@tensor(pi)@R.T@w]
    return Y
def physical(pi,prior):
    if pi[0]<=0:return False
    S=np.diag([1/.15]*3+[1.]);P=S@pseudoinertia(pi)@S/20
    return bool(np.min(np.linalg.eigvalsh(P))>0 and prior['mass_bounds_kg'][0]-1e-8<=pi[0]<=prior['mass_bounds_kg'][1]+1e-8 and np.max(abs(pi[1:4]/pi[0]))<=prior['com_bound_m']+1e-9 and np.all(np.diag(pseudoinertia(pi))[:3]<=pi[0]*prior['support_half_extent_m']**2+1e-9))

"""Analytic C2 scalar jets, normalized-time polynomials and endpoint bumps."""
from dataclasses import dataclass
import numpy as np

@dataclass
class JetSegment:
    start: float
    duration: float
    coefficients: np.ndarray

    @classmethod
    def connect(cls,start,duration,initial,terminal,bump=0.):
        if duration<=0 or not np.isfinite(duration):raise ValueError('positive duration')
        p,v,a=np.asarray(initial,float);q,w,b=np.asarray(terminal,float);T=duration
        c0=p;c1=v*T;c2=a*T*T/2
        A=q-c0-c1-c2;B=w*T-c1-2*c2;C=b*T*T-2*c2
        c=np.array([c0,c1,c2,10*A-4*B+C/2,-15*A+7*B-C,6*A-3*B+C/2,0.])
        c+=float(bump)*np.array([0,0,0,64,-192,192,-64])
        return cls(float(start),float(T),c)

    def jet(self,t):
        x=np.clip((t-self.start)/self.duration,0.,1.)
        return np.array([np.polynomial.polynomial.polyval(x,np.polynomial.polynomial.polyder(self.coefficients,k))/self.duration**k for k in range(3)])

    def record(self):return dict(start_s=self.start,duration_s=self.duration,coefficients=self.coefficients.tolist())

    @classmethod
    def restore(cls,r):return cls(r['start_s'],r['duration_s'],np.asarray(r['coefficients']))

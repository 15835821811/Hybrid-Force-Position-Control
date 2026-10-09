"""Causal non-overlapping momentum blocks; data SVD before physical constraints."""
import numpy as np
from scipy.optimize import minimize
from .momentum_regressor import parameters,pseudoinertia,physical,regressor,state_jacobian

class InertialEstimator:
    def __init__(self,prior,cfg):
        self.prior=prior;self.cfg=cfg;self.pi0=parameters(prior['mass_kg'],prior['com_m'],prior['inertia_com_kg_m2']);self.pi=self.pi0.copy()
        self.scales=np.asarray(prior['parameter_scales']);self.cov=np.diag(prior['prior_covariance_scaled_diagonal']);self.blocks=[];self.anchor=None;self.last_fit=-1.;self.rank=0
        self.status='UNOBSERVABLE';self.reason='no informative blocks';self.singular=np.zeros(10);self.V=np.eye(10);self.accepted=False;self.predict=[];self.cross_checks=[]
        self.absolute_excitation=False
    def add(self,estimate,robot_momentum,impulse_at_O):
        if not estimate.valid:return
        Y=regressor(estimate.p,estimate.R,estimate.v,estimate.w)
        G=state_jacobian(estimate.p,estimate.R,estimate.v,estimate.w,self.pi0)
        # Inflation covers the prior mass range for this local linearization;
        # this is an approximate EIV covariance, not a certified probability law.
        C=4*G@estimate.covariance@G.T
        if self.anchor is None:self.anchor=(estimate.time,Y,robot_momentum.copy(),impulse_at_O.copy(),C);return
        t,A,h,J,Ca=self.anchor
        if estimate.time-t<self.cfg['identification_block_s']-1e-8:return
        DY=Y-A;dh=-(robot_momentum-h);dJ=impulse_at_O-J
        # Prediction is evaluated BEFORE ingesting this new block.
        scale=np.asarray(self.prior['momentum_noise_floor'])
        covariance=C+Ca+np.diag(scale**2);L=np.linalg.cholesky(covariance)
        if np.linalg.norm(np.linalg.solve(L,dJ))>5 and np.linalg.norm(np.linalg.solve(L,dh))>5:self.absolute_excitation=True
        if self.blocks:self.predict.append({'time':estimate.time,'posterior':float(np.linalg.norm(np.linalg.solve(L,DY@self.pi-dh))),'prior':float(np.linalg.norm(np.linalg.solve(L,DY@self.pi0-dh))),'rank':self.rank})
        self.cross_checks.append({'time':estimate.time,'robot_vs_interface':(dh-dJ).tolist()})
        self.blocks.append((DY,dh,L));self.anchor=(estimate.time,Y,robot_momentum.copy(),impulse_at_O.copy(),C)
        if estimate.time-self.last_fit>=self.cfg['identification_update_s']-1e-9:self.fit();self.last_fit=estimate.time
    def fit(self):
        A=np.vstack([np.linalg.solve(L,a) for a,b,L in self.blocks]);b=np.concatenate([np.linalg.solve(L,b) for a,b,L in self.blocks]);self.solve(A,b,self.absolute_excitation)
    def solve(self,A,b,absolute_excitation=True):
        As=A*self.scales
        # Free-motion data is homogeneous: never update the global scale from
        # numerical noise. Remove that direction before the DATA SVD.
        if not absolute_excitation:
            direction=self.pi0/self.scales;direction/=np.linalg.norm(direction)
            As=As@(np.eye(10)-np.outer(direction,direction))
        U,s,Vh=np.linalg.svd(As,full_matrices=False)
        self.singular=np.pad(s,(0,max(0,10-len(s))))[:10];self.V=Vh
        threshold=max(self.cfg['information_singular_min'],(s[0] if len(s) else 0)*self.cfg['information_relative_min'])
        self.rank=int(np.count_nonzero(s>threshold));self.accepted=False
        if self.rank==0:self.reason='data rank zero; prior retained';return
        V=Vh[:self.rank].T;B=As@V;rhs=b-A@self.pi0
        z=np.linalg.lstsq(B,rhs,rcond=None)[0]
        def pi_of(x):return self.pi0+self.scales*(V@x)
        def constraints(x):
            pi=pi_of(x);P=pseudoinertia(pi);S=np.diag([1/.15]*3+[1.]);m=pi[0];cb=self.prior['com_bound_m'];lo,hi=self.prior['mass_bounds_kg']
            return np.r_[np.linalg.eigvalsh(S@P@S/20)-1e-8,(m-lo)/20,(hi-m)/20,(m*cb-pi[1:4])/3,(m*cb+pi[1:4])/3,(m*.15**2-np.diag(P)[:3])/.45]
        normal=max(1.,np.linalg.norm(B)**2)
        result=minimize(lambda x:.5*np.sum((B@x-rhs)**2)/normal,z if np.all(constraints(z)>=0) else np.zeros(self.rank),jac=lambda x:B.T@(B@x-rhs)/normal,
            constraints={'type':'ineq','fun':constraints},method='SLSQP',options={'ftol':1e-12,'maxiter':150})
        candidate=pi_of(result.x)
        if not result.success or not physical(candidate,self.prior):self.reason='physical constrained fit rejected: '+result.message;return
        self.pi=candidate;self.accepted=True;self.reason='data-observable directions updated; nullspace prior retained'
        C0=np.diag(self.prior['prior_covariance_scaled_diagonal']);N=np.eye(10)-V@V.T
        self.cov=N@C0@N.T+V@np.diag(1/s[:self.rank]**2)@V.T
        self.status='IDENTIFIED_IN_TESTED_SUBSPACE' if self.rank==10 else 'PARTIALLY_IDENTIFIED'
        # FULL_IDENTIFICATION_VALIDATED requires external heldout truth + prediction evidence.
    def snapshot(self):
        return {'pi':self.pi.tolist(),'rank':self.rank,'status':self.status,'accepted':self.accepted,'reason':self.reason,'singular_values':self.singular.tolist(),'right_singular_vectors_scaled':self.V.tolist(),'covariance_scaled':self.cov.tolist(),'physical':physical(self.pi,self.prior),'blocks':len(self.blocks),'absolute_excitation_detected':self.absolute_excitation,'covariance_scope':'local EIV approximation; correlated estimator blocks; not calibrated confidence'}

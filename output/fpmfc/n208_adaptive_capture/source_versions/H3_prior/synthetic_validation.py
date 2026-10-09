"""Independent Newton-Euler ODE with known external wrench: identification only.

This synthetic object is not the robot task plant and provides no capture claim.
"""
import copy
import json
import time
import numpy as np
from scipy.integrate import solve_ivp
from scipy.spatial.transform import Rotation
from .common import ROOT,config,save,init_ledger
from .momentum_regressor import parameters,regressor,physical
from .inertial_estimator import InertialEstimator

def run():
    started=time.process_time();m=25.;c=np.array([.012,-.01,.005]);R0=Rotation.from_rotvec([.3,-.2,.4]).as_matrix();Ic=R0@np.diag([.2,.31,.38])@R0.T
    def rhs(t,y):
        q=y[6:10]/np.linalg.norm(y[6:10]);R=Rotation.from_quat(q).as_matrix();w=y[10:13]
        F=np.array([.7*np.sin(1.2*t)+.2,.6*np.cos(.9*t),.5*np.sin(1.7*t+.3)])
        M=np.array([.08*np.cos(1.3*t),.06*np.sin(.7*t+.4),.07*np.cos(1.6*t+.2)])
        dq=.5*np.r_[q[3]*w+np.cross(q[:3],w),-q[:3]@w]
        dw=np.linalg.solve(Ic,R.T@M-np.cross(w,Ic@w))
        return np.r_[y[3:6],F/m,dq,dw,F,M+np.cross(y[:3],F)]
    y0=np.r_[[.8,.2,.9],[.01,-.02,.03],Rotation.from_rotvec([.2,.1,-.3]).as_quat(),[.08,-.06,.1],np.zeros(6)]
    sol=solve_ivp(rhs,[0,12],y0,t_eval=np.linspace(0,12,121),rtol=1e-11,atol=1e-13);assert sol.success
    Ys=[]
    for y in sol.y.T:
        R=Rotation.from_quat(y[6:10]).as_matrix();w=R@y[10:13];p=y[:3]-R@c;v=y[3:6]-np.cross(w,R@c);Ys.append(regressor(p,R,v,w))
    DY=np.diff(Ys,axis=0);dJ=np.diff(sol.y[13:19].T,axis=0);truth=parameters(m,c,Ic)
    residual=np.einsum('nij,j->ni',DY,truth)-dJ;assert np.max(abs(residual))<1e-8
    split=70;results=[];prior=config('target_prior')
    for mass_prior in [15.,32.]:
        pr=copy.deepcopy(prior);pr['mass_kg']=mass_prior;est=InertialEstimator(pr,config());est.solve(DY[:split].reshape(-1,10)/1e-4,dJ[:split].reshape(-1)/1e-4)
        prediction=np.einsum('nij,j->ni',DY[split:],est.pi)-dJ[split:]
        assert est.accepted and est.rank==10 and physical(est.pi,prior) and np.linalg.norm(est.pi-truth)<1e-5 and np.max(abs(prediction))<1e-8
        results.append({'mass_prior':mass_prior,'fit':est.snapshot(),'parameter_error_norm':float(np.linalg.norm(est.pi-truth)),'future_prediction_max_error':float(np.max(abs(prediction)))})
    bad=InertialEstimator(prior,config());bad.solve(np.eye(10)*100,np.r_[-5.,np.zeros(3),.3,.3,.3,np.zeros(3)]*100)
    assert bad.accepted and physical(bad.pi,prior)
    result={'passed':True,'scope':'independent synthetic forced Newton-Euler ODE, no robot/target actuator addition','duration_s':12.,'ode_function_evaluations':sol.nfev,'train_blocks':split,'future_validation_blocks':len(DY)-split,'fixed_origin_integral_residual':float(np.max(abs(residual))),'prior_comparison':results,'nonphysical_unconstrained_data_fit_remains_physical':bad.snapshot(),'cpu_s':time.process_time()-started}
    path=ROOT/'synthetic_newton_euler_validation.json';save(path,result);ledger=init_ledger();ledger['cpu_s']+=result['cpu_s'];ledger['isolated_tests'].append({'path':path.name,'passed':True,'cpu_s':result['cpu_s'],'robot_physics_steps':0});save(ROOT/'run_ledger.json',ledger)
    print(json.dumps({'passed':True,'max_residual':result['fixed_origin_integral_residual'],'prediction_errors':[r['future_prediction_max_error'] for r in results]}),flush=True)
    return result

if __name__=='__main__':run()

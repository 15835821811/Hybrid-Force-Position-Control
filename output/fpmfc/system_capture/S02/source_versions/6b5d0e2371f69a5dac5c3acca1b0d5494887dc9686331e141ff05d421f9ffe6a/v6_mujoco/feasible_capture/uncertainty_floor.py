"""Independent Riccati necessary-condition audit; no robot or state feedback."""
import numpy as np
from scipy.linalg import solve_discrete_are
from .benchmark import mission
from v6_mujoco.adaptive_capture.common import config
from .common import ROOT,PROJECT_ROOT,save,sha

def run():
    cfg=mission('noisy');sensor=config('sensors')['noisy']
    h=sensor['pose_period_s'];delay=sensor['delay_s'];q=cfg['process_linear_accel'];R=sensor['position_sigma_m']**2;H=np.array([[1.,0.]])
    F=np.array([[1.,h],[0.,1.]])
    Q=q*q*np.array([[h**3/3,h*h/2],[h*h/2,h]])
    prior=solve_discrete_are(F.T,H.T,Q,np.array([[R]]))
    def update(P):
        K=P@H.T/((H@P@H.T).item()+R);A=np.eye(2)-K@H
        return A@P@A.T+R*(K@K.T)
    post=update(prior);P0=np.diag([R,.1]);P=P0.copy()
    for _ in range(10000):P=update(F@P@F.T+Q)
    residual=float(np.max(abs(post-update(F@post@F.T+Q))));independent_error=float(np.max(abs(P-post)))
    initial_order=np.linalg.eigvalsh(P0-post)
    lower=cfg['margin_sigma']*np.sqrt(post[1,1]+q*q*delay)
    assert residual<1e-15 and independent_error<1e-14 and initial_order.min()>0
    assert lower>cfg['capture']['linear_m_s']+1e-5
    result={'status':'CONDITIONAL_IMPLEMENTED_GUARD_INCOMPATIBILITY','sample_period_s':h,'minimum_delay_s':delay,'position_measurement_variance_m2':R,
            'linear_acceleration_density_m_s32':q,'process_multiplier_minimum':1.,'scalar_posterior_covariance':post,
            'initial_covariance_minus_fixed_point_eigenvalues':initial_order,'riccati_fixed_point_residual':residual,'iterated_vs_DARE_error':independent_error,
            'minimum_velocity_3sigma_lower_bound_m_s':float(lower),'capture_linear_limit_m_s':cfg['capture']['linear_m_s'],
            'assumptions':['three independent translation CV blocks, position-only observation with fixed stated covariance',
                'sample spacing6ms; control-time measurement age at least12ms; no additional observations',
                'each interval process density at least.003; covariance initialized as diag(R,.1) after first pose',
                'exact covariance recursion; rounding residual checked independently; no manual covariance shrink'],
            'scope':'lower bound on the implemented guard allowance, NOT an actual estimation-error or sensor impossibility bound; contact can only increase translation covariance in this model',
            'sensor_configuration':sensor,'implementation_sha':{p:sha(PROJECT_ROOT/p) for p in ['v6_mujoco/feasible_capture/uncertainty_floor.py','v6_mujoco/adaptive_capture/state_estimator.py','v6_mujoco/feasible_capture/estimator_adapter.py','configs/adaptive_capture/sensors.yaml','configs/n209_paper_system.yaml']}}
    save(ROOT/'uncertainty_floor.json',result);print('minimum implemented linear3sigma',lower);return result

if __name__=='__main__':run()

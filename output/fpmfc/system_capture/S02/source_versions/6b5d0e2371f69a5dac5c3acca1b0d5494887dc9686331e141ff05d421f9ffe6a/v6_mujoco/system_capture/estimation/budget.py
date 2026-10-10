"""Diagnostic interface covariance transport. Does not replace capture guard."""
import numpy as np
from v6_mujoco.adaptive_capture.contracts import skew

def interface_jacobian(R,w,r_local):
    r=R@np.asarray(r_local);J=np.eye(12)
    J[:3,3:6]=-skew(r)
    J[6:9,3:6]=-skew(w)@skew(r)
    J[6:9,9:12]=-skew(r)
    return J

def relative_covariance(target,offset_target,robot_R=None,robot_w=None,offset_tool=None,
                        robot_P=None,cross_P=None,calibration_P=None,time_derivative=None,time_sigma=0.):
    Jg=interface_jacobian(target.R,target.w,offset_target)
    Je=interface_jacobian(np.eye(3) if robot_R is None else robot_R,
        np.zeros(3) if robot_w is None else robot_w,np.zeros(3) if offset_tool is None else offset_tool)
    Pr=np.zeros((12,12)) if robot_P is None else np.asarray(robot_P)
    C=np.zeros((12,12)) if cross_P is None else np.asarray(cross_P)
    P=Jg@target.covariance@Jg.T+Je@Pr@Je.T-Je@C@Jg.T-Jg@C.T@Je.T
    if calibration_P is not None:P+=calibration_P
    if time_derivative is not None:P+=time_sigma**2*np.outer(time_derivative,time_derivative)
    return (P+P.T)/2

def guard_margins(e,cfg,offset):
    """Exact legacy formula, including its omissions; never substituted online."""
    sig=cfg['margin_sigma'];P=e.covariance;r=np.linalg.norm(offset)
    s=[np.sqrt(max(0.,np.linalg.eigvalsh(P[k:k+3,k:k+3])[-1])) for k in (0,3,6,9)]
    out=sig*np.array([s[0]+r*s[1],s[1],s[2]+r*s[3],s[3]])
    out[:2]+=[cfg.get('position_bias_bound_m',0.)+r*cfg.get('rotation_bias_bound_rad',0.),cfg.get('rotation_bias_bound_rad',0.)]
    return out

def limits(cfg):
    c=cfg['capture']
    return np.array([c['translation_m'],np.deg2rad(c['rotation_deg']),c['linear_m_s'],np.deg2rad(c['angular_deg_s'])])

def diagnostic(e,cfg,offset,actual_relative=None):
    P=relative_covariance(e,offset);m=guard_margins(e,cfg,offset)
    # Norm-bounded pose biases at B and rotation; velocity transport sensitivity
    # to orientation bias is separately disclosed, although legacy guard omits it.
    full=3*np.array([np.sqrt(max(0.,np.linalg.eigvalsh(P[k:k+3,k:k+3])[-1])) for k in (0,3,6,9)])
    bias=np.array([cfg.get('position_bias_bound_m',0.)+np.linalg.norm(offset)*cfg.get('rotation_bias_bound_rad',0.),
        cfg.get('rotation_bias_bound_rad',0.),np.linalg.norm(e.w)*np.linalg.norm(offset)*cfg.get('rotation_bias_bound_rad',0.),0.])
    return dict(legacy_margin=m,legacy_remaining_budget=limits(cfg)-m,
        complete_geometry_margin=full+bias,complete_geometry_remaining_budget=limits(cfg)-full-bias,
        relative_covariance=P,actual_relative=actual_relative,
        robot_navigation_encoder='SIMULATION_ASSUMPTION: exact; zero robot/target cross covariance',
        clock_and_calibration='SIMULATION_ASSUMPTION: exact timestamps and fixed mounts; zero additional uncertainty',
        scope='counterfactual necessary information budget; not actual guard invocation or joint 99.7% guarantee')

"""Source nullspace reading and a thin specialization of the real project HQP."""
from types import SimpleNamespace
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from ...fpmfc.controller import FPMFCHQP,FPMFCControllerConfig
from ...hierarchical_qp import QPConfig
from ...fpmfc.shape import ArmShapeSample
from ..contracts import ControlProposal,NamedConstraint
from .common import assumptions,wrap
from .model import SRSSpec
from .kinematics import IndependentSRS,shape_from_points,rx

def source_velocity(terms,shape,ref,literal=False,feedback=True):
    cfg=assumptions()['controller']
    e=np.r_[ref.T[:3,3]-terms.p,Rotation.from_matrix(ref.T[:3,:3]@terms.R.T).as_rotvec()]
    demand=np.r_[ref.v,ref.w]+(cfg['pose_gain_s_inv']*e if feedback else 0)
    J=terms.Jg;U,s,Vh=np.linalg.svd(J,full_matrices=True)
    if s[-1]<cfg['numerical_rank_tolerance']:raise ValueError('GENERALIZED_JACOBIAN_SINGULAR')
    primary=Vh[:6].T@((U.T@demand)/s);n=Vh[-1]
    slope=shape.J@n
    if abs(slope)<cfg['numerical_rank_tolerance']:raise ValueError('SHAPE_NULLSPACE_SINGULAR')
    shape_command=ref.psidot+(cfg['shape_gain_s_inv']*float(wrap(ref.psi-shape.angle)) if feedback else 0)
    scalar=(shape_command-(0 if literal else shape.J@primary))/slope
    velocity=primary+n*scalar
    return SimpleNamespace(velocity=velocity,primary=primary,primary_residual=J@velocity-demand,
        shape_residual=float(shape.J@velocity-shape_command),null=n,rank_values=s,
        primary_null_residual=float(np.linalg.norm(J@n)),base_null_velocity=terms.A@n)

class EngineShape:
    def __init__(self,m):
        self.m=m;self.base=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'srs_base')
        self.joints=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,'srs_joint'+str(i)) for i in (2,4,6)]
        self.V=rx(assumptions()['model']['mount_rotation_x_rad'])[:,2]
    def full(self,d):
        R=d.xmat[self.base].reshape(3,3);p=d.xpos[self.base];points=[];jac=[]
        for joint in self.joints:
            point=d.xanchor[joint];J=np.zeros((3,self.m.nv))
            mujoco.mj_jac(self.m,d,J,None,point,int(self.m.jnt_bodyid[joint]))
            points.append(R.T@(point-p));jac.append(R.T@J[:,6:])
        return shape_from_points(*points,jac[1]-jac[0],jac[2]-jac[0],self.V)
    def sample(self,d):
        s=self.full(d)
        return ArmShapeSample(s.angle,s.S,s.E,s.W,s.length,s.projection_norm,s.reference_projection_norm,False)
    def jacobian(self,d):return self.full(d).J

class SRSHQP(FPMFCHQP):
    """Unmodified project two-level objectives/solver with SRS-only bounds."""
    def __init__(self,m):
        a=assumptions()['controller'];self.srs=a
        cfg=FPMFCControllerConfig(position_gain=a['pose_gain_s_inv'],orientation_gain=a['pose_gain_s_inv'],shape_gain=a['shape_gain_s_inv'],
            position_weight=a['hqp_position_weight'],orientation_weight=a['hqp_orientation_weight'],shape_weight=a['hqp_shape_weight'],base_reaction_weight=a['hqp_base_weight'],
            smoothness_weight=a['hqp_smoothness_weight'],primary_regularization=a['hqp_primary_regularization'],level1_position_tolerance_m_s=a['hqp_lock_tolerance'],level1_angular_tolerance_rad_s=a['hqp_lock_tolerance'],
            linear_speed_limit_m_s=10.,angular_speed_limit_rad_s=10.)
        qc=QPConfig(qp_tolerance=1e-10,feasibility_tolerance=1e-9,qp_max_iterations=3000,admm_rho=.01,admm_sigma=1e-12)
        super().__init__(SRSSpec(),m,(),EngineShape(m),controller_config=cfg,constraint_config=qc)
    def _bounds(self,q):
        a=self.srs;h=a['task_period_s'];v=a['joint_velocity_limit_rad_s'];acc=a['joint_acceleration_limit_rad_s2'];lim=a['joint_position_limit_rad']
        low=np.maximum.reduce([np.full(7,-v),self.previous_velocity-acc*h,(-lim-q)/h])
        high=np.minimum.reduce([np.full(7,v),self.previous_velocity+acc*h,(lim-q)/h])
        self.last_velocity_lower=low;self.last_velocity_upper=high
        return low,high
    def _clearance_constraints(self,d,mapping):return np.zeros((0,7)),np.zeros(0),float('inf')
    def command(self,d,ref):
        return self.solve_fpmfc(d,target_position=ref.T[:3,3],target_velocity=ref.v,target_rotation=ref.T[:3,:3],target_angular_velocity=ref.w,target_arm_angle_rad=ref.psi,target_arm_angle_velocity_rad_s=ref.psidot)

def servo_torque(m,d,qref,dqref,ddqref,statuses):
    a=assumptions()['controller'];M=np.zeros((m.nv,m.nv));mujoco.mj_fullM(m,M,d.qM)
    S=M[6:,6:]-M[6:,:6]@np.linalg.solve(M[:6,:6],M[:6,6:]);bias=d.qfrc_bias[6:]-M[6:,:6]@np.linalg.solve(M[:6,:6],d.qfrc_bias[:6])
    wn=a['servo_natural_frequency_rad_s'];z=a['servo_damping_ratio']
    torque=S@(ddqref+wn**2*(qref-d.qpos[7:])+2*z*wn*(dqref-d.qvel[6:]))+bias
    maximum=float(np.max(abs(torque)));valid=maximum<=a['torque_limit_nm'] and np.isfinite(torque).all()
    constraints=(NamedConstraint('joint_torque','task_safety','N m',1.,'declared SRS torque servo limit',maximum,a['torque_limit_nm']),)
    return ControlProposal(float(d.time),torque,'PRECONTACT',statuses,constraints,False,'NONE' if valid else 'SIM_ABORT','CONTROL' if valid else 'TORQUE_LIMIT',bool(valid))

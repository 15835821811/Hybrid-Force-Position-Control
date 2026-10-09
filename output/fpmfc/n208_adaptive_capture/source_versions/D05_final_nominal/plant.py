"""Private truth and initialization. No online controller imports this module."""
from dataclasses import dataclass
import copy
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from v6_mujoco.geometry_capture.design import compile_model
from v6_mujoco.end_to_end_capture.adapter import initialize, latch_id, pads
from v6_mujoco.model import body_id
from v6_mujoco.postgrasp.physics import joint_slices

@dataclass(frozen=True)
class TruthConfig:
    mass: float
    com: tuple
    inertia: tuple
    omega_body: tuple
    contact_scale: float=1.

def nominal_truth():
    # Initial angular velocity obtained from the historical nominal coordinate convention.
    m=compile_model();d=initialize(m);_,tv=joint_slices(m,'target_free_joint')
    return TruthConfig(20.,(0.,0.,0.),tuple(map(tuple,.3*np.eye(3))),tuple(d.qvel[tv][3:]))

def supported_truth(mass,center,half_lengths,rotation_vector,omega_body,contact_scale=1.):
    """Uniform rotated interior box: positive distribution with explicit support."""
    c=np.asarray(center);a=np.asarray(half_lengths);R=Rotation.from_rotvec(rotation_vector).as_matrix()
    assert np.all(abs(c)+abs(R)@a<=.15)
    covariance=R@np.diag(a*a/3)@R.T
    I=mass*(np.trace(covariance)*np.eye(3)-covariance)
    return TruthConfig(float(mass),tuple(c),tuple(map(tuple,I)),tuple(omega_body),contact_scale)

class Plant:
    def __init__(self,truth,dt=.002,solver_tolerance=None):
        self.truth=truth;self.model=compile_model();m=self.model;b=body_id(m,'tumbling_target')
        vals,R=np.linalg.eigh(np.asarray(truth.inertia));assert min(vals)>0 and max(vals)<sum(vals)-max(vals)
        if np.linalg.det(R)<0:R[:,0]*=-1
        quat=Rotation.from_matrix(R).as_quat()
        m.body_mass[b]=truth.mass;m.body_ipos[b]=truth.com;m.body_inertia[b]=vals;m.body_iquat[b]=np.r_[quat[3],quat[:3]]
        m.opt.timestep=dt
        if solver_tolerance is not None:m.opt.tolerance=solver_tolerance
        m.geom_solref[pads(m),0]*=truth.contact_scale
        mujoco.mj_setConst(m,mujoco.MjData(m))
        self.data=initialize(m);_,tv=joint_slices(m,'target_free_joint');self.data.qvel[tv][3:]=truth.omega_body
        mujoco.mj_forward(m,self.data)
        assert m.nu==7 and all(m.actuator_trnid[:,0]!=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,'target_free_joint'))
        self.fixed={k:getattr(m,k).copy() for k in ('body_mass','body_inertia','body_ipos','body_iquat','site_pos','site_quat','eq_solref','eq_solimp')}
    def latch(self):
        m,d=self.model,self.data;q=d.qpos.copy();v=d.qvel.copy()
        d.eq_active[latch_id(m)]=True;m.geom_contype[pads(m)]=0;m.geom_conaffinity[pads(m)]=0
        assert np.array_equal(q,d.qpos) and np.array_equal(v,d.qvel)
    def step(self,torque):
        self.data.ctrl[:]=torque
        assert not np.any(self.data.xfrc_applied) and not np.any(self.data.qfrc_applied)
        mujoco.mj_step(self.model,self.data)
    def assert_frozen(self):
        assert all(np.array_equal(getattr(self.model,k),v) for k,v in self.fixed.items())

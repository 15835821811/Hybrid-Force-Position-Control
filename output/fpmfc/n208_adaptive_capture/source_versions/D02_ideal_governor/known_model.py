"""Compile a NEW control model from known XML plus TargetPrior, never plant copy."""
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from v6_mujoco.model import PROJECT_ROOT, default_model_spec, body_id
from v6_mujoco.geometry_capture.design import from_xml
from v6_mujoco.end_to_end_capture.adapter import initialize
from v6_mujoco.postgrasp.physics import joint_slices, body_jacobian

def compile_known(prior):
    root=ET.fromstring((PROJECT_ROOT/'models/flexiv_rizon4s_n206_tool_scene.xml').read_text(encoding='utf-8'))
    inertial=root.find('.//body[@name="tumbling_target"]/inertial')
    inertial.attrib.clear()
    I=np.asarray(prior['inertia_com_kg_m2'])
    inertial.attrib.update(mass=str(prior['mass_kg']),pos=' '.join(map(str,prior['com_m'])), fullinertia=' '.join(map(str,[I[0,0],I[1,1],I[2,2],I[0,1],I[0,2],I[1,2]])))
    m=from_xml(ET.tostring(root,encoding='unicode'))
    m.geom_contype[:]=0; m.geom_conaffinity[:]=0
    d=initialize(m); d.eq_active[:]=False
    return m,d

def set_measured(m,d,packet,estimate):
    spec=default_model_spec();bq,bv=spec.base_slices(m);qj,vj=spec.joint_addresses(m)
    d.qpos[bq]=packet.base_pose;d.qvel[bv]=packet.base_velocity
    d.qpos[qj]=packet.joint_position;d.qvel[vj]=packet.joint_velocity
    tq,tv=joint_slices(m,'target_free_joint')
    quat=Rotation.from_matrix(estimate.R).as_quat()
    d.qpos[tq]=np.r_[estimate.p,quat[3],quat[:3]]
    d.qvel[tv]=np.r_[estimate.v,estimate.R.T@estimate.w]
    d.time=packet.time;d.eq_active[:]=False;d.ctrl[:]=packet.actuator_torque
    mujoco.mj_forward(m,d)

def robot_momentum(m,d,O=np.zeros(3)):
    """Known bodies only; fixed world axes and fixed world origin O."""
    h=np.zeros(6);target=body_id(m,'tumbling_target')
    for b in range(1,m.nbody):
        if b==target or m.body_mocapid[b]>=0 or not m.body_mass[b]:continue
        v=body_jacobian(m,d,b,d.xipos[b])@d.qvel
        p=m.body_mass[b]*v[:3];R=d.ximat[b].reshape(3,3)
        h[:3]+=p;h[3:]+=R@np.diag(m.body_inertia[b])@R.T@v[3:]+np.cross(d.xipos[b]-O,p)
    return h

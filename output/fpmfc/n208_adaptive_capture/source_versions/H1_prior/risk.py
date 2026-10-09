"""Independent-model collision energy proxy; does not predict contact peak force."""
import numpy as np
import mujoco
from v6_mujoco.model import site_id
from v6_mujoco.postgrasp.physics import body_jacobian

def impact_proxy(m,d):
    tool=site_id(m,'postgrasp_tool_interface');grasp=site_id(m,'target_grasp_site')
    Je=body_jacobian(m,d,int(m.site_bodyid[tool]),d.site_xpos[tool]);Jg=body_jacobian(m,d,int(m.site_bodyid[grasp]),d.site_xpos[grasp]);J=Je[:3]-Jg[:3];n=d.site_xmat[grasp].reshape(3,3)[:,2]
    M=np.zeros((m.nv,m.nv));mujoco.mj_fullM(m,M,d.qM);j=n@J
    mass=1/float(j@np.linalg.solve(M,j));speed=float(j@d.qvel)
    je=n@Je[:3];robot_upper=1/float(je@np.linalg.solve(M,je))
    return {'effective_mass_kg':mass,'robot_only_upper_mass_kg':robot_upper,'normal_relative_speed_m_s':speed,'normal_energy_j':.5*mass*speed*speed,'upper_energy_j':.5*robot_upper*speed*speed,'scope':'prelatch independent free target adds nonnegative inverse effective mass; robot-only upper bound. Local energy proxy, not peak force'}

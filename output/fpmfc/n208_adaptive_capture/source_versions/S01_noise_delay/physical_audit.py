"""Evaluation-only physical wrench, power and Newton-Euler residuals."""
import copy
import numpy as np
import mujoco
from v6_mujoco.model import site_id,body_id,default_model_spec
from v6_mujoco.postgrasp.physics import joint_slices,body_jacobian

def inspect(m,d):
    g=site_id(m,'target_grasp_site');e=site_id(m,'postgrasp_tool_interface');b=body_id(m,'tumbling_target');point=d.site_xpos[g]
    Jt=body_jacobian(m,d,b,point);Jr=body_jacobian(m,d,int(m.site_bodyid[e]),point)
    _,tv=joint_slices(m,'target_free_joint');_,bv=default_model_spec().base_slices(m)
    wt=np.linalg.solve(Jt[:,tv].T,d.qfrc_constraint[tv]);wr=np.linalg.solve(Jr[:,bv].T,d.qfrc_constraint[bv])
    reconstructed=Jt.T@wt+Jr.T@wr;power=wt@(Jt@d.qvel)+wr@(Jr@d.qvel)
    Jc=body_jacobian(m,d,b,d.xipos[b]);eps=1e-6;js=[]
    for sign in [-1,1]:
        scratch=copy.copy(d);mujoco.mj_integratePos(m,scratch.qpos,d.qvel,sign*eps);mujoco.mj_forward(m,scratch);js.append(body_jacobian(m,scratch,b,scratch.xipos[b]))
    accel=Jc@d.qacc+(js[1]-js[0])@d.qvel/(2*eps);twist=Jc@d.qvel
    R=d.ximat[b].reshape(3,3);Iw=R@np.diag(m.body_inertia[b])@R.T;moment_c=wt[3:]+np.cross(point-d.xipos[b],wt[:3])
    return {'physical_target_wrench_at_G_world':wt,'physical_robot_wrench_at_G_world':wr,'interface_generalized_reconstruction_error':float(np.linalg.norm(reconstructed-d.qfrc_constraint)),
        'interface_power_residual_w':float(power-d.qfrc_constraint@d.qvel),'interface_net_force_world_n':wt[:3]+wr[:3],'interface_net_moment_at_G_world_nm':wt[3:]+wr[3:],
        'target_newton_residual_n':m.body_mass[b]*accel[:3]-wt[:3],
        'target_euler_residual_nm':Iw@accel[3:]+np.cross(twist[3:],Iw@twist[3:])-moment_c}

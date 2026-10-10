"""Cheap kinematic ranking, never a safety certificate or plant trajectory."""
import copy
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from v6_mujoco.model import site_id
from v6_mujoco.collision import signed_distance
from v6_mujoco.geometry_capture.design import planning_minimum
from v6_mujoco.postgrasp.physics import body_jacobian,joint_slices
from v6_mujoco.adaptive_capture.known_model import set_measured

def screen(owner,packet,e,reference,horizon):
    m=owner.model;d=mujoco.MjData(m);set_measured(m,d,packet,e);r=copy.deepcopy(reference)
    h=owner.hqp;sid=site_id(m,'flange_site');_,tv=joint_slices(m,'target_free_joint')
    # End-effector nullspace and base reaction are separately reported.
    minimum=1e9;base_peak=0.;speed=0.;tracking=0.;first=None;steps=0;invalid=None
    end=min(horizon,max(.2,r.arrival-packet.time));dt=.1
    for k in range(int(np.ceil(end/dt))+1):
        mujoco.mj_forward(m,d);elapsed=k*dt
        # Current CA mean, not future sensor packets or true inertia.
        ee=__import__('dataclasses').replace(e,time=packet.time+elapsed,p=e.p+e.v*elapsed+.5*e.a*elapsed**2,
            R=Rotation.from_rotvec(e.w*elapsed+.5*e.alpha*elapsed**2).as_matrix()@e.R,v=e.v+e.a*elapsed,w=e.w+e.alpha*elapsed)
        r.shaped=ee;ref=r.sample_at(packet.time+elapsed)
        mapping,_=h.reaction_velocity_map(d);drift=d.qvel-mapping@d.qvel[owner.vids]
        J=body_jacobian(m,d,int(m.site_bodyid[sid]),d.site_xpos[sid]);G=J@mapping
        error=np.r_[ref.position_world_m-d.site_xpos[sid],Rotation.from_matrix(ref.rotation_world@d.site_xmat[sid].reshape(3,3).T).as_rotvec()]
        desired=np.r_[ref.linear_velocity_world_m_s,ref.angular_velocity_world_rad_s]+np.r_[[3.]*3,[4.]*3]*error-J@drift
        pinv=G.T@np.linalg.inv(G@G.T+1e-7*np.eye(6));primary=pinv@desired;N=np.eye(7)-np.linalg.pinv(G)@G
        sh=owner.shape.sample(d)
        if sh.singular:invalid='SHAPE_SINGULARITY';break
        g=owner.shape.jacobian(d);psierr=(ref.arm_angle_rad-sh.angle_rad+np.pi)%(2*np.pi)-np.pi
        z=N@g;secondary=z*(ref.arm_angle_velocity_rad_s+2*psierr-g@primary)/(g@z+1e-5)
        dq=primary+secondary;speed=max(speed,float(np.max(abs(dq)/owner.spec.velocity_limits_rad_s)))
        if first is None:first=dict(dq=dq.copy(),null_task_norm=float(np.linalg.norm(G@secondary)),base_reaction=mapping[owner.base]@secondary)
        dq=np.clip(dq,-owner.spec.velocity_limits_rad_s,owner.spec.velocity_limits_rad_s)
        velocity=drift+mapping@dq;base_peak=max(base_peak,float(np.linalg.norm(velocity[owner.base][3:])))
        distances=np.array([signed_distance(m,d,p)-planning_minimum(p) for p in h.pairs]);minimum=min(minimum,float(min(distances)))
        tracking=max(tracking,float(np.linalg.norm(error[:3])))
        if np.any(d.qpos[owner.qids]<h.joint_lower-.02) or np.any(d.qpos[owner.qids]>h.joint_upper+.02):invalid='KINEMATIC_JOINT_BOX';break
        d.qvel[:]=velocity;mujoco.mj_integratePos(m,d.qpos,velocity,dt);d.time+=dt;steps+=1
    # A coarse screen ranks candidates. Only plainly unbounded jets/positions
    # are rejected here; sampled distance penetration is left to dynamics.
    return dict(model='CA_kinematic_screen',valid=invalid is None,reason=invalid,steps=steps,
        min_margin_m=minimum,peak_base_rad_s=base_peak,max_speed_ratio=speed,max_tracking_m=tracking,
        first_action=first,score=[max(0.,-minimum)/.01,max(0.,speed-1.),tracking/.008,base_peak/.02],
        scope='ranking only; ignores servo/contact dynamics, not actuation authorization')

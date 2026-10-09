"""Synchronized copy observations and phase-specific safety gates."""
import copy
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from v6_mujoco.model import default_model_spec,site_id,body_id
from v6_mujoco.collision import build_collision_pairs,signed_distance
from v6_mujoco.postgrasp.physics import body_jacobian,momenta
from v6_mujoco.postgrasp_campaign.engine import observe as inherited_observe,capture_passed
from v6_mujoco.fpmfc.target import target_from_config
from .adapter import extra_pairs,latch_id,pads
from .common import preconfig,contactconfig,config,legacy,PHASE,read,C1

ZERO_REFERENCE={"rotation":np.eye(3),"translation":np.zeros(3)}

def observe(m,real,trajectory,shape,H0=None):
    # The inherited observer's single equality assumption is explicitly checked.
    assert m.neq==1 and latch_id(m)==0
    row=inherited_observe(m,real,ZERO_REFERENCE,build_collision_pairs(m),H0)
    d=copy.copy(real);mujoco.mj_forward(m,d);t=float(real.time)
    target=target_from_config(preconfig()).sample(t);bid=body_id(m,"tumbling_target");flange=site_id(m,"flange_site")
    R=d.site_xmat[flange].reshape(3,3);p=d.site_xpos[flange].copy()
    jac=body_jacobian(m,d,int(m.site_bodyid[flange]),p);twist=jac@d.qvel
    target_R=d.xmat[bid].reshape(3,3)
    recess=float(read(C1/"pairing_manifest.json")["terminal_reference"]["tool_face_recess_m"])
    face_position=p-R[:,2]*recess
    face_velocity=twist[:3]+np.cross(twist[3:],face_position-p)
    grasp=site_id(m,"target_grasp_site");grasp_jac=body_jacobian(m,d,int(m.site_bodyid[grasp]),d.site_xpos[grasp]);grasp_twist=grasp_jac@d.qvel
    active=bool(d.eq_active[latch_id(m)]); applicable=t<=8+1e-9
    ref=trajectory.sample(min(t,8.))
    distances=np.array([signed_distance(m,d,pair) for pair in extra_pairs(m)])
    intended=[];unintended=0;contact_peak=0.;normal=0.;penetration=0.
    for i in range(d.ncon):
        c=d.contact[i];f=np.zeros(6);mujoco.mj_contactForce(m,d,i,f)
        if {int(c.geom1),int(c.geom2)}==set(pads(m)):
            intended.append(i);contact_peak=max(contact_peak,float(np.linalg.norm(f[:3])));normal+=float(f[0]);penetration=max(penetration,-float(c.dist))
        else:unintended+=1
    row.update(target_position_world_m=d.xpos[bid].copy(),target_rotation_world=target_R.copy(),target_reference_position_world_m=target.center_position_world_m,target_reference_rotation_world=target.center_rotation_world,
        target_prediction_position_error_m=float(np.linalg.norm(d.xpos[bid]-target.center_position_world_m)),target_prediction_rotation_error_rad=float(Rotation.from_matrix(target.center_rotation_world.T@target_R).magnitude()),
        flange_position_world_m=p,flange_rotation_world=R.copy(),flange_twist_world=twist,approach_reference_position_world_m=ref.position_world_m,approach_reference_rotation_world=ref.rotation_world,approach_reference_twist_world=np.r_[ref.linear_velocity_world_m_s,ref.angular_velocity_world_rad_s],
        physical_tool_face_position_world_m=face_position,physical_tool_face_relative_linear_speed_m_s=float(np.linalg.norm(face_velocity-grasp_twist[:3])),physical_flange_relative_angular_speed_rad_s=float(np.linalg.norm(twist[3:]-grasp_twist[3:])),
        approach_position_error_m=float(np.linalg.norm(p-ref.position_world_m)),approach_rotation_error_rad=float(Rotation.from_matrix(ref.rotation_world.T@R).magnitude()),approach_shape_error_rad=float(abs((shape.sample(d).angle_rad-ref.arm_angle_rad+np.pi)%(2*np.pi)-np.pi)),
        approach_reference_applicable=applicable,holding_error_applicable=active,wrench_applicable=bool(active or intended),
        intentional_contact_count=len(intended),unintended_contact_count=unintended,contact_peak_force_n=contact_peak,contact_normal_force_n=normal,contact_penetration_m=penetration,extra_pair_distances_m=distances,
        local_postlatch_time_s=t-8,robot_momentum_world=np.zeros(6))
    physical=momenta(m,d);tj=body_jacobian(m,d,bid,d.xipos[bid]);velocity=tj@d.qvel; mass=m.body_mass[bid];radius=d.xipos[bid]-np.asarray(physical["center_world_m"])
    inertia=d.ximat[bid].reshape(3,3)@np.diag(m.body_inertia[bid])@d.ximat[bid].reshape(3,3).T
    row["robot_momentum_world"]=np.r_[np.asarray(physical["linear_momentum_world_kg_m_s"])-mass*velocity[:3],np.asarray(physical["angular_momentum_about_center_world_kg_m2_s"])-inertia@velocity[3:]-np.cross(radius,mass*velocity[:3])]
    return row

def safety(row,initial,m,had_contact=False):
    if not all(np.all(np.isfinite(v)) for v in row.values()):return "MOMENTUM_OR_NUMERICS_FAILED","nonfinite"
    for key,limit in [("linear_momentum_world_kg_m_s",1e-4),("angular_momentum_about_center_world_kg_m2_s",1e-5)]:
        if np.linalg.norm(np.asarray(row[key])-initial[key])>limit:return "MOMENTUM_OR_NUMERICS_FAILED",key
    if np.any(row["qfrc_applied"]) or np.any(row["xfrc_applied"]) or np.any(row["solver_warning_count"]):return "MOMENTUM_OR_NUMERICS_FAILED","external force or solver warning"
    for key,limit in [("solver_residual",1e-7),("wrench_reconstruction_error",1e-8),("action_reaction_force_error_n",1e-8),("action_reaction_moment_geometry_error_nm",1e-8)]:
        if row[key]>limit:return "MOMENTUM_OR_NUMERICS_FAILED",key
    spec=default_model_spec();qids,vids=spec.joint_addresses(m)
    joints=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n) for n in spec.joint_names]
    limits=m.jnt_range[joints];post=bool(row["holding_error_applicable"])
    state_failure="POSTGRASP_SAFETY_FAILED" if post else "PRECONTACT_TRACKING_FAILED"
    if np.any(row["joint_position_rad"]<limits[:,0]-1e-8) or np.any(row["joint_position_rad"]>limits[:,1]+1e-8) or np.any(np.abs(row["joint_velocity_rad_s"])>spec.velocity_limits_rad_s+1e-8) or np.any(np.abs(row["ctrl_nm"])>spec.torque_limits_nm+1e-8) or np.any(np.abs(row["actuator_force_nm"])>spec.torque_limits_nm+1e-8):return state_failure,"joint/torque limit"
    if row["minimum_noncontact_clearance_m"]<.04:return "UNINTENDED_CONTACT_OR_COLLISION","original 40 mm pair"
    for pair,distance in zip(extra_pairs(m),row["extra_pair_distances_m"]):
        minimum=-contactconfig()["acceptance"]["maximum_penetration_m"] if pair.category=="intended_surface" else config()["new_collision_margin_m"]
        if distance<minimum-config()["numerical_penetration_tolerance_m"]:return "UNINTENDED_CONTACT_OR_COLLISION",pair.name
    if row["unintended_contact_count"]:return "UNINTENDED_CONTACT_OR_COLLISION","unplanned physical contact"
    if row["load_fraction"]>1:return state_failure,"actual rho > 1"
    if post:
        if row["interface_translation_error_m"]>.0005 or row["interface_rotation_error_deg"]>.1:return "POSTGRASP_SAFETY_FAILED","holding pose"
    elif row["contact_peak_force_n"]>20 or row["contact_penetration_m"]>.002:return "UNINTENDED_CONTACT_OR_COLLISION","prelatch 20 N/2 mm contact gate"
    if not had_contact and not row["intentional_contact_count"] and not post:
        if row["target_prediction_position_error_m"]>config()["target_propagation_position_tolerance_m"] or row["target_prediction_rotation_error_rad"]>config()["target_propagation_rotation_tolerance_rad"]:return "TARGET_PROPAGATION_MISMATCH","precontact analytic diagnostic"
    return None,None

def capture(row):return capture_passed(row,legacy()["design"]["latch"])

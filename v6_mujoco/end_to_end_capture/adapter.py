"""Name-addressed full model; initialization is the only physical state writer."""
import copy
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from v6_mujoco.model import default_model_spec,geom_id,body_id,site_id
from v6_mujoco.collision import build_collision_pairs,CollisionPair
from v6_mujoco.postgrasp.physics import joint_slices,object_id
from v6_mujoco.fpmfc.target import target_from_config
from .common import MODEL,C1,read,preconfig,config

def compile_model(scenario="nominal"):
    _,assets=default_model_spec()._xml_and_assets()
    m=mujoco.MjModel.from_xml_string(MODEL.read_text(encoding="utf-8").replace('meshdir="../assets/meshes"','meshdir="."'),assets)
    s=config()["scenarios"][scenario];m.opt.timestep=s["dt"]
    tid=body_id(m,"tumbling_target");m.body_mass[tid]=s["mass"];m.body_inertia[tid]=s["inertia"]
    m.geom_pos[geom_id(m,"workspace_obstacle_0")]=[10,10,10] # Frozen C1 obstacle placement.
    mujoco.mj_setConst(m,mujoco.MjData(m))
    assert (m.nq,m.nv,m.nu,m.neq)==(21,19,7,1)
    return m

def addresses(m):
    names=["target_free_joint","base_free_joint"]+[f"joint{i}" for i in range(1,8)]
    return {n:{"qpos":list(range(*joint_slices(m,n)[0].indices(m.nq))),"qvel":list(range(*joint_slices(m,n)[1].indices(m.nv)))} for n in names}

def latch_id(m):return object_id(m,mujoco.mjtObj.mjOBJ_EQUALITY,"postgrasp_latch")
def pads(m):return [geom_id(m,n) for n in ("gripper_contact_pad","target_contact_plate")]

def initialize(m,spin_scale=1.):
    d=mujoco.MjData(m);spec=default_model_spec()
    key=object_id(m,mujoco.mjtObj.mjOBJ_KEY,"simscape_home")
    # Use the named model home key only during t=0 initialization.
    mujoco.mj_resetDataKeyframe(m,d,key)
    qj,vj=spec.joint_addresses(m);d.qpos[qj]=spec.home_joint_position
    bq,bv=spec.base_slices(m);d.qpos[bq]=[0,0,0,1,0,0,0];d.qvel[:]=0
    sample=target_from_config(preconfig()).sample(0);tq,tv=joint_slices(m,"target_free_joint")
    quat=Rotation.from_matrix(sample.center_rotation_world).as_quat()
    d.qpos[tq]=np.r_[sample.center_position_world_m,quat[3],quat[:3]]
    d.qvel[tv]=np.r_[sample.center_linear_velocity_world_m_s,spin_scale*sample.center_rotation_world.T@sample.angular_velocity_world_rad_s]
    d.eq_active[latch_id(m)]=False;d.ctrl[:]=0
    mujoco.mj_forward(m,d)
    return d

def extra_pairs(m):
    robot=["satellite_base_collision"]+[f"link{i}_collision" for i in range(8)]+["gripper_contact_pad"]
    return tuple(CollisionPair(a+"__"+b,geom_id(m,a),geom_id(m,b),"intended_surface" if a=="gripper_contact_pad" else "target_distance") for a in robot for b in ("tumbling_target_geom","target_contact_plate") if (a,b)!=("gripper_contact_pad","target_contact_plate"))

def contract():
    m=compile_model();d=initialize(m); ids=addresses(m)
    interactions=[]
    for i in range(m.ngeom):
        for j in range(i+1,m.ngeom):
            if (m.geom_contype[i]&m.geom_conaffinity[j]) or (m.geom_contype[j]&m.geom_conaffinity[i]):interactions.append([mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,k) for k in (i,j)])
    related=[p for p in interactions if any(n in p for n in ("gripper_contact_pad","target_contact_plate"))]
    assert related==[["target_contact_plate","gripper_contact_pad"]]
    return {"model_adapter_contract":{"dimensions":[m.nq,m.nv,m.nu],"addresses":ids,"actuators":[mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_ACTUATOR,i) for i in range(m.nu)],"latch_id":latch_id(m),"latch_initial_active":bool(d.eq_active[latch_id(m)]),"site_local_frames":{n:{"position":m.site_pos[site_id(m,n)].tolist(),"quaternion":m.site_quat[site_id(m,n)].tolist()} for n in ("flange_site","postgrasp_tool_interface","target_grasp_site")},"tool_face":"flange_position - flange_z * original terminal_reference.tool_face_recess_m; distinct from calibrated weld site","reaction_mapping":"robot base and seven named joints only; target rows zero; model.nv sized matrices"},"initialization_contract":{"qpos":d.qpos.tolist(),"qvel":d.qvel.tolist(),"target_velocity_convention":"free-joint linear world, angular local body; checked by world Jacobian and free propagation tests","target_from":"original target config sample(0)","robot_from":"named home key and unchanged home joints","no_terminal_state_source":True},"contact_pair_contract":{"material":"unchanged original pad/plate friction solref solimp margin","initial_mask_interactions":interactions,"pad_mask_interactions":related,"latch_event":"disable only the two pads; enumeration proves no other physical mask interaction removed","original_40mm_pairs":[p.name for p in build_collision_pairs(m)],"new_distance_pairs":[{"name":p.name,"coverage":"signed-distance only; no collision response","minimum_m":-.002 if p.category=="intended_surface" else config()["new_collision_margin_m"]} for p in extra_pairs(m)],"pad_cube_relation":"cube surface coincides with physical plate front face; only pad-cube gets original 2mm contact penetration allowance; all link/cube and link/plate pairs remain checked","numerical_penetration_tolerance_m":config()["numerical_penetration_tolerance_m"],"blanket_target_or_end_effector_exclusions":False}}

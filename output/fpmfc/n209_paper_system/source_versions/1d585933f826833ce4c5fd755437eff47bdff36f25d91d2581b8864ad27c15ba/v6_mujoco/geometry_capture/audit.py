"""G0/G1: only saved states and static isolated data; never mj_step."""
import copy
import json
import numpy as np
import mujoco
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from v6_mujoco.model import PROJECT_ROOT,default_model_spec,geom_id,body_id,site_id
from v6_mujoco.postgrasp.physics import joint_slices,digest
from v6_mujoco.postgrasp_campaign.io import save,identity
from v6_mujoco.end_to_end_capture.common import C1,ROOT as OLD,MODEL,arrays,preconfig,verify_design
from v6_mujoco.end_to_end_capture.adapter import compile_model,initialize,addresses,extra_pairs
from v6_mujoco.fpmfc.target import target_from_config
from .geometry import load_stl,topology,mesh_box,convex_box_sat

ROOT=PROJECT_ROOT/'output/fpmfc/n206_geometry_capture'

def transform(p,R):
    T=np.eye(4);T[:3,:3]=np.asarray(R).reshape(3,3);T[:3,3]=p;return T

def quatR(q):return Rotation.from_quat(np.roll(q,-1)).as_matrix()
def bodyT(m,d,n):
    i=body_id(m,n);return transform(d.xpos[i],d.xmat[i])
def siteT(m,d,n):
    i=site_id(m,n);return transform(d.site_xpos[i],d.site_xmat[i])
def world_vertices(m,d,n):
    g=geom_id(m,n); mesh=m.geom_dataid[g]; v=m.mesh_vert[m.mesh_vertadr[mesh]:m.mesh_vertadr[mesh]+m.mesh_vertnum[mesh]]
    return v@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]

def set_body_free(m,d,name,T):
    q,_=joint_slices(m,name);quat=Rotation.from_matrix(T[:3,:3]).as_quat();d.qpos[q]=np.r_[T[:3,3],quat[3],quat[:3]]

def historical(m):
    d=initialize(m);source=arrays(C1/'precontact/C1/trace.npz');old=default_model_spec().compile_model()
    for name,ids in addresses(m).items():
        if name=='target_free_joint':continue
        q,v=joint_slices(old,name);d.qpos[ids['qpos']]=source['qpos'][-1,q];d.qvel[ids['qvel']]=source['qvel'][-1,v]
    target=target_from_config(preconfig()).sample(8)
    set_body_free(m,d,'target_free_joint',transform(target.center_position_world_m,target.center_rotation_world))
    d.time=8;mujoco.mj_forward(m,d);return d

def ideal(m,seed=None):
    """Whole robot rigid pose correction in diagnostic copy, no IK assumption.

    Link7 result is independent of chosen upstream posture. Other pair results
    describe only this explicitly labelled static posture, not reachability.
    """
    d=historical(m) if seed is None else copy.copy(seed)
    shift=siteT(m,d,'target_grasp_site')@np.linalg.inv(siteT(m,d,'postgrasp_tool_interface'))
    set_body_free(m,d,'base_free_joint',shift@bodyT(m,d,'base_link_0'))
    mujoco.mj_forward(m,d);return d

def distance(m,d,a,b):
    segment=np.zeros(6);value=mujoco.mj_geomDistance(m,d,geom_id(m,a),geom_id(m,b),2.,segment)
    return float(value),segment

def ordered_box_witness(m,d,box,segment):
    """Label endpoints geometrically, not by assumed 3.3.2 pair order.

    Mesh-box raw fromto is reversed in the installed runtime. Box signed SDF
    residual identifies its endpoint independently of any velocity probe.
    """
    g=geom_id(m,box);points=segment.reshape(2,3);local=(points-d.geom_xpos[g])@d.geom_xmat[g].reshape(3,3)
    q=abs(local)-m.geom_size[g];sdf=np.linalg.norm(np.maximum(q,0),axis=1)+np.minimum(np.max(q,axis=1),0)
    reverse=bool(abs(sdf[0])<abs(sdf[1]));ordered=points[::-1].copy() if reverse else points.copy()
    return ordered.reshape(6),reverse,abs(sdf).tolist()

def state_audit(m,d,label):
    target=bodyT(m,d,'tumbling_target');result={'label':label,'time_s':float(d.time),'qpos':d.qpos.tolist(),'pairs':[]}
    for p in extra_pairs(m):
        a=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,p.geom_a);b=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,p.geom_b)
        value,rawseg=distance(m,d,a,b);seg,reversed_,residuals=ordered_box_witness(m,d,b,rawseg);direction=(seg[3:]-seg[:3])/value if abs(value)>1e-12 and value<2 else np.zeros(3)
        item={'pair':p.name,'signed_distance_m':value,'closest_points_world_m':seg.reshape(2,3).tolist(),'raw_fromto_world_m':rawseg.tolist(),'raw_segment_reversed_by_box_surface_membership':reversed_,'box_surface_residuals_m':residuals,
              'closest_points_target_m':((seg.reshape(2,3)-target[:3,3])@target[:3,:3]).tolist(),
              'separation_direction_world':direction.tolist(),'separation_direction_target':(target[:3,:3].T@direction).tolist(),
              'geoms':[{'name':n,'body':mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_BODY,int(m.geom_bodyid[g])),
                        'T_world_geom':transform(d.geom_xpos[g],d.geom_xmat[g]).tolist(),
                        'T_world_body':transform(d.xpos[m.geom_bodyid[g]],d.xmat[m.geom_bodyid[g]]).tolist()} for n,g in [(a,p.geom_a),(b,p.geom_b)]]}
        if a=='link7_collision':
            g=p.geom_b;vertices=(world_vertices(m,d,a)-d.geom_xpos[g])@d.geom_xmat[g].reshape(3,3)
            mesh=m.geom_dataid[p.geom_a];f=m.mesh_face[m.mesh_faceadr[mesh]:m.mesh_faceadr[mesh]+m.mesh_facenum[mesh]]
            item['independent_mesh_box']=mesh_box(vertices,f,m.geom_size[g])
            item['independent_convex_sat']=convex_box_sat(vertices,m.geom_size[g])
        result['pairs'].append(item)
    result['T_target_link7']=(np.linalg.inv(target)@bodyT(m,d,'link7')).tolist()
    result['interface_translation_error_m']=float(np.linalg.norm(siteT(m,d,'target_grasp_site')[:3,3]-siteT(m,d,'postgrasp_tool_interface')[:3,3]))
    return result

def run():
    verify_design();m=compile_model();home=initialize(m);trace=arrays(OLD/'E_nominal/trace.npz')
    assert mujoco.__version__=='3.3.2'
    states={}
    for t in [7.360,7.362]:
        i=int(np.argmin(abs(trace['time_s']-t)));assert abs(trace['time_s'][i]-t)<1e-9
        d=copy.copy(home);d.qpos[:]=trace['qpos'][i];d.qvel[:]=trace['qvel'][i];d.time=trace['time_s'][i];mujoco.mj_forward(m,d);states[f'n205_actual_{t:.3f}']=d
    states['historical_C1_actual_8']=historical(m);states['ideal_fixed_mating']=ideal(m)
    F=np.linalg.inv(bodyT(m,home,'link7'))@siteT(m,home,'flange_site')
    E=np.linalg.inv(siteT(m,home,'flange_site'))@siteT(m,home,'postgrasp_tool_interface')
    G=np.linalg.inv(bodyT(m,home,'tumbling_target'))@siteT(m,home,'target_grasp_site')
    fixed=G@np.linalg.inv(E)@np.linalg.inv(F)
    save(ROOT/'terminal_mating_contract.json',{'T_link7_flange':F.tolist(),'T_flange_tool':E.tolist(),'T_target_grasp':G.tolist(),
         'T_target_link7_fixed':fixed.tolist(),'equation':'T_target_link7 = T_target_grasp inv(T_flange_tool) inv(T_link7_flange)',
         'flange_joint_count':int(m.body_jntnum[body_id(m,'flange')]),'legal_task_freedoms':[],
         'upstream_posture_or_common_world_pose_can_change_fixed_relative_geometry':False,
         'interface_semantics':'site-based weld fixes all six pose components; no evidence for yaw or surface sliding freedom',
         'pad_front_flange_z_m':float(m.geom_pos[geom_id(m,'gripper_contact_pad'),2]+m.geom_size[geom_id(m,'gripper_contact_pad'),1]),
         'original_interface_from_N201_calibrated_frozen_site_not_recalibrated_here':True})
    meshes={}
    for i in range(8):
        n=f'link{i}_collision';g=geom_id(m,n);mid=m.geom_dataid[g];path=PROJECT_ROOT/f'assets/meshes/rizon4s/collision/link{i}.stl'
        raw,f=load_stl(path);compiled=world_vertices(m,home,n);b=int(m.geom_bodyid[g]);expected=raw@home.xmat[b].reshape(3,3).T+home.xpos[b]
        err=max(cKDTree(compiled).query(expected)[0].max(),cKDTree(expected).query(compiled)[0].max())
        reconstructed=world_vertices(m,home,n) # compiled geom already includes mesh principal-axis compensation
        hull=convex_box_sat(raw,np.ones(3))
        meshes[n]={'source':str(path.relative_to(PROJECT_ROOT)),'sha256':digest(path),'topology':topology(raw,f),
             'compiled_vertex_count':int(m.mesh_vertnum[mid]),'compiled_face_count':int(m.mesh_facenum[mid]),
             'mesh_pos':m.mesh_pos[mid].tolist(),'mesh_quat':m.mesh_quat[mid].tolist(),
             'geom_pos_compiled':m.geom_pos[g].tolist(),'geom_quat_compiled':m.geom_quat[g].tolist(),
             'raw_to_compiled_world_hausdorff_m':float(err),'collision_representation':'MuJoCo convex hull of the same STL vertices',
             'display_representation':'same collision STL mesh, not separate high-resolution CAD',
             'no_double_application':'world = geom_xpos + geom_xmat @ compiled_vertex; raw source uses body transform once'}
    out=[state_audit(m,d,k) for k,d in states.items()]
    tests={};d=states['ideal_fixed_mating'];basegap=distance(m,d,'link7_collision','tumbling_target_geom')[0]
    H=transform([.4,-.2,.3],Rotation.from_rotvec([.3,-.6,.2]).as_matrix());c=copy.copy(d)
    for body,joint in [('base_link_0','base_free_joint'),('tumbling_target','target_free_joint')]:set_body_free(m,c,joint,H@bodyT(m,d,body))
    mujoco.mj_forward(m,c);v,seg=distance(m,d,'link7_collision','tumbling_target_geom');v2,seg2=distance(m,c,'link7_collision','tumbling_target_geom')
    tests['rigid_transform_distance_error_m']=abs(v-v2)
    tests['rigid_transform_witness_error_m']=float(np.max(abs(seg.reshape(2,3)@H[:3,:3].T+H[:3,3]-seg2.reshape(2,3))))
    tests['mesh_world_max_error_m']=max(x['raw_to_compiled_world_hausdorff_m'] for x in meshes.values())
    tests['fixed_chain_error']=float(np.max(abs(fixed-np.asarray(out[-1]['T_target_link7']))))
    samples=[];n=siteT(m,d,'target_grasp_site')[:3,2]
    for dx in np.linspace(-.0001,.0001,21):
        c=copy.copy(d);B=bodyT(m,c,'base_link_0');B[:3,3]-=dx*n;set_body_free(m,c,'base_free_joint',B);mujoco.mj_forward(m,c)
        v,seg=distance(m,c,'link7_collision','tumbling_target_geom');samples.append({'retreat_m':float(dx),'distance_m':v,'segment_world_m':seg.tolist()})
    tests['microtranslation_samples']=samples
    tests['retreat_distance_slope']=float(np.polyfit([x['retreat_m'] for x in samples],[x['distance_m'] for x in samples],1)[0])
    tests['max_adjacent_distance_change_m']=float(np.max(abs(np.diff([x['distance_m'] for x in samples]))))
    tests['threshold_7_360_pass_7_362_fail']=bool(out[0]['pairs'][-3]['signed_distance_m']>=.00099 and out[1]['pairs'][-3]['signed_distance_m']<.00099)
    legacy=copy.copy(m);legacy.opt.disableflags|=int(mujoco.mjtDisableBit.mjDSBL_NATIVECCD)
    legacyvals={k:distance(legacy,d,'link7_collision','tumbling_target_geom')[0] for k,d in states.items()}
    settings={'mujoco':mujoco.__version__,'disableflags':int(m.opt.disableflags),'enableflags':int(m.opt.enableflags),'nativeccd':True,
              'query_distmax_m':2.,'ccd_tolerance':float(m.opt.ccd_tolerance),'ccd_iterations':int(m.opt.ccd_iterations),
              'constraint_tolerance':float(m.opt.tolerance),'iterations':int(m.opt.iterations),
              'geom_margin_m':{mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,i):float(m.geom_margin[i]) for i in range(m.ngeom)},
              'legacy_comparison_only_not_acceptance':legacyvals,'adopted_detector':'unchanged frozen nativeccd'}
    result={'source_commit':'f2c4cf5cc12dd299589dc4e0a3f32d52ed085403','source_identity':identity([MODEL,OLD/'E_nominal/trace.npz',C1/'precontact/C1/trace.npz']),
            'new_physics_steps':0,'settings':settings,'meshes':meshes,'states':out,'tests':tests,
            'classification':'TERMINAL_MATING_GEOMETRY_INCOMPATIBLE' if basegap<.00099 else 'INSUFFICIENT_GEOMETRIC_EVIDENCE'}
    save(ROOT/'geometry_audit.json',result)
    print(json.dumps({'classification':result['classification'],'distances':{x['label']:x['pairs'][-3]['signed_distance_m'] for x in out},'tests':{k:v for k,v in tests.items() if k!='microtranslation_samples'},'link7_topology':meshes['link7_collision']['topology']},indent=2))

if __name__=='__main__':run()

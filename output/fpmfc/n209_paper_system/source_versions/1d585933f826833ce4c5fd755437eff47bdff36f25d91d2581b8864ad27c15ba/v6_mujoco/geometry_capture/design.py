"""Frozen route B: a positive-mass rigid spacer/pad simulation hypothesis."""
import copy
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
import yaml
from v6_mujoco.model import PROJECT_ROOT,default_model_spec,geom_id,body_id
from v6_mujoco.collision import CollisionPair,build_collision_pairs,signed_distance
from v6_mujoco.postgrasp.physics import momenta
from v6_mujoco.postgrasp_campaign.io import save,identity,read
from v6_mujoco.end_to_end_capture.adapter import compile_model as old_compile,initialize,extra_pairs as old_pairs
from .audit import ROOT,MODEL,ideal,siteT,state_audit
from .physics import momenta

CONFIG=PROJECT_ROOT/'configs/n206_geometry_capture.yaml'
NEW_MODEL=PROJECT_ROOT/'models/flexiv_rizon4s_n206_tool_scene.xml'
def config():return yaml.safe_load(CONFIG.read_text(encoding='utf-8'))

def dimensions(delta):
    c=config()['installation'];radius=c['pad_radius_m'];length=delta+c['original_pad_front_flange_z_m'];spacer=length-c['pad_thickness_m']
    mass=c['assumed_density_kg_m3']*np.pi*radius**2*length
    return {'increment_m':delta,'spacer_length_m':spacer,'pad_front_z_m':length,'radius_m':radius,'mass_kg':mass,
            'com_flange_m':[0.,0.,length/2], 'diaginertia_kg_m2':[mass*(3*radius**2+length**2)/12]*2+[mass*radius**2/2]}

def xml_candidate(delta):
    params=dimensions(delta)
    if params['spacer_length_m']<config()['installation']['minimum_spacer_length_m']:return None
    root=ET.fromstring(MODEL.read_text(encoding='utf-8'));flange=root.find('.//body[@name="flange"]')
    # Preserve the old massless flange inertial frame explicitly after moving
    # its geom; otherwise the compiler chooses a different unused COM frame.
    ET.SubElement(flange,'inertial',{'pos':'0 0 -0.0052','mass':'0','diaginertia':'0 0 0'})
    # Existing link7/flange body frames and all old inertias are untouched.
    tool=ET.SubElement(flange,'body',{'name':'n206_rigid_tool','pos':'0 0 0'})
    ET.SubElement(tool,'inertial',{'mass':str(params['mass_kg']),'pos':' '.join(map(str,params['com_flange_m'])),'diaginertia':' '.join(map(str,params['diaginertia_kg_m2']))})
    for name in ['postgrasp_tool_interface','gripper_contact_pad']:
        element=flange.find(f'./*[@name="{name}"]');flange.remove(element);p=np.fromstring(element.get('pos'),sep=' ');p[2]+=delta;element.set('pos',' '.join(map(str,p)));tool.append(element)
    s=params['spacer_length_m']
    ET.SubElement(tool,'geom',{'name':'n206_spacer','type':'cylinder','pos':f'0 0 {s/2}','size':f'{params["radius_m"]} {s/2}','density':'0','rgba':'0.15 0.5 0.7 1','contype':'0','conaffinity':'0'})
    ET.indent(root,space='  ');return ET.tostring(root,encoding='unicode')+'\n'

def from_xml(xml):
    _,assets=default_model_spec()._xml_and_assets();m=mujoco.MjModel.from_xml_string(xml.replace('meshdir="../assets/meshes"','meshdir="."'),assets)
    m.geom_pos[geom_id(m,'workspace_obstacle_0')]=[10,10,10];mujoco.mj_setConst(m,mujoco.MjData(m));return m

def compile_model(scenario='nominal'):
    m=from_xml(NEW_MODEL.read_text(encoding='utf-8'));s=config()['scenarios'][scenario];m.opt.timestep=s['dt'];t=body_id(m,'tumbling_target');m.body_mass[t]=s['mass'];m.body_inertia[t]=s['inertia'];mujoco.mj_setConst(m,mujoco.MjData(m));return m

def extra_pairs(m):
    pairs=list(old_pairs(m));g=geom_id(m,'n206_spacer')
    for name in ['tumbling_target_geom','target_contact_plate']:
        pairs.append(CollisionPair('n206_spacer__'+name,g,geom_id(m,name),'target_distance'))
    for tool in ['n206_spacer','gripper_contact_pad']:
        for name in ['satellite_base_collision']+[f'link{i}_collision' for i in range(7)]+['workspace_obstacle_0']:
            pairs.append(CollisionPair(tool+'__'+name,geom_id(m,tool),geom_id(m,name),'tool_robot'))
    return tuple(pairs)

def planning_minimum(pair):
    if pair.category=='intended_surface':return -.002
    return .045 if pair.category in ['tool_robot','workspace','satellite','self'] else .002

def run():
    assert read(ROOT/'geometry_audit.json')['classification']=='TERMINAL_MATING_GEOMETRY_INCOMPATIBLE'
    old=old_compile();results=[];selected=None
    for delta in config()['installation_candidates_m']:
        dims=dimensions(delta);xml=xml_candidate(delta)
        if xml is None:
            results.append({**dims,'qualified':False,'reason':'NO_POSITIVE_EXTERNAL_SPACER: pad back lies behind frozen flange seat; no unverified recess or bore invented'});continue
        m=from_xml(xml);d=ideal(m)
        distances=[{'pair':p.name,'distance_m':signed_distance(m,d,p),'planning_minimum_m':planning_minimum(p)} for p in build_collision_pairs(m)+extra_pairs(m)]
        ok=all(x['distance_m']>=x['planning_minimum_m'] for x in distances)
        old_same=True
        for b in range(1,old.nbody):
            name=mujoco.mj_id2name(old,mujoco.mjtObj.mjOBJ_BODY,b);j=body_id(m,name)
            old_same&=all(np.array_equal(getattr(old,k)[b],getattr(m,k)[j]) for k in ['body_mass','body_inertia','body_ipos','body_iquat'])
        assert old_same and (m.nq,m.nv,m.nu)==(21,19,7)
        row={**dims,'qualified':ok,'all_pairs':distances,'old_body_inertias_identical':old_same,'nq_nv_nu':[m.nq,m.nv,m.nu],
             'static_posture_scope':'old joint angles used only as arbitrary rigid assembly seed; exact mating from fixed chain, not an IK or planning result',
             'assembly_exemptions':[{'pair':'n206_spacer--link7','reason':'explicit bolted-seat simulation assumption at flange; attached rigid part, not target contact'},
                                   {'pair':'n206_spacer--pad','reason':'contiguous tool body, internal face'},
                                   {'pair':'pad--link7','reason':'same fixed assembly; shifted pad has positive clearance, not a target exemption'}]}
        results.append(row)
        if ok and selected is None:selected=(dims,xml,m,d)
    save(ROOT/'candidate_geometry_results.json',{'frozen_candidates_m':config()['installation_candidates_m'],'results':results,'selection_does_not_use_dynamics':True})
    if selected is None:
        save(ROOT/'selected_design.json',{'status':'NEEDS_INTERFACE_DESIGN_CONFIRMATION','reason':'no frozen candidate passes supported geometry'});return
    dims,xml,m,d=selected;NEW_MODEL.write_text(xml,encoding='utf-8',newline='\n')
    home=initialize(m);E=np.linalg.inv(siteT(m,home,'flange_site'))@siteT(m,home,'postgrasp_tool_interface')
    save(ROOT/'selected_design.json',{'status':'STATIC_GEOMETRY_CANDIDATE_SELECTED_NOT_DYNAMICALLY_QUALIFIED','route':'B','hardware_design_assumption_changed':True,
         'hardware_approved':False,'parameters':dims,'T_flange_tool':E.tolist(),'model_identity':identity([NEW_MODEL,CONFIG]),
         'connection_assumption':config()['installation']['connection_assumption'],'selection':'minimum frozen increment with positive external spacer and all static pair margins',
         'margin_scope':'one exact ideal pose; perturbations and full approach still require qualification'})
    save(ROOT/'changed_mass_inertia.json',{'added_body':'n206_rigid_tool','parameters':dims,'original_bodies_identical':True,'joint_count':7,
         'home_system':momenta(m,home),'ideal_static_system':momenta(m,d),
         'lever_arm_change_flange_m':[0,0,dims['increment_m']], 'interface_weld_parameters_unchanged':bool(np.array_equal(m.eq_solref,old.eq_solref) and np.array_equal(m.eq_solimp,old.eq_solimp) and np.array_equal(m.eq_data,old.eq_data)),
         'required_requalification':['new whole-system momentum and locked inertia','interface physical wrench and power at shifted site','candidate2 restricted load envelope with new lever arm','capture rejection and no state mutation','seven actuator joint load limits'],
         'historical_qualification_inherited':False})
    save(ROOT/'selected_ideal_geometry.json',state_audit(m,d,'N206 exact mating; static seed posture'))
    print({'selected_increment_mm':dims['increment_m']*1000,'mass_kg':dims['mass_kg'],'spacer_mm':dims['spacer_length_m']*1000})

if __name__=='__main__':run()

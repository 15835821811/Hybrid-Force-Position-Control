"""New distal composite inertia and shifted interface; frozen candidate2 loads."""
import copy
import time
import numpy as np
import mujoco
from v6_mujoco.model import PROJECT_ROOT,body_id,site_id
from v6_mujoco.postgrasp_campaign.io import save,save_npz,read,identity
from v6_mujoco.postgrasp_calibration import load_tests as inherited
from .audit import ROOT,ideal
from .design import compile_model,NEW_MODEL

def fixture(parameters,timestep=.002):
    m=compile_model();d=ideal(m);ids=[body_id(m,n) for n in ['link7','n206_rigid_tool']];mass=m.body_mass[ids].sum();com=(m.body_mass[ids,None]*d.xipos[ids]).sum(0)/mass
    inertia=np.zeros((3,3))
    for b in ids:
        r=d.xipos[b]-com;R=d.ximat[b].reshape(3,3);inertia+=R@np.diag(m.body_inertia[b])@R.T+m.body_mass[b]*((r@r)*np.eye(3)-np.outer(r,r))
    eig,Rr=np.linalg.eigh(inertia)
    if np.linalg.det(Rr)<0:Rr[:,0]*=-1
    target=body_id(m,'tumbling_target');Pt=d.xipos[target];Rt=d.ximat[target].reshape(3,3);site=site_id(m,'target_grasp_site');Ps=d.site_xpos[site];Rs=d.site_xmat[site].reshape(3,3)
    fmt=inherited.fmt;quat=inherited.quat
    bodies=[]
    for name,joint,p,R,mass_,I,site_name in [('tumbling_target','target_free_joint',Pt,Rt,m.body_mass[target],m.body_inertia[target],'target_grasp_site'),('flange','tool_free_joint',com,Rr,mass,eig,'postgrasp_tool_interface')]:
        bodies.append(f'<body name="{name}" pos="{fmt(p)}" quat="{fmt(quat(R))}"><freejoint name="{joint}"/><inertial pos="0 0 0" mass="{mass_}" diaginertia="{fmt(I)}"/><site name="{site_name}" pos="{fmt(R.T@(Ps-p))}" quat="{fmt(quat(R.T@Rs))}"/></body>')
    xml=f'''<mujoco model="n206_distal_composite_fixture"><compiler angle="radian"/><option timestep="{timestep}" gravity="0 0 0" integrator="RK4" iterations="50" tolerance="1e-10"/><worldbody>{''.join(bodies)}</worldbody><equality><weld name="postgrasp_latch" site1="postgrasp_tool_interface" site2="target_grasp_site" solref="{fmt(parameters['solref'])}" solimp="{fmt(parameters['solimp'])}" torquescale="{parameters['torquescale_m']}"/></equality></mujoco>'''
    return mujoco.MjModel.from_xml_string(xml),{'source_model_identity':identity([NEW_MODEL]),'composite_bodies':['link7','n206_rigid_tool'],'mass_kg':float(mass),'com_world_m':com.tolist(),'inertia_world_kg_m2':inertia.tolist(),'principal_inertia_kg_m2':eig.tolist(),'interface_world_m':Ps.tolist(),'scope':'distal rigid composite, excludes upstream arm and manufacturing flexibility'},xml

def run():
    path=ROOT/'interface_tests';path.mkdir(exist_ok=True)
    assert not (path/'result.json').exists(), 'preserve completed fixture evidence'
    contract=read(PROJECT_ROOT/'output/fpmfc/postgrasp_campaign/restricted_fixture/input_contract.json')
    parameters=read(PROJECT_ROOT/'output/fpmfc/postgrasp_campaign/campaign_manifest.json')['connection_parameters']
    save(path/'frozen_scope.json',{'reason':'Added mass and shifted point alter full 6D distal mobility; repeat only restricted fixture directions and three original fine checks. Do not repeat old controller campaigns or tune weld.',
         'cases':contract['cases'],'fine_cases':contract['fine_cases'],'parameters':parameters,'actual_envelope':{'force_n':50,'moment_nm':2,'rho_max':1},'damping_retuned':False,'old_full_boundary_qualification':'FAILED','implementation_identity':identity([__file__,inherited.__file__])})
    ledger=read(ROOT/'run_ledger.json');ledger['interface_tests']=[];save(ROOT/'run_ledger.json',ledger)
    cases={x['name']:x for x in contract['cases']};schedule=[(n,.002) for n in cases]+[(n,.001) for n in contract['fine_cases']];results=[]
    original=inherited.fixture;inherited.fixture=fixture
    try:
        for name,dt in schedule:
            label=name+('_fine' if dt==.001 else '_coarse');ledger=read(ROOT/'run_ledger.json');ledger['interface_tests'].append({'label':label,'status':'RUNNING','duration_s':.9,'dt':dt});save(ROOT/'run_ledger.json',ledger)
            row,a,geometry,xml=inherited.test_case(parameters,cases[name],dt)
            save_npz(path/(label+'.npz'),a);(path/(label+'.xml')).write_text(xml,encoding='utf-8',newline='\n')
            row['passed']=bool(row['pose_holding_passed'] and row['numerics_passed'] and row['max_interface_load_fraction']<=1)
            if dt==.001:
                with np.load(path/(name+'_coarse.npz')) as z:diff={k:float(np.max(abs(z[k]-a[k][::2]))) for k in ['translation_error_m','rotation_error_deg','interface_load_fraction']}
                row['fine_differences']=diff;row['passed']&=diff['translation_error_m']<=.00005 and diff['rotation_error_deg']<=.01 and diff['interface_load_fraction']<=.02
            row['label']=label;row['geometry']=geometry;save(path/(label+'.json'),row);results.append(row)
            ledger['interface_tests'][-1]['status']='PASSED' if row['passed'] else 'FAILED';save(ROOT/'run_ledger.json',ledger)
            print({'fixture':label,'passed':row['passed'],'rho':row['max_interface_load_fraction']},flush=True)
            if not row['passed']:break
    finally:inherited.fixture=original
    result={'passed':len(results)==len(schedule) and all(r['passed'] for r in results),'results':results,'scope':'only frozen restricted input profile/directions; no full-envelope or arbitrary hardware certification','formal_robot_attempts':0}
    save(path/'result.json',result);return result

if __name__=='__main__':run()

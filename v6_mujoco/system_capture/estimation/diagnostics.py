"""Read-only diagnostics from persisted estimates and independent observations."""
import numpy as np
from scipy.spatial.transform import Rotation
from .common import OUT,OLD,read,save,sha
from .offline import KINDS,public_offset

def components(e):
    return dict(component_rms=np.sqrt(np.mean(e**2,axis=0)),component_peak=np.max(abs(e),axis=0),
        norm_rms=[np.sqrt(np.mean(np.sum(e[:,k:k+3]**2,axis=1))) for k in (0,3,6,9)],
        norm_peak=[np.max(np.linalg.norm(e[:,k:k+3],axis=1)) for k in (0,3,6,9)])

def augment_offline():
    offset=public_offset();summary={}
    for path in sorted((OUT/'offline').glob('*/comparison.npz')):
        name=path.parent.name
        if name.startswith('development_'):continue
        with np.load(path) as a:z={k:a[k] for k in a.files}
        if name.startswith('archived_'):
            run=name[len('archived_'):].rsplit('_',1)[0]
            with np.load(OLD/'runs'/run/'trace.npz') as a:
                truth=dict(p=a['target_position_world_m'],R=a['target_rotation_world'],v=a['target_geometric_twist_world'][:,:3],w=a['target_geometric_twist_world'][:,3:])
                contact=a['intentional_contact_count']>0
                actual=np.column_stack([a['interface_translation_error_m'],np.deg2rad(a['interface_rotation_error_deg']),np.linalg.norm(a['interface_relative_twist_world'][:,:3],axis=1),np.linalg.norm(a['interface_relative_twist_world'][:,3:],axis=1)])
                execution=np.column_stack([a['approach_position_error_m'],a['approach_rotation_error_rad']])
                tool_p=a['tool_position_world_m'];tool_R=a['tool_rotation_world'];flange_tw=a['flange_twist_world']
                tool_v=flange_tw[:,:3]+np.cross(flange_tw[:,3:],tool_p-a['flange_position_world_m'])
                tool_w=flange_tw[:,3:]
                original_guard_mask=a['phase_code']==3
            from v6_mujoco.adaptive_capture.known_model import compile_known
            from v6_mujoco.model import site_id
            cfg=read(OLD/'runs'/run/'config.json');m,d=compile_known(cfg['prior'],cfg['mission']['solver_tolerance'])
            quat=m.site_quat[site_id(m,'target_grasp_site')];Rbg=Rotation.from_quat(quat[[1,2,3,0]]).as_matrix()
        else:
            family=name[len('validation_'):].rsplit('_',1)[0]
            with np.load(OUT/'offline'/('signal_'+family)/'truth.npz') as a:truth={k:a[k] for k in a.files}
            contact=truth['contact'];actual=None;execution=None;original_guard_mask=None
        r=z['estimated_R']@offset;rt=truth['R']@offset
        pG=z['estimated_p']+r;pGt=truth['p']+rt
        vG=z['estimated_v']+np.cross(z['estimated_w'],r);vGt=truth['v']+np.cross(truth['w'],rt)
        err=np.column_stack([pG-pGt,z['error'][:,3:6],vG-vGt,z['error'][:,9:12]])
        content=dict(time=z['time'],target_geometry_error=z['error'],target_grasp_error=err)
        if actual is not None:
            est_rel=np.column_stack([np.linalg.norm(pG-tool_p,axis=1),Rotation.from_matrix(np.transpose(tool_R,(0,2,1))@(z['estimated_R']@Rbg)).magnitude(),np.linalg.norm(vG-tool_v,axis=1),np.linalg.norm(z['estimated_w']-tool_w,axis=1)])
            content.update(actual_tool_target_error=actual,counterfactual_estimated_relative_error=est_rel,
                controller_reference_execution_error=execution,original_guard_call_mask=original_guard_mask)
        first=np.flatnonzero(contact);contact_time=z['time'][first[0]] if len(first) else None
        valid=z['valid'];windows=dict(all_valid=valid,precontact=valid&~contact)
        if contact_time is not None:windows['contact_transition_100ms']=valid&(abs(z['time']-contact_time)<=.1)
        desc={k:components(err[mask]) for k,mask in windows.items() if np.any(mask)}
        dest=path.parent/'reference_point_diagnostics.npz';np.savez_compressed(dest,**content)
        summary[name]=dict(windows=desc,first_contact_s=contact_time,sha256=sha(dest),
            actual_new_guard_calls=0,original_guard_calls=int(sum(original_guard_mask)) if original_guard_mask is not None else 0,
            scope='offline estimate comparison; historical physical errors/execution are from original trajectory; new estimate relative errors are counterfactual')
    save(OUT/'reference_point_diagnostics.json',summary)
    return summary

def audit_completed_safety():
    """Apply the unchanged evaluator to every persisted completed step.

    This covers the last E0 step, whose subsequent estimator call failed before
    the online evaluator could run. Only a model is initialized; no integration.
    """
    from .persistence import records,validate
    from v6_mujoco.adaptive_capture.plant import Plant,TruthConfig
    from v6_mujoco.adaptive_capture.evaluation import safety
    from pathlib import Path
    attempts=read(OUT/'run_ledger.json')['attempts']
    dest=OUT/'completed_prefix_safety_audit.json'
    if dest.exists():
        cached=read(dest)
        if set(cached)=={a['name'] for a in attempts} and all(
            cached[a['name']].get('audit_code_sha256')==sha(Path(__file__)) and
            cached[a['name']]['raw_index_sha256']==sha(OUT/'runs'/a['name']/'raw/index.json')
            for a in attempts):return cached
    result={}
    arrays=lambda row:{k:np.array(v) if isinstance(v,list) else v for k,v in row.items()}
    for attempt in attempts:
        name=attempt['name'];folder=OUT/'runs'/name;checked=validate(folder/'raw')
        cfg=read(folder/'config.json')
        plant=Plant(TruthConfig(**cfg['truth_evaluation_only']),cfg['dt'],cfg['mission']['solver_tolerance'])
        first=None;violations=[];n=0;last=0.
        for r in records(folder/'raw'):
            if r['kind']=='initial_state':first=arrays(r['observation'])
            if r['kind'] not in ('initial_state','step_completed'):continue
            row=arrays(r['observation']);kind,detail=safety(row,first,plant.model)
            if kind:violations.append(dict(time=r['time'],category=kind,detail=detail))
            if r['kind']=='step_completed':n+=1;last=r['time']
        result[name]=dict(passed=not violations,completed_steps_checked=n,max_time_s=last,
            violations=violations,raw_index_sha256=checked['index_sha256'],
            audit_code_sha256=sha(Path(__file__)),
            new_physics_steps=0,scope='unchanged original safety evaluator on all committed completed observations, including exception endpoint')
    save(dest,result)
    return result

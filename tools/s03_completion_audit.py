"""Bounded publication audit of saved S03 evidence, identities and media."""
import sys
import time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from v6_mujoco.system_capture.planning.contracts import OUT,PROJECT,read,save,sha,LIMITS,used
from v6_mujoco.system_capture.planning.report import folder_for

def main():
    book=read(OUT/'run_ledger.json');names=['B00']+[r['name'] for r in book['attempts']];b=read(OUT/'candidate_budget.json')
    checks={};details={}
    checks['attempt_cap']=len(book['attempts'])<=12
    checks['candidate_cap']=b['evaluations']==len(b['entries'])<=1000
    checks['predictive_cap']=b['predictive_steps']+b['replay_predictive_steps']<=1000000 and b['reserved_steps']==0
    checks['cpu_cap']=used()<=43200
    checks['wall_cap']=time.time()-book['started_epoch']<=43200
    checks['two_implementations']=set(p.parent.name for p in (OUT/'versions').glob('*/identity.json'))=={'V1','V2'}
    v1=read(OUT/'versions/V1/planning_contract.json');v2=read(OUT/'versions/V2/planning_contract.json')
    checks['one_scientific_parameter_family']={k:v for k,v in v1.items() if k!='version'}=={k:v for k,v in v2.items() if k!='version'}
    checks['all_sources_recoverable']=all(x['recovery'] and sha(PROJECT/x['recovery'])==x['expected_sha256'] for changes in read(OUT/'source_recoverability.json')['changed_files_by_run'].values() for x in changes)
    checks['dual_replay']=all(read(OUT/'runs'/n/(kind+'_replay.json'))['passed'] for n in names for kind in ['actuator','decision'])
    safety={n:read(OUT/'diagnostics'/('safety_'+n+'.json')) for n in names}
    checks['all_completed_steps_safety_checked']=all(v['passed'] and v['checked_completed_steps']==v['expected_completed_steps'] for v in safety.values())
    details['safety_completed_steps']={n:v['checked_completed_steps'] for n,v in safety.items()}
    checks['H1_skip_explicit']=(OUT/'regression_skipped.json').exists()
    checks['historical_gaps_retained']=read(OUT/'qualification.json')['historical_source_gap']=='RETAINED' and read(OUT/'qualification.json')['noise_domain']=='NOT_ADMITTED'
    fairness={}
    for scene,a,c in [('nominal','P00','P01'),('H2','P02','P03')]:
        x=read(OUT/'runs'/a/'config.json');y=read(OUT/'runs'/c/'config.json')
        mx=dict(x['mission']);my=dict(y['mission']);mx.pop('s03_method');my.pop('s03_method')
        fairness[scene]=dict(same_mission_except_method=mx==my,same_truth=x['truth_evaluation_only']==y['truth_evaluation_only'],same_sensor=x['sensors']==y['sensors'],same_prior=x['prior']==y['prior'])
    details['paired_fairness']=fairness;checks['paired_fairness']=all(all(v.values()) for v in fairness.values())
    from v6_mujoco.system_capture.planning.report import measures,base_qpos_slice
    checks['base_joint_named_layout']=base_qpos_slice()==slice(7,14)
    with np.load(folder_for('B00')/'trace.npz') as z:
        bound=float(np.trapz(np.linalg.norm(z['base_omega_world_rad_s'],axis=1),z['time_s']))
    checks['base_drift_angular_path_bound']=measures('B00')['base_attitude_peak_rad']<=bound+1e-6
    q=read(OUT/'same_model_comparison.json');checks['failed_pair_scope']=all(p['full_task_relative_reduction_percent'] is None for scene in q['pairs'].values() for p in scene.values() if p['status']!='QUALIFIED_FULL_TASK_PAIR')
    prefix=q['pairs']['nominal']['joint']['baseline_metrics']
    checks['truncated_window_labels']=prefix['postgrasp_metrics_status']=='NOT_EVALUATED' and prefix['latch_time_s'] is None and prefix['detumbling'] is None and not prefix['full_task'] and prefix['run_outcome']['detumbling'] is True
    repeats={}
    for a,c in [('D00_R1','P01'),('D01','P03')]:
        with np.load(folder_for(a)/'trace.npz') as x,np.load(folder_for(c)/'trace.npz') as y:
            repeats[a+'/'+c]={k:bool(np.array_equal(x[k],y[k])) for k in ['time_s','qpos','qvel','ctrl_nm','estimate_covariance','progress_s']}
    details['deterministic_repeats_not_independent_samples']=repeats
    media={};countfig=countvid=0
    for n in names:
        dest=OUT/'visualizations/runs'/n;fm=read(dest/'figure_manifest.json');vm=read(dest/'video_manifest.json');m=read(folder_for(n)/'metrics.json')
        flags=dict(trace_sha=vm['trace_sha256']==sha(folder_for(n)/'trace.npz'),actual_end=abs(vm['physical_end_time_s']-m['end_time_s'])<1e-10,
            eight_videos=len(vm['videos'])==8,eleven_diagnostics=len(fm)==11,zero_new_physics=vm['physics_steps']==0,
            body_basis=vm['body_camera_max_orthogonality_error']<1e-6,
            tracking=read(dest/'tracking_consistency.json')['position_error_max_difference_m']<1e-10,
            hashes=all(sha(dest/(k+'.mp4'))==v['sha256'] for k,v in vm['videos'].items()) and all(sha(dest/(k+'.'+ext))==v[ext+'_sha256'] for k,v in fm.items() for ext in ['png','pdf']))
        media[n]=flags;countfig+=len(fm)+int((dest/'local_planning.pdf').exists());countvid+=len(vm['videos'])
    details['media']=media;checks['media']=all(all(v.values()) for v in media.values())
    countfig+=1+len(read(OUT/'visualizations/figures/scientific_manifest.json'))
    refresh=dict(stage='S03',evidence_entries=len(names),figures=countfig,videos=countvid,physics_steps=0,historical_media_regenerated=False,checks=media,
        sources={p.relative_to(PROJECT).as_posix():sha(p) for p in [PROJECT/'tools/s03_visualization.py',PROJECT/'tools/s03_scientific_figures.py',PROJECT/'tools/s02_visualization.py',PROJECT/'tools/s02_render.py']})
    save(OUT/'visualizations/refresh_manifest.json',refresh)
    save(OUT/'completion_audit.json',dict(passed=all(checks.values()),checks=checks,details=details,scope='saved evidence integrity and bounded negative-result delivery; no task success promotion'))
    print(dict(passed=all(checks.values()),failed=[k for k,v in checks.items() if not v],figures=countfig,videos=countvid))
    if not all(checks.values()):raise SystemExit(1)

if __name__=='__main__':main()

"""Independent read-only content/metric audit; creates only its own audit result."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from scipy.spatial.transform import Rotation
from v6_mujoco.system_capture.paper_bridge.common import OUT,PROJECT,read,save,sha
from v6_mujoco.system_capture.paper_bridge.phase import verify

def run():
    manifest=verify();ledger=read(OUT/'run_ledger.json');errors=[];checks={};missing=[]
    for kind in ['evaluations','dynamics','replays']:
        for row in ledger[kind]:
            if row['status'] not in ['COMPLETED','FAILED']:errors.append('unresolved '+row['key'])
            if sha(PROJECT/row['result_file'])!=row['result_sha256']:errors.append('result digest '+row['key'])
    for archive in (OUT/'source_versions').iterdir():
        # Archive directory names are content-map hashes; use the two retained manifests.
        candidates=[manifest]+[read(p) for p in (OUT/'manifest_history').glob('*.json')]
        identity=next((c['runtime_identity'] for c in candidates if c['runtime_hash']==archive.name),None)
        if identity is None:errors.append('unknown archived identity '+archive.name);continue
        for rel,h in identity.items():
            if sha(archive/rel)!=h:errors.append('source archive '+archive.name+'/'+rel)
    for row in ledger['dynamics']:
        folder=(PROJECT/row['result_file']).parent;m=read(folder/'metrics.json');path=folder/'trace.npz'
        if not path.exists():
            missing.append(row['key'])
            if m.get('status')!='IMPLEMENTATION_ERROR':errors.append('unexpected absent trace '+row['key'])
            continue
        identity=read(folder/'identity.json')
        if sha(path)!=identity['trace_sha256']:errors.append('trace digest '+row['key'])
        with np.load(path,allow_pickle=False) as z:a={k:z[k] for k in z.files}
        n=len(a['time_s']);checks[row['key']]={}
        def check(key,got,expected,atol=1e-10):
            delta=float(np.max(np.abs(np.asarray(got)-np.asarray(expected))))
            checks[row['key']][key]=delta
            if delta>atol:errors.append(row['key']+' '+key)
        if len(a['applied_ctrl'])!=n-1 or len(a['integration_dt_s'])!=n-1:errors.append('input/state length '+row['key'])
        if not all(np.isfinite(v).all() for v in a.values()):errors.append('nonfinite '+row['key'])
        if not np.all(np.diff(a['time_s'])>0):errors.append('nonmonotonic time '+row['key'])
        check('time_step_sequence',np.diff(a['time_s']),a['integration_dt_s'])
        check('initial_qpos',a['qpos'][0],identity['initial_state_qpos'])
        check('initial_qvel',a['qvel'][0],identity['initial_state_qvel'])
        check('position_definition',a['flange_position_world_m']-a['reference_position_world_m'],a['position_error_m'])
        re=Rotation.from_matrix(np.einsum('nij,nkj->nik',a['flange_rotation_world'],a['reference_rotation_world'])).as_rotvec()
        check('rotation_definition',re,a['rotation_error_rad'])
        w=np.linalg.norm(a['base_angular_velocity_world_rad_s'],axis=1);t=a['time_s']
        check('base_peak',w.max(),m['base_peak_angular_speed_rad_s'])
        check('base_rms',np.sqrt(np.trapz(w*w,t)/max(t[-1],1e-12)),m['base_rms_angular_speed_rad_s'])
        check('end_time',t[-1],m['end_time_s'])
        check('actual_applied_torque_peak',abs(a['applied_ctrl']).max(),m['actual_applied_peak_torque_nm'])
        for field,constraint in [('position_error_m','path_position_error_m'),('rotation_error_rad','path_rotation_error_rad')]:
            check(constraint,np.linalg.norm(a[field],axis=1).max(),m['constraints'][constraint]['value'])
        if m['completed_horizon']:
            check('complete_endpoint_time',t[-1],m['T_s'],1e-8)
        rp=read(OUT/'replays'/(row['key']+'.json'))
        if not rp['passed'] or rp['trace_sha256']!=sha(path):errors.append('replay '+row['key'])
    cells=[]
    for p in sorted((OUT/'optimization_runs').glob('*/result.json')):
        c=read(p);cell=p.parent.name;recorded=[r for r in ledger['evaluations'] if r['key'].startswith('C2_'+cell+'_')]
        if len(recorded)!=120 or c['evaluations']!=120:errors.append('unfair cell '+cell)
        possible=[r for r in recorded if r['parameters']['T']==c['best_position'][0] and r['parameters']['psi']==c['best_position'][1]]
        if not possible or not any(read(PROJECT/r['result_file'])==c['best_rollout'] for r in possible):errors.append('best is not a recorded evaluation '+cell)
        cells.append(cell)
    if len(cells)!=6:errors.append('missing optimization cell')
    if missing!=['author_point_source']:errors.append('unrecognized archival gaps')
    if len(ledger['dynamics'])>12 or len(ledger['evaluations'])>800:errors.append('budget exceeded')
    result={'passed_current_candidate_content_checks':not errors,'whole_campaign_archive_complete':not missing,
      'errors':errors,'retained_missing_original_traces':missing,'dynamics_checked':len(checks),'optimization_cells_checked':cells,
      'max_metric_discrepancies':checks,'evaluations':len(ledger['evaluations']),'dynamics_attempts':len(ledger['dynamics']),
      'S00_and_runtime_hashes_unchanged':True,'validator_sha256':sha(Path(__file__)),'scope':'Exact content identity, independent metric recomputation and original-trajectory linkage; no new physical attempt'}
    save(OUT/'independent_evidence_audit.json',result)
    print({k:v for k,v in result.items() if k!='max_metric_discrepancies'},flush=True)
    if errors:raise SystemExit(1)

if __name__=='__main__':run()

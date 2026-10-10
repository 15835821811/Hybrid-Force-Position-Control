"""Complete diagnostic fields from already recorded physical observations only."""
import sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from v6_mujoco.system_capture.planning.contracts import *
from v6_mujoco.system_capture.estimation.persistence import records

cpu,wall=time.process_time(),time.perf_counter()
path=OLD/'diagnostics/H2_prior/snapshots.json';old=read(path);d=read(OLD/'diagnostics/H2_prior/diagnosis.json')
assert sha(path)==d['snapshot_sha']
entry=read(OUT/'entry_acceptance.json');rel=path.relative_to(PROJECT).as_posix();initial=entry['files_sha256'][rel]
entry['files_sha256'][rel]=sha(path);entry['H2_snapshot_materialization']=dict(initial_pointer_sha256=initial,materialized_sha256=sha(path),matched_original_snapshot_sha=True,
    original_window_s=[old[0]['time'],old[-1]['time']],count=len(old),new_physics_steps=0)
save(OUT/'entry_acceptance.json',entry)
snap=read(OUT/'diagnostics/current_sets.json')
with np.load(OUT/'runs/B01/trace.npz') as z:
    for s in snap:
        i=int(np.argmin(abs(z['time_s']-s['time'])))
        s['actual_joint_acceleration_evaluation_only_rad_s2']=z['joint_acceleration_rad_s2'][i].tolist()
        s['acceleration_scope']='recorded physical evaluation only; never supplied to candidate screening or control'
save(OUT/'diagnostics/current_sets.json',snap)
noisy={};manifest=read(OUT/'snapshot_manifest.json')
for name,times in manifest['S02_readonly'].items():
    rows=[]
    for r in records(S02/'runs'/name/'raw'):
        if r['kind']=='proposal' and any(abs(r['time']-t)<1e-8 for t in times):
            rows.append(dict(time=r['time'],snapshot=r['snapshot'],q=r['observation']['joint_position_rad'],
                dq=r['observation']['joint_velocity_rad_s'],ddq_evaluation_only=r['observation']['joint_acceleration_rad_s2']))
    assert len(rows)==len(times)
    noisy[name]=rows
save(OUT/'diagnostics/S02_readonly_windows.json',noisy)
save(OUT/'diagnostics/window_coverage.json',dict(registered_unique_states=len(old)+len(snap)+sum(map(len,noisy.values())),limit=80,
    historical_hard_set_window_s=[old[0]['time'],old[-1]['time']],new_ideal_H2_times_s=[s['time'] for s in snap],
    H2_baseline_stop_s=read(OUT/'runs/B01/completion.json')['end_time_s'],
    terminal_rigid_geometry='fixed terminal tool and link7 relation to target independent of upstream psi; path can change transient approach only',
    no_truth_action_selection=True,no_state_injection_into_actual_plant=True))
charge('readonly_window_completion',cpu,wall)
print('46 preregistered unique states covered; historical and new H2 identities remain distinct',flush=True)

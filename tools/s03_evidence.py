"""Read-only extraction of finite-search coverage and signed named sensitivities."""
import csv
import argparse
import sys
import time
import numpy as np
import mujoco
from collections import Counter
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from v6_mujoco.system_capture.planning.contracts import OUT,read,save,sha,charge
from v6_mujoco.system_capture.planning.report import folder_for

def shape_trajectory(name):
    from v6_mujoco.adaptive_capture.plant import Plant,TruthConfig
    from v6_mujoco.geometry_capture.reference import shape_model
    from v6_mujoco.system_capture.estimation.persistence import records
    from v6_mujoco.adaptive_capture.evaluation import safety
    f=folder_for(name);cfg=read(f/'config.json');m=Plant(TruthConfig(**cfg['truth_evaluation_only']),cfg['dt'],cfg['mission']['solver_tolerance']).model;d=mujoco.MjData(m)
    with np.load(f/'trace.npz') as z:d.qpos[:]=z['qpos'][0];d.qvel[:]=z['qvel'][0]
    mujoco.mj_forward(m,d);shape=shape_model(m,d);rows=[];first=None;checked=0;violations=[]
    for r in records(f/'raw'):
        if r['kind']=='initial_state':first={k:np.asarray(v) if isinstance(v,list) else v for k,v in r['observation'].items()}
        if r['kind']=='step_completed':
            row={k:np.asarray(v) if isinstance(v,list) else v for k,v in r['observation'].items()}
            category,detail=safety(row,first,m);checked+=1
            if category:violations.append(dict(time_s=r['time'],category=category,detail=detail))
        if r['kind']!='proposal' or round(r['time']/.002)%10:continue
        obs=r['observation'];ref=r['snapshot']['controller'].get('reference')
        if not obs['approach_reference_applicable'] or not ref:continue
        d.qpos[:]=obs['qpos'];d.qvel[:]=obs['qvel'];mujoco.mj_forward(m,d);s=shape.sample(d)
        rows.append(dict(time_s=r['time'],actual_rad=s.angle_rad,reference_rad=ref['arm_angle_rad'],
            reference_velocity_rad_s=ref['arm_angle_velocity_rad_s'],reference_acceleration_rad_s2=ref['arm_angle_acceleration_rad_s2'],singular=s.singular))
    if rows:
        actual=np.unwrap([r['actual_rad'] for r in rows]);desired=np.unwrap([r['reference_rad'] for r in rows]);actual+=2*np.pi*round((desired[0]-actual[0])/(2*np.pi))
        for i,r in enumerate(rows):r.update(actual_rad=float(actual[i]),reference_rad=float(desired[i]))
    save(OUT/'diagnostics'/('shape_'+name+'.json'),dict(run=name,samples=rows,physics_steps=0,
        scope='read-only forward kinematics on saved actual qpos and recorded task reference; not controller feedback',
        source_raw_sha256=sha(f/'raw/index.json')))
    expected=read(f/'completion.json')['physical_steps']
    save(OUT/'diagnostics'/('safety_'+name+'.json'),dict(run=name,checked_completed_steps=checked,expected_completed_steps=expected,
        passed=checked==expected and not violations,violations=violations,physics_steps=0,
        scope='unchanged physical safety evaluator on every committed completed-step observation, including the terminal D00 prefix; does not prove future safety or recovery'))

def extract(defer_ledger=False):
    cpu,wall=time.process_time(),time.perf_counter()
    a=read(OUT/'candidate_authority.json');signed=[]
    for s in a['states']:
        base=s['results'][0];b={x['pair']:x for x in base['prediction']['named_minima']}
        for i,r in enumerate(s['results'][1:],1):
            dim=next(j for j,(x,y) in enumerate(zip(r['theta'],base['theta'])) if x!=y)
            for n in r['prediction']['named_minima']:
                m=b[n['pair']]
                # MuJoCo's absent/disabled-distance sentinel is not a length.
                if abs(n['margin_m'])>=1e6 or abs(m['margin_m'])>=1e6:continue
                signed.append(dict(time_s=s['time'],dimension=list(a['effects'])[dim],candidate=i,
                    sign='+' if r['theta'][dim]>base['theta'][dim] else '-',pair=n['pair'],
                    threshold_m=n['threshold_m'],baseline_margin_m=m['margin_m'],candidate_margin_m=n['margin_m'],
                    delta_margin_m=n['margin_m']-m['margin_m']))
    dest=OUT/'diagnostics/authority_signed_margins.csv'
    with dest.open('w',newline='',encoding='utf-8') as f:
        w=csv.DictWriter(f,fieldnames=list(signed[0]));w.writeheader();w.writerows(signed)
    book=read(OUT/'run_ledger.json');budget=read(OUT/'candidate_budget.json');byrun=Counter(x['run'] for x in budget['entries'])
    records={}
    for row in book['attempts']:
        name=row['name'];f=folder_for(name);g=read(f/'progress_governor.json');m=read(f/'metrics.json')
        terminal=g[-1] if g else {};candidates=[];sequence=[]
        for r in g:
            cs=r['candidates'];chosen=r.get('selected');chosen_c=next((x for x in cs if x.get('index')==chosen),None) if chosen is not None else None
            sequence.append(dict(time_s=r['time'],selected=chosen,chosen_fraction=r.get('chosen_fraction'),
                arrival_s=chosen_c['reference']['arrival'] if chosen_c else None,
                theta=chosen_c['theta'] if chosen_c else None,
                screen_count=sum(bool(c.get('screen')) for c in cs),dynamic_count=sum('prediction' in c for c in cs),
                cache_valid_until=r.get('valid_until'),failure=r.get('failure')))
        for c in terminal.get('candidates',[]):
            screen=c.get('screen') or {};p=c.get('prediction',c if 'fraction' in c else {})
            named=[n for n in p.get('named_minima',[]) if abs(n['margin_m'])<1e6]
            candidates.append(dict(index=c.get('index'),theta=c.get('theta'),fraction=c.get('fraction'),
                jet_valid=c.get('jet_valid'),jet_reason=c.get('reason') if 'theta' in c else None,
                screening=screen,dynamic_evaluated=bool(p),prediction={k:p[k] for k in ['verified','reason','steps','horizon_reached_s','max_tracking_error_m','max_linear_residual_m_s','max_angular_residual_rad_s','min_linear_constraint_slack_mixed_units','max_rho','peak_base_rad_s','terminal_progress'] if k in p},
                smallest_finite_named_margins=sorted(named,key=lambda x:x['margin_m'])[:8]))
        records[name]=dict(status=row['status'],actual_end_time_s=row['end_time_s'],actual_guard_calls=m['actual_guard_calls'],
            last_candidate_record_time_s=terminal.get('time'),last_candidates_are_terminal_evaluation=abs(terminal.get('time',-1)-row['end_time_s'])<1e-8,
            actual_safety_violations=m['actual_safety_violations'],postgrasp_window=m['final_window_status'],
            named_evaluations=byrun[name],planning_sequence=sequence,last_candidates=candidates,
            stop_scope='external persistence error; see original traceback' if row['status']=='IMPLEMENTATION_ERROR' else 'finite implemented policy; no universal infeasibility certificate',
            source=dict(governor_sha256=sha(f/'progress_governor.json'),completion_sha256=sha(f/'completion.json')))
    save(OUT/'search_coverage.json',dict(runs=records,named_evaluations_by_run=dict(byrun),
        family='five deterministic members per planning event, at most two full prior rollouts; B2 couples path/shape in two members',
        parameter_box_exhausted=False,current_terminal_hardset_certified=False,
        current_set_scope='independent B01 certificates at 12 preregistered times through 5.8 s; historical H2 certificates separate',
        signed_authority=dict(file=dest.relative_to(OUT).as_posix(),sha256=sha(dest),states=4,forecast_horizon_s=.6,
            sentinel_policy='abs(margin)>=1e6 excluded; missing distances are not physical margins',
            sign='positive is greater predicted minimum distance for that pair; maxima across pairs are not simultaneous gains')))
    for name in ['B00']+[r['name'] for r in book['attempts']]:shape_trajectory(name)
    if defer_ledger:save(OUT/'evidence_operation_receipt.json',dict(kind='evidence_extraction',cpu_s=time.process_time()-cpu,wall_s=time.perf_counter()-wall,new_physics_steps=0))
    else:charge('evidence_extraction',cpu,wall,new_physics_steps=0)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--defer-ledger',action='store_true');extract(p.parse_args().defer_ledger)

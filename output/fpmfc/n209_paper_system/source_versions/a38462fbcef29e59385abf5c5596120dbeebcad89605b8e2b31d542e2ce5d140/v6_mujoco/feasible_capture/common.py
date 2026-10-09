import hashlib
import json
import os
import time
from pathlib import Path
from v6_mujoco.model import PROJECT_ROOT

ROOT = PROJECT_ROOT / 'output/fpmfc/n209_paper_system'
OLD = PROJECT_ROOT / 'output/fpmfc/n208_adaptive_capture'
PAPER = PROJECT_ROOT / 'paper/system_paper'
BASE = 'b1af09f89226fa6f2ae1b362f3d6225b3363cc17'

def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda: f.read(1024 * 1024), b''):
            h.update(b)
    return h.hexdigest()

def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))

def save(path, obj):
    from v6_mujoco.adaptive_capture.runner import jsonable
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(jsonable(obj), ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    os.replace(tmp, path)

def identity():
    paths = list((PROJECT_ROOT/'v6_mujoco').rglob('*.py'))
    paths += list((PROJECT_ROOT/'configs').rglob('*.yaml'))
    paths += list((PROJECT_ROOT/'models').glob('*.xml'))
    paths += list((PROJECT_ROOT/'assets/meshes').rglob('*.stl'))
    return {p.relative_to(PROJECT_ROOT).as_posix():sha(p) for p in sorted(paths)}

def ledger():
    p = ROOT/'run_ledger.json'
    if not p.exists():
        save(p, {'started_epoch':time.time(), 'cpu_s':0., 'attempts':[], 'operations':[], 'frozen':False,
                 'limits':{'development':8, 'regression':4, 'holdout':36, 'ablation':6, 'step_sensitivity':2, 'implementation_reserve':4},
                 'cpu_budget_s':28800, 'wall_budget_s':28800})
    return read(p)

def charge(kind, cpu, wall, detail):
    book=ledger(); book['cpu_s']+=cpu
    book['operations'].append({'kind':kind, 'cpu_s':cpu, 'wall_s':wall, 'detail':detail, 'epoch':time.time()})
    save(ROOT/'run_ledger.json', book)

def prepare():
    ROOT.mkdir(parents=True, exist_ok=True); PAPER.mkdir(parents=True, exist_ok=True); ledger()
    if (ROOT/'source_manifest.json').exists():
        manifest=read(ROOT/'source_manifest.json')
        for rel,item in manifest['sources'].items():
            if rel.startswith('output/fpmfc/n208_') or rel=='paper/N208_PASSIVE_TARGET_ESTIMATION_CAPTURE_REPORT.md':
                if sha(PROJECT_ROOT/rel)!=item['sha256']:raise RuntimeError('Historical evidence changed: '+rel)
        print('Existing source manifest preserved; historical byte identities verified.');return
    required=['qualification_matrix.json','comparison.json','run_ledger.json','experiment_manifest.json',
              'requirements_and_parameter_policy.md','state_estimation_summary.json','identifiability_report.json',
              'truth_access_audit.json','momentum_feasibility.json']
    paths=[OLD/p for p in required]+[PROJECT_ROOT/'paper/N208_PASSIVE_TARGET_ESTIMATION_CAPTURE_REPORT.md']
    for name in ['D06_final_gain3','D05_final_nominal','H1_prior','H1_identified','H2_prior','H2_identified','H3_prior','H3_identified','S01_noise_delay']:
        paths += [p for p in (OLD/'runs'/name).iterdir() if p.is_file()]
        paths += list((OLD/'source_versions'/name).glob('*.py'))
    sources=['controller','state_estimator','relative_reference','governor','inertial_estimator','information_gate','known_model']
    paths += [PROJECT_ROOT/('v6_mujoco/adaptive_capture/'+p+'.py') for p in sources]
    paths += [PROJECT_ROOT/p for p in ['v6_mujoco/fpmfc/controller.py','v6_mujoco/hierarchical_qp.py','v6_mujoco/geometry_capture/planning.py','v6_mujoco/collision.py']]
    entries={}
    for p in paths:
        with p.open('rb') as f: head=f.read(100)
        if head.startswith(b'version https://git-lfs.github.com/spec'):raise RuntimeError('unhydrated LFS '+str(p))
        entries[p.relative_to(PROJECT_ROOT).as_posix()]={'sha256':sha(p),'bytes':p.stat().st_size}
    save(ROOT/'source_manifest.json',{'base_commit':BASE,'sources':entries,'scope':'historical evidence, read only; no historical requalification'})
    save(ROOT/'baseline_contract.json',{'base_commit':BASE,'B0':'N208, feedback disabled; any common numerical repair explicitly shared',
         'B1':'B0 plus audited timing/reference interface and predictive progress governor',
         'B2':'B1 without predictive progress governor','historical_seen':['D06','D05','H1','H2','H3','S01'],
         'capture_limits':[0.0001,0.05,0.001,0.2],'capture_units':['m','deg','m/s','deg/s'],
         'hold_limits':[0.0005,0.1],'hold_units':['m','deg'],'load':'norm(F)/50+norm(M)/2 <= 1',
         'approach_deadline_s':20,'post_duration_s':20,'final_window_s':2,
         'new_robot_attempts':0,'no_push':True,'N210_N211':'PLAN_ONLY'})
    print(json.dumps({'prepared_sources':len(entries),'root':str(ROOT)}))

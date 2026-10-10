"""S01 bounded lifecycle and content identities, independent of old ledgers."""
import contextlib
import os
import platform
import shutil
import sys
import time
from pathlib import Path
import mujoco
import numpy as np
import scipy
from .common import *

ACTIVE_CPU_START=None
def book():return read(OUT/'run_ledger.json')
def resource_used():
    return sum(x['cpu_s'] for x in book()['operations'])+(time.process_time()-ACTIVE_CPU_START if ACTIVE_CPU_START is not None else 0)
def remaining_cpu():return max(0,read(PROTOCOL)['budgets']['cpu_s']-resource_used())
def runtime_identity():
    from . import control,evaluate,tests,registry,optimize,replay
    paths=set(Path(mod.__file__).resolve() for mod in list(sys.modules.values()) if getattr(mod,'__file__',None) and str(Path(mod.__file__).resolve()).startswith(str(PROJECT/'v6_mujoco')) and Path(mod.__file__).suffix=='.py')
    paths|={p for p in Path(__file__).parent.glob('*.py') if p.name not in ('report.py','figures.py')}
    paths|={PROTOCOL,*CONFIG.glob('*.yaml'),PROJECT/'models/paper_compat/srs.xml',Path(__file__).parent/'source_ambiguities.md'}
    return {p.relative_to(PROJECT).as_posix():sha(p) for p in sorted(paths)}
def environment():return {'python':platform.python_version(),'python_executable':sys.executable,'mujoco':mujoco.__version__,'numpy':np.__version__,'scipy':scipy.__version__,'platform':platform.platform(),'logical_cpus':os.cpu_count(),'processor':platform.processor(),'threads':{k:os.environ.get(k) for k in ['OPENBLAS_NUM_THREADS','OMP_NUM_THREADS']}}
def prepare():
    if git('rev-parse','HEAD')!=BASE and git('merge-base',BASE,'HEAD')!=BASE:raise RuntimeError('wrong baseline')
    if git('branch','--show-current')!=read(PROTOCOL)['branch']:raise RuntimeError('wrong S01 branch')
    OUT.mkdir(parents=True,exist_ok=True)
    if (OUT/'phase_manifest.json').exists():verify();print('S01 already frozen');return
    from .model import generate
    from .registry import models,algorithms
    generate();s=source();assert sha(PROJECT/s['source']['file'])==s['source']['sha256']
    if not (OUT/'run_ledger.json').exists():save(OUT/'run_ledger.json',{'evaluations':[],'dynamics':[],'replays':[],'operations':[],'candidate':'C1_frozen_source_compatible_residual_hierarchy','new_flexiv_attempts':0,'next_phases_started':[]})
    frozen=runtime_identity();archive=OUT/'source_versions'/digest(frozen)
    for rel,h in frozen.items():
        p=archive/rel;p.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(PROJECT/rel,p)
    old_inputs={p.relative_to(PROJECT).as_posix():sha(p) for p in (PROJECT/'output/fpmfc/system_capture/S00').rglob('*') if p.is_file()}
    old_inputs[s['source']['file']]=s['source']['sha256']
    save(OUT/'phase_manifest.json',{'phase':'S01','input_commit':BASE,'branch':git('branch','--show-current'),'protocol':read(PROTOCOL),'runtime_identity':frozen,'runtime_hash':digest(frozen),'source_archive':archive.relative_to(PROJECT).as_posix(),'environment':environment(),'read_only_input_hashes':old_inputs,'frozen_at_epoch':time.time(),'stage_scope':'SRS precontact only; historical S00 and Flexiv untouched'})
    shutil.copyfile(Path(__file__).parent/'source_ambiguities.md',OUT/'source_ambiguities.md')
    save(OUT/'source_contract.json',{'source':s,'assumptions':assumptions(),'pdf_visual_review':{'pdf_pages':[4,5,6,7,8,9,10,11],'printed_pages':[961,962,963,964,965,966,967,968],'method':'pdftoppm page images and text independently inspected; tables and formulas checked against images'},'source_parameter_completeness':'SOURCE_LIMITED','classification':'SOURCE_COMPATIBLE_UNDER_ASSUMPTIONS','config_hashes':{p.name:sha(p) for p in CONFIG.glob('*.yaml')}})
    dest=PROJECT/'experiments/system_capture/registry/S01';save(dest/'models.json',models());save(dest/'algorithms.json',algorithms())
    print({'prepared':'S01','runtime_hash':digest(frozen),'budget':read(PROTOCOL)['budgets']},flush=True)
def verify():
    m=read(OUT/'phase_manifest.json')
    for p,h in m['runtime_identity'].items():
        if sha(PROJECT/p)!=h:raise RuntimeError('frozen S01 runtime changed: '+p)
    for p,h in m['read_only_input_hashes'].items():
        if sha(PROJECT/p)!=h:raise RuntimeError('historical input changed: '+p)
    if environment()!=m['environment']:raise RuntimeError('frozen environment changed')
    return m
def reserve(kind,key,parameters):
    b=book();items=b[kind];found=next((r for r in items if r['key']==key),None)
    if found:
        if found['parameters']!=plain(parameters):raise RuntimeError('resume parameters changed '+key)
        if found['status'] not in ('COMPLETED','FAILED'):raise RuntimeError('unresolved existing attempt; never restart '+key)
        if sha(PROJECT/found['result_file'])!=found['result_sha256']:raise RuntimeError('resume result digest '+key)
        return found,False
    caps={'evaluations':800,'dynamics':12,'replays':12}
    if len(items)>=caps[kind] or remaining_cpu()<=0:raise RuntimeError('BUDGET_EXHAUSTED')
    row={'key':key,'parameters':plain(parameters),'status':'RUNNING','pid':os.getpid(),'started_epoch':time.time(),'runtime_hash':read(OUT/'phase_manifest.json')['runtime_hash']}
    items.append(row);save(OUT/'run_ledger.json',b);return row,True
def finish(kind,key,path,status):
    b=book();r=next(x for x in b[kind] if x['key']==key);r.update(status=status,result_file=path.relative_to(PROJECT).as_posix(),result_sha256=sha(path),finished_epoch=time.time());save(OUT/'run_ledger.json',b)
def evaluation(key,T,psi,dt=None):
    from .evaluate import kinematic,Evaluation
    parameters={'T':float(T),'psi':float(psi),'dt':float(dt or assumptions()['optimization']['planning_dt_s'])};r,new=reserve('evaluations',key,parameters)
    if not new:
        path=PROJECT/r['result_file'];result=read(path)
        with np.load(path.with_suffix('.npz'),allow_pickle=False) as a:trace={k:a[k] for k in a.files}
        return Evaluation(result,trace)
    path=OUT/'optimization_runs/evaluations'/(key+'.json');result=kinematic(T,psi,dt,remaining_cpu());save(path,result.metrics);np.savez_compressed(path.with_suffix('.npz'),**result.trace)
    finish('evaluations',key,path,'COMPLETED' if result.feasible else 'FAILED');return result
def dynamic(key,T,psi,method='source_nullspace',dt=None):
    from .evaluate import dynamics
    parameters={'T':float(T),'psi':float(psi),'method':method,'dt':float(dt or assumptions()['controller']['physics_dt_s'])}
    # Exact duplicates, including failed conditions, are referenced without rerun.
    old=next((r for r in book()['dynamics'] if r['parameters']==parameters),None)
    if old:
        if old['status'] not in ('COMPLETED','FAILED'):raise RuntimeError('pending duplicate '+old['key'])
        return old['key'],read(PROJECT/old['result_file'])
    row,new=reserve('dynamics',key,parameters)
    if not new:return key,read(PROJECT/row['result_file'])
    print('S01 dynamics start '+key,flush=True)
    path=OUT/'dynamics_runs'/key/'metrics.json';result=dynamics(T,psi,method,dt,remaining_cpu());save(path,result.metrics);np.savez_compressed(path.parent/'trace.npz',**result.trace)
    save(path.parent/'identity.json',{'parameters':parameters,'runtime_hash':row['runtime_hash'],'trace_sha256':sha(path.parent/'trace.npz'),'initial_state_qpos':result.trace['qpos'][0].tolist(),'initial_state_qvel':result.trace['qvel'][0].tolist()})
    finish('dynamics',key,path,'COMPLETED' if result.feasible else 'FAILED');print('S01 dynamics '+key+' '+result.metrics['status']+' feasible='+str(result.feasible),flush=True);return key,result.metrics
def run_all(resume=False):
    verify()
    if not read(OUT/'equation_tests.json')['passed'] or not read(OUT/'model_checks.json')['passed']:raise RuntimeError('mathematical/model tests required')
    if not resume and (book()['evaluations'] or book()['dynamics']):raise RuntimeError('--resume required for existing work')
    from .optimize import run
    run()
def replay_all(resume=False):
    verify()
    if not resume and book()['replays']:raise RuntimeError('--resume required')
    from .replay import run
    run()
@contextlib.contextmanager
def operation(mode):
    global ACTIVE_CPU_START
    OUT.mkdir(parents=True,exist_ok=True);lock=OUT/'operation.lock';fd=os.open(lock,os.O_CREAT|os.O_EXCL|os.O_WRONLY);os.write(fd,str(os.getpid()).encode());os.close(fd)
    wall=time.perf_counter();ACTIVE_CPU_START=time.process_time()
    try:yield
    finally:
        if (OUT/'run_ledger.json').exists():
            b=book();b['operations'].append({'mode':mode,'pid':os.getpid(),'cpu_s':time.process_time()-ACTIVE_CPU_START,'wall_s':time.perf_counter()-wall,'scope':'entire single process; per-evaluation CPU is a subset, not added again'});save(OUT/'run_ledger.json',b)
        ACTIVE_CPU_START=None;lock.unlink()
def main(mode,resume=False):
    with operation(mode):
        if mode=='prepare':prepare()
        elif mode=='test':
            verify();from .tests import run
            if not run():return 1
        elif mode=='run':run_all(resume)
        elif mode=='replay':replay_all(resume)
        elif mode=='report':
            verify();from .report import report
            return 0 if report() else 1
    return 0

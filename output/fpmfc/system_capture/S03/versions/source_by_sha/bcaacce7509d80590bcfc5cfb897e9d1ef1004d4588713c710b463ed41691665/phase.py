"""S03-only gates. A failed task is retained; no implicit retuning or next phase."""
import os
import time
import contextlib
from .contracts import *

RUNTIME_FILES=['parameterization.py','trajectory_segment.py','screening.py','prediction.py','planner.py','controller_adapter.py']

def freeze():
    p=read(OUT/'planning_contract.json');version=p['version'];path=OUT/'versions'/version/'identity.json'
    identity_now={f'v6_mujoco/system_capture/planning/{n}':sha(PROJECT/'v6_mujoco/system_capture/planning'/n) for n in RUNTIME_FILES}
    identity_now.update({n:sha(PROJECT/n) for n in ['v6_mujoco/system_capture/adapters.py','v6_mujoco/system_capture/estimation/runner.py','v6_mujoco/system_capture/estimation/replay.py']})
    doc=dict(version=version,source=identity_now,contract=sha(OUT/'planning_contract.json'))
    if path.exists():
        if read(path)!=doc:raise RuntimeError('frozen version changed; explicit implementation amendment required')
    else:
        import shutil
        for rel in identity_now:
            dest=path.parent/rel;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(PROJECT/rel,dest)
        save(path.parent/'planning_contract.json',p);save(path,doc)
    return doc

@contextlib.contextmanager
def operation_lock():
    path=OUT/'operation.lock';fd=os.open(path,os.O_CREAT|os.O_EXCL|os.O_WRONLY);os.write(fd,str(os.getpid()).encode());os.close(fd)
    try:yield
    finally:path.unlink()

def run(resume=False):
    from ..estimation.runner import run_one
    if not read(OUT/'module_tests.json')['passed'] or read(OUT/'candidate_authority.json')['status']!='PASS':raise RuntimeError('module/authority gates required')
    if not (OUT/'runs/B00/reused_result.json').exists():raise RuntimeError('B0 compatibility required')
    freeze();ctx=Context()
    with operation_lock():
        for name in ['D00','D01','P00','P01','P02','P03','R00']:
            book=read(OUT/'run_ledger.json');attempts={x['name']:x for x in book['attempts']}
            if name in attempts:
                if not resume:raise FileExistsError(name)
                if attempts[name]['status'] in ('IMPLEMENTATION_ERROR','PERSISTENCE_ERROR','REGISTERED_BEFORE_DYNAMICS','RUNNING'):raise RuntimeError('implementation failure retained; repair dependency before continuing')
                continue
            if any(x['actual_safety_violations'] for x in attempts.values()):
                save(OUT/'run_admission_stop.json',dict(reason='ACTUAL_SAFETY_VIOLATION',not_run=name));break
            if name=='R00':
                admitted=all(read(OUT/'runs'/n/'metrics.json')['postgrasp_detumbling'] for n in ['P01','P03'])
                if not admitted:
                    save(OUT/'regression_skipped.json',dict(run='R00',reason='nominal/H2 B2 full task admission not achieved'));break
            result=run_one(name,ctx)
            if result['status'] in ('IMPLEMENTATION_ERROR','PERSISTENCE_ERROR','INCOMPLETE_BUDGET'):break
    return True

def replay(resume=False):
    from ..estimation.replay import replay_one
    ctx=Context()
    with operation_lock():
        for e in read(OUT/'run_ledger.json')['attempts']:
            for kind in ['actuator','decision']:
                dest=OUT/'runs'/e['name']/(kind+'_replay.json')
                if dest.exists():
                    if not resume:raise FileExistsError(dest)
                    if not read(dest)['passed']:return False
                    continue
                if not replay_one(e['name'],kind,ctx)['passed']:return False
    return True

def main(mode,resume=False):
    if mode=='prepare':print(prepare());return 0
    if mode=='test':
        from .tests import run_tests
        if not run_tests()['passed']:return 1
        from .diagnostics import run_authority
        if (OUT/'runs/B01/completion.json').exists():return 0 if run_authority()['status']=='PASS' else 1
        print('B0 compatibility and H2 baseline pending; no formal planning admission');return 0
    if mode=='run':return 0 if run(resume) else 1
    if mode=='replay':return 0 if replay(resume) else 1
    if mode=='report':
        from .report import report
        report();return 0
    raise ValueError(mode)

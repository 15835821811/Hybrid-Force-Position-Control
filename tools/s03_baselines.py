"""Exact-byte C2 compatibility gate, then the one new B0 H2 attempt."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from v6_mujoco.system_capture.planning.contracts import *
from v6_mujoco.system_capture.estimation.replay import replay_one
from v6_mujoco.system_capture.estimation.runner import run_one

def main():
    context=Context();f=S02/'runs/E0_C2';c=read(f/'config.json')
    # Runtime modules are original frozen bytes; only orchestration injection
    # points/CLI differ, and the full decision replay tests their equivalence.
    changed={p:dict(expected=h,current=sha(PROJECT/p)) for p,h in c['source_identity'].items() if sha(PROJECT/p)!=h}
    allowed={'v6_mujoco/system_capture/estimation/runner.py','v6_mujoco/system_capture/estimation/replay.py','v6_mujoco/system_capture/cli.py'}
    if set(changed)-allowed:raise RuntimeError('B0 frozen code differs: '+str(set(changed)-allowed))
    save(OUT/'B0_source_compatibility.json',dict(frozen_algorithm_digest=digest(c['source_identity']),source='per-E0_C2 runtime SHA, not publication commit',
        changed_orchestration_only=changed,unchanged_files=len(c['source_identity'])-len(changed),requires_full_decision_equivalence=True))
    for kind in ('actuator','decision'):
        path=OUT/'runs/B00'/(kind+'_replay.json')
        result=read(path) if path.exists() else replay_one('B00',kind,context,source_folder=f)
        if not result['passed']:raise RuntimeError('B00 compatibility failed')
    save(OUT/'runs/B00/reused_result.json',dict(source=f.relative_to(PROJECT).as_posix(),metrics_sha256=sha(f/'metrics.json'),
        config_sha256=sha(f/'config.json'),raw_sha256=sha(f/'raw/index.json'),new_physical_attempt=False,
        metrics=read(f/'metrics.json'),dual_replay=True))
    b=read(OUT/'run_ledger.json')
    if not b['reused']:b['reused']=[dict(name='B00',source='S02/E0_C2',strict_dual_replay=True)];save(OUT/'run_ledger.json',b)
    if not any(x['name']=='B01' for x in b['attempts']):run_one('B01',context)

if __name__=='__main__':main()

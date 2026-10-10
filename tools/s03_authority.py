import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from v6_mujoco.system_capture.planning.contracts import *
from v6_mujoco.system_capture.planning.diagnostics import run_authority

def account_baseline():
    p=OUT/'candidate_budget.json';b=read(p)
    if b.get('baseline_accounted'):return
    d=read(OUT/'runs/B01/completion.json');log=read(OUT/'runs/B01/progress_governor.json')
    for row in log:
        for candidate in row['candidates']:
            b['evaluations']+=1;b['entries'].append(dict(id=b['evaluations'],run='B01',method='B0_scalar',time_s=row['time'],theta=[candidate['fraction']],model='independent_frozen_prior_dynamics'))
    b['predictive_steps']+=d['predictive_steps'];b['replay_predictive_steps']+=read(OUT/'runs/B00/decision_replay.json')['predictive_steps']
    # B00's completed first safe brake calls use 3*10 steps each. This is
    # derived from the unchanged fixed loop and source log, not a new sample.
    import numpy as np
    with np.load(S02/'runs/E0_C2/trace.npz') as z:
        ticks=np.rint(z['time_s']/.002).astype(int)%10==0;post=z['holding_error_applicable'].astype(bool)
        assert np.all(z['brake_alpha'][post]==1.), 'cannot infer short-circuited brake predictions'
        latch=read(S02/'runs/E0_C2/completion.json')['latch_time_s']
        calls=int(np.count_nonzero(ticks & post))+int(round(latch/.002)%10!=0)+1
    b['replay_predictive_steps']+=calls*30
    b['baseline_accounted']=True;b['B00_brake_steps_scope']=dict(steps=calls*30,calls=calls,source='recorded alpha=1, task/latch times and frozen 3-model/10-step loop; derived, not direct instrumentation')
    save(p,b)

if __name__=='__main__':
    account_baseline();print({k:v for k,v in run_authority().items() if k not in ('states',)},flush=True)

import argparse
import subprocess
import sys
from .common import ROOT,read,save,prepare,ledger

def main():
    p=argparse.ArgumentParser();g=p.add_mutually_exclusive_group(required=True)
    for op in ['prepare','audit-existing','develop','regression','freeze','validate-holdout','replay-only','paper','all','test']:g.add_argument('--'+op,action='store_true')
    p.add_argument('--resume',action='store_true');p.add_argument('--budget',type=int,default=8)
    p.add_argument('--name');p.add_argument('--sensor',default='ideal');p.add_argument('--scenario',default='nominal');p.add_argument('--method',default='B1',choices=['B0','B1','B2']);a=p.parse_args()
    if a.prepare:prepare();return
    if a.audit_existing:
        for n in ['H2_prior','S01_noise_delay','D05_final_nominal']:
            if a.resume and (ROOT/'diagnostics'/n/'diagnosis.json').exists():continue
            subprocess.run([sys.executable,'-m','v6_mujoco.feasible_capture.qp_diagnostics',n],check=True)
        return
    if a.test:
        from .selftest import run
        run();return
    if a.develop or a.regression:
        if not a.name:p.error('--name required; attempts cannot be overwritten')
        if a.budget>8:p.error('development budget cannot exceed8')
        from .benchmark import run
        run(a.name,a.sensor,a.method,a.scenario,'development' if a.develop else 'regression');return
    if a.replay_only:
        from .benchmark import replay
        for x in ledger()['attempts']:
            if a.name and x['name']!=a.name:continue
            if a.resume and (ROOT/'runs'/x['name']/'validation.json').exists():continue
            replay(x['name'])
        return
    if a.freeze or a.validate_holdout:
        raise SystemExit('NOT_ADMITTED: final selected nominal/H2/S01/H1 regressions must pass; inspect qualification_matrix.json')
    if a.paper:
        from .report import run
        run();return
    if a.all:
        prepare()
        subprocess.run([sys.executable,'-m','v6_mujoco.feasible_capture','--audit-existing','--resume'],check=True)
        subprocess.run([sys.executable,'-m','v6_mujoco.feasible_capture','--test'],check=True)
        raise SystemExit('DEVELOPMENT_CHECKPOINT: resume existing ledger; explicit version selection required before dependent validation')

if __name__=='__main__':main()

import argparse
import subprocess
import sys
from .common import ROOT,read,save,prepare,ledger

def main():
    p=argparse.ArgumentParser();g=p.add_mutually_exclusive_group(required=True)
    for op in ['prepare','audit-existing','develop','regression','freeze','validate-holdout','replay-only','paper','all','test','visualize']:g.add_argument('--'+op,action='store_true')
    p.add_argument('--resume',action='store_true');p.add_argument('--budget',type=int,default=8)
    p.add_argument('--name');p.add_argument('--validation-tag',default='');p.add_argument('--sensor',default='ideal');p.add_argument('--scenario',default='nominal');p.add_argument('--method',default='B1',choices=['B0','B1','B2']);a=p.parse_args()
    if a.prepare:prepare();return
    if a.visualize:
        from .visualization import run,SELECTED
        run(a.name or SELECTED,a.resume);return
    if a.audit_existing:
        for n in ['H2_prior','S01_noise_delay','D05_final_nominal']:
            if a.resume and (ROOT/'diagnostics'/n/'diagnosis.json').exists():continue
            subprocess.run([sys.executable,'-m','v6_mujoco.feasible_capture.qp_diagnostics',n],check=True)
        return
    if a.test:
        from .selftest import run
        run();return
    if a.develop or a.regression:
        if not a.name:
            if a.regression:
                from .campaign import regression_resume
                regression_resume();return
            print('DEVELOPMENT_CLOSED: two recorded candidate versions; selected V2. Existing failures are not retried.');return
        if a.budget>8:p.error('development budget cannot exceed8')
        if a.develop and sum(x['category']=='development' for x in ledger()['attempts'])>=a.budget:p.error('requested development budget reached')
        from .benchmark import run
        run(a.name,a.sensor,a.method,a.scenario,'development' if a.develop else 'regression');return
    if a.replay_only:
        from .benchmark import replay
        if not a.name:
            from .campaign import replay_resume
            replay_resume();return
        for x in ledger()['attempts']:
            if a.name and x['name']!=a.name:continue
            filename='validation'+('_'+a.validation_tag if a.validation_tag else '')+'.json'
            if a.resume and (ROOT/'runs'/x['name']/filename).exists():continue
            replay(x['name'],a.validation_tag)
        return
    if a.freeze or a.validate_holdout:
        from .campaign import freeze
        result=freeze();print(result['status']);return
    if a.paper:
        from .report import run
        run();return
    if a.all:
        from .campaign import all_resume
        all_resume();return

if __name__=='__main__':main()

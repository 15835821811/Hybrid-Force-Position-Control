"""Explicit admission and bounded N206 continuous attempts."""
import argparse
from .common import *

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--prepare',action='store_true');parser.add_argument('--all',action='store_true');parser.add_argument('--scenario',choices=config()['scenario_order']);parser.add_argument('--resume',action='store_true');parser.add_argument('--validate-only',action='store_true');args=parser.parse_args()
    if args.prepare:prepare()
    from .runner import run
    from .validator import validate,sensitivity
    if args.validate_only:
        for entry in read(ROOT/'run_ledger.json')['formal_runs']:validate(entry['scenario'])
    elif args.scenario:
        run(args.scenario,args.resume);validate(args.scenario)
    elif args.all:
        for scene in config()['scenario_order']:
            if scene!='nominal' and not read(ROOT/'nominal/metrics.json')['end_to_end_passed']:
                ledger=read(ROOT/'run_ledger.json');ledger['skipped'][scene]='NOT_RUN_NOMINAL_FAILURE';save(ROOT/'run_ledger.json',ledger);save(ROOT/scene/'skipped.json',{'status':'NOT_RUN_NOMINAL_FAILURE','formal_trajectory_count':0});continue
            result=run(scene,args.resume)
            if not (ROOT/scene/'validation.json').exists():validate(scene)
        if all((ROOT/s/'metrics.json').exists() and read(ROOT/s/'metrics.json')['end_to_end_passed'] for s in ['nominal','fine']):sensitivity()

if __name__=='__main__':main()

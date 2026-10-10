"""N205 unified runner with explicit stage admission and immutable resume."""
import argparse
from .common import *

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("--prepare",action="store_true");p.add_argument("--scenario",choices=("nominal","fine","light","heavy"));p.add_argument("--all",action="store_true");p.add_argument("--resume",action="store_true");p.add_argument("--validate-only",action="store_true");p.add_argument("--report",action="store_true");p.add_argument("--test",action="store_true");args=p.parse_args()
    if args.prepare:prepare();print("N205 design frozen and inputs verified",flush=True)
    if args.test or args.scenario or args.all:
        from .selftest import run as tests
        tests()
    if args.scenario or args.all:
        from .runner import run
        from .validator import validate,sensitivity
        order=config()["scenario_order"]
        for scenario in order if args.all else [args.scenario]:
            if scenario!="nominal":
                previous=order[order.index(scenario)-1];path=ROOT/("E_"+previous)/"metrics.json"
                if not path.exists() or not read(path)["end_to_end_passed"] or not read(path.with_name("validation.json"))["passed"]:
                    ledger=read(ROOT/"run_ledger.json");ledger["skipped"][scenario]="upstream scenario did not pass; no automatic retry or parameter change";save(ROOT/"run_ledger.json",ledger);continue
            already=(ROOT/("E_"+scenario)/"validation.json").exists()
            result=run(scenario,args.resume)
            if not (already and args.resume):validate(scenario)
            if scenario=="fine" and result["end_to_end_passed"]:
                if not sensitivity()["passed"]:raise RuntimeError("MOMENTUM_OR_NUMERICS_FAILED: step sensitivity")
    if args.validate_only:
        from .validator import validate
        for item in read(ROOT/"run_ledger.json")["formal_runs"]:validate(item["scenario"])
    if args.report:
        from .report import build
        build()

if __name__=="__main__":main()

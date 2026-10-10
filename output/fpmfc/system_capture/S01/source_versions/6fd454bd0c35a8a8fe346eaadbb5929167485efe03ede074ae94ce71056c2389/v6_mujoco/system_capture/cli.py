import argparse
import json

def main(argv=None):
    p=argparse.ArgumentParser(description='One explicitly selected system stage; S00 or S01.')
    p.add_argument('--phase',required=True)
    p.add_argument('--mode',required=True,choices=['prepare','test','run','replay','report'])
    p.add_argument('--resume',action='store_true')
    a=p.parse_args(argv)
    if a.phase=='S01':
        from .paper_bridge.phase import main as s01_main
        return s01_main(a.mode,a.resume)
    if a.phase!='S00':
        print(json.dumps({'phase':a.phase,'status':'NOT_IMPLEMENTED','executed':False}));return 2
    from . import phase
    if a.mode=='prepare':phase.prepare();return 0
    if a.mode=='test':return 0 if phase.tests() else 1
    if a.mode in ('run','replay'):
        ok=phase.replays(a.resume)
        if a.mode=='replay':return 0 if ok else 1
    from .report import report
    return 0 if report() else 1

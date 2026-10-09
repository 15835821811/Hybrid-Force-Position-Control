"""One-time validation freeze and independently generated passive-object cases."""
import argparse
import dataclasses
import time
import json
from pathlib import Path
import numpy as np
import yaml
from .common import ROOT,CONFIG,config,read,save,runtime_identity,init_ledger,digest

def execution_identity():
    auxiliary={'audit.py','selftest.py','synthetic_validation.py','validate.py','figures.py','render.py','report.py','campaign.py','completion_audit.py'}
    return {p:h for p,h in runtime_identity().items() if not (p.startswith('v6_mujoco/adaptive_capture/') and Path(p).name in auxiliary) and not p.endswith('/holdout_truth.yaml')}

def verify_freeze():
    frozen=read(ROOT/'experiment_manifest.json')['execution_identity'];current=execution_identity()
    if current!=frozen:raise RuntimeError('FROZEN_EXECUTION_IDENTITY_CHANGED: '+str([p for p in frozen.keys()|current.keys() if frozen.get(p)!=current.get(p)]))
    return True

def freeze(selected):
    ledger=init_ledger()
    if ledger['frozen'] or (ROOT/'experiment_manifest.json').exists():raise RuntimeError('already frozen; no re-freeze')
    if any(x['status']=='RUNNING' for x in ledger['attempts']):raise RuntimeError('active attempt')
    metrics=read(ROOT/'runs'/selected/'metrics.json');validation=read(ROOT/'runs'/selected/'validation.json')
    assert metrics['continuous_task_completed'] and metrics['postgrasp_detumbling'] and validation['actuator_replay_passed'] and validation['decision_replay_passed']
    selected_cfg=read(ROOT/'runs'/selected/'config.json')
    for p,h in execution_identity().items():
        if p in selected_cfg['identity'] and h!=selected_cfg['identity'][p]:raise RuntimeError('selected run is not current executable '+p)
    save(ROOT/'experiment_manifest.json',{'schema':'n208_frozen_validation_v1','baseline_commit':config()['baseline_commit'],'frozen_epoch':time.time(),'selected_development_run':selected,'execution_identity':execution_identity(),'mission':config(),'prior':config('target_prior'),'sensor_modes':config('sensors'),'budget':{'development':6,'holdout':6,'fine':1,'pressure':2,'compatibility':1,'total':16},'qualification_not_inherited_from_N206':True})
    ledger['frozen']=True;ledger['frozen_epoch']=time.time();save(ROOT/'run_ledger.json',ledger)
    # These declared geometric distributions are instantiated only after freeze.
    from .plant import supported_truth
    definitions=[('H1',27.,[.012,-.008,.005],[.105,.085,.095],[.15,-.2,.1],[.7,-.5,4.]),('H2',14.,[-.01,.012,-.008],[.08,.11,.07],[-.12,.22,.17],[-.6,.8,5.5]),('H3',30.,[0.,0.,0.],[.15,.15,.15],[0.,0.,0.],[0.,0.,3.])]
    cases={}
    for name,m,c,half,rot,w in definitions:
        truth=supported_truth(m,c,half,rot,np.deg2rad(w));cases[name]={'truth':json.loads(json.dumps(dataclasses.asdict(truth),default=float)),'mass_support':{'shape':'uniform interior box','center_m':c,'half_extent_m':half,'rotation_vector':rot},'sensor_mode':'ideal','scope':'H3 spherical principal-axis free-motion degeneracy' if name=='H3' else 'nonzero COM, rotated nonspherical inertia, multi-axis spin'}
    text=yaml.safe_dump({'frozen_before_construction':True,'cases':cases},sort_keys=False)
    (CONFIG/'holdout_truth.yaml').write_text(text,encoding='utf-8',newline='\n')
    save(ROOT/'holdout_identity.json',{'sha256':digest(CONFIG/'holdout_truth.yaml'),'constructed_epoch':time.time()})
    return cases

def execute(kind,case=None,mode='prior'):
    verify_freeze()
    from .runner import run
    from .plant import TruthConfig,nominal_truth
    if kind=='holdout':
        assert digest(CONFIG/'holdout_truth.yaml')==read(ROOT/'holdout_identity.json')['sha256']
        s=config('holdout_truth')['cases'][case];return run(case+'_'+mode,s['sensor_mode'],'holdout',TruthConfig(**s['truth']),mode=mode)
    if kind=='fine':return run('F01_fine','ideal','fine',dt=.001,mode='prior')
    if kind=='noise':return run('S01_noise_delay','noisy','pressure',mode='identified')
    if kind=='contact':return run('S02_contact_mismatch','ideal','pressure',truth=dataclasses.replace(nominal_truth(),contact_scale=1.5),mode='identified')
    raise ValueError(kind)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('action',choices=['freeze','holdout','fine','noise','contact']);p.add_argument('--selected');p.add_argument('--case');p.add_argument('--mode',default='prior');a=p.parse_args()
    if a.action=='freeze':freeze(a.selected)
    else:execute(a.action,a.case,a.mode)

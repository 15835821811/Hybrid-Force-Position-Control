"""Resume a bounded selected-candidate campaign; failures are immutable."""
import subprocess
import sys
import time
from .common import ROOT,PAPER,PROJECT_ROOT,read,save,sha,identity,ledger

REGRESSIONS=[('R01_V2_nominal','nominal','ideal'),('R02_V2_H2','H2_prior','ideal'),('R03_V2_S01','nominal','noisy'),('R04_V2_H1','H1_prior','ideal')]

def dispatch(*args):
    subprocess.run([sys.executable,'-m','v6_mujoco.feasible_capture',*args],check=True)

def admission():
    results={}
    for name,scenario,sensor in REGRESSIONS:
        path=ROOT/'runs'/name/'metrics.json'
        if not path.exists():results[name]={'passed':False,'status':'NOT_EVALUATED'};continue
        m=read(path);results[name]={'passed':bool(m['continuous_task_completed'] and m['postgrasp_detumbling'] and not m['actual_safety_violations']),
                                  'status':m['status'],'source_key':m['source_key'],'metrics_sha256':sha(path)}
    return {'selected_version':'V2','all_seen_regressions_pass':all(x['passed'] for x in results.values()),'regressions':results,
            'selection_basis':'single final V2 startup soft-residual revision; no third candidate, no holdout-based selection'}

def freeze():
    a=admission();book=ledger()
    # This campaign never calls a not-yet-validated generator. A passing future
    # candidate requires a new protocol review; current failed version is closed.
    result={**a,'status':'NOT_FROZEN_REGRESSION_FAILED' if not a['all_seen_regressions_pass'] else 'ADMISSION_REQUIRES_REPLAY_AND_GENERATOR_REVIEW',
            'frozen':False,'holdout_authorized':False,'implementation_identity':identity(),
            'protocol_sha256':sha(PAPER/'experiment_protocol.md'),'parameter_policy_sha256':sha(ROOT/'parameter_policy.md'),
            'reason':'Selected V2 fails seen task completion; dependent independent experiment stage is not admitted.',
            'scope':'release/evidence identity only; NOT a validation freeze'}
    save(ROOT/'frozen_manifest.json',result)
    contract={'status':'PLANNED_NOT_MATERIALIZED','protocol':'paper/system_paper/experiment_protocol.md','protocol_sha256':sha(PAPER/'experiment_protocol.md'),
              'random_algorithm':'NumPy PCG64','physical_seed':2091009,'sensor_seeds':[209201,209202,209203],
              'clusters':6,'paired_runs':36,'B2_preselected_clusters':[3,6],'fine_preselected_clusters':[1,4],
              'mass_distribution':'positive uniform rotated box inside target cube; explicit support and triangle checks',
              'generator_execution':'NOT_IMPLEMENTED_OR_RUN_BECAUSE_ADMISSION_FAILED','no_hidden_scenario_values':True}
    save(ROOT/'holdout_generator_contract.json',contract)
    save(ROOT/'holdout_identity.json',{'status':'NOT_EVALUATED','reason':'NOT_ADMITTED','generated_scenarios':[],'attempts':0,'contract_sha256':sha(ROOT/'holdout_generator_contract.json')})
    return result

def regression_resume():
    names={x['name'] for x in ledger()['attempts']}
    for name,scenario,sensor in REGRESSIONS:
        if name in names:continue
        dispatch('--regression','--name',name,'--scenario',scenario,'--sensor',sensor)

def replay_resume():
    for x in ledger()['attempts']:
        p=ROOT/'runs'/x['name']
        if (p/'validation_timestamp_audited.json').exists():continue
        if not (p/'trace.npz').exists():continue
        dispatch('--replay-only','--name',x['name'],'--validation-tag','timestamp_audited')

def all_resume():
    from .common import prepare
    prepare();dispatch('--audit-existing','--resume');dispatch('--test')
    regression_resume();replay_resume()
    subprocess.run([sys.executable,'-m','v6_mujoco.feasible_capture.supporting_audit'],check=True)
    subprocess.run([sys.executable,'-m','v6_mujoco.feasible_capture.uncertainty_floor'],check=True)
    subprocess.run([sys.executable,'-m','v6_mujoco.feasible_capture.log_integrity'],check=True)
    freeze();dispatch('--paper')
    print('N209 bounded campaign complete; inspect qualification_matrix.json. N210/N211 not started.')

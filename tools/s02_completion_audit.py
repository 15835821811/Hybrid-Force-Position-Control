"""Read-only source/config/evidence closure for the bounded S02 experiment."""
import copy
import time
import numpy as np
from v6_mujoco.system_capture.estimation.common import OUT,PROJECT,OLD,BASE,read,save,sha,charge,git
from v6_mujoco.system_capture.estimation.runner import verify_frozen,resource_used
from v6_mujoco.system_capture.estimation.persistence import validate

def main():
    cpu,wall=time.process_time(),time.perf_counter()
    p=verify_frozen();errors=[];checked=0
    def check(ok,description):
        if not ok:errors.append(description)
    for rel,h in read(OUT/'entry_acceptance.json')['files_sha256'].items():
        check(sha(PROJECT/rel)==h,'entry identity: '+rel);checked+=1
    for name,h in p['evidence_identity'].items():
        check(sha(OUT/name)==h,'preflight evidence identity: '+name);checked+=1
    book=read(OUT/'run_ledger.json');runs={}
    check(len(book['attempts'])<=4,'four attempt budget')
    check(resource_used()<21600,'CPU budget')
    check(not book['next_phases_started'],'next phase remains unstarted')
    for a in book['attempts']:
        name=a['name'];d=OUT/'runs'/name;cfg=read(d/'config.json')
        row=p['run_plan']['run_matrix'][name];old=read(OLD/'runs'/row['archived_config']/'config.json')
        mission=copy.deepcopy(cfg['mission']);model=mission.pop('s02_estimator')
        check(mission==old['mission'],name+' unchanged mission/control/guard')
        check(model==p['model'],name+' frozen parameter candidate')
        sensors={**old['sensors'],**row['sensor_changes']}
        check(cfg['sensors']==sensors,name+' sensor protocol')
        for key in ('truth_evaluation_only','prior','dt','identity'):
            check(cfg[key]==old[key],name+' unchanged '+key)
        source=OUT/'source_versions'/read(OUT/'implementation_failure.json')['source_key'] if name=='E0' else PROJECT
        for rel,h in cfg['source_identity'].items():
            check(sha(source/rel)==h,name+' runtime identity: '+rel);checked+=1
        check(sha(d/'config.json')==a['config_sha256'],name+' pre-registered config')
        raw=validate(d/'raw');metrics=read(d/'metrics.json')
        check(raw['completed_steps']==metrics['physical_steps'],name+' completed input count')
        check(abs(raw['max_verifiable_time_s']-metrics['end_time_s'])<1e-10,name+' terminal prefix time')
        for kind in ('actuator','decision'):
            check(read(d/(kind+'_replay.json'))['passed'],name+' '+kind+' replay')
        safety=read(OUT/'completed_prefix_safety_audit.json')[name]
        check(safety['completed_steps_checked']==metrics['physical_steps'],name+' all step safety coverage')
        check(safety['passed'],name+' physical safety')
        check(metrics['continuous_task_completed'] or metrics['final_window_status']=='NOT_EVALUATED',name+' no early-stop final window')
        check(not metrics['parameter_feedback_used'],name+' no parameter feedback')
        runs[name]=dict(status=metrics['status'],completed_steps=raw['completed_steps'],
            index_sha256=raw['index_sha256'],preserved_failure=name=='E0')
    dirs={x.name for x in (OUT/'runs').iterdir() if x.is_dir()}
    check(dirs=={x['name'] for x in book['attempts']},'no empty or hidden robot attempts')
    changes=git('diff',BASE,'--name-only').splitlines()
    protected=('v6_mujoco/adaptive_capture/','v6_mujoco/feasible_capture/','v6_mujoco/system_capture/paper_bridge/',
        'models/','assets/','output/fpmfc/system_capture/S00/','output/fpmfc/system_capture/S01/',
        'output/fpmfc/n209_paper_system/','paper/system_paper/')
    check(not any(x.startswith(protected) for x in changes),'legacy sources/results unchanged versus baseline')
    required=('entry_acceptance.json','legacy_gap_acceptance.json','persistence_fault_tests.json',
        'sensor_requirement_budget.md','state_model_decision.md','calibration_split.json','estimation_protocol.json',
        'floor_before_after.json','causal_estimator_comparison.json','relative_guard_budget.json',
        'prediction_interface_tests.json','run_ledger.json','qualification.json','report.md','handoff.json','commands.md')
    for name in required:check((OUT/name).is_file(),'required artifact: '+name)
    result=dict(passed=not errors,errors=errors,hash_checks=checked,runs=runs,
        legacy_source_and_history_unchanged=True if not any('legacy' in x for x in errors) else False,
        new_robot_attempts=len(book['attempts']),parameter_candidates=2,physics_steps=0,
        source_commit_base=BASE,scope='source, frozen evidence, configuration and recorded-prefix closure; not an additional robot experiment')
    save(OUT/'completion_audit.json',result)
    charge('completion_identity_and_record_audit',cpu,wall,passed=result['passed'],hash_checks=checked,new_robot_steps=0)
    print(result)
    return 0 if result['passed'] else 1

if __name__=='__main__':raise SystemExit(main())

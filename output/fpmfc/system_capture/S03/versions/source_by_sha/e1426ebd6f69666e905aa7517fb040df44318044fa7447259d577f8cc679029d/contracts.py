"""Persistent budgets and preregistered domains; no plant information here."""
import copy
import time
from pathlib import Path
from ..common import PROJECT,OLD,read,save,sha,digest,environment,git,clean

OUT=PROJECT/'output/fpmfc/system_capture/S03'
S02=PROJECT/'output/fpmfc/system_capture/S02'
BASE='b048891c57d3ec0df72ec53ab87e20f2fe9aa1ac'
BRANCH='codex/system-s03-pm-feasible-planning'
LIMITS=dict(cpu_s=43200,wall_s=43200,attempts=12,candidates=1000,predictive_steps=1000000)

def charge(kind,cpu,wall,**fields):
    b=read(OUT/'run_ledger.json');b['operations'].append(dict(kind=kind,cpu_s=time.process_time()-cpu,wall_s=time.perf_counter()-wall,**fields));save(OUT/'run_ledger.json',b)

def used():
    b=read(OUT/'run_ledger.json')
    return sum(e.get('budget_charge_s',e.get('cpu_s',0.)) for k in ('operations','attempts','replays') for e in b[k])

def plan():
    matrix={}
    for name,method,scene,kind in [
        ('B01','B0_scalar','H2','baseline'),('D00','B2_time_path_shape','nominal','development'),
        ('D01','B2_time_path_shape','H2','development'),
        ('D02','B2_time_path_shape','nominal','development_reserved'),('D03','B2_time_path_shape','H2','development_reserved'),
        ('P00','B1_time_path','nominal','paired'),('P01','B2_time_path_shape','nominal','paired'),
        ('P02','B1_time_path','H2','paired'),('P03','B2_time_path_shape','H2','paired'),
        ('R00','B2_time_path_shape','H1','seen_regression')]:matrix[name]=dict(method=method,scene=scene,kind=kind)
    return dict(phase='S03',maximum_full_robot_attempts=12,journal_block_records=200,run_matrix=matrix,
        B00='Reuse E0_C2 only after exact identity and both full replays; otherwise one new baseline attempt',
        implementation_versions_max=2,implementation_retry_reserve=1,limits=LIMITS,
        observations='original ideal SensorPacket through frozen C2 CA18; no truth velocities',
        realtime='NON_REALTIME_SIMULATION; plant paused during computation; NOT_CERTIFIED',
        comparison='B1/B2 same search count, solver, weights, safety; B1 shape correction fixed zero',
        no_automatic_next_stage=True,push_authorized_by='final paragraph of user objective',
        visual_scope='refresh S03 run views including five views, continuous body-side and tracking; historical evidence retained')

def config(name):
    row=read(OUT/'experiment_manifest.json')['run_matrix'][name]
    source={'nominal':'R01_V2_nominal','H2':'R02_V2_H2','H1':'R04_V2_H1'}[row['scene']]
    c=copy.deepcopy(read(OLD/'runs'/source/'config.json'))
    c['mission']['s02_estimator']=read(PROJECT/'configs/system_capture/algorithms/s02_acceleration.json')
    ideal=read(S02/'runs/E0_C2/config.json')['sensors']
    assert c['sensors']==ideal, 'do not silently replace a measurement protocol'
    c['mission']['s03_method']=row['method']
    if row['method']!='B0_scalar':c['mission']['s03']=read(OUT/'planning_contract.json')
    c.update(phase='S03',run_id=name,source_identity=identity(),protocol_sha256=sha(OUT/'experiment_manifest.json'),
        changes=dict(planner=row['method'],estimator='frozen C2 CA18',physics='unchanged'))
    return c

def identity():
    old=read(S02/'runs/E0_C2/config.json')['source_identity']
    paths={p for p in old if p.endswith(('.py','.xml','.yaml','.json','.stl'))}
    paths|={p.relative_to(PROJECT).as_posix() for p in (PROJECT/'v6_mujoco/system_capture/planning').glob('*.py')}
    return {p:sha(PROJECT/p) for p in sorted(paths) if (PROJECT/p).exists() and not p.endswith('/report.py')}

def prepare():
    if git('branch','--show-current')!=BRANCH or git('merge-base',BASE,'HEAD')!=BASE:raise RuntimeError('wrong base/branch')
    OUT.mkdir(parents=True,exist_ok=True)
    if not (OUT/'run_ledger.json').exists():save(OUT/'run_ledger.json',dict(phase='S03',started_epoch=time.time(),limits=LIMITS,attempts=[],operations=[],replays=[],reused=[],
        resource_scope='all instrumented experiment/replay/report processes; initial shell/code inspection reserved conservatively',
        initial_inspection_charge_s=120))
    b=read(OUT/'run_ledger.json')
    if not b['operations']:b['operations'].append(dict(kind='initial_read_only_inspection',budget_charge_s=120,cpu_s=None,exact=False));save(OUT/'run_ledger.json',b)
    if not (OUT/'experiment_manifest.json').exists():save(OUT/'experiment_manifest.json',plan())
    if not (OUT/'candidate_budget.json').exists():save(OUT/'candidate_budget.json',dict(evaluations=0,cache_hits=0,predictive_steps=0,replay_predictive_steps=0,kinematic_steps=0,entries=[]))
    cpu,wall=time.process_time(),time.perf_counter();inputs={}
    names=[S02/x for x in ['report.md','qualification.json','handoff.json','run_ledger.json','sensor_requirement_budget.md','online_relative_guard_budget.json']]
    names += [S02/'runs'/n/'guard_call_analysis.json' for n in ['E0_C2','E1','E2']]
    for phase in ['S00','S01']:
        names += [PROJECT/'output/fpmfc/system_capture'/phase/x for x in ['report.md','qualification.json','handoff.json']]
    names += [OLD/'qp_failure_diagnosis.json',OLD/'diagnostics/H2_prior/snapshots.json',OLD/'diagnostics/H2_prior/diagnosis.json',PROJECT/'output/fpmfc/system_capture/S00/source_traceability.csv']
    for p in names:
        if not p.exists():raise FileNotFoundError(p)
        inputs[p.relative_to(PROJECT).as_posix()]=sha(p)
    runs={}
    for name in ['E0','E0_C2','E1','E2']:
        f=S02/'runs'/name;c=read(f/'config.json');d=read(f/'completion.json')
        for p in [f/'config.json',f/'completion.json',f/'raw/index.json']:inputs[p.relative_to(PROJECT).as_posix()]=sha(p)
        runs[name]=dict(status=d['status'],end_time_s=d['end_time_s'],physical_steps=d['physical_steps'],
            source_digest=digest(c['source_identity']),raw_index_sha256=sha(f/'raw/index.json'))
    save(OUT/'entry_acceptance.json',dict(status='PASS_WITH_RETAINED_HISTORICAL_GAPS',baseline=BASE,branch=BRANCH,
        files_sha256=inputs,archived_runs=runs,S02_retained='ACCURACY_IMPROVED_GUARD_NOT_COMPATIBLE',S01='PARTIAL',C1='MISSING_ORIGINAL_TRACE',environment=environment()))
    save(OUT/'measurement_domain.json',dict(main='IDEAL_SENSOR_PACKET_CA18',noise_domain='NOT_ADMITTED',sensor=read(S02/'runs/E0_C2/config.json')['sensors'],
        estimator=read(PROJECT/'configs/system_capture/algorithms/s02_acceleration.json'),startup_contact_issue='retained independently for S02R; no new S02R attempts'))
    save(OUT/'snapshot_manifest.json',dict(frozen_before_candidate_evaluation=True,max_states=80,
        ideal_H2_times_s=[0.04,1.,2.,3.,4.,4.6,4.8,5.,5.2,5.4,5.6,5.8],
        authority_subset_s=[0.04,3.,4.6,5.2],
        historical_H2='reuse all 26 archived last-0.5s snapshots; original estimator/identity retained',
        S02_readonly={'E1':[7.52,7.72,7.92,8.02],'E2':[8.32,8.52,8.72,8.82]},
        count_upper_bound=46,selection='fixed regular approach times plus preregistered terminal windows; no favorable-state selection'))
    charge('entry_audit',cpu,wall)
    return dict(status='PREPARED',historical_physics_steps_reexecuted=0)

class Context:
    out=OUT
    def __init__(self):self.plan=read(OUT/'experiment_manifest.json')
    def verify(self):
        if self.wall_expired():raise RuntimeError('INCOMPLETE_BUDGET: 12h wall')
        return dict(source_identity=identity())
    def remaining_cpu(self):return LIMITS['cpu_s']-used()
    def wall_expired(self):return time.time()-read(OUT/'run_ledger.json')['started_epoch']>=LIMITS['wall_s']
    def config(self,name):return config(name)
    def controller(self,cfg,replay=False):
        from .controller_adapter import controller
        wrapped=controller(cfg,replay=replay)
        if hasattr(wrapped.legacy.predictor,'budget'):wrapped.legacy.predictor.budget.run=cfg.get('run_id','unknown')
        return wrapped

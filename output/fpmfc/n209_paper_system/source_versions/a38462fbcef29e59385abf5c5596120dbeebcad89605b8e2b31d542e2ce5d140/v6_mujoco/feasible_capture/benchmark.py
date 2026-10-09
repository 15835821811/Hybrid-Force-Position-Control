"""Serial bounded runner. Lock spans an operation; every attempted plant is counted."""
import contextlib
import copy
import gzip
import json
import os
import shutil
import time
import traceback
from pathlib import Path
import numpy as np
import yaml
from v6_mujoco.adaptive_capture.common import config as old_config
from v6_mujoco.adaptive_capture.plant import Plant,nominal_truth,TruthConfig
from v6_mujoco.adaptive_capture.sensors import SensorFrontend
from v6_mujoco.adaptive_capture.evaluation import observe,safety,capture,summarize
from v6_mujoco.adaptive_capture.runner import jsonable
from v6_mujoco.adaptive_capture.validate import packet_from_dict
from .common import ROOT,OLD,PROJECT_ROOT,read,save,sha,identity,ledger,charge
from .controller_adapter import FeasibleController,NoVerifiedControl

@contextlib.contextmanager
def campaign_lock():
    ROOT.mkdir(parents=True,exist_ok=True);path=ROOT/'campaign.lock'
    fd=os.open(path,os.O_CREAT|os.O_EXCL|os.O_WRONLY)
    os.write(fd,str(os.getpid()).encode());os.close(fd)
    try:yield
    finally:path.unlink()

def mission(sensor,method='B1'):
    cfg=old_config();custom=yaml.safe_load((PROJECT_ROOT/'configs/n209_paper_system.yaml').read_text(encoding='utf-8'))
    cfg.update(custom);cfg['method']=method;cfg['parameter_feedback']=False
    cfg['shape_reference']=sensor!='ideal' and method!='B0'
    if sensor!='ideal':
        cfg.update(position_bias_bound_m=5e-6,rotation_bias_bound_rad=6e-6,
                   process_linear_accel=.03 if method=='B0' else cfg['n209']['noisy_process_linear_accel'],
                   process_angular_accel=.2 if method=='B0' else cfg['n209']['noisy_process_angular_accel'])
    return cfg

def clean_diagnostics(x):
    x=jsonable(x)
    if isinstance(x,dict):return {k:clean_diagnostics(v) for k,v in x.items()}
    if isinstance(x,list):return [clean_diagnostics(v) for v in x]
    if isinstance(x,float) and not np.isfinite(x):return None
    return x

def archive_sources(ident):
    import hashlib
    key=hashlib.sha256(json.dumps(ident,sort_keys=True).encode()).hexdigest();dest=ROOT/'source_versions'/key
    if not dest.exists():
        for rel in ident:
            if rel.endswith('.py'):
                path=dest/rel;path.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(PROJECT_ROOT/rel,path)
        save(dest/'identity.json',ident)
    return key

def run(name,sensor='ideal',method='B1',scenario='nominal',category='development',dt=.002):
    with campaign_lock():return _run(name,sensor,method,scenario,category,dt)

def _run(name,sensor,method,scenario,category,dt):
    book=ledger();out=ROOT/'runs'/name;cfg=mission(sensor,method);prior=old_config('target_prior');scfg=old_config('sensors')[sensor].copy()
    if category not in book['limits']:raise ValueError(category)
    if out.exists() or any(x['name']==name for x in book['attempts']):raise FileExistsError('recorded attempts never implicitly retry')
    if len(book['attempts'])>=60 or sum(x['category']==category for x in book['attempts'])>=book['limits'][category]:raise RuntimeError('ATTEMPT_BUDGET')
    if book['cpu_s']>=book['cpu_budget_s'] or time.time()-book['started_epoch']>=book['wall_budget_s']:raise RuntimeError('INCOMPLETE_BUDGET')
    if category=='development' and book['frozen']:raise RuntimeError('ALREADY_FROZEN')
    if category in ['holdout','ablation','step_sensitivity'] and not book['frozen']:raise RuntimeError('NOT_FROZEN')
    versions={x['version'] for x in book['attempts'] if x['category']=='development'}|{cfg['n209']['version']}
    if len(versions)>2:raise RuntimeError('MAX_TWO_METHOD_VERSIONS')
    truth=nominal_truth() if scenario=='nominal' else TruthConfig(**read(OLD/'runs'/scenario/'config.json')['truth_evaluation_only'])
    ident=identity();source_key=archive_sources(ident);out.mkdir(parents=True)
    start=time.time();cpu=time.process_time();entry={'name':name,'category':category,'version':cfg['n209']['version'],'scenario':scenario,
        'sensor':sensor,'method':method,'status':'RUNNING','started_epoch':start,'source_key':source_key}
    book['attempts'].append(entry);save(ROOT/'run_ledger.json',book)
    save(out/'config.json',{'mission':cfg,'prior':prior,'sensors':scfg,'truth_evaluation_only':truth,'dt':dt,'identity':ident,'source_key':source_key})
    rows=[];packets=[];events=[];posteriors=[];control_states=[];latencies=[];c=None;plant=None;status='RUNNING';reason=None;torque=np.zeros(7);last_phase=None;last_progress=-1
    try:
        plant=Plant(truth,dt,cfg['solver_tolerance']);c=FeasibleController(prior,cfg);front=SensorFrontend(scfg);first=observe(plant,c)
        servo_stride=round(cfg['servo_s']/dt);task_stride=round(cfg['task_s']/dt)
        for step in range(round((cfg['approach_deadline_s']+cfg['post_duration_s'])/dt)+1):
            t=float(plant.data.time);latch=False
            if step%servo_stride==0:
                p=front.sample(plant.model,plant.data);decision='CONTROL'
                call_start=time.perf_counter()
                try:torque,latch=c.update(p,step%task_stride==0)
                except NoVerifiedControl as ex:status=str(ex);reason=c.predictor.log[-1] if c.predictor else c.abort_reason;decision=status
                latencies.append(time.perf_counter()-call_start)
                packets.append({'packet':jsonable(p),'torque':torque.copy(),'latch':latch,'decision':decision})
                if c.abort_reason and status=='RUNNING':status='NO_VERIFIED_CONTROL';reason=c.abort_reason
                if latch:
                    true_ok=capture(observe(plant,c));events.append({'time':t,'state':'LATCH_REQUEST','estimated_gate':copy.deepcopy(c.last_gate),'true_capture':true_ok})
                    plant.latch()
                    if not true_ok:status='FALSE_CAPTURE';reason='estimated approval fails independent actual gate'
                if c.phase!=last_phase:events.append({'time':t,'state':c.phase});last_phase=c.phase
                if step%task_stride==0:
                    posteriors.append({'time':t,**c.estimator.snapshot()})
                    control_states.append({'time':t,'estimate':c.estimate,'raw_target':getattr(c.reference,'raw',c.estimate),'reference_target':c.reference.estimate,
                        'reference':c.latest_reference,'gate':copy.deepcopy(c.last_gate),'qref':c.qref.copy(),'velocity':c.vel.copy(),'start_velocity':c.start.copy(),
                        's':c.reference.progress/8,'s_dot_filter':c.reference.rate/8,'s_ddot_filter':c.reference.accel/8,'phase':c.phase,
                        'pi_est':c.estimator.pi.copy(),'pi_ctrl':c.information_gate.model_pi.copy() if c.information_gate.model_pi is not None else None})
                    if c.tasks and abs(c.tasks[-1]['time']-t)<1e-8 and (not c.tasks[-1]['success'] or any(c.tasks[-1]['bound_conflicts'])) and status=='RUNNING':status='HQP_FAILED';reason=c.tasks[-1]
            plant.data.ctrl[:]=torque;row=observe(plant,c);rows.append(row)
            failure,detail=safety(row,first,plant.model)
            if failure:events.append({'time':t,'state':'ACTUAL_SAFETY_VIOLATION','category':failure,'detail':detail})
            if failure and status=='RUNNING':status=failure;reason=detail
            if c.latch_time is None and t>=cfg['approach_deadline_s']-1e-9 and status=='RUNNING':status='APPROACH_TIMEOUT';reason=c.last_gate
            if c.latch_time is not None and t>=c.latch_time+cfg['post_duration_s']-1e-9 and status=='RUNNING':status='COMPLETED'
            if book['cpu_s']+time.process_time()-cpu>book['cpu_budget_s'] or time.time()-book['started_epoch']>book['wall_budget_s']:status='INCOMPLETE_BUDGET';reason='predeclared resource limit'
            if int(t)>last_progress:
                last_progress=int(t);print(json.dumps({'run':name,'t':round(t,3),'phase':c.phase,'s':c.reference.progress/8,'rho':row['load_fraction'],'status':status}),flush=True)
                save(out/'progress.json',{'time':t,'phase':c.phase,'status':status,'wall_s':time.time()-start})
            if status!='RUNNING':break
            plant.step(torque)
        plant.assert_frozen()
    except Exception:status='IMPLEMENTATION_ERROR';reason=traceback.format_exc()
    if status=='RUNNING':status='TIME_BOUND_REACHED'
    if rows:
        a,result=summarize(rows,status,c.latch_time,cfg);np.savez_compressed(out/'trace.npz',**a);result['trace_sha256']=sha(out/'trace.npz')
    else:result={'status':status,'end_time_s':0.,'continuous_task_completed':False,'full_window_evaluated':False}
    result.update(reason=clean_diagnostics(reason),cpu_s=time.process_time()-cpu,wall_s=time.time()-start,source_key=source_key,
        actual_safety_violations=[x for x in events if x['state']=='ACTUAL_SAFETY_VIOLATION'],method=method,scenario=scenario,sensor=sensor)
    events.append({'time':result['end_time_s'],'state':status,'reason':clean_diagnostics(reason)})
    if c:
        for file,data in [('tasks',c.tasks),('hqp_diagnostics',c.hqp.records),('posteriors',posteriors),('control_states',control_states),
                          ('estimator_events',getattr(c.state_filter,'log',[])),('governor',[] if c.governor is None else c.governor.log),
                          ('progress_governor',[] if c.predictor is None else c.predictor.log)]:save(out/(file+'.json'),clean_diagnostics(data))
        save(out/'identification.json',{'final':c.estimator.snapshot(),'prediction_before_assimilation':c.estimator.predict})
        if latencies:
            z=np.asarray(latencies);result['control_latency_s']={k:float(np.quantile(z,q)) for k,q in [('p50',.5),('p95',.95),('p99',.99),('max',1.)]}
            result['deadline_miss_fraction']={'2ms':float(np.mean(z>.002)),'20ms':float(np.mean(z>.02))}
            result['latency_scope']='runner wall time for every controller call, including terminal exceptions'
        result['predictive_steps']=0 if c.predictor is None else c.predictor.physics_steps
        result['predictive_task_solves']=0 if c.predictor is None else c.predictor.task_solves
        result['parameter_feedback_used']=c.feedback_used
    with gzip.open(out/'packets.jsonl.gz','wt',encoding='utf-8') as f:
        for p in packets:f.write(json.dumps(jsonable(p),separators=(',',':'))+'\n')
    save(out/'events.json',events)
    result['cpu_s']=time.process_time()-cpu;result['wall_s']=time.time()-start
    result['resource_scope']='includes simulation, internal predictions and artifact serialization; process CPU excludes OS/tool orchestration'
    save(out/'metrics.json',result)
    book=ledger();book['cpu_s']+=result['cpu_s'];next(x for x in book['attempts'] if x['name']==name).update(status=status,end_time_s=result['end_time_s'],cpu_s=result['cpu_s'],finished_epoch=time.time())
    save(ROOT/'run_ledger.json',book);save(ROOT/'development_ledger.json',{'attempts':[x for x in book['attempts'] if x['category']=='development'],'limits':{'versions':2,'attempts':8}})
    print(json.dumps({'run':name,'status':status,'end_time_s':result['end_time_s'],'reason':clean_diagnostics(reason)},ensure_ascii=False),flush=True)
    return result

def replay(name):
    with campaign_lock():
        out=ROOT/'runs'/name;cfg=read(out/'config.json');cpu=time.process_time();wall=time.perf_counter()
        cls=controller_for_replay(cfg)
        z=np.load(out/'trace.npz');plant=Plant(TruthConfig(**cfg['truth_evaluation_only']),cfg['dt'],cfg['mission']['solver_tolerance']);errors=np.zeros(2)
        for i,t in enumerate(z['time_s']):
            if z['eq_active'][i,0] and not plant.data.eq_active[0]:plant.latch()
            errors=np.maximum(errors,[max(abs(plant.data.qpos-z['qpos'][i])),max(abs(plant.data.qvel-z['qvel'][i]))])
            if i+1<len(z['time_s']):plant.step(z['ctrl_nm'][i])
        plant.assert_frozen();c=cls(cfg['prior'],cfg['mission']);error=0.;same=True;n=0;state_error=0.;state_same=True
        states={round(x['time']/cfg['mission']['servo_s']):x for x in read(out/'control_states.json')}
        with gzip.open(out/'packets.jsonl.gz','rt',encoding='utf-8') as f:
            for line in f:
                row=json.loads(line);p=packet_from_dict(row['packet']);decision='CONTROL'
                try:
                    tau,latch=c.update(p,round(p.time/cfg['mission']['servo_s'])%round(cfg['mission']['task_s']/cfg['mission']['servo_s'])==0)
                    error=max(error,float(max(abs(tau-row['torque']))));same &= latch==row['latch']
                except RuntimeError as ex:
                    if str(ex) not in ['NO_VERIFIED_CONTROL','ESTIMATE_UNRELIABLE']:raise
                    decision=str(ex)
                same &= decision==row['decision'];n+=1
                saved=states.get(round(p.time/cfg['mission']['servo_s']))
                if saved:
                    actual=clean_diagnostics({'estimate':c.estimate,'reference':c.latest_reference,'gate':c.last_gate,
                                             'qref':c.qref,'velocity':c.vel,'phase':c.phase,'s':c.reference.progress/8,
                                             'pi_est':c.estimator.pi})
                    eq,delta=compare_tree({k:saved[k] for k in actual},actual)
                    state_same &= eq;state_error=max(state_error,delta)
        result={'actuator_replay_passed':bool(errors[0]<1e-10 and errors[1]<1e-9),'qpos_error':errors[0],'qvel_error':errors[1],
                'decision_replay_passed':bool(error<1e-10 and same and state_same and state_error<1e-10),'torque_error':error,'events_match':bool(same),'packets':n,
                'estimated_reference_guard_parameter_state_match':bool(state_same),'max_intermediate_state_error':state_error,
                'trace_sha256':sha(out/'trace.npz'),'validator_identity':identity(),'cpu_s':time.process_time()-cpu,'wall_s':time.perf_counter()-wall,
                'scope':'t0 initialization once, actuator/events only; separate packet decisions; neither substitutes for task success'}
        dest=out/'validation.json'
        if dest.exists():raise FileExistsError(dest)
        save(dest,result);charge('dual_replay',result['cpu_s'],result['wall_s'],name)
        print(json.dumps({k:v for k,v in result.items() if k!='validator_identity'}),flush=True);return result

def controller_for_replay(cfg):
    """Archived per-run sources, never silently replay V1 using V2."""
    import importlib.util
    import sys
    modules=['adaptive_capture.'+n for n in ['contracts','momentum_regressor','known_model','state_estimator','inertial_estimator','relative_reference','risk','information_gate','governor','controller']]
    modules+=['feasible_capture.'+n for n in ['estimator_adapter','reference_shaper','controller_adapter','progress_governor']]
    paths=['v6_mujoco/'+x.replace('.','/')+'.py' for x in modules]
    for rel in ['v6_mujoco/fpmfc/controller.py','v6_mujoco/hierarchical_qp.py','v6_mujoco/geometry_capture/planning.py','v6_mujoco/run.py']:
        if sha(PROJECT_ROOT/rel)!=cfg['identity'][rel]:raise RuntimeError('historical shared dependency changed: '+rel)
    if all(sha(PROJECT_ROOT/rel)==cfg['identity'][rel] for rel in paths):return FeasibleController
    for module,rel in zip(modules,paths):
        saved=ROOT/'source_versions'/cfg['source_key']/rel
        if sha(saved)!=cfg['identity'][rel]:raise RuntimeError('archived source mismatch: '+rel)
        name='v6_mujoco.'+module;spec=importlib.util.spec_from_file_location(name,saved);obj=importlib.util.module_from_spec(spec)
        sys.modules[name]=obj;spec.loader.exec_module(obj)
    return sys.modules['v6_mujoco.feasible_capture.controller_adapter'].FeasibleController


def compare_tree(a,b):
    if isinstance(a,dict):
        if not isinstance(b,dict) or set(a)!=set(b):return False,0.
        values=[compare_tree(a[k],b[k]) for k in a]
    elif isinstance(a,list):
        if not isinstance(b,list) or len(a)!=len(b):return False,0.
        values=[compare_tree(x,y) for x,y in zip(a,b)]
    elif isinstance(a,(float,int)) and not isinstance(a,bool):
        if not isinstance(b,(float,int)):return False,0.
        return True,abs(float(a)-float(b))
    else:return a==b,0.
    return all(x[0] for x in values),max([x[1] for x in values] or [0.])

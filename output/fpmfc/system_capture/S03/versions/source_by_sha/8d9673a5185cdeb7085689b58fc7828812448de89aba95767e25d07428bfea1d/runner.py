"""Four-attempt S02 runner using S00 contracts and unchanged physical modules."""
import copy
import time
import traceback
from pathlib import Path
import numpy as np
from ..contracts import ControllerSetup,encode,decode,plain
from ..adapters import ControllerAdapter,SensorAdapter,PlantAdapter
from ..scheduler import ClockSpec
from ..common import save,clean,sha,digest,environment
from .common import OUT,OLD,PROJECT,PROTOCOL,read,identity
from .controller import AccelerationController
from .persistence import Journal,records,completed_inputs,validate,atomic_json

def controller(cfg):
    setup=ControllerSetup(cfg['prior'],cfg['mission'])
    return ControllerAdapter(AccelerationController(plain(setup.prior),plain(setup.algorithm_config)))

def snapshot(adapter):
    c=adapter.legacy;f=c.state_filter
    result=clean(dict(controller=adapter.last_snapshot,posterior=f.s,latest_snapshot=f.latest_snapshot,
        history=list(f.history),samples=f.n,measurement_time=f.time,contact=f.contact,
        innovation=f.innovation,parameter=adapter.parameters,guard_call_count=len(c.guard_calls),
        predictor_steps=c.predictor.physics_steps,predictor_solves=c.predictor.task_solves,
        last_prediction=c.predictor.log[-1] if c.predictor.log else None))
    if hasattr(c,'planning_snapshot'):result['planning']=clean(c.planning_snapshot())
    return result

def resource_used():
    b=read(OUT/'run_ledger.json')
    return sum(x.get('budget_charge_s',x.get('cpu_s',0.)) for kind in ('operations','attempts','replays') for x in b[kind])

def verify_frozen():
    p=read(OUT/'estimation_protocol.json')
    for rel,h in p['source_identity'].items():
        if sha(PROJECT/rel)!=h:raise RuntimeError('frozen source changed: '+rel)
    if environment()!=p['environment']:raise RuntimeError('frozen environment changed')
    return p

def run_one(name,context=None):
    out=OUT if context is None else context.out
    protocol=verify_frozen() if context is None else context.verify()
    plan=read(PROTOCOL) if context is None else context.plan
    book=read(out/'run_ledger.json')
    if len(book['attempts'])>=plan['maximum_full_robot_attempts'] or any(x['name']==name for x in book['attempts']):raise RuntimeError('attempt limit/no implicit retries')
    remaining=21600-resource_used() if context is None else context.remaining_cpu()
    if remaining<60:raise RuntimeError('INCOMPLETE_BUDGET: do not start another attempt')
    if context is None:
        row=plan['run_matrix'][name];cfg=copy.deepcopy(read(OLD/'runs'/row['archived_config']/'config.json'))
        cfg['mission']['s02_estimator']=read(PROJECT/'configs/system_capture/algorithms/s02_acceleration.json')
        cfg['sensors'].update(row['sensor_changes'])
        cfg.update(phase='S02',run_id=name,source_identity=protocol['source_identity'],
            protocol_sha256=sha(OUT/'estimation_protocol.json'),changes=dict(estimator='CA18',predictor='P18 read-only snapshot',sensors=row['sensor_changes']))
    else:cfg=context.config(name)
    folder=out/'runs'/name;folder.mkdir(parents=True,exist_ok=False)
    atomic_json(folder/'config.json',cfg)
    start,cpu=time.perf_counter(),time.process_time();entry=dict(name=name,status='REGISTERED_BEFORE_DYNAMICS',started_epoch=time.time(),config_sha256=sha(folder/'config.json'))
    book['attempts'].append(entry);save(out/'run_ledger.json',book)
    journal=Journal(folder/'raw',plan['journal_block_records']);status='RUNNING';reason=None
    plant=None;wrapped=None;first=None;last=0.;physical_steps=0;violations=[];latencies=[];next_print=0.;raw_validation=None;close_error=None
    try:
        from v6_mujoco.adaptive_capture.plant import Plant,TruthConfig
        from v6_mujoco.adaptive_capture.sensors import SensorFrontend
        from v6_mujoco.adaptive_capture.evaluation import observe,safety,capture
        plant=Plant(TruthConfig(**cfg['truth_evaluation_only']),cfg['dt'],cfg['mission']['solver_tolerance'])
        wrapped=controller(cfg) if context is None else context.controller(cfg)
        c=wrapped.legacy;front=SensorAdapter(SensorFrontend(cfg['sensors']));plant_adapter=PlantAdapter(plant)
        clocks=ClockSpec(cfg['dt'],cfg['mission']['servo_s'],cfg['mission']['task_s'])
        first=observe(plant,c)
        journal.append(dict(kind='initial_state',time=0.,observation=first));journal.commit()
        for step in range(round((cfg['mission']['approach_deadline_s']+cfg['mission']['post_duration_s'])/cfg['dt'])+1):
            t=float(plant.data.time);last=t;p=front.sample(plant.model,plant.data)
            journal.append(dict(kind='packet_received',step=step,time=t,packet=encode(p)))
            now=time.perf_counter()
            try:proposal=wrapped.update(p,clocks.task_tick(t))
            finally:latencies.append(time.perf_counter()-now)
            obs=observe(plant,c)
            violation,detail=safety(obs,first,plant.model)
            if violation:violations.append(dict(time=t,category=violation,detail=detail));status=violation;reason=detail
            if status=='RUNNING' and not proposal.actuation_valid:
                status=proposal.decision;reason=clean(c.predictor.log[-1] if c.predictor.log else c.abort_reason)
            if status=='RUNNING' and c.tasks and abs(c.tasks[-1]['time']-t)<1e-8 and (not c.tasks[-1]['success'] or any(c.tasks[-1]['bound_conflicts'])):
                status='HQP_FAILED';reason=clean(c.tasks[-1])
            if status=='RUNNING' and np.any(abs(proposal.tau7)>c.spec.torque_limits_nm+1e-8):
                status='INTERFACE_COMMAND_LIMIT';reason='proposal exceeds unchanged actuator limits'
            if status=='RUNNING' and proposal.latch_request and not capture(obs):
                status='FALSE_CAPTURE';reason='estimated approval fails independent actual gate';violations.append(dict(time=t,category=status))
            if status=='RUNNING' and c.latch_time is None and t>=cfg['mission']['approach_deadline_s']-1e-9:status='APPROACH_TIMEOUT';reason=clean(c.last_gate)
            if status=='RUNNING' and c.latch_time is not None and t>=c.latch_time+cfg['mission']['post_duration_s']-1e-9:status='COMPLETED'
            if time.process_time()-cpu>remaining or (context is not None and context.wall_expired()):status='INCOMPLETE_BUDGET';reason='shared process CPU or elapsed wall limit'
            journal.append(dict(kind='proposal',step=step,time=t,proposal=encode(proposal),snapshot=snapshot(wrapped),observation=obs,status=status,reason=clean(reason)))
            if t>=next_print:
                print(dict(run=name,t=round(t,3),phase=c.phase,status=status,steps=physical_steps),flush=True);next_print=t+1.
                atomic_json(folder/'progress.json',dict(time=t,status=status,cpu_s=time.process_time()-cpu,wall_s=time.perf_counter()-start))
            if status!='RUNNING':break
            # Application and successful integration are separate evidence.
            if proposal.latch_request:plant_adapter.replay_event(True)
            plant.data.ctrl[:]=proposal.tau7
            journal.append(dict(kind='input_applied',step=step,time=t,tau=proposal.tau7,latch=proposal.latch_request))
            plant_adapter.replay_step(proposal.tau7);physical_steps+=1
            after=observe(plant,c)
            journal.append(dict(kind='step_completed',step=step,time=float(plant.data.time),observation=after))
        plant_adapter.assert_frozen()
    except Exception:
        status='IMPLEMENTATION_ERROR';reason=traceback.format_exc()
        try:journal.append(dict(kind='exception',time=float(plant.data.time) if plant else 0.,reason=reason))
        except Exception:pass
    finally:
        # Raw data is closed and verified BEFORE metrics, arrays or plots.
        try:raw_validation=journal.close()
        except Exception:close_error=traceback.format_exc();status='PERSISTENCE_ERROR'
        last=float(plant.data.time) if plant else 0.
        finish=dict(status=status,reason=clean(reason),end_time_s=last,physical_steps=physical_steps,
            raw_validation=raw_validation,persistence_error=close_error,
            max_verifiable_time_s=raw_validation['max_verifiable_time_s'] if raw_validation else read(folder/'raw/index.json')['max_completed_time_s'],
            actual_safety_violations=violations,journal_io_wall_s=journal.io_s,journal_serialization_wall_s=journal.serialization_s,
            cpu_s=time.process_time()-cpu,wall_s=time.perf_counter()-start,
            latch_time_s=wrapped.legacy.latch_time if wrapped else None,
            predictive_steps=wrapped.legacy.predictor.physics_steps if wrapped else 0,
            predictive_task_solves=wrapped.legacy.predictor.task_solves if wrapped else 0,
            actual_guard_calls=len(wrapped.legacy.guard_calls) if wrapped else 0,
            parameter_feedback_used=wrapped.legacy.feedback_used if wrapped else False)
        atomic_json(folder/'completion.json',finish)
        book=read(out/'run_ledger.json');next(x for x in book['attempts'] if x['name']==name).update(finish)
        save(out/'run_ledger.json',book)
    # Summaries are rebuildable exclusively from committed original records.
    if raw_validation:
        try:
            summarize_run(name,out)
            if wrapped:
                c=wrapped.legacy
                for key,data in [('guard_calls',c.guard_calls),('progress_governor',c.predictor.log),('estimator_events',c.state_filter.log),('tasks',c.tasks)]:save(folder/(key+'.json'),clean(data))
                save(folder/'timing.json',dict(control_s=np.quantile(latencies,[.5,.95,.99,1.]),
                    estimator_update_s=np.quantile(c.state_filter.update_times,[.5,.95,.99,1.]),
                    snapshot_predict_s=np.quantile(c.state_filter.predict_times,[.5,.95,.99,1.]) if c.state_filter.predict_times else [],
                    quantiles=[.5,.95,.99,1.],failed_controller_calls=int(status in ('NO_VERIFIED_CONTROL','ESTIMATE_UNRELIABLE','IMPLEMENTATION_ERROR')),
                    realtime='NOT_CERTIFIED'))
        except Exception:
            atomic_json(folder/'summary_error.json',dict(error=traceback.format_exc(),raw_preserved=True))
            raise
    return finish

def summarize_run(name,out=OUT):
    from v6_mujoco.adaptive_capture.evaluation import summarize
    folder=out/'runs'/name;done=read(folder/'completion.json');cfg=read(folder/'config.json')
    checked=validate(folder/'raw');rows=[];last_completed=None
    for r in records(folder/'raw'):
        if r['kind']=='proposal':rows.append(r['observation'])
        elif r['kind']=='step_completed':last_completed=r['observation']
    if last_completed is not None and (not rows or last_completed['time_s']>rows[-1]['time_s']+1e-12):
        rows.append(last_completed)  # Actual completed endpoint, no invented estimator update.
    if rows:
        rows=[{k:np.array(v) if isinstance(v,list) else v for k,v in r.items()} for r in rows]
        arrays,result=summarize(rows,done['status'],done['latch_time_s'],cfg['mission'])
        # ctrl_nm is the last applied input at observation time; never the final
        # unexecuted proposal. Actuator replay reads raw completed pairs only.
        np.savez_compressed(folder/'trace.npz',**arrays)
    else:result=dict(full_window_evaluated=False,continuous_task_completed=False)
    result.update(done,raw_validation=checked,final_window_status='EVALUATED' if result['full_window_evaluated'] else 'NOT_EVALUATED')
    save(folder/'metrics.json',clean(result));return result

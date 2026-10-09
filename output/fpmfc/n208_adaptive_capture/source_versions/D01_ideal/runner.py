"""Budgeted single continuous real plant; packet control and separate truth audit."""
import argparse
import dataclasses
import json
import time
import traceback
import numpy as np
from .common import ROOT,config,read,save,save_npz,runtime_identity,init_ledger,digest
from .plant import Plant,nominal_truth,TruthConfig
from .sensors import SensorFrontend
from .controller import Controller
from .evaluation import observe,safety,capture,summarize

def jsonable(value):
    if dataclasses.is_dataclass(value):return {f.name:jsonable(getattr(value,f.name)) for f in dataclasses.fields(value)}
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    if isinstance(value,(list,tuple)):return [jsonable(x) for x in value]
    if isinstance(value,dict):return {k:jsonable(v) for k,v in value.items()}
    return value

def run(name,sensor_mode='ideal',category='development',truth=None,dt=None):
    ledger=init_ledger();cfg=config();prior=config('target_prior');sensor_cfg=config('sensors')[sensor_mode];out=ROOT/'runs'/name
    if out.exists() or any(x['name']==name for x in ledger['attempts']):raise FileExistsError('no overwrite/retry: '+name)
    if len(ledger['attempts'])>=cfg['max_attempts']:raise RuntimeError('TOTAL_BUDGET_EXHAUSTED')
    if category=='development' and (ledger['frozen'] or sum(x['category']=='development' for x in ledger['attempts'])>=cfg['max_development']):raise RuntimeError('DEVELOPMENT_BUDGET_OR_FREEZE')
    if ledger['cpu_s']>=cfg['cpu_budget_s'] or time.time()-ledger['started_epoch']>=cfg['wall_budget_s']:raise RuntimeError('RESOURCE_BUDGET')
    if category!='development' and not ledger['frozen']:raise RuntimeError('NOT_FROZEN')
    truth=truth or nominal_truth();dt=dt or cfg['dt_s'];start=time.time();cpu=time.process_time();identity=runtime_identity();out.mkdir(parents=True)
    entry={'name':name,'category':category,'started_epoch':start,'status':'RUNNING','identity':identity};ledger['attempts'].append(entry);save(ROOT/'run_ledger.json',ledger)
    save(out/'config.json',{'mission':cfg,'prior':prior,'sensors':sensor_cfg,'sensor_mode':sensor_mode,'truth_evaluation_only':jsonable(truth),'dt':dt,'identity':identity})
    rows=[];events=[];packets=[];posteriors=[];c=None;status='RUNNING';reason=None;last_phase=None;last_rank=None;last_progress=-1;torque=np.zeros(7)
    try:
        plant=Plant(truth,dt);c=Controller(prior,cfg);sensor=SensorFrontend(sensor_cfg)
        first=observe(plant,c);servo_stride=round(cfg['servo_s']/dt);task_stride=round(cfg['task_s']/dt);nmax=round((cfg['approach_deadline_s']+cfg['post_duration_s'])/dt)
        for step in range(nmax+1):
            t=float(plant.data.time);latch=False
            if step%servo_stride==0:
                packet=sensor.sample(plant.model,plant.data);torque,latch=c.update(packet,step%task_stride==0);packets.append({'packet':jsonable(packet),'torque':torque.tolist(),'latch':latch})
                if not c.estimate.valid and t>cfg['stale_limit_s']+.05:status='SENSOR_UNAVAILABLE';reason='no valid causal estimate'
                if latch:
                    before=observe(plant,c);true_ok=capture(before)
                    events.append({'time_s':t,'state':'LATCH_REQUEST','estimated_gate':c.last_gate,'evaluation_true_capture':true_ok})
                    # Always record the actual estimator decision. A false gate
                    # is a failed experiment, not replaced by a truth decision.
                    plant.latch()
                    if not true_ok:status='FALSE_CAPTURE';reason='estimator-authorized latch violates independent true gate'
                if c.phase!=last_phase:events.append({'time_s':t,'state':c.phase});last_phase=c.phase
                if step%task_stride==0:
                    posteriors.append({'time_s':t,**c.estimator.snapshot()})
                if step%task_stride==0 and c.tasks and abs(c.tasks[-1]['time']-t)<1e-8 and (not c.tasks[-1]['success'] or any(c.tasks[-1]['bound_conflicts'])):status='HQP_FAILED';reason=c.tasks[-1]
            plant.data.ctrl[:]=torque
            row=observe(plant,c);rows.append(row)
            failure,detail=safety(row,first,plant.model)
            if failure and status=='RUNNING':status=failure;reason=detail
            if c.latch_time is None and t>=cfg['approach_deadline_s']-1e-9:status='APPROACH_TIMEOUT';reason=c.last_gate
            if c.latch_time is not None and t>=c.latch_time+cfg['post_duration_s']-1e-9 and status=='RUNNING':status='COMPLETED'
            if time.time()-ledger['started_epoch']>cfg['wall_budget_s'] or ledger['cpu_s']+time.process_time()-cpu>cfg['cpu_budget_s']:status='RESOURCE_BUDGET';reason='pre-registered cumulative resource limit'
            progress=int(t)
            if progress>last_progress:
                print(json.dumps({'run':name,'t':round(t,3),'phase':c.phase,'rank':c.estimator.rank,'rho':row['load_fraction'],'position_mm':row['approach_position_error_m']*1000,'gate':c.last_gate}),flush=True);last_progress=progress
                save(out/'progress.json',{'time_s':t,'phase':c.phase,'status':status,'wall_s':time.time()-start})
            if status!='RUNNING':break
            plant.step(torque)
        plant.assert_frozen()
    except Exception:status='IMPLEMENTATION_ERROR';reason=traceback.format_exc()
    if status=='RUNNING':status='TIME_BOUND_REACHED'
    events.append({'time_s':rows[-1]['time_s'] if rows else 0.,'state':status,'reason':reason})
    if rows:
        a,result=summarize(rows,status,c.latch_time,cfg);save_npz(out/'trace.npz',a);result['trace_sha256']=digest(out/'trace.npz')
    else:result={'status':status,'end_time_s':0.,'continuous_task_completed':False}
    result.update(reason=reason,cpu_s=time.process_time()-cpu,wall_s=time.time()-start,identity=identity)
    if c is not None:
        save(out/'tasks.json',c.tasks);save(out/'identification.json',{'final':c.estimator.snapshot(),'prediction_before_assimilation':c.estimator.predict,'impulse_crosscheck':c.estimator.cross_checks})
        result['control_latency_s']={'mean':float(np.mean(c.timings)),'max':max(c.timings),'p99':float(np.quantile(c.timings,.99))}
    # JSONL packets preserve exact asynchronous delivery, permitting decision replay.
    import gzip
    with gzip.open(out/'packets.jsonl.gz','wt',encoding='utf-8') as f:
        for p in packets:f.write(json.dumps(p,separators=(',',':'))+'\n')
    save(out/'events.json',events);save(out/'posteriors.json',posteriors);save(out/'metrics.json',result)
    ledger=init_ledger();ledger['cpu_s']+=result['cpu_s'];ledger['attempts'][-1].update(status=status,end_time_s=result['end_time_s'],finished_epoch=time.time(),cpu_s=result['cpu_s']);save(ROOT/'run_ledger.json',ledger)
    print(json.dumps({'run':name,'status':status,'reason':reason,'end':result['end_time_s']}),flush=True)
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('name');p.add_argument('--sensor',default='ideal');a=p.parse_args();run(a.name,a.sensor)

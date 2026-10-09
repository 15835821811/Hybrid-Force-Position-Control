"""Budgeted single continuous real plant; packet control and separate truth audit."""
import argparse
import dataclasses
import json
import time
import traceback
import shutil
from pathlib import Path
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

def run(name,sensor_mode='ideal',category='development',truth=None,dt=None,mode='prior',template_duration=None):
    ledger=init_ledger();cfg=config();prior=config('target_prior');sensor_cfg=config('sensors')[sensor_mode];out=ROOT/'runs'/name
    if sensor_mode=='noisy':
        cfg.update(position_bias_bound_m=5e-6,rotation_bias_bound_rad=6e-6,process_linear_accel=.03,process_angular_accel=.2)
    if mode not in ['prior','identified']:raise ValueError(mode)
    cfg['parameter_feedback']=mode=='identified'
    if template_duration is not None:
        if category!='development':raise ValueError('duration cannot change after development')
        cfg['template_duration_s']=float(template_duration)
    if out.exists() or any(x['name']==name for x in ledger['attempts']):raise FileExistsError('no overwrite/retry: '+name)
    if len(ledger['attempts'])>=cfg['max_attempts']:raise RuntimeError('TOTAL_BUDGET_EXHAUSTED')
    category_limits={'development':6,'holdout':6,'fine':1,'pressure':2,'compatibility':1}
    if category not in category_limits or sum(x['category']==category for x in ledger['attempts'])>=category_limits[category]:raise RuntimeError('CATEGORY_BUDGET_EXHAUSTED')
    if category=='development' and (ledger['frozen'] or sum(x['category']=='development' for x in ledger['attempts'])>=cfg['max_development']):raise RuntimeError('DEVELOPMENT_BUDGET_OR_FREEZE')
    if ledger['cpu_s']>=cfg['cpu_budget_s'] or time.time()-ledger['started_epoch']>=cfg['wall_budget_s']:raise RuntimeError('RESOURCE_BUDGET')
    if category!='development' and not ledger['frozen']:raise RuntimeError('NOT_FROZEN')
    if category!='development':
        from .campaign import verify_freeze
        verify_freeze()
    truth=truth or nominal_truth();dt=dt or cfg['dt_s'];start=time.time();cpu=time.process_time();identity=runtime_identity();out.mkdir(parents=True)
    entry={'name':name,'category':category,'started_epoch':start,'status':'RUNNING','identity':identity};ledger['attempts'].append(entry);save(ROOT/'run_ledger.json',ledger)
    snapshot=ROOT/'source_versions'/name;snapshot.mkdir(parents=True,exist_ok=False)
    for source in Path(__file__).parent.glob('*.py'):shutil.copy2(source,snapshot/source.name)
    save(out/'config.json',{'mission':cfg,'prior':prior,'sensors':sensor_cfg,'sensor_mode':sensor_mode,'mode':mode,'truth_evaluation_only':jsonable(truth),'dt':dt,'identity':identity})
    rows=[];events=[];packets=[];posteriors=[];c=None;status='RUNNING';reason=None;last_phase=None;last_rank=None;last_progress=-1;torque=np.zeros(7)
    try:
        plant=Plant(truth,dt);c=Controller(prior,cfg);sensor=SensorFrontend(sensor_cfg)
        first=observe(plant,c);servo_stride=round(cfg['servo_s']/dt);task_stride=round(cfg['task_s']/dt);nmax=round((cfg['approach_deadline_s']+cfg['post_duration_s'])/dt)
        for step in range(nmax+1):
            t=float(plant.data.time);latch=False
            if step%servo_stride==0:
                packet=sensor.sample(plant.model,plant.data);torque,latch=c.update(packet,step%task_stride==0);packets.append({'packet':jsonable(packet),'torque':torque.tolist(),'latch':latch})
                if c.abort_reason is not None:status='PREDICTION_SAFETY_INFEASIBLE';reason=c.abort_reason
                if not c.estimate.valid and t>cfg['stale_limit_s']+.05:status='SENSOR_UNAVAILABLE';reason='no valid causal estimate'
                if latch:
                    before=observe(plant,c);true_ok=capture(before)
                    events.append({'time_s':t,'state':'LATCH_REQUEST','estimated_gate':c.last_gate,'confirmation_start_s':c.confirm_start,'confirmation_duration_s':t-c.confirm_start,'evaluation_true_capture':true_ok})
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
        if c.estimator.blocks:
            save_npz(out/'regression_blocks.npz',{'DY':np.asarray([a for a,b,L in c.estimator.blocks]),'dh':np.asarray([b for a,b,L in c.estimator.blocks]),'cholesky':np.asarray([L for a,b,L in c.estimator.blocks])})
        result['control_latency_s']={'mean':float(np.mean(c.timings)),'max':max(c.timings),'p99':float(np.quantile(c.timings,.99))} if c.timings else None
        result['parameter_feedback_used']=c.feedback_used
        result['mode']=mode
        result['parameter_feedback_reason']=c.information_gate.reason
        if c.governor is not None:save(out/'governor.json',c.governor.log)
    # JSONL packets preserve exact asynchronous delivery, permitting decision replay.
    import gzip
    with gzip.open(out/'packets.jsonl.gz','wt',encoding='utf-8') as f:
        for p in packets:f.write(json.dumps(p,separators=(',',':'))+'\n')
    save(out/'events.json',jsonable(events));save(out/'posteriors.json',jsonable(posteriors));save(out/'metrics.json',jsonable(result))
    ledger=init_ledger();ledger['cpu_s']+=result['cpu_s'];next(x for x in ledger['attempts'] if x['name']==name).update(status=status,end_time_s=result['end_time_s'],finished_epoch=time.time(),cpu_s=result['cpu_s']);save(ROOT/'run_ledger.json',ledger)
    print(json.dumps({'run':name,'status':status,'reason':reason,'end':result['end_time_s']}),flush=True)
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('name');p.add_argument('--sensor',default='ideal');p.add_argument('--mode',choices=['prior','identified'],default='prior');p.add_argument('--duration',type=float);a=p.parse_args();run(a.name,a.sensor,mode=a.mode,template_duration=a.duration)

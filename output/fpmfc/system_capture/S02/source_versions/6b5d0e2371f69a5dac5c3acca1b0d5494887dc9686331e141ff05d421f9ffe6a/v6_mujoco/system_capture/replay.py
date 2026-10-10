"""One t0 actuator replay or one archival-controller adapter decision replay.

The direct archived controller executes once inside the adapter; its raw return
is compared with the new contract AND the independent saved original run. This
does not duplicate full decision passes or exceed the four-replay S00 budget.
"""
import copy
import gzip
import json
import os
import sys
import time
import traceback
from pathlib import Path
import numpy as np
from .common import PROJECT,OUT,OLD,read,save,sha,clean
from .contracts import plain
from .adapters import packet_from_legacy,PlantAdapter
from .registry import controller
from .contracts import ControllerSetup
from .scheduler import ClockSpec,decisions
from .profiling import SampledProfiler,summarize

def compare(a,b):
    a=plain(a);b=plain(b)
    if isinstance(a,dict):
        if not isinstance(b,dict) or set(a)!=set(b):return False,0.
        pairs=[compare(a[k],b[k]) for k in a]
    elif isinstance(a,list):
        if not isinstance(b,list) or len(a)!=len(b):return False,0.
        pairs=[compare(x,y) for x,y in zip(a,b)]
    elif isinstance(a,(float,int)) and not isinstance(a,bool) and isinstance(b,(float,int)) and not isinstance(b,bool):
        return bool(abs(a-b)<1e-10),float(abs(a-b))
    else:return a==b,0.
    return all(v[0] for v in pairs),max([v[1] for v in pairs] or [0.])

def worker(name,kind):
    start=time.perf_counter();cpu=time.process_time();folder=OLD/'runs'/name;cfg=read(folder/'config.json')
    manifest=read(OUT/'phase_manifest.json');key=name+'__'+kind
    result={'run':name,'kind':kind,'passed':False,'input_commit':manifest['input_commit'],'source_content_hash':manifest['source_content_hash'],
      'source_key':cfg['source_key'],'trace_sha256':sha(folder/'trace.npz'),'new_physical_attempts':0,'physics_steps':0,'predictive_steps':0,
      'identity_policy':'isolated runtime directory; exact archived Python bytes and verified non-Python dependencies; no module replacement/monkey-patch'}
    try:
        runtime=Path.cwd()
        for rel,digest in cfg['identity'].items():
            if sha(runtime/rel)!=digest:raise RuntimeError('runtime identity mismatch '+rel)
        import v6_mujoco.adaptive_capture.plant as plant_module
        import v6_mujoco.feasible_capture.controller_adapter as controller_module
        for module in [plant_module,controller_module]:
            if not Path(module.__file__).is_relative_to(runtime):raise RuntimeError('escaped archived runtime imports')
        with np.load(folder/'trace.npz',allow_pickle=False) as archive:
            a={k:archive[k] for k in ['time_s','qpos','qvel','ctrl_nm','eq_active','interface_geom_masks','estimate_covariance']}
        max_cpu=float(os.environ['S00_CPU_REMAINING'])
        if kind=='actuator':
            from v6_mujoco.end_to_end_capture.adapter import pads
            plant=plant_module.Plant(plant_module.TruthConfig(**cfg['truth_evaluation_only']),cfg['dt'],cfg['mission']['solver_tolerance'])
            wrapped=PlantAdapter(plant);errors=np.zeros(2);event_times=[];masks_match=True;time_error=0.;steps=0
            for i,t in enumerate(a['time_s']):
                if a['eq_active'][i,0] and not plant.data.eq_active[0]:wrapped.replay_event(True);event_times.append(float(t))
                errors=np.maximum(errors,[np.max(abs(plant.data.qpos-a['qpos'][i])),np.max(abs(plant.data.qvel-a['qvel'][i]))])
                masks=np.array([plant.model.geom_contype[pads(plant.model)],plant.model.geom_conaffinity[pads(plant.model)]])
                masks_match &= np.array_equal(masks,a['interface_geom_masks'][i])
                time_error=max(time_error,abs(plant.data.time-t))
                if i+1<len(a['time_s']):wrapped.replay_step(a['ctrl_nm'][i]);steps+=1
                if time.process_time()-cpu>max_cpu:raise RuntimeError('INCOMPLETE_BUDGET')
            wrapped.assert_frozen()
            result.update(passed=bool(errors[0]<1e-10 and errors[1]<1e-9 and masks_match and time_error<1e-10),qpos_max_error=float(errors[0]),qvel_max_error=float(errors[1]),
                masks_match=bool(masks_match),time_max_error_s=time_error,latch_times_s=event_times,physics_steps=steps,
                state_initializations=1,saved_intermediate_state_injections=0,actuator_channels=plant.model.nu)
        elif kind=='decision':
            from v6_mujoco.adaptive_capture.validate import packet_from_dict
            wrapped=controller('n209_scalar_governor',ControllerSetup(cfg['prior'],cfg['mission']))
            c=wrapped.legacy;clocks=ClockSpec(cfg['dt'],cfg['mission']['servo_s'],cfg['mission']['task_s'])
            states={round(x['time']/cfg['mission']['servo_s']):x for x in read(folder/'control_states.json')}
            expected=[]
            with gzip.open(folder/'packets.jsonl.gz','rt',encoding='utf-8') as f:expected=[json.loads(line) for line in f]
            stats={'packets':0,'torque_max_error':0.,'state_max_error':0.,'packet_mapping_max_error':0.,'states_checked':0,
                'events_match':True,'intermediate_match':True,'direct_return_matches_adapter':True,'packet_roundtrip_match':True}
            ts_audits=[];mismatches=[];final_decision=None
            profiler=SampledProfiler(manifest['profile_packet_indices'][name])
            def packets():
                from .adapters import packet_to_legacy
                for row in expected:
                    old=packet_from_dict(row['packet']);p=packet_from_legacy(old)
                    ok,err=compare(clean(old),clean(packet_to_legacy(p)))
                    stats['packet_roundtrip_match'] &= ok;stats['packet_mapping_max_error']=max(stats['packet_mapping_max_error'],err)
                    yield p
            def observe(i,p,proposal,adapter):
                nonlocal final_decision
                row=expected[i];stats['packets']+=1;final_decision=proposal.decision
                raw=adapter.last_raw
                stats['direct_return_matches_adapter'] &= bool(np.array_equal(proposal.tau7,raw['tau7']) and proposal.latch_request==raw['latch_request'] and proposal.phase==raw['phase'] and proposal.decision==raw['decision'])
                if proposal.actuation_valid:stats['torque_max_error']=max(stats['torque_max_error'],float(np.max(abs(proposal.tau7-row['torque']))))
                stats['events_match'] &= proposal.decision==row['decision'] and proposal.latch_request==row['latch']
                saved=states.get(i)
                if saved is not None:
                    saved=copy.deepcopy(saved);est=saved['estimate'];k=round(p.t_control/cfg['dt'])
                    if est is not None and p.t_control==0 and not est['valid'] and est['measurement_time']==-1:
                        delta=float(np.max(abs(np.asarray(est['covariance'])-a['estimate_covariance'][k])))
                        if delta>1e-14:
                            ts_audits.append({'time':0.,'field':'estimate.covariance','original_difference':delta,
                              'replacement':'same-tick independently copied trace; known invalid-t0 alias only; no input/action changed'})
                            est['covariance']=a['estimate_covariance'][k].tolist()
                    actual=plain(adapter.last_snapshot)
                    ok,err=compare({k:saved[k] for k in actual},actual)
                    stats['states_checked']+=1;stats['intermediate_match'] &= ok;stats['state_max_error']=max(stats['state_max_error'],err)
                    if not ok and len(mismatches)<10:
                        mismatches.append({'time_s':p.t_control,'fields':[k for k in actual if not compare(saved[k],actual[k])[0]]})
                if i%1000==0:print(json.dumps({'run':name,'packet':i,'t':p.t_control,'phase':proposal.phase}),flush=True)
                if time.process_time()-cpu>max_cpu:raise RuntimeError('INCOMPLETE_BUDGET')
            decisions(packets(),wrapped,clocks,observe,profiler)
            metrics=read(folder/'metrics.json')
            final_match=stats['packets']==len(expected) and (final_decision=='CONTROL' if metrics['status']=='COMPLETED' else final_decision==metrics['status'])
            result.update(stats,passed=bool(stats['torque_max_error']<1e-10 and stats['state_max_error']<1e-10 and all(stats[k] for k in ['events_match','intermediate_match','direct_return_matches_adapter','packet_roundtrip_match']) and final_match),
                timestamp_snapshot_audit=ts_audits,final_outcome_matches=bool(final_match),historical_task_status=metrics['status'],mismatches=mismatches,
                predictive_steps=c.predictor.physics_steps if c.predictor else 0,predictive_task_solves=c.predictor.task_solves if c.predictor else 0,
                original_validation_sha256=sha(folder/'validation_timestamp_audited.json'),original_control_states_sha256=sha(folder/'control_states.json'),
                scope='one archived controller stream composed with new adapter; raw return equality plus original recorded trajectory/packet/state equivalence, not independent performance evidence')
            save(OUT/'runtime_profiles'/(name+'.json'),{'records':profiler.records,'summary':summarize(profiler.records),
                'selected_packet_indices':manifest['profile_packet_indices'][name],
                'scope':'inclusive observed calls at prespecified same controller states; nested costs overlap; failed/invalid returns included; no extra state rollout for profiling; instrumentation overhead included'})
        else:raise ValueError(kind)
    except Exception:
        result['error']=traceback.format_exc()
    finally:
        result.update(cpu_s=time.process_time()-cpu,wall_s=time.perf_counter()-start)
        save(OUT/'replays'/(key+'.json'),result)
    print(json.dumps(result,ensure_ascii=False),flush=True)
    return 0 if result['passed'] else 1

if __name__=='__main__':raise SystemExit(worker(sys.argv[1],sys.argv[2]))

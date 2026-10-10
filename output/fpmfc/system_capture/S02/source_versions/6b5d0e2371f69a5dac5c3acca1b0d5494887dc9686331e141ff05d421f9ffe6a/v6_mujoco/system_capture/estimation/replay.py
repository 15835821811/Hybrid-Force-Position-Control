"""Independent t0 actuator replay and public-packet decision replay."""
import time
import traceback
import numpy as np
from ..contracts import decode,encode,plain
from ..common import clean,save,sha
from ..replay import compare
from ..scheduler import ClockSpec
from .common import OUT,read
from .persistence import records,completed_inputs,validate
from .runner import controller,snapshot,verify_frozen,resource_used

def deterministic(x):
    if isinstance(x,dict):return {k:deterministic(v) for k,v in x.items() if k not in ('latency_s',)}
    if isinstance(x,list):return [deterministic(v) for v in x]
    return x

def replay_one(name,kind):
    verify_frozen();folder=OUT/'runs'/name;cfg=read(folder/'config.json');done=read(folder/'completion.json')
    dest=folder/(kind+'_replay.json')
    if dest.exists():raise FileExistsError('replay already recorded')
    checked=validate(folder/'raw');cpu,wall=time.process_time(),time.perf_counter();remaining=21600-resource_used()
    result=dict(run=name,kind=kind,passed=False,raw_validation=checked,physics_steps=0,predictive_steps=0,
        state_initializations=1,intermediate_plant_state_injections=0)
    try:
        if kind=='actuator':
            from v6_mujoco.adaptive_capture.plant import Plant,TruthConfig
            from v6_mujoco.end_to_end_capture.adapter import pads
            plant=Plant(TruthConfig(**cfg['truth_evaluation_only']),cfg['dt'],cfg['mission']['solver_tolerance'])
            first=next(records(folder/'raw'))['observation'];qerr=float(np.max(abs(plant.data.qpos-first['qpos'])));verr=float(np.max(abs(plant.data.qvel-first['qvel'])))
            masks=True;events=[];n=0;t_error=0.
            for applied,row in completed_inputs(folder/'raw'):
                if applied['latch']:plant.latch();events.append(applied['time'])
                plant.step(np.asarray(applied['tau']));obs=row['observation'];n+=1
                qerr=max(qerr,float(np.max(abs(plant.data.qpos-obs['qpos']))));verr=max(verr,float(np.max(abs(plant.data.qvel-obs['qvel']))))
                t_error=max(t_error,abs(plant.data.time-row['time']))
                actual=np.array([plant.model.geom_contype[pads(plant.model)],plant.model.geom_conaffinity[pads(plant.model)]])
                masks &= np.array_equal(actual,obs['interface_geom_masks']) and np.array_equal(plant.data.eq_active,obs['eq_active'])
                if time.process_time()-cpu>remaining:raise RuntimeError('INCOMPLETE_BUDGET')
            plant.assert_frozen()
            result.update(passed=qerr<1e-10 and verr<1e-9 and masks and t_error<1e-10,
                qpos_max_error=qerr,qvel_max_error=verr,time_max_error_s=t_error,masks_match=bool(masks),
                event_times=events,physics_steps=n,final_time_s=float(plant.data.time),
                terminal_unapplied_proposals_excluded=True)
        elif kind=='decision':
            wrapped=controller(cfg);clocks=ClockSpec(cfg['dt'],cfg['mission']['servo_s'],cfg['mission']['task_s'])
            n=0;max_error=0.;same=True;mismatches=[];actual=None;final_decision=None
            for r in records(folder/'raw'):
                if r['kind']=='packet_received':
                    p=decode(r['packet']);actual=wrapped.update(p,clocks.task_tick(p.t_control));n+=1
                elif r['kind']=='proposal':
                    saved=decode(r['proposal']);ok,err=compare(plain(actual),plain(saved));same &= ok;max_error=max(max_error,err)
                    ok,err=compare(deterministic(snapshot(wrapped)),deterministic(r['snapshot']));same &= ok;max_error=max(max_error,err)
                    if not ok and len(mismatches)<10:mismatches.append(dict(time=r['time'],kind='intermediate_snapshot'))
                    final_decision=actual.decision
                if time.process_time()-cpu>remaining:raise RuntimeError('INCOMPLETE_BUDGET')
            terminal_matches=(final_decision=='CONTROL' if done['status'] in ('COMPLETED','APPROACH_TIMEOUT') else final_decision==done['status'])
            result.update(passed=bool(same and max_error<1e-10 and terminal_matches),packets=n,
                max_intermediate_error=max_error,proposals_and_snapshots_match=bool(same),mismatches=mismatches,
                terminal_decision=final_decision,recorded_stop_reason=done['status'],stop_reason_match=terminal_matches,
                predictive_steps=wrapped.legacy.predictor.physics_steps,actual_guard_calls=len(wrapped.legacy.guard_calls))
        else:raise ValueError(kind)
    except Exception:result['error']=traceback.format_exc()
    finally:
        result.update(cpu_s=time.process_time()-cpu,wall_s=time.perf_counter()-wall)
        save(dest,clean(result));book=read(OUT/'run_ledger.json');book['replays'].append(dict(run=name,kind=kind,
            passed=result['passed'],cpu_s=result['cpu_s'],wall_s=result['wall_s'],result_sha256=sha(dest),
            physics_steps=result['physics_steps'],predictive_steps=result['predictive_steps']));save(OUT/'run_ledger.json',book)
    print(dict(run=name,replay=kind,passed=result['passed'],cpu_s=result['cpu_s']),flush=True)
    return result

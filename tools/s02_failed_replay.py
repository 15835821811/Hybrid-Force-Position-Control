"""Run in an exact archived source runtime, with external original evidence root."""
import os
import time
import traceback
from pathlib import Path
import numpy as np
from v6_mujoco.system_capture.estimation.common import read,save,sha
from v6_mujoco.system_capture.estimation.persistence import records,completed_inputs,validate
from v6_mujoco.system_capture.estimation.runner import controller,snapshot
from v6_mujoco.system_capture.estimation.replay import deterministic
from v6_mujoco.system_capture.replay import compare
from v6_mujoco.system_capture.contracts import decode,plain
from v6_mujoco.system_capture.scheduler import ClockSpec

def main():
    out=Path(os.environ['S02_EVIDENCE_ROOT']);folder=out/'runs/E0';cfg=read(folder/'config.json')
    for rel,h in cfg['source_identity'].items():
        if sha(Path.cwd()/rel)!=h:raise RuntimeError('archived runtime identity mismatch '+rel)
    cpu,wall=time.process_time(),time.perf_counter()
    from v6_mujoco.adaptive_capture.plant import Plant,TruthConfig
    plant=Plant(TruthConfig(**cfg['truth_evaluation_only']),cfg['dt'],cfg['mission']['solver_tolerance'])
    qerr=verr=0.;n=0
    for app,row in completed_inputs(folder/'raw'):
        if app['latch']:plant.latch()
        plant.step(np.array(app['tau']));obs=row['observation'];n+=1
        qerr=max(qerr,float(np.max(abs(plant.data.qpos-obs['qpos']))));verr=max(verr,float(np.max(abs(plant.data.qvel-obs['qvel']))))
    plant.assert_frozen()
    act=dict(passed=qerr<1e-10 and verr<1e-9,qpos_max_error=qerr,qvel_max_error=verr,physics_steps=n,
        final_time_s=float(plant.data.time),state_initializations=1,intermediate_plant_state_injections=0,
        cpu_s=time.process_time()-cpu,wall_s=time.perf_counter()-wall,scope='completed committed input prefix only; E0 implementation error retained')
    save(folder/'actuator_replay.json',act);print('E0 actuator '+str(act),flush=True)
    cpu,wall=time.process_time(),time.perf_counter();wrapped=controller(cfg);clocks=ClockSpec(cfg['dt'],cfg['mission']['servo_s'],cfg['mission']['task_s'])
    same=True;err=0.;count=0;caught=None;caught_time=None;recorded_exception=None
    for row in records(folder/'raw'):
        if row['kind']=='packet_received':
            p=decode(row['packet']);count+=1
            try:proposal=wrapped.update(p,clocks.task_tick(p.t_control))
            except Exception as exc:caught=type(exc).__name__+': '+str(exc);caught_time=p.t_control
        elif row['kind']=='proposal':
            if caught:raise RuntimeError('replay raised before recorded failure')
            ok,delta=compare(plain(proposal),plain(decode(row['proposal'])));same &=ok;err=max(err,delta)
            ok,delta=compare(deterministic(snapshot(wrapped)),deterministic(row['snapshot']));same &=ok;err=max(err,delta)
        elif row['kind']=='exception':recorded_exception=row['reason'].strip().splitlines()[-1]
        if count and count%1000==0 and row['kind']=='proposal':print('E0 decisions '+str(count),flush=True)
    stop=caught==recorded_exception and caught_time==read(folder/'completion.json')['end_time_s']
    dec=dict(passed=bool(same and err<1e-10 and stop),packets=count,max_intermediate_error=err,
        proposals_and_snapshots_match=bool(same),stop_reason_match=bool(stop),recorded_exception=recorded_exception,
        replay_exception=caught,exception_time_s=caught_time,predictive_steps=wrapped.legacy.predictor.physics_steps,
        cpu_s=time.process_time()-cpu,wall_s=time.perf_counter()-wall,
        scope='exact original frozen implementation, no repaired data; expected failure reproduced')
    save(folder/'decision_replay.json',dec);print('E0 decision '+str(dec),flush=True)
if __name__=='__main__':main()

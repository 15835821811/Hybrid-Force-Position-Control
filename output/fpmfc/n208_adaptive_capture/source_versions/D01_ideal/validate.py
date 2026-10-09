"""Two distinct replays: actuator-input plant replay and packet decision replay."""
import argparse
import gzip
import json
import time
from pathlib import Path
import numpy as np
from .common import ROOT,read,save,digest,init_ledger,runtime_identity
from .plant import Plant,TruthConfig
from .controller import Controller
from .contracts import SensorPacket,PoseObservation
from v6_mujoco.postgrasp_campaign.engine import integration_state

def packet_from_dict(d):
    poses=tuple(PoseObservation(x['stamp'],np.array(x['position']),np.array(x['rotation']),np.array(x['covariance'])) for x in d['poses'])
    return SensorPacket(d['time'],*[np.array(d[k]) for k in ['base_pose','base_velocity','joint_position','joint_velocity','actuator_torque','wrench_target_at_grasp_world']],d['contact'],poses)

def run(name,decisions=True):
    out=ROOT/'runs'/name;cfg=read(out/'config.json');metrics=read(out/'metrics.json');cpu=time.process_time();started=time.time()
    with np.load(out/'trace.npz') as z:a={k:z[k] for k in z.files}
    plant=Plant(TruthConfig(**cfg['truth_evaluation_only']),cfg['dt']);errors=np.zeros(2);event_i=None
    # Fresh t=0 initializer. No readback state injection, including at latch.
    for i,t in enumerate(a['time_s']):
        if a['eq_active'][i,0] and not plant.data.eq_active[0]:plant.latch();event_i=i
        plant.data.ctrl[:]=a['ctrl_nm'][i]
        errors=np.maximum(errors,[np.max(abs(plant.data.qpos-a['qpos'][i])),np.max(abs(plant.data.qvel-a['qvel'][i]))])
        if i+1<len(a['time_s']):plant.step(a['ctrl_nm'][i])
    plant.assert_frozen();torque_error=0.;decision_matches=True;state_error=0.;packet_count=0
    if decisions:
        current=runtime_identity()
        # Refuse to claim historical decision replay under changed controller.
        online=['controller.py','state_estimator.py','inertial_estimator.py','relative_reference.py','known_model.py','momentum_regressor.py','contracts.py']
        for path,sha in cfg['identity'].items():
            if any(path.endswith('/'+n) for n in online) and current.get(path)!=sha:raise RuntimeError('controller identity changed; use archived source or mark decision replay unavailable: '+path)
        c=Controller(cfg['prior'],cfg['mission'])
        with gzip.open(out/'packets.jsonl.gz','rt',encoding='utf-8') as f:
            for line in f:
                row=json.loads(line);packet=packet_from_dict(row['packet']);step=round(packet.time/cfg['mission']['servo_s']);task_stride=round(cfg['mission']['task_s']/cfg['mission']['servo_s'])
                tau,latch=c.update(packet,step%task_stride==0);torque_error=max(torque_error,float(np.max(abs(tau-row['torque']))));decision_matches&=bool(latch==row['latch']);packet_count+=1
    result={'actuator_replay_passed':bool(np.all(errors<np.array([1e-10,1e-9]))),'qpos_max_error':float(errors[0]),'qvel_max_error':float(errors[1]),'latch_event_index':event_i,'physics_steps':len(a['time_s'])-1,
        'decision_replay_passed':bool(torque_error<1e-10 and decision_matches) if decisions else None,'decision_torque_max_error':torque_error if decisions else None,'packets':packet_count,'truth_states_used_for_feedback':False,'trace_sha256':digest(out/'trace.npz'),'validator_identity':runtime_identity(),'cpu_s':time.process_time()-cpu,'wall_s':time.time()-started}
    path=out/'validation.json'
    if path.exists():path=out/('validation_'+str(int(time.time()))+'.json')
    save(path,result);ledger=init_ledger();ledger['cpu_s']+=result['cpu_s'];ledger['replays'].append({'run':name,'path':str(path.relative_to(ROOT)),'cpu_s':result['cpu_s'],'plant_steps':result['physics_steps'],'decision_packets':packet_count});save(ROOT/'run_ledger.json',ledger)
    print(json.dumps({k:v for k,v in result.items() if k!='validator_identity'}),flush=True);return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('name');p.add_argument('--plant-only',action='store_true');a=p.parse_args();run(a.name,not a.plant_only)

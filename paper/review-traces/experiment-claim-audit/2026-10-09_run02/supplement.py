from audit_inspect import *
import numpy as np,gzip
ROOT=PAPER.parent;BASE=ROOT/'output/fpmfc/n209_paper_system';out={}
for a in read(BASE/'run_ledger.json')['attempts']:
    p=BASE/'runs'/a['name'];assert p/'trace.npz' in FILES.values()
    with np.load(p/'trace.npz',allow_pickle=False) as z:
        arr={k:z[k] for k in ['time_s','flange_position_world_m','physical_tool_face_position_world_m','joint_position_rad','joint_velocity_rad_s','estimated_p','estimated_R','estimated_v','estimated_w','qpos','ctrl_nm','estimated_capture_values','estimated_capture_margins','parameter_pi']}
    gov=read(p/'progress_governor.json');cfg=read(p/'config.json');states=read(p/'control_states.json');est=read(p/'estimator_events.json');events=read(p/'events.json')
    obs=[];packet_times=[];packet_torques=[]
    with gzip.open(p/'packets.jsonl.gz','rt',encoding='utf-8') as f:
        for line in f:
            d=json.loads(line);packet_times.append(d['packet']['time']);packet_torques.append(d['torque'])
            for pose in d['packet']['poses']:obs.append(pose)
    stamps=[e['stamp'] for e in est]
    latches=[e for e in events if e['state']=='LATCH_REQUEST']
    out[a['name']]={'tool_offset_min':float(np.linalg.norm(arr['physical_tool_face_position_world_m']-arr['flange_position_world_m'],axis=1).min()),'tool_offset_max':float(np.linalg.norm(arr['physical_tool_face_position_world_m']-arr['flange_position_world_m'],axis=1).max()),'predictor_min_s':min(g['latency_s'] for g in gov),'predictor_p50_s':float(np.median([g['latency_s'] for g in gov])),'predictor_calls':len(gov),'packet_time_error':float(np.max(abs(np.array(packet_times)-arr['time_s']))),'packet_torque_max_difference':float(np.max(abs(np.array(packet_torques)-arr['ctrl_nm']))),'raw_pose_count':len(obs),'estimator_events':len(est),'event_timestamps_strict':bool(np.all(np.diff(stamps)>0)),'max_measurement_event_delay':None if not est else max(e['arrival']-e['stamp'] for e in est),'latches':[{'time':e['time'],'gate_values':e['estimated_gate']['values'],'margins':e['estimated_gate']['margins'],'limits':e['estimated_gate']['limits'],'all_gate_inequalities':bool(np.all(np.array(e['estimated_gate']['values'])+e['estimated_gate']['margins']<=e['estimated_gate']['limits'])),'true_capture':e['true_capture']} for e in latches]}
    if a['sensor']=='noisy':out[a['name']]['example_pose']=obs[0]
cfgs={a['name']:read(BASE/'runs'/a['name']/'config.json') for a in read(BASE/'run_ledger.json')['attempts']}
def diff(a,b,path=''):
    if isinstance(a,dict) and isinstance(b,dict):
        return sum((diff(a.get(k),b.get(k),path+'.'+k) for k in set(a)|set(b)),[])
    return [] if a==b else [{'path':path,'first':a,'second':b}]
out['mission_diff_D02_D04']=diff(cfgs['D02_V1_H2']['mission'],cfgs['D04_V2_H2']['mission'])
out['sensor_old_new_diff']=diff(read(ROOT/'output/fpmfc/n208_adaptive_capture/runs/S01_noise_delay/config.json')['sensors'],cfgs['D03_V1_S01']['sensors'])
out['old_noisy_process']={k:v for k,v in read(ROOT/'output/fpmfc/n208_adaptive_capture/runs/S01_noise_delay/config.json')['mission'].items() if 'process' in k}
(HERE/'supplement_metrics.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
print(json.dumps(out,indent=2))

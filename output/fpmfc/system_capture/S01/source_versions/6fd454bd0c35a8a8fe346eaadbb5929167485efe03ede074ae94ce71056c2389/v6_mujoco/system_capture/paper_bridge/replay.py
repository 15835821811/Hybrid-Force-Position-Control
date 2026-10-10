"""Independent actuator replay: one initialization, recorded inputs, no state injection."""
import time
import mujoco
import numpy as np
from .common import OUT,PROJECT,assumptions,read,save,sha
from .model import compile_model
from .phase import book,reserve,finish,remaining_cpu

def run():
    results=[]
    for original in book()['dynamics']:
        if original['status'] not in ('COMPLETED','FAILED'):raise RuntimeError('dynamics attempt unresolved: '+original['key'])
        key=original['key'];folder=(PROJECT/original['result_file']).parent;identity=read(folder/'identity.json');path=OUT/'replays'/(key+'.json')
        assert sha(folder/'trace.npz')==identity['trace_sha256']
        row,new=reserve('replays',key,{'trace_sha256':identity['trace_sha256'],'original_parameters':original['parameters']})
        if not new:results.append(read(PROJECT/row['result_file']));continue
        cpu=time.process_time();wall=time.perf_counter();m=compile_model(original['parameters']['dt']);d=mujoco.MjData(m)
        with np.load(folder/'trace.npz',allow_pickle=False) as archive:a={k:archive[k] for k in archive.files}
        d.qpos[:]=a['qpos'][0];d.qvel[:]=a['qvel'][0];d.time=float(a['time_s'][0]);mujoco.mj_forward(m,d)
        maxq=0.;maxv=0.;maxt=0.
        for i,tau in enumerate(a['applied_ctrl']):
            if remaining_cpu()<=0:raise RuntimeError('BUDGET_EXHAUSTED')
            m.opt.timestep=float(a['integration_dt_s'][i]);d.ctrl[:]=tau;mujoco.mj_step(m,d)
            maxq=max(maxq,float(abs(d.qpos-a['qpos'][i+1]).max()));maxv=max(maxv,float(abs(d.qvel-a['qvel'][i+1]).max()));maxt=max(maxt,abs(d.time-a['time_s'][i+1]))
        tolerances=assumptions()['numerical_tests'];passed=maxq<=tolerances['replay_qpos'] and maxv<=tolerances['replay_qvel'] and maxt<1e-10 and len(a['applied_ctrl'])==len(a['qpos'])-1
        result={'run':key,'passed':bool(passed),'qpos_max_error':maxq,'qvel_max_error':maxv,'time_max_error_s':maxt,'physics_steps':len(a['applied_ctrl']),'state_initializations':1,'saved_intermediate_state_injections':0,
          'actuator_channels':7,'trace_sha256':identity['trace_sha256'],'original_status':read(folder/'metrics.json')['status'],'cpu_s':time.process_time()-cpu,'wall_s':time.perf_counter()-wall,'independent_replay':True,'not_a_new_performance_attempt':True}
        save(path,result);finish('replays',key,path,'COMPLETED' if passed else 'FAILED');results.append(result);print('S01 replay '+key+' '+str(passed),flush=True)
    save(OUT/'replay_summary.json',{'passed':len(results)==len(book()['dynamics']) and all(r['passed'] for r in results),'count':len(results),'results':results})

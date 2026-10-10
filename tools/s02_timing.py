"""Missing legacy/new per-call timing, from existing public packets only."""
import gzip
import json
import time
import numpy as np
from v6_mujoco.system_capture.estimation.common import OUT,OLD,PROJECT,read,save,charge,sha
from v6_mujoco.system_capture.estimation.runner import verify_frozen
from v6_mujoco.system_capture.estimation.filter import AccelerationEstimator,predict_snapshot
from v6_mujoco.feasible_capture.estimator_adapter import TimedEstimator
from v6_mujoco.adaptive_capture.validate import packet_from_dict

def main():
    verify_frozen();cpu,wall=time.process_time(),time.perf_counter()
    path=OLD/'runs/R03_V2_S01/packets.jsonl.gz'
    with gzip.open(path,'rt',encoding='utf-8') as f:packets=[packet_from_dict(json.loads(line)['packet']) for line in f]
    cfg=read(OLD/'runs/R03_V2_S01/config.json')['mission']
    cfg['s02_estimator']=read(PROJECT/'configs/system_capture/algorithms/s02_acceleration.json')
    results={};arrays={}
    for name,cls in (('CV',TimedEstimator),('CA',AccelerationEstimator)):
        f=cls(cfg);updates=[];forecasts=[]
        for packet in packets:
            start=time.perf_counter();e=f.update(packet);updates.append(time.perf_counter()-start)
            start=time.perf_counter()
            if name=='CA':predict_snapshot(e,e.time+.012)
            else:f.propagate(e.p,e.R,e.v,e.w,e.covariance,.012)
            forecasts.append(time.perf_counter()-start)
        arrays[name+'_update_s']=updates;arrays[name+'_blind_12ms_s']=forecasts
        results[name]=dict(calls=len(packets),update_quantiles_s=np.quantile(updates,[.5,.95,.99,1.]),
            blind_12ms_quantiles_s=np.quantile(forecasts,[.5,.95,.99,1.]))
    dest=OUT/'timing_calls.npz';np.savez_compressed(dest,**arrays)
    save(OUT/'estimator_timing.json',dict(source='archived R03 public packets; no robot integration',
        source_sha256=sha(path),raw_sha256=sha(dest),by_method=results,quantiles=[.5,.95,.99,1.],
        scope='single host sample, sequential method order, concurrent robot process; includes Python logging and snapshot copies; no hardware realtime certification',
        physics_steps=0,additional_parameter_selection=False))
    charge('per_call_estimator_timing',cpu,wall,new_robot_steps=0,updates=2*len(packets),blind_forecasts=2*len(packets))
    print('per-call timing recorded',len(packets))

if __name__=='__main__':main()

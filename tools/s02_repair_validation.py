"""Validate implementation repair on the original failed public packet prefix."""
import time
import numpy as np
from v6_mujoco.system_capture.estimation.common import OUT,read,save,charge
from v6_mujoco.system_capture.estimation.filter import AccelerationEstimator,angular_step
from v6_mujoco.system_capture.estimation.persistence import records
from v6_mujoco.system_capture.contracts import decode
from v6_mujoco.system_capture.adapters import packet_to_legacy,estimate_from_legacy

cpu,wall=time.process_time(),time.perf_counter()
cfg=read(OUT/'runs/E0/config.json');f=AccelerationEstimator(cfg['mission']);worst=0.;n=0;failed=[]
for row in records(OUT/'runs/E0/raw'):
    if row['kind']=='packet_received':
        e=f.update(packet_to_legacy(decode(row['packet'])));n+=1
        low=float(np.linalg.eigvalsh(e.full_covariance).min());worst=min(worst,low)
        try:estimate_from_legacy(e,cfg['mission'],'CONTACT' if e.process_contact else 'FREE')
        except Exception as ex:failed.append(dict(time=e.time,error=str(ex)))
result=dict(passed=not failed,packets=n,min_eigenvalue=worst,last_time_s=e.time,
    failures=failed,source='original E0 public packets, including terminal failed packet',
    physics_steps=0,parameter_change=False,noise_matrix='positive Gauss-Legendre noise-factor integral, no clipping or shrink')
save(OUT/'covariance_repair_tests.json',result);charge('covariance_repair_recorded_packets',cpu,wall,**result);print(result)

"""Truth access for evaluation only. No evaluator values feed the controller."""
import copy
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from v6_mujoco.geometry_capture.observer import observe as old_observe,safety as old_safety,capture
from v6_mujoco.geometry_capture.reference import Reference
from v6_mujoco.postgrasp.physics import body_jacobian
from v6_mujoco.model import body_id

PHASES={'OBSERVE':0,'RENDEZVOUS':1,'RELATIVE_APPROACH':2,'CAPTURE_WINDOW':3,'LATCH':4,'GRASP_VERIFY':5,'DAMP_TRANSFER':6,'HOLD':7,'ABORT':8}

def observe(plant,controller,first=None):
    m,d=plant.model,plant.data;e=controller.estimate
    reference=controller.reference if e is not None and e.valid else controller.reference.nominal
    row=old_observe(m,d,reference,controller.shape)
    # Old analytic-free-motion diagnostic remains visible but is not a safety
    # gate for arbitrary non-spherical passive dynamics. It is not set to zero.
    row['phase_code']=PHASES[controller.phase]
    row['progress_s']=controller.reference.progress
    row['reference_rate']=controller.reference.rate
    row['latch_local_s']=float(d.time)-controller.latch_time if controller.latch_time is not None else -1.
    row['approach_reference_applicable']=controller.latch_time is None
    row['estimated_p']=e.p.copy() if e is not None else np.zeros(3)
    row['estimated_R']=e.R.copy() if e is not None else np.eye(3)
    row['estimated_v']=e.v.copy() if e is not None else np.zeros(3)
    row['estimated_w']=e.w.copy() if e is not None else np.zeros(3)
    row['estimate_covariance']=e.covariance.copy() if e is not None else np.zeros((12,12))
    row['estimate_valid']=e.valid if e is not None else False
    row['measurement_age_s']=float(d.time)-e.measurement_time if e is not None else -1.
    row['innovation']=e.innovation.copy() if e is not None else np.zeros(6)
    row['parameter_pi']=controller.estimator.pi.copy();row['parameter_rank']=controller.estimator.rank
    s=copy.copy(d);mujoco.mj_forward(m,s);b=body_id(m,'tumbling_target');twist=body_jacobian(m,s,b,s.xpos[b])@s.qvel
    row['target_geometric_twist_world']=twist
    row['estimate_error']=np.r_[row['estimated_p']-s.xpos[b],Rotation.from_matrix(row['estimated_R']@s.xmat[b].reshape(3,3).T).as_rotvec(),row['estimated_v']-twist[:3],row['estimated_w']-twist[3:]]
    return row

def safety(row,first,m):
    # True safety may abort a simulation; never authorize latch, fix estimates,
    # change gains, move a frame, or choose a more favourable result window.
    return old_safety(row,first,m,had_contact=True)

def summarize(rows,status,latch_time,cfg):
    a={k:np.array([r[k] for r in rows]) for k in rows[0]};t=a['time_s'];complete=latch_time is not None and abs(t[-1]-latch_time-cfg['post_duration_s'])<1e-7 and status=='COMPLETED'
    W=None if latch_time is None else [latch_time+cfg['post_duration_s']-cfg['evaluation_width_s'],latch_time+cfg['post_duration_s']]
    window=np.zeros(len(t),bool) if W is None else (t>=W[0]-1e-9)&(t<=W[1]+1e-9)
    spin=np.rad2deg(np.linalg.norm(a['target_omega_world_rad_s'],axis=1));rel=np.rad2deg(np.linalg.norm(a['target_base_relative_omega_world_rad_s'],axis=1));perf=(spin<=cfg['performance']['world_deg_s'])&(rel<=cfg['performance']['relative_deg_s'])
    first=lambda mask:float(t[np.flatnonzero(mask)[0]]) if np.any(mask) else None
    valid=a['estimate_valid'].astype(bool);post=a['holding_error_applicable'].astype(bool)
    enter=np.r_[perf[0],perf[1:]&~perf[:-1]];leave=np.r_[False,~perf[1:]&perf[:-1]]
    work={k:float(np.trapz(a[k],t)) for k in ['actuator_power_w','passive_power_w','all_constraint_power_w','latch_constraint_power_w']}
    result={'status':status,'end_time_s':float(t[-1]),'latch_time_s':latch_time,'evaluation_window_s':W,'continuous_task_completed':complete,'full_window_evaluated':complete,'postgrasp_detumbling':bool(complete and np.all(perf[window])),
        'world_window_max_deg_s':float(max(spin[window])) if complete else None,'relative_window_max_deg_s':float(max(rel[window])) if complete else None,'first_performance_entry_s':first(enter),'performance_exit_times_s':t[leave].tolist(),
        'max_P_drift_kg_m_s':float(np.max(np.linalg.norm(a['linear_momentum_world_kg_m_s']-a['linear_momentum_world_kg_m_s'][0],axis=1))),'max_H_drift_kg_m2_s':float(np.max(np.linalg.norm(a['angular_momentum_about_center_world_kg_m2_s']-a['angular_momentum_about_center_world_kg_m2_s'][0],axis=1))),
        'max_load_fraction':float(max(a['load_fraction'])),'peak_contact_force_n':float(max(a['contact_peak_force_n'])),'first_contact_s':first(a['intentional_contact_count']>0),'work_j':work,
        'energy_balance_residual_j':float(a['kinetic_energy_j'][-1]-a['kinetic_energy_j'][0]-sum(work[k] for k in ['actuator_power_w','passive_power_w','all_constraint_power_w'])),
        'state_error_rms_components':np.sqrt(np.mean(a['estimate_error'][valid]**2,axis=0)).tolist() if np.any(valid) else None,'state_error_max_components':np.max(abs(a['estimate_error'][valid]),axis=0).tolist() if np.any(valid) else None,
        'max_post_translation_m':float(max(a['interface_translation_error_m'][post])) if np.any(post) else None,'max_post_rotation_deg':float(max(a['interface_rotation_error_deg'][post])) if np.any(post) else None,
        'final_identification_rank':int(a['parameter_rank'][-1]),'parameter_feedback_used':False,'mode':'shadow identification / fixed prior control','real_time_ready':False,'hardware_validated':False}
    return a,result

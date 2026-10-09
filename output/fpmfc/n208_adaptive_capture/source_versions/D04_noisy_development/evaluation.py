"""Truth access for evaluation only. No evaluator values feed the controller."""
import copy
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from v6_mujoco.geometry_capture.observer import observe as old_observe,safety as old_safety,capture
from v6_mujoco.geometry_capture.reference import Reference
from v6_mujoco.postgrasp.physics import body_jacobian
from v6_mujoco.model import body_id,default_model_spec
from v6_mujoco.geometry_capture.design import extra_pairs,config as geometry_config
from .physical_audit import inspect as physical_inspect

PHASES={'OBSERVE':0,'RENDEZVOUS':1,'RELATIVE_APPROACH':2,'CAPTURE_WINDOW':3,'LATCH':4,'GRASP_VERIFY':5,'DAMP_TRANSFER':6,'HOLD':7,'ABORT':8}
GEOMETRY_CONFIG=geometry_config() # immutable N206 contract, read once per process

def observe(plant,controller,first=None):
    m,d=plant.model,plant.data;e=controller.estimate
    reference=controller.reference if e is not None and e.valid else controller.reference.nominal
    row=old_observe(m,d,reference,controller.shape)
    # Old analytic-free-motion diagnostic remains visible but is not a safety
    # gate for arbitrary non-spherical passive dynamics. It is not set to zero.
    row['phase_code']=PHASES[controller.phase]
    row['progress_s']=controller.reference.progress
    row['reference_rate']=controller.reference.rate
    row['reference_acceleration']=controller.reference.accel
    row['reference_jerk']=controller.reference.jerk
    row['estimated_capture_values']=np.asarray(controller.last_gate.get('values',[0.]*4))
    row['estimated_capture_margins']=np.asarray(controller.last_gate.get('margins',[0.]*4))
    row['capture_confirmation_start_s']=controller.confirm_start if controller.confirm_start is not None else -1.
    row['impact_effective_mass_kg']=controller.impact.get('effective_mass_kg',0.)
    row['impact_upper_energy_j']=controller.impact.get('upper_energy_j',0.)
    row['brake_alpha']=controller.governor.alpha if controller.governor is not None else 1.
    row['parameter_feedback_used']=controller.feedback_used
    row['latch_local_s']=float(d.time)-controller.latch_time if controller.latch_time is not None else -1.
    row['local_postlatch_time_s']=row['latch_local_s']
    row['approach_reference_applicable']=controller.latch_time is None
    row['estimated_p']=e.p.copy() if e is not None else np.zeros(3)
    row['estimated_R']=e.R.copy() if e is not None else np.eye(3)
    row['estimated_v']=e.v.copy() if e is not None else np.zeros(3)
    row['estimated_w']=e.w.copy() if e is not None else np.zeros(3)
    row['estimate_covariance']=e.covariance.copy() if e is not None else np.zeros((12,12))
    row['estimate_valid']=e.valid if e is not None else False
    row['measurement_age_s']=float(d.time)-e.measurement_time if e is not None else -1.
    row['estimate_output_time_s']=e.time if e is not None else -1.
    row['estimate_current_tick']=e is not None and abs(float(d.time)-e.time)<1e-9
    row['innovation']=e.innovation.copy() if e is not None else np.zeros(6)
    row['parameter_pi']=controller.estimator.pi.copy();row['parameter_rank']=controller.estimator.rank
    s=copy.copy(d);mujoco.mj_forward(m,s);b=body_id(m,'tumbling_target');twist=body_jacobian(m,s,b,s.xpos[b])@s.qvel
    row['target_geometric_twist_world']=twist
    row['estimate_error']=np.r_[row['estimated_p']-s.xpos[b],Rotation.from_matrix(row['estimated_R']@s.xmat[b].reshape(3,3).T).as_rotvec(),row['estimated_v']-twist[:3],row['estimated_w']-twist[3:]]
    row.update(physical_inspect(m,s))
    return row

def safety(row,first,m):
    # True safety may abort a simulation; never authorize latch, fix estimates,
    # change gains, move a frame, or choose a more favourable result window.
    if not all(np.all(np.isfinite(v)) for v in row.values()):return 'MOMENTUM_OR_NUMERICS_FAILED','nonfinite'
    for key,limit in [('linear_momentum_world_kg_m_s',1e-4),('angular_momentum_about_center_world_kg_m2_s',1e-5)]:
        if np.linalg.norm(np.asarray(row[key])-first[key])>limit:return 'MOMENTUM_OR_NUMERICS_FAILED',key
    if np.any(row['qfrc_applied']) or np.any(row['xfrc_applied']) or np.any(row['solver_warning_count']):return 'MOMENTUM_OR_NUMERICS_FAILED','external force or solver warning'
    for key,limit in [('solver_residual',1e-7),('wrench_reconstruction_error',1e-8),('action_reaction_force_error_n',1e-8),('action_reaction_moment_geometry_error_nm',1e-8)]:
        if row[key]>limit:return 'MOMENTUM_OR_NUMERICS_FAILED',key
    spec=default_model_spec();jids=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n) for n in spec.joint_names];limits=m.jnt_range[jids]
    post=bool(row['holding_error_applicable']);kind='POSTGRASP_SAFETY_FAILED' if post else 'PRECONTACT_TRACKING_FAILED'
    if np.any(row['joint_position_rad']<limits[:,0]-1e-8) or np.any(row['joint_position_rad']>limits[:,1]+1e-8) or np.any(abs(row['joint_velocity_rad_s'])>spec.velocity_limits_rad_s+1e-8) or np.any(abs(row['ctrl_nm'])>spec.torque_limits_nm+1e-8) or np.any(abs(row['actuator_force_nm'])>spec.torque_limits_nm+1e-8):return kind,'joint/torque limit'
    if row['minimum_noncontact_clearance_m']<.04:return 'UNINTENDED_CONTACT_OR_COLLISION','original 40 mm pair'
    for pair,distance in zip(extra_pairs(m),row['extra_pair_distances_m']):
        limit=-.002 if pair.category=='intended_surface' else (.04 if pair.category=='tool_robot' else GEOMETRY_CONFIG['new_collision_margin_m'])
        if distance<limit-GEOMETRY_CONFIG['numerical_penetration_tolerance_m']:return 'UNINTENDED_CONTACT_OR_COLLISION',pair.name
    if row['unintended_contact_count']:return 'UNINTENDED_CONTACT_OR_COLLISION','unplanned physical contact'
    if row['load_fraction']>1:return kind,'actual rho > 1'
    for key,limit in [('interface_generalized_reconstruction_error',1e-8),('interface_power_residual_w',1e-8),('target_newton_residual_n',1e-6),('target_euler_residual_nm',1e-7)]:
        if np.linalg.norm(row[key])>limit:return 'MOMENTUM_OR_NUMERICS_FAILED',key
    if post:
        if row['interface_translation_error_m']>.0005 or row['interface_rotation_error_deg']>.1:return 'POSTGRASP_SAFETY_FAILED','holding pose'
    elif row['contact_peak_force_n']>20 or row['contact_penetration_m']>.002:return 'UNINTENDED_CONTACT_OR_COLLISION','prelatch 20 N/2 mm contact gate'
    return None,None

def summarize(rows,status,latch_time,cfg):
    a={k:np.array([r[k] for r in rows]) for k in rows[0]};t=a['time_s'];complete=bool(latch_time is not None and abs(t[-1]-latch_time-cfg['post_duration_s'])<1e-7 and status=='COMPLETED')
    W=None if latch_time is None else [latch_time+cfg['post_duration_s']-cfg['evaluation_width_s'],latch_time+cfg['post_duration_s']]
    window=np.zeros(len(t),bool) if W is None else (t>=W[0]-1e-9)&(t<=W[1]+1e-9)
    spin=np.rad2deg(np.linalg.norm(a['target_omega_world_rad_s'],axis=1));rel=np.rad2deg(np.linalg.norm(a['target_base_relative_omega_world_rad_s'],axis=1));perf=(spin<=cfg['performance']['world_deg_s'])&(rel<=cfg['performance']['relative_deg_s'])
    first=lambda mask:float(t[np.flatnonzero(mask)[0]]) if np.any(mask) else None
    valid=a['estimate_valid'].astype(bool)
    if 'estimate_current_tick' in a:valid &= a['estimate_current_tick'].astype(bool)
    post=a['holding_error_applicable'].astype(bool)
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
    if np.any(valid):
        sig=np.sqrt(np.maximum(0,np.diagonal(a['estimate_covariance'][valid],axis1=1,axis2=2)))
        result['empirical_marginal_3sigma_coverage']=np.mean(abs(a['estimate_error'][valid])<=3*sig,axis=0).tolist()
        result['covariance_scope']='single-trajectory empirical marginal coverage; no calibrated joint safety probability'
    return a,result

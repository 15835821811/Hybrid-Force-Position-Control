"""Task-level descriptive summaries. Never turn time samples into trial counts."""
import numpy as np
from scipy.spatial.transform import Rotation
from v6_mujoco.model import default_model_spec
from .common import ROOT,read,save,sha,ledger


def run():
    output=[]
    for entry in ledger()['attempts']:
        path=ROOT/'runs'/entry['name']
        if not (path/'metrics.json').exists():
            output.append({**entry,'analysis_status':'INCOMPLETE'});continue
        m=read(path/'metrics.json');cfg=read(path/'config.json');v=read(path/'validation.json') if (path/'validation.json').exists() else {}
        row={**entry,**m,'validation':v,'config_sha256':sha(path/'config.json'),'metrics_sha256':sha(path/'metrics.json')}
        z=np.load(path/'trace.npz');t=z['time_s'];mask=z['estimate_valid'].astype(bool);base=z['base_omega_world_rad_s']
        row['base_peak_omega_deg_s']=float(np.rad2deg(np.linalg.norm(base,axis=1)).max())
        row['base_rms_omega_deg_s']=float(np.rad2deg(np.sqrt(np.mean(np.sum(base**2,axis=1)))))
        R=Rotation.from_quat(z['qpos'][:,[4,5,6,3]]);row['base_max_attitude_drift_deg']=float(np.rad2deg((R*R[0].inv()).magnitude()).max())
        row['physical_capture_at_latch']='NOT_EVALUATED'
        if m['latch_time_s'] is not None:
            k=int(np.argmin(abs(t-m['latch_time_s'])))
            row['physical_capture_at_latch']={'translation_m':float(z['interface_translation_error_m'][k]),'rotation_deg':float(z['interface_rotation_error_deg'][k]),
                'linear_m_s':float(z['physical_tool_face_relative_linear_speed_m_s'][k]),'angular_deg_s':float(np.rad2deg(z['physical_flange_relative_angular_speed_rad_s'][k]))}
        row['post_window_status']='EVALUATED' if m['full_window_evaluated'] else 'NOT_EVALUATED'
        row['maximum_measurement_age_s']=float(z['measurement_age_s'][mask].max()) if any(mask) else None
        row['estimator_by_phase']={}
        for phase in np.unique(z['phase_code']):
            select=mask & (z['phase_code']==phase)
            if any(select):row['estimator_by_phase'][str(int(phase))]={'sample_count':int(select.sum()),'RMS_components':np.sqrt(np.mean(z['estimate_error'][select]**2,axis=0)).tolist()}
        row['min_noncontact_clearance_m']=float(z['minimum_noncontact_clearance_m'].min())
        row['peak_actual_torque_nm']=float(abs(z['actuator_force_nm']).max())
        row['minimum_joint_velocity_margin_rad_s']=float(np.min(default_model_spec().velocity_limits_rad_s-abs(z['joint_velocity_rad_s'])))
        rows=read(path/'control_states.json');gates=[x['gate'] for x in rows if x.get('gate',{}).get('limits')]
        row['capture_uncertainty_alone_blocks_samples']=0;row['capture_gate_samples']=len(gates)
        if gates:
            G=np.array([x['margins'] for x in gates]);L=np.array(gates[0]['limits'])
            row['capture_uncertainty_alone_blocks_samples']=int(np.any(G>L,axis=1).sum())
            row['minimum_capture_uncertainty_margin']=G.min(axis=0).tolist();row['capture_limits']=L.tolist()
        logs=read(path/'progress_governor.json');row['progress_interventions']=sum(x['chosen_fraction'] is not None and x['chosen_fraction']<1 for x in logs)
        row['candidate_rejections']=sum(not c['verified'] for x in logs for c in x['candidates'])
        row['predictor_max_latency_s']=max([x['latency_s'] for x in logs] or [0.])
        row['latency_record_caveat']=m.get('latency_scope','Legacy c.timings omits terminal exception; predictor_max_latency_s includes it. Do not equate legacy reported max to worst final call.')
        row['failure_mechanism']='NONE' if m['continuous_task_completed'] else ('PREDICTIVE_TASK_ADMISSION_REJECTION' if m['status']=='NO_VERIFIED_CONTROL' else m['status'])
        save(path/'analysis.json',row);output.append(row)
    finished=[x for x in output if 'continuous_task_completed' in x]
    holdouts=[x for x in finished if x['category']=='holdout']
    result={'scope':'all recorded attempts; development/regression are seen, correlated and not independent probability trials',
            'attempts':len(output),'finished':len(finished),'complete_tasks':sum(x['continuous_task_completed'] for x in finished),
            'actual_safety_violation_runs':sum(bool(x['actual_safety_violations']) for x in finished),
            'independent_holdout_pairs':len(holdouts)//2,'independent_physical_clusters':0 if not holdouts else None,
            'paired_effect':'NOT_EVALUATED' if not holdouts else 'REQUIRES_CLUSTER_ANALYSIS',
            'population_success_probability':'NOT_ESTIMATED','p_value':'NOT_EVALUATED','bootstrap':'NOT_EVALUATED_NO_ADMITTED_HOLDOUTS',
            'runs':output}
    save(ROOT/'statistics.json',result);return result

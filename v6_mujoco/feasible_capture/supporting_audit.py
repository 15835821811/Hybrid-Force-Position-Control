"""Read-only historical analysis; no plant trajectories or action feedback."""
import gzip
import time
import numpy as np
from scipy.spatial.transform import Rotation
from v6_mujoco.adaptive_capture.validate import packet_from_dict
from v6_mujoco.adaptive_capture.momentum_regressor import tensor
from .common import ROOT,OLD,PROJECT_ROOT,read,save,sha,charge
from .benchmark import mission
from .estimator_adapter import TimedEstimator

def sensor_audit():
    path=OLD/'runs/S01_noise_delay';z=np.load(path/'trace.npz');cfg=mission('noisy');filter=TimedEstimator(cfg);rows=[]
    with gzip.open(path/'packets.jsonl.gz','rt',encoding='utf-8') as f:
        import json
        for line in f:
            row=json.loads(line);packet=packet_from_dict(row['packet']);e=filter.update(packet);i=round(packet.time/.002)
            if not e.valid:continue
            truth_twist=z['target_geometric_twist_world'][i]
            err=np.r_[e.p-z['target_position_world_m'][i],Rotation.from_matrix(e.R@z['target_rotation_world'][i].T).as_rotvec(),e.v-truth_twist[:3],e.w-truth_twist[3:]]
            rows.append({'time':packet.time,'error':err,'sigma':np.sqrt(np.diag(e.covariance)),
                         'velocity_margin_linear_m_s':3*np.sqrt(max(np.linalg.eigvalsh(e.covariance[6:9,6:9]))),
                         'velocity_margin_angular_rad_s':3*np.sqrt(max(np.linalg.eigvalsh(e.covariance[9:,9:])))})
    t=np.array([x['time'] for x in rows]);E=np.array([x['error'] for x in rows]);S=np.array([x['sigma'] for x in rows]);mask=(t>=1)&(t<=7)
    oldmask=(z['time_s']>=1)&(z['time_s']<=7);oldE=z['estimate_error'][oldmask]
    report={'historical_packets_sha':sha(path/'packets.jsonl.gz'),'historical_trace_sha':sha(path/'trace.npz'),
        'implementation_sha':{p:sha(PROJECT_ROOT/p) for p in ['v6_mujoco/feasible_capture/estimator_adapter.py','v6_mujoco/feasible_capture/supporting_audit.py','configs/n209_paper_system.yaml']},
        'scope':'same historical packets, changed estimator only; not a new trajectory or closed-loop success',
        'window_s':[1,7],'B0_rms_components':np.sqrt(np.mean(oldE**2,axis=0)),
        'new_rms_components':np.sqrt(np.mean(E[mask]**2,axis=0)),
        'new_empirical_marginal_3sigma_coverage':np.mean(abs(E[mask])<=3*S[mask],axis=0),
        'innovation_samples':len(filter.log),'control_samples':len(rows),'NIS_mean':np.mean([x['NIS'] for x in filter.log]),
        'new_linear_velocity_3sigma_median_m_s':np.median([x['velocity_margin_linear_m_s'] for x in rows]),
        'new_angular_velocity_3sigma_median_rad_s':np.median([x['velocity_margin_angular_rad_s'] for x in rows]),
        'diagnosis':'N208 .03/.2 acceleration density and current contact-mode replay propagate measurement changes into direct target/reference velocities; simultaneous hard conflict appears at7.54s. This is a mechanism hypothesis plus same-packet estimator evidence, not an isolated closed-loop causal effect.',
        'timing_support':'Fixed delay causal stamp updates; within-batch sorting; older duplicate/out-of-order rejection;0.25s historical process modes',
        'coverage_scope':'marginal empirical samples are temporally correlated; no calibrated joint probability',
        'guard_reference_separated':True}
    save(ROOT/'sensor_reference_audit.json',report);save(ROOT/'diagnostics/S01_noise_delay/new_estimator_redecision.json',rows)
    return report

def identification_audit():
    source=read(OLD/'identifiability_report.json');paired=read(OLD/'comparison.json')['paired_comparisons'];out={}
    for name in ['H1_prior','H1_identified','H3_prior','H3_identified']:
        p=OLD/'runs'/name;cfg=read(p/'config.json');truth=cfg['truth_evaluation_only'];blocks=np.load(p/'regression_blocks.npz')
        scales=np.array(cfg['prior']['parameter_scales']);raw=np.vstack(blocks['DY']);white=np.vstack([np.linalg.solve(L,A) for A,L in zip(blocks['DY'],blocks['cholesky'])])
        singular_raw=np.linalg.svd(raw*scales,compute_uv=False);singular_white=np.linalg.svd(white*scales,compute_uv=False)
        history=[]
        for x in read(p/'posteriors.json'):
            pi=np.array(x['pi']);m=pi[0];com=pi[1:4]/m;Ic=tensor(pi)-m*((com@com)*np.eye(3)-np.outer(com,com));I=np.array(truth['inertia'])
            history.append({'time':x['time_s'],'mass_error_pct':100*abs(m-truth['mass'])/truth['mass'],
                            'com_error_mm':1000*np.linalg.norm(com-truth['com']),'inertia_error_pct':100*np.linalg.norm(Ic-I)/np.linalg.norm(I),
                            'rank':x['rank'],'singular_values':x['singular_values']})
        first_parameter_time=None
        gov=read(p/'governor.json')
        if gov:
            pi0=np.array(gov[0]['model_pi'])
            first_parameter_time=next((x['time'] for x in gov if max(abs(np.array(x['model_pi'])-pi0))>1e-12),None)
        out[name]={'historical_report':source['runs'][name], 'unregularized_scaled_raw_singular_values':singular_raw,
                   'unregularized_scaled_whitened_singular_values':singular_white,'first_parameter_change_in_prediction_s':first_parameter_time,
                   'history_path':'diagnostics/'+name+'/identification_history.json','control_benefit':'CONTROL_BENEFIT_NOT_DEMONSTRATED',
                   'pair':paired[name.split('_')[0]],'blocks_sha':sha(p/'regression_blocks.npz')}
        save(ROOT/'diagnostics'/name/'identification_history.json',history)
    result={'runs':out,'new_robot_attempts':0,'prior_variants_kg':[15,20,32],'full_parameter_convergence_claim':False,
            'scope':'re-analysis of archived N208 data; no online retuning, no active excitation, no posterior backfill; raw/white SVD excludes regularization',
            'rank10_vs_com5mm':'H1 rank10 does not meet COM accuracy target; threshold unchanged'}
    save(ROOT/'identifiability_supporting_audit.json',result);return result

def run():
    cpu=time.process_time();wall=time.perf_counter();s=sensor_audit();identification_audit()
    charge('supporting_readonly_audits',time.process_time()-cpu,time.perf_counter()-wall,'sensor_reference and inertial support')
    print('sensor old/new RMS',s['B0_rms_components'],s['new_rms_components'])

if __name__=='__main__':run()

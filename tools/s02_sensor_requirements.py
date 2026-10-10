"""Offline sensing requirements, never a new estimator candidate or robot run."""
import time
import numpy as np
from v6_mujoco.system_capture.estimation.common import OUT,PROJECT,OLD,read,save
from v6_mujoco.system_capture.estimation.runner import verify_frozen
from v6_mujoco.system_capture.estimation.filter import white_jerk
from v6_mujoco.system_capture.estimation.offline import public_offset
from v6_mujoco.system_capture.estimation.budget import limits

def transition(h):return np.array([[1.,h,.5*h*h],[0.,1.,h],[0.,0.,1.]])

def posterior(period,noise,density):
    F=transition(period);Q=white_jerk(period,density);P=np.eye(3)
    for _ in range(3000):
        P=F@P@F.T+Q;K=P[:,0]/(P[0,0]+noise**2)
        A=np.eye(3);A[:,0]-=K
        P=A@P@A.T+noise**2*np.outer(K,K)
    return (P+P.T)/2

def main():
    verify_frozen();cpu,wall=time.process_time(),time.perf_counter()
    model=read(PROJECT/'configs/system_capture/algorithms/s02_acceleration.json')
    cfg=read(OLD/'runs/R03_V2_S01/config.json');sensor=cfg['sensors'];mission=cfg['mission']
    length=np.linalg.norm(public_offset());gates=limits(mission);factor=model['contact_multiplier']
    # Small declared design table, not a search over estimator parameters.
    scenarios=[(.006,.012,1.),(.006,0.,1.),(.006,.012,.5),(.006,0.,.5),
        (.002,.002,1.),(.002,0.,1.),(.002,.002,.5),(.002,0.,.5)]
    rows=[]
    for period,delay,noise_scale in scenarios:
        ps=posterior(period,sensor['position_sigma_m']*noise_scale,model['linear_jerk_density']*factor)
        rs=posterior(period,sensor['rotation_sigma_rad']*noise_scale,model['angular_jerk_density']*factor)
        ages=delay+np.arange(0,period-1e-10,cfg['dt'])
        margins=[]
        for age in ages:
            F=transition(age)
            P=F@ps@F.T+white_jerk(age,model['linear_jerk_density']*factor)
            R=F@rs@F.T+white_jerk(age,model['angular_jerk_density']*factor)
            sig_p,sig_v=np.sqrt(P[0,0]),np.sqrt(P[1,1]);sig_R,sig_w=np.sqrt(R[0,0]),np.sqrt(R[1,1])
            m=mission['margin_sigma']*np.array([sig_p+length*sig_R,sig_R,sig_v+length*sig_w,sig_w])
            m[:2]+=[mission['position_bias_bound_m']+length*mission['rotation_bias_bound_rad'],mission['rotation_bias_bound_rad']]
            margins.append(m)
        maximum=np.max(margins,axis=0);remaining=gates-maximum
        rows.append(dict(pose_period_s=period,delay_s=delay,noise_scale=noise_scale,
            position_sigma_m=sensor['position_sigma_m']*noise_scale,
            rotation_sigma_rad=sensor['rotation_sigma_rad']*noise_scale,ages_s=ages,
            maximum_legacy_margins=maximum,minimum_remaining_budget=remaining,
            necessary_only_all_four_positive=bool(np.all(remaining>0))))
    baseline=np.array(read(OUT/'floor_before_after.json')['new_CA']['numerical_periodic_diagnostic']['contact']['margins'])
    error=float(np.max(abs(np.max(baseline,axis=0)-rows[0]['maximum_legacy_margins'])))
    assert error<1e-10,error
    data=dict(scope='zero-angular-rate isotropic steady local CA contact model; numerical sensing-design necessary conditions only, no robot rollout or new estimator tuning',
        preserved_estimator_parameters=model,original_bias_bounds_unchanged=True,
        additional_hardware_uncertainty='not budgeted: clock, calibration, encoder/navigation uncertainty and correlations must consume remaining reserve',
        actual_error_validation='NOT_PERFORMED_FOR_HYPOTHETICAL_SENSORS',
        deployed=False,robot_attempts=0,baseline_floor_agreement_max_abs=error,rows=rows,
        cpu_s=time.process_time()-cpu,wall_s=time.perf_counter()-wall)
    save(OUT/'sensor_requirement_diagnostic.json',data)
    lines=['','## Offline sensing-design diagnostic','',
        'This eight-row design table holds the selected CA densities and contact multiplier fixed. It is not another estimator candidate or an E3 robot result. Original bias bounds remain unchanged. Robot/clock/calibration errors remain zero only under the stated simulation assumptions.','',
        '|Pose period (ms)|Delay (ms)|Noise scale|Max linear margin (mm/s)|Max angular margin (deg/s)|All four necessary budgets positive|',
        '|---:|---:|---:|---:|---:|---|']
    for r in rows:
        m=r['maximum_legacy_margins']
        lines.append(f"|{r['pose_period_s']*1000:g}|{r['delay_s']*1000:g}|{r['noise_scale']:g}|{m[2]*1000:.4f}|{np.rad2deg(m[3]):.4f}|{r['necessary_only_all_four_positive']}|")
    lines+=['','Noise scale 1 means 10 micrometres / 35 microradians per component; scale 0.5 means 5 micrometres / 17.5 microradians. Ages cover all phases of the unchanged 2 ms controller clock.','',
        'These are stationary zero-rate covariance recursions, not actual-error guarantees, contact-transition validation, hardware specifications or authorization to replace the sensor. A positive remaining budget must still cover tracking error and all extra hardware errors. The observed startup/latch innovation spikes are not solved by this table.','']
    path=OUT/'sensor_requirement_budget.md';base=path.read_text(encoding='utf-8').split('\n## Offline sensing-design diagnostic')[0]
    path.write_text(base+'\n'.join(lines),encoding='utf-8')
    print({'baseline_agreement':error,'rows':[(r['pose_period_s'],r['delay_s'],r['noise_scale'],r['necessary_only_all_four_positive']) for r in rows]})

if __name__=='__main__':main()

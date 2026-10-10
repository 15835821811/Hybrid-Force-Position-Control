"""Explicit S02 orchestration and admission; no cross-stage execution."""
import contextlib
import io
import os
import time
import unittest
from .common import OUT,PROJECT,PROTOCOL,BASE,BRANCH,read,save,sha,digest,environment,identity,prepare,charge

def tests():
    from .tests import suite,PersistenceProperties
    cpu,wall=time.process_time(),time.perf_counter();stream=io.StringIO();PersistenceProperties.results=[]
    result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite())
    (OUT/'tests.log').write_text(stream.getvalue(),encoding='utf-8')
    doc=dict(passed=result.wasSuccessful(),tests_run=result.testsRun,failures=len(result.failures),errors=len(result.errors),
        physics_steps=0,scope='bounded synthetic, model initialization and isolated persistence tests; no complete robot attempts')
    save(OUT/'prediction_interface_tests.json',doc)
    save(OUT/'persistence_fault_tests.json',dict(passed=result.wasSuccessful(),cases=PersistenceProperties.results,
        commit_period='200 records, normally 0.1 s for four events per 2 ms cycle; final and initial blocks explicitly flushed',
        forced_termination_scope='only indexed committed blocks guaranteed recoverable; in-memory tail deliberately ignored; no power-loss guarantee'))
    charge('properties_and_fault_injection',cpu,wall,**doc)
    print(stream.getvalue(),flush=True)
    return result.wasSuccessful()

def offline_and_freeze():
    from .offline import preregister,calibrate,synthetic_validation,historical_validation,floor
    from .runner import resource_used
    preregister()
    selected=read(OUT/'calibration_selection.json') if (OUT/'calibration_selection.json').exists() else calibrate()
    m=selected['model']
    syn=read(OUT/'synthetic_validation.json') if (OUT/'synthetic_validation.json').exists() else synthetic_validation(m)
    hist=read(OUT/'historical_estimator_comparison.json') if (OUT/'historical_estimator_comparison.json').exists() else historical_validation(m)
    f=read(OUT/'floor_before_after.json') if (OUT/'floor_before_after.json').exists() else floor(m)
    noisy=hist['R03_V2_S01'];improved=noisy['CA']['norm_rms'][2]<noisy['CV']['norm_rms'][2]
    info=noisy['CA']['information']['necessary_condition_fraction']>0
    admission=dict(status='LIMITED_DIAGNOSTIC_ADMISSION' if improved and info else 'ESTIMATOR_GUARD_MISMATCH_IN_TESTED_SCOPE',
        accuracy_improved_in_seen_noisy_prefix=improved,necessary_information_compatible_in_seen_prefix=info,
        actual_guard_called_in_offline_comparison=False,
        contact_covariance_limit='fixed contact periodic margins exceed original velocity/angular gates; not qualified for general contact',
        ideal_limit='ideal replay has larger peak error and NIS around contact; E0 must preserve full task before E1',
        sequential_gates=read(PROTOCOL)['E2_E3_admission'])
    save(OUT/'offline_admission.json',admission)
    save(OUT/'causal_estimator_comparison.json',dict(synthetic=syn,archived=hist,
        split_sha256=sha(OUT/'calibration_split.json'),definition='component and 3D norms; measurement/current/blind forecasts, NIS6, NEES12 and empirical marginal coverage',
        no_population_inference=True))
    save(OUT/'relative_guard_budget.json',dict(
        per_time_arrays='offline/*/comparison.npz: legacy_remaining_budget and full_remaining_budget',
        full_geometry='Jp_theta=-skew(Rr), Jv_theta=-skew(omega)skew(Rr), Jv_omega=-skew(Rr)',
        cross_terms='all target state cross terms preserved; optional robot/target cross covariance and calibration/time terms implemented',
        robot_and_clock='SIMULATION_ASSUMPTION: exact; no hardware conclusion',
        guard='unchanged legacy marginal norm formula; full propagation diagnostic only',
        bias='norm bounded pose bias; missing orientation-to-velocity bias term disclosed and included in full diagnostic',
        condition='uncertainty plus bias < physical gate is necessary only; actual relative error, geometry, loads, 40 ms and latch checks remain required'))
    (OUT/'state_model_decision.md').write_text(
        '# S02 state model decision\n\nSelected '+selected['selected']+' from two frozen density multipliers of the same 18-dimensional model. '+
        'Densities come from independent development acceleration increments, not old CV units or guard-only tuning. '+
        'Selection used state error, blind prediction error, NIS and coverage. The validation seeds and [2,6] s suffix were frozen first.\n\n'+
        'State order: dp_W,dtheta_W,dv_W,domega_W,da_W,dalpha_W. Means p,R,v,omega,a,alpha. '+
        'Velocity and acceleration start at zero with declared public variances. No true twist, COM or inertia is read. '+
        'White jerk spectral intensities are the squares of the saved densities (m/s^(5/2), rad/s^(5/2)). '+
        'Translation Q is the integrated white-jerk matrix. Rotation and angular variational covariance use RK4 substeps <=2 ms. '+
        'Left-error reset uses the left Jacobian, checked independently by group perturbation; this new formulation does not rewrite archived CV.\n\n'+
        'Past propagation uses causal mode history. Forecast holds the latest observed mode with no future measurements/contact truth. '+
        'The governor combines this covariance with an independent prior-plant mean; this is an approximation, not a joint consistent probability model. '+
        'Guard thresholds, multiplier, shaping, candidates and horizon remain identical.\n\n'+
        'Known limits: ideal contact spikes have increased NIS; free noisy information improves but prolonged contact margins remain incompatible. '+
        'Admission is to at most four frozen diagnostic trials, conditional on E0 and subsequent safety checks.\n',encoding='utf-8')
    (OUT/'sensor_requirement_budget.md').write_text(
        '# Capture information budget\n\nThe original CV conditional linear 3sigma floor is '+str(f['old_CV']['recomputed_linear_3sigma_m_s']*1000)+
        ' mm/s under 6 ms pose sampling, age >=12 ms, 10 micrometre noise and 0.003 m/s^(3/2) acceleration density. '+
        'This is a computed covariance allowance bound, not actual-error or all-estimator impossibility.\n\n'+
        'The new CA numerical periodic free-mode diagnostic at ages 12/14/16 ms gives about 0.362/0.369/0.376 mm/s linear guard margin, '+
        'leaving about 0.638/0.631/0.624 mm/s for relative tracking. The contact-mode diagnostic is about 1.266–1.367 mm/s and leaves no linear budget. '+
        'These are fixed-protocol, zero-angular-rate diagnostics; the time-series files retain the actual nonlinear ranges.\n\n'+
        'All target position/orientation and velocity/angular-velocity cross terms and lever-arm orientation derivatives are propagated offline. '+
        'Tool velocity includes omega cross fixed flange-to-interface offset. Robot navigation/encoders, timestamps and fixed calibration are exact SIMULATION_ASSUMPTION channels, not demonstrated hardware capabilities. '+
        'With hardware their covariance, correlations, time bias and calibration must consume additional budget. A 3sigma direction approximation is not an overall or all-time 99.7% guarantee.\n\n'+
        'E3 removes only target pose delay as an authorized diagnostic. No hardware protocol replacement is deployed. '+
        'Further sensing requirements must address contact-transition bandwidth and both angular/linear uncertainty, with positive tracking reserve, not only free-motion RMS.\n',encoding='utf-8')
    if not read(OUT/'prediction_interface_tests.json')['passed']:raise RuntimeError('tests failed')
    if (OUT/'estimation_protocol.json').exists():
        from .runner import verify_frozen
        return verify_frozen()
    frozen=dict(phase='S02',baseline=BASE,branch=BRANCH,frozen_epoch=time.time(),
        model=m,run_plan=read(PROTOCOL),admission=admission,source_identity=identity(),environment=environment(),
        evidence_identity={p.name:sha(p) for p in [OUT/'calibration_split.json',OUT/'calibration_selection.json',OUT/'prediction_interface_tests.json',OUT/'persistence_fault_tests.json',OUT/'synthetic_validation.json',OUT/'historical_estimator_comparison.json']},
        remaining_cpu_budget_s=21600-resource_used(),scope='S02 only; no automatic S03; full physical attempts include implementation failures')
    save(OUT/'estimation_protocol.json',frozen)
    return frozen

@contextlib.contextmanager
def operation_lock():
    path=OUT/'operation.lock';fd=os.open(path,os.O_CREAT|os.O_EXCL|os.O_WRONLY)
    os.write(fd,str(os.getpid()).encode());os.close(fd)
    try:yield
    finally:path.unlink()

def run(resume=False):
    from .runner import run_one
    p=read(OUT/'estimation_protocol.json')
    with operation_lock():
        for name in read(PROTOCOL)['run_order']:
            book=read(OUT/'run_ledger.json');entries={x['name']:x for x in book['attempts']}
            if name in entries:
                if not resume:raise RuntimeError('existing attempt; use --resume for next registered condition only')
                if entries[name]['status'] in ('REGISTERED_BEFORE_DYNAMICS','RUNNING','IMPLEMENTATION_ERROR','PERSISTENCE_ERROR'):
                    raise RuntimeError('incomplete/interface failure retained; dependent runs stopped')
                continue
            if name!='E0':
                e0=read(OUT/'runs/E0/metrics.json')
                if not (e0['continuous_task_completed'] and e0['postgrasp_detumbling']):
                    save(OUT/'run_admission_stop.json',dict(reason='E0_TASK_NOT_PRESERVED',not_run=[x for x in read(PROTOCOL)['run_order'] if x not in entries]));break
                if p['admission']['status']!='LIMITED_DIAGNOSTIC_ADMISSION':break
                if any(e.get('actual_safety_violations') or e['status'] in ('IMPLEMENTATION_ERROR','PERSISTENCE_ERROR','FALSE_CAPTURE') for e in entries.values()):break
            run_one(name)
    return True

def replay(resume=False):
    from .replay import replay_one
    with operation_lock():
        for e in read(OUT/'run_ledger.json')['attempts']:
            for kind in ('actuator','decision'):
                path=OUT/'runs'/e['name']/(kind+'_replay.json')
                if path.exists():
                    if not resume:raise FileExistsError('replay already exists; use --resume')
                    if not read(path)['passed']:return False
                    continue
                if not replay_one(e['name'],kind)['passed']:return False
    return True

def main(mode,resume=False):
    if mode=='prepare':print(prepare());return 0
    if mode=='test':
        if not tests():return 1
        offline_and_freeze();return 0
    if mode=='run':return 0 if run(resume) else 1
    if mode=='replay':return 0 if replay(resume) else 1
    if mode=='report':
        from .report import report
        return 0 if report() else 1
    raise ValueError(mode)

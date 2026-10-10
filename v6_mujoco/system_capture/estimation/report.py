"""S02 evidence synthesis. Reports failures and unexecuted conditions explicitly."""
import json
import time
import numpy as np
from scipy.spatial.transform import Rotation
from .common import OUT,PROJECT,OLD,BASE,BRANCH,read,save,sha,digest,charge
from .runner import resource_used,verify_frozen
from .offline import public_offset
from .budget import diagnostic
from .filter import AccelerationSnapshot
from .persistence import records,validate

def actual_guard_analysis(name):
    path=OUT/'runs'/name/'guard_calls.json'
    calls=read(path) if path.exists() else []
    if not calls:return dict(calls=0,scope='no actual guard invocation; counterfactual budgets remain separate')
    values=np.array([r['values'] for r in calls]);margins=np.array([r['margins'] for r in calls])
    limits=np.array([r['limits'] for r in calls])
    names=('translation','rotation','linear_velocity','angular_velocity')
    result=dict(calls=len(calls),instantaneous_passes=sum(r['passed'] for r in calls),
        start_time_s=calls[0]['time'],end_time_s=calls[-1]['time'],
        four_gate_exceedance_counts=dict(zip(names,np.sum(values+margins>limits,axis=0).tolist())),
        information_alone_exceedance_counts=dict(zip(names,np.sum(margins>=limits,axis=0).tolist())),
        last_actual_call=calls[-1],
        scope='actual unchanged guard calls; a single passing call does not establish the 40 ms confirmation or latch/task success')
    save(path.parent/'guard_call_analysis.json',result)
    return result

def run_diagnostics(name):
    d=OUT/'runs'/name;cfg=read(d/'config.json');offset=public_offset();rows=[]
    for r in records(d/'raw'):
        if r['kind']!='proposal':continue
        e=r['snapshot']['latest_snapshot'];e=AccelerationSnapshot(**e)
        row=r['observation'];P=diagnostic(e,cfg['mission'],offset)
        actual_p=np.array(row['target_position_world_m']);actual_R=np.array(row['target_rotation_world']);tw=np.array(row['target_geometric_twist_world'])
        rg=e.R@offset;rt=actual_R@offset
        point_error=np.r_[e.p+rg-actual_p-rt,Rotation.from_matrix(e.R@actual_R.T).as_rotvec(),
            e.v+np.cross(e.w,rg)-tw[:3]-np.cross(tw[3:],rt),e.w-tw[3:]]
        true_rel=np.r_[row['interface_translation_error_m'],np.deg2rad(row['interface_rotation_error_deg']),
            np.linalg.norm(np.array(row['interface_relative_twist_world'])[:3]),np.linalg.norm(np.array(row['interface_relative_twist_world'])[3:])]
        rows.append(dict(time=r['time'],geometry_error=row['estimate_error'],grasp_error=point_error,
            actual_relative=true_rel,execution_error=[row['approach_position_error_m'],row['approach_rotation_error_rad']],
            remaining=P['legacy_remaining_budget'],complete_remaining=P['complete_geometry_remaining_budget'],valid=e.valid,
            real_guard_called=r['snapshot']['guard_call_count']>(rows[-1]['guard_count'] if rows else 0),guard_count=r['snapshot']['guard_call_count']))
    if not rows:return {}
    data={k:np.array([r[k] for r in rows]) for k in rows[0]};np.savez_compressed(d/'relative_budget.npz',**data)
    valid=data['valid'];mask=valid&(data['time']>=.2);safe=np.all(data['remaining']>0,axis=1)&valid
    # Longest consecutive compatible information interval, separate from actual
    # guard calls/physical alignment/contact/confirmation.
    longest=0;current=0
    for b in safe:current=current+1 if b else 0;longest=max(longest,current)
    return dict(actual_guard_calls=int(data['guard_count'][-1]),
        counterfactual_necessary_condition_fraction=float(np.mean(safe[mask])) if np.any(mask) else None,
        longest_information_only_interval_s=longest*cfg['dt'],
        minimum_remaining_budget=data['remaining'][mask].min(axis=0).tolist() if np.any(mask) else None,
        grasp_component_rms=np.sqrt(np.mean(data['grasp_error'][mask]**2,axis=0)).tolist() if np.any(mask) else None,
        grasp_component_peak=np.max(abs(data['grasp_error'][valid]),axis=0).tolist() if np.any(valid) else None,
        geometry_velocity_all_valid_rms_m_s=float(np.sqrt(np.mean(np.sum(data['geometry_error'][valid,6:9]**2,axis=1)))) if np.any(valid) else None,
        geometry_velocity_all_valid_peak_m_s=float(np.max(np.linalg.norm(data['geometry_error'][valid,6:9],axis=1))) if np.any(valid) else None,
        geometry_velocity_peak_time_s=float(data['time'][np.flatnonzero(valid)[np.argmax(np.linalg.norm(data['geometry_error'][valid,6:9],axis=1))]]) if np.any(valid) else None,
        true_relative_peak=np.max(data['actual_relative'],axis=0).tolist(),raw_sha256=sha(d/'relative_budget.npz'),
        interpretation='information-only intervals are not capture windows; all startup/transition peaks retained')

def account_unmeasured_preflight():
    path=OUT/'run_ledger.json';b=read(path)
    if any(x['kind']=='preflight_accounting_amendment' for x in b['operations']):return
    # The first archival offline invocation stopped at metric serialization;
    # no CPU timer was persisted. Retain that gap and conservatively charge
    # 300 CPU seconds, exceeding the entire observed invocation wall interval.
    b['operations'].append(dict(kind='preflight_accounting_amendment',cpu_s=None,budget_charge_s=300.,
        actual_cpu_s='NOT_RECORDED',basis='conservative 300 s charge for initial archival summary failure, early manual tests and postprocessing before ledger instrumentation',
        failure='undefined correlation for constant H2 innovation caused JSON NaN rejection; comparison.npz was already written',
        repair='correlation explicitly null; saved completed comparisons reused, remaining H2/noisy comparisons executed',
        full_robot_attempts=0,not_an_exact_cpu_measurement=True))
    save(path,b)

def report():
    cpu,wall=time.process_time(),time.perf_counter();verify_frozen();account_unmeasured_preflight()
    # Correct generated prose while preserving the byte-frozen experiment
    # orchestrator and original E0 source. No runtime/parameter revision.
    path=OUT/'state_model_decision.md'
    prose=path.read_text(encoding='utf-8').replace(
        'Rotation and angular variational covariance use RK4 substeps <=2 ms.',
        'The repaired C2 rotation mean and angular variational transition use RK4 substeps <=2 ms; process covariance is a positive-weight Gauss-Legendre integral of noise-factor outer products. The original E0 direct-Q RK4 defect and source are retained separately.')
    path.write_text(prose,encoding='utf-8')
    path=OUT/'sensor_requirement_budget.md'
    prose=path.read_text(encoding='utf-8').replace(
        'E3 removes only target pose delay as an authorized diagnostic.',
        'The authorized E3 design removes only target pose delay, but E3 was removed from the remaining run plan when the E0 implementation repair consumed an attempt. No zero-delay robot-task result is claimed.')
    prose=prose.replace('No zero-delay result is claimed.','No zero-delay robot-task result is claimed.')
    path.write_text(prose,encoding='utf-8')
    from .diagnostics import augment_offline,audit_completed_safety
    timing=OUT/'safety_audit_timing.json'
    if timing.exists():
        b=read(OUT/'run_ledger.json')
        if not any(x['kind']=='completed_prefix_safety_audit' for x in b['operations']):
            b['operations'].append(dict(kind='completed_prefix_safety_audit',**read(timing)))
            save(OUT/'run_ledger.json',b)
    sensing=OUT/'sensor_requirement_diagnostic.json'
    if sensing.exists():
        b=read(OUT/'run_ledger.json');r=read(sensing)
        if not any(x['kind']=='offline_sensing_requirement_diagnostic' for x in b['operations']):
            b['operations'].append(dict(kind='offline_sensing_requirement_diagnostic',
                cpu_s=r['cpu_s'],wall_s=r['wall_s'],new_robot_steps=0,estimator_parameter_change=False,
                scope=r['scope']))
            save(OUT/'run_ledger.json',b)
    augment_offline()
    safety_audit=audit_completed_safety()
    book=read(OUT/'run_ledger.json');runs={};diagnostics={};replays={};guards={}
    for attempt in book['attempts']:
        name=attempt['name'];d=OUT/'runs'/name;validate(d/'raw')
        runs[name]=read(d/'metrics.json');diagnostics[name]=run_diagnostics(name)
        guards[name]=actual_guard_analysis(name)
        replays[name]={k:read(d/(k+'_replay.json')) if (d/(k+'_replay.json')).exists() else {'passed':False,'status':'NOT_EVALUATED'} for k in ('actuator','decision')}
    save(OUT/'online_relative_guard_budget.json',diagnostics)
    persistence={n:dict(blocks=r['raw_validation']['block_count'],records=r['raw_validation']['records'],
        io_wall_s=r['journal_io_wall_s'],serialization_wall_s=r['journal_serialization_wall_s'],
        io_fraction_of_run_wall=r['journal_io_wall_s']/r['wall_s'],
        serialization_fraction_of_run_wall=r['journal_serialization_wall_s']/r['wall_s'],
        nominal_commit_period_s=.1,max_verifiable_time_s=r['max_verifiable_time_s'],
        scope='measured within run; I/O and serialization separate; no isolated no-journal speedup claim')
        for n,r in runs.items()}
    save(OUT/'persistence_performance.json',persistence)
    syn=read(OUT/'synthetic_validation.json');hist=read(OUT/'historical_estimator_comparison.json');floor=read(OUT/'floor_before_after.json')
    all_replays=all(v['passed'] for pair in replays.values() for v in pair.values()) and bool(runs)
    violations=[dict(run=n,**v) for n,r in runs.items() for v in r['actual_safety_violations']]
    for n,r in safety_audit.items():
        for v in r['violations']:
            item=dict(run=n,**v)
            if item not in violations:violations.append(item)
    noisy_pass=any(n in ('E1','E2') and r['continuous_task_completed'] and r['postgrasp_detumbling'] and not r['actual_safety_violations'] for n,r in runs.items())
    ideal_name='E0_C2' if 'E0_C2' in runs else 'E0'
    ideal_pass=ideal_name in runs and runs[ideal_name]['continuous_task_completed'] and runs[ideal_name]['postgrasp_detumbling']
    if noisy_pass:overall='INFORMATION_AND_TASK_PASSED_IN_TESTED_CASES'
    elif ideal_pass:overall='ACCURACY_IMPROVED_GUARD_NOT_COMPATIBLE'
    else:overall='CURRENT_ESTIMATOR_PROTOCOL_NOT_QUALIFIED'
    full={n:dict(status=runs[n]['status'],end_time_s=runs[n]['end_time_s'],task_completed=runs[n]['continuous_task_completed'],
        postgrasp_detumbling=runs[n]['postgrasp_detumbling'],final_window=runs[n]['final_window_status']) if n in runs else dict(status='NOT_RUN',reason='sequential admission or four-attempt repair budget; no empty result directory') for n in ('E0','E0_C2','E1','E2','E3')}
    q=dict(overall=overall,implementation=dict(status='C2_PSD_REPAIR_IMPLEMENTED_ORIGINAL_E0_FAILURE_RETAINED',tests=read(OUT/'prediction_interface_tests.json'),repair=read(OUT/'covariance_repair_tests.json')),
        legacy_C1_record_loss_retained='MISSING_ORIGINAL_TRACE; S01 PARTIAL unchanged; limited user-authorized exception only',
        source_compatibility_scope='S01 SOURCE_COMPATIBLE_UNDER_ASSUMPTIONS retained; CA18 is a new local motion model, not the source paper algorithm',
        timestamp_causality='TESTED: measurement-time assimilation, arrived-only observations, historical modes, stale/duplicate/out-of-order rejection',
        truth_isolation='TESTED_PUBLIC_PACKET_BOUNDARY: no target qvel/qacc/mass/COM/inertia in estimator; truth evaluation separate',
        covariance_prediction_consistency='TESTED_FULL_P18: acceleration cross terms retained, correct P12 marginal, no online mutation; predictor prior mean/covariance combination remains approximate',
        state_error_validation=dict(scope='six held-out signal suffixes plus four already-seen archive regressions; startup and contact peaks retained',
            seen_noisy_velocity_rms_mm_s={k:hist['R03_V2_S01'][k]['norm_rms'][2]*1000 for k in ('CV','CA')},
            seen_noisy_scoring_interval_s=hist['R03_V2_S01']['CA']['interval_s'],
            seen_noisy_all_valid_velocity_rms_mm_s={k:hist['R03_V2_S01'][k]['all_prefix']['norm_rms'][2]*1000 for k in ('CV','CA')},
            seen_noisy_all_valid_velocity_peak_mm_s={k:hist['R03_V2_S01'][k]['all_prefix']['norm_peak'][2]*1000 for k in ('CV','CA')},
            startup_caveat='CA valid-startup peak is larger than CV; initial covariance consumes the budget, so low settled RMS is not a peak guarantee',
            ideal_contact_caveat='CA has larger ideal contact peak/NIS than CV; no uniform improvement claim'),
        uncertainty_calibration_scope='development-only density calibration; empirical marginal coverage and NIS/NEES on correlated signals, no joint/all-time confidence certificate',
        guard_information_compatibility=dict(free_noisy_prefix='NECESSARY_CONDITION_SUPPORTED_AFTER_INITIALIZATION_IN_TESTED_SIGNALS',
            sustained_contact='NOT_QUALIFIED: numerical contact-mode periodic linear/angular margins exceed unchanged gates',
            actual_calls={n:r['actual_guard_calls'] for n,r in runs.items()},
            actual_call_analysis=guards,
            overall_scope='no capture information claim for approach prefixes where guard was never called'),
        sensor_profile='E1/E2 original 6 ms,12 ms,10 um,35 urad and biases; E0/E0_C2 ideal; E3 NOT_RUN (its plan changed only target pose delay)',
        full_task_status_by_run=full,
        planning_or_governor_failure_remaining={n:r['reason'] for n,r in runs.items() if r['status']=='NO_VERIFIED_CONTROL'},
        real_safety_violations=violations,dual_replay=dict(passed=all_replays,by_run={n:{k:v['passed'] for k,v in pair.items()} for n,pair in replays.items()}),
        next_stage_admission=dict(S03_automatically_started=False,ideal_measurement='FROZEN_CA_INTERFACE_SUPPORTED' if ideal_pass else 'USE_PREVIOUSLY_VERIFIED_IDEAL_BASELINE_ONLY',
            nonideal_measurement='SUPPORTED_ONLY_IN_ACTUALLY_PASSED_CASES' if noisy_pass else 'NOT_ADMITTED',
            outstanding=['contact uncertainty/innovation validation','governor/planning failures retained','hardware sensing/calibration not validated']),
        hardware='NOT_VALIDATED',full_inertia_identification='NOT_CLAIMED_BY_S02',realtime='NOT_CERTIFIED')
    save(OUT/'qualification.json',q)
    from .fig1_errors import main as f1
    from .fig2_budget import main as f2
    from .fig3_contact import main as f3
    from .fig4_outcomes import main as f4
    for fun in (f1,f2,f3,f4):fun()
    captions={
      'f1_causal_errors':'CV denotes the original constant-velocity estimator; CA is the new 18-state local-acceleration estimator. (a,b) World-frame geometric-origin linear and angular velocity error norms on archived R03_V2_S01, all times retained; gray marks invalid initialization before 24 ms. Symmetric-log axes use linear thresholds 0.05 mm/s and 0.05 deg/s. (c,d) Euclidean norm-RMS of blind errors on the synthetic bias_noise held-out interval [2,6] s, after causal [0,2) s warmup. Forecasts assimilate no future observations. Endpoint truncation leaves 667/666/665/664 origins for horizons 0/6/12/16 ms, matched between methods. These are correlated samples, not independent trials.',
      'f2_guard_budget':'(a,b) Archived R03_V2_S01; (c,d) synthetic bias_noise over [0,6] s, with [0,2) causal warmup before the dash-dot line and [2,6] held-out scoring. Gray at the origin marks invalid initialization before 24 ms; the wider light band marks prescribed synthetic contact process mode from 1.2 to 4.2 s, not measured robot contact. Remaining budget uses unchanged legacy 3sigma formulas; dotted CA full geometry is a diagnostic and does not replace the guard. Symmetric-log linear thresholds are 0.2 mm/s and 0.2 deg/s. Negative budget leaves no tracking reserve. Positive linear budget alone does not imply feasibility: angular budget can remain negative. R03 actual guard calls = 0; all these checks are counterfactual necessary conditions.',
      'f3_contact_motion':'Corrected E0_C2 first intended geometric contact (intentional contact count becomes positive, not a nonzero-force criterion) plus/minus 100 ms, fixed before looking at outcomes. All three curves describe world-frame target geometric-origin speed norms. This unchanged baseline has reference shaping disabled: the target reference copies the causal estimate, so those curves coincide; it is not the flange path. Panel (b) retains the contact estimation spike. Panel (c), added during result review to expose the larger latch-transition spike just outside the fixed contact window, shows every valid sample over the full trajectory and labels the maximum. This is a diagnostic presentation amendment, not a changed task-evaluation window. If the recorded prefix ends within this interval, the window is truncated there. Missing current estimates or references at an exception endpoint are left absent, never extrapolated.',
      'f4_task_outcomes':'Planned conditions and repair attempts: E0 and E0_C2 are two attempts at the same ideal condition. Bars show actual duration, markers peak actual load fraction with the unchanged limit at one. Blue circles require full duration, the final detumbling gate and no recorded safety violation; orange crosses indicate failure. Row text distinguishes implementation failure and control rejection. NOT RUN is reserved for absent ledger attempts, with E3 omitted for the repair budget; a missing summary is labelled separately. Early-stop final two-second windows remain NOT_EVALUATED.'}
    (OUT/'figures/README.md').write_text('# S02 scientific figures\n\nRun generators from the repository root as python -m v6_mujoco.system_capture.estimation.fig1_errors (and fig2_budget, fig3_contact, fig4_outcomes). Copies here record the exact figure source; use the package entry points and plot_style.py for reproduction.\n\n'+'\n\n'.join(f'[{name}.pdf]({name}.pdf): {caption}' for name,caption in captions.items() if (OUT/'figures'/(name+'.pdf')).exists())+'\n',encoding='utf-8')
    (OUT/'figures/latex_includes.tex').write_text('\n\n'.join('\\begin{figure*}[t]\n\\centering\n\\includegraphics[width=\\textwidth]{'+name+'.pdf}\n\\caption{'+caption.replace('_','\\_')+'}\n\\end{figure*}' for name,caption in captions.items() if (OUT/'figures'/(name+'.pdf')).exists()),encoding='utf-8')
    import shutil
    for path in (PROJECT/'v6_mujoco/system_capture/estimation').glob('fig*.py'):
        shutil.copyfile(path,OUT/'figures'/path.name)
    shutil.copyfile(PROJECT/'v6_mujoco/system_capture/estimation/plot_style.py',OUT/'figures/plot_style.py')
    lines=['# S02 估计器—传感器—捕获门报告','',f'**{overall}**。工程实现与资格分开；以下均限于已执行条件。','',
        '## Material Passport','',f'模式：代码实验与验证。基线 `{BASE}`；分支 `{BRANCH}`。数据和输入 SHA 见 entry_acceptance.json；冻结接口见 estimation_protocol.json。','',
        'S01 的 PARTIAL、原 C1 的 MISSING_ORIGINAL_TRACE 原样保留。没有回填历史原始记录，也没有重跑 S01。S02 的18维局部加速度模型是新工程方案，不是原论文算法或精确作者数值复现。','',
        '## 方法与冻结边界','',
        '状态 p,R,v,omega,a,alpha 使用世界系左误差；平移白 jerk 精确离散，姿态与变分矩阵按≤2 ms RK4传播，角过程协方差用正权Gauss-Legendre噪声因子积分。开发集只用于两个谱密度候选的选择；选择发生在原E0实现版本，开发数组和当时的测试保留在 pre_repair/，PSD修复后未重选参数。测试种子及 [2,6] s 校验后段事先冻结。修复后复查同一校验集属于实现回归，不新增独立样本。速度/加速度均以零均值和公开方差初始化。真实质量、COM、惯量、初始目标速度与未来观测均未进入估计器。','',
        '传感器、N206工具、N209 V2路径/HQP/参考整形、候选集合、预测时域、捕获门与抓后阻尼不变。完整 P18 及交叉项进入只读快照预测；12维接口只是正确边际。独立先验植物均值与估计协方差的组合仍是近似。历史CV源码字节保持，版本化预测器差分由测试约束。','',
        '## 离线证据','',
        '|校验序列 [2,6] s|CV速度RMS (mm/s)|CA速度RMS (mm/s)|CV峰值 (mm/s)|CA峰值 (mm/s)|CA最小分量3σ覆盖|',
        '|---|---:|---:|---:|---:|---:|']
    for name,pair in syn.items():
        a,b=pair['CV'],pair['CA'];lines.append(f"|{name}|{a['norm_rms'][2]*1000:.4f}|{b['norm_rms'][2]*1000:.4f}|{a['norm_peak'][2]*1000:.4f}|{b['norm_peak'][2]*1000:.4f}|{min(b['marginal_3sigma_coverage']):.4f}|")
    a,b=hist['R03_V2_S01']['CV'],hist['R03_V2_S01']['CA']
    lines+=['',f"已有噪声R03回归的预定义 [0.2,7.84] s 区间内，速度三维RMS由 {a['norm_rms'][2]*1000:.6f} 降到 {b['norm_rms'][2]*1000:.6f} mm/s。若包含全部有效启动样本，RMS分别为 {a['all_prefix']['norm_rms'][2]*1000:.6f} 和 {b['all_prefix']['norm_rms'][2]*1000:.6f} mm/s；有效样本峰值反而从 {a['all_prefix']['norm_peak'][2]*1000:.6f} 增到 {b['all_prefix']['norm_peak'][2]*1000:.6f} mm/s。更早的无效初始化峰值也在图中完整保留。低稳定段RMS不构成启动峰值改善或全时刻捕获条件。这是已见回归，不是新目标泛化。理想名义/H1 接触附近的新模型误差峰值及NIS也高于CV。所有有效样本与启动段、接触前后、测量时刻/控制时刻/盲预测结果见原始比较数组。",'',
        f"旧CV条件性下界独立复算为 {floor['old_CV']['recomputed_linear_3sigma_m_s']*1000:.7f} mm/s。新CA固定自由过程诊断约0.362–0.376 mm/s；持续接触过程约1.266–1.367 mm/s，仍超过1 mm/s线速度门，角速度裕量也有超门。该接触结论限于对应固定协议数值递推，不是所有状态的全局不可能性证明。",'',
        '完整几何协方差保留位置—姿态、速度—角速度及力臂姿态项。目标几何原点、目标接口、工具—目标真实误差、参考执行误差分别存储。机器人导航/关节、时间戳和固定标定的零预算明确为 SIMULATION_ASSUMPTION。逐时刻剩余预算不能用全程RMS替代；3σ不代表整体/全时域99.7%保证。','',
        '传感器需求另作八行离线设计诊断，估计参数与偏置界保持冻结。在零角速度、各向同性、固定接触模式的协方差递推中，仅将6 ms采样协议的延迟降为0，最坏线/角裕量仍约1.075 mm/s和0.203 deg/s。2 ms采样、2 ms延迟及原噪声的对应数值约0.818 mm/s和0.155 deg/s，留下少量必要余量；仍需覆盖实际跟踪和额外硬件误差。该表不验证过渡峰值、实际误差或任务，不是新估计候选、E3机器人结果或传感器部署。详见 sensor_requirement_budget.md。','',
        '## 完整机器人尝试','', '|条件|结果|实际时长(s)|真实guard调用|完整评价窗|最大真实载荷比例|','|---|---|---:|---:|---|---:|']
    for name in ('E0','E0_C2','E1','E2','E3'):
        if name in runs:
            r=runs[name];lines.append(f"|{name}|{r['status']}|{r['end_time_s']:.6f}|{r['actual_guard_calls']}|{r['final_window_status']}|{r['max_load_fraction']:.6f}|")
        else:lines.append(f'|{name}|NOT_RUN|—|—|NOT_EVALUATED|—|')
    lines+=['','在线目标几何原点速度误差（所有有效控制时刻，包含启动和事件尖峰）：','',
        '|尝试|RMS (mm/s)|峰值 (mm/s)|峰值时刻(s)|','|---|---:|---:|---:|']
    for name,d in diagnostics.items():
        if d.get('geometry_velocity_all_valid_rms_m_s') is not None:
            lines.append(f"|{name}|{d['geometry_velocity_all_valid_rms_m_s']*1000:.6f}|{d['geometry_velocity_all_valid_peak_m_s']*1000:.6f}|{d['geometry_velocity_peak_time_s']:.6f}|")
    lines+=['','真实捕获门调用与反事实预算分开统计。下表为真实调用中“估计相对误差+原裕量”超门的次数；四项可能同时失败，不能相加作为独立样本数。单次通过也不等于40 ms确认或任务通过。','',
        '|尝试|调用数|单次通过数|平移超门|姿态超门|线速度超门|角速度超门|','|---|---:|---:|---:|---:|---:|---:|']
    for name,g in guards.items():
        if g['calls']:
            counts=g['four_gate_exceedance_counts']
            lines.append(f"|{name}|{g['calls']}|{g['instantaneous_passes']}|{counts['translation']}|{counts['rotation']}|{counts['linear_velocity']}|{counts['angular_velocity']}|")
        else:lines.append(f'|{name}|0|—|—|—|—|—|')
    lines+=['','与原R03在预测残差检查处停止、从未调用真实捕获门不同，本轮实际调用和失败项以 guard_call_analysis.json 为准。末次调用的各项误差、裕量和原门槛均保留。规划拒绝和捕获信息不足可以同时存在，不能仅根据最终停止原因判定估计已准入。']
    for name,r in runs.items():
        if r['status']=='NO_VERIFIED_CONTROL':
            reasons=sorted({c['reason'] for c in r['reason']['candidates']})
            g=guards[name]
            lines+=['',f"{name} 在 {r['end_time_s']:.3f} s 的全部预测候选因 {', '.join(reasons)} 被拒绝。真实门共调用 {g['calls']} 次，单次通过 {g['instantaneous_passes']} 次；仅不确定性裕量自身超出线速度/角速度门的次数分别为 {g['information_alone_exceedance_counts']['linear_velocity']} / {g['information_alone_exceedance_counts']['angular_velocity']}。未执行最终拒绝提案，未将该停止解释为安全恢复。"]
    lines+=['','E0在7.814 s的接触过程切换处触发协方差非半正定错误。原始冻结源码、3907个物理步与终止异常全部保留。原因是直接RK4积分角白jerk协方差遗漏h^5方向方差；修复采用正权噪声因子积分，没有裁剪特征值或缩小P。记录包复现、修复回归和新增反例测试分开保存。E0_C2占第二次尝试；剩余E1/E2最多各一次，E3因四次上限取消。没有换基线或重新调参。','',
        '原20 s接近上限、锁紧后20 s及最后2 s评价保持。没有按事件结果挑选评价窗。NO_VERIFIED_CONTROL 是算法拒绝/仿真终止，不是安全恢复动作。零guard调用的接近轨迹不构成捕获信息已验证。','',
        '## 记录保护与重放','',f"{len(runs)} 条真实尝试在动力学前登记。原始SensorPacket、控制提案、内部P18/模式历史、输入已施加与积分完成事件分开保存。所有已提交块校验后才汇总。五类注入故障保留可验证前缀；强制终止只保证已提交块，未提交尾部不补造。独立执行器与测量包决策双重放：{all_replays}。真实安全违规数：{len(violations)}。",'',
        '接收姿态的R数组在预飞单元测试中触发SciPy 1.11只读buffer限制；在任何完整机器人运行前改为独立可写R副本，P18仍不可写。早期离线H2创新恒定，自相关未定义导致JSON拒绝NaN；已保存的比较数组保留，未定义相关明确为null。相关失败日志和非精确CPU计量范围保留。','',
        '## 计算与限制','',f"当前计入预算 {resource_used():.3f} s / 21600 s，包括预飞遗漏的保守300 s计费；该项不是实测CPU。物理步数 {sum(r['physical_steps'] for r in runs.values())}，在线预测步数 {sum(r['predictive_steps'] for r in runs.values())}。单次更新/预测/控制耗时、失败调用及I/O耗时见逐运行日志。未认证硬实时。",'',
        '本轮四次尝试不是统计验证矩阵；没有硬件验证、全惯性辨识或辨识反馈收益。SRS源参数缺口与C1缺失继续保留。最多四类新科学图；本轮未生成新视频，旧媒体保持。','',
        '## S03交接','',
        '不自动执行S03。仅将实际得到支持的测量域及冻结配置交接；持续接触不确定性、理想接触创新尖峰和治理器拒绝仍是未解决项。非理想任务未通过时，不准入非理想验证域；规划研究可从此前已验证的理想测量基线开始。','',
        '[资格矩阵](qualification.json) · [双重放及原始运行](runs/) · [科学图](figures/README.md) · [交接](handoff.json)','']
    (OUT/'report.md').write_text('\n'.join(lines),encoding='utf-8')
    text='\n'.join(lines[lines.index('## 方法与冻结边界'):lines.index('## 记录保护与重放')])
    (PROJECT/'paper/system_framework/s02_estimation_section.md').write_text('# S02 observation and capture-information extension\n\n'+text+'\n\nS01 remains SOURCE_COMPATIBLE_UNDER_ASSUMPTIONS with PARTIAL replay coverage and the original C1 trace loss. S02 does not establish hardware sensing, joint safety probability or global contact robustness.\n',encoding='utf-8')
    charge('report_and_scientific_figures',cpu,wall,new_robot_steps=0)
    evidence={p.relative_to(PROJECT).as_posix():sha(p) for p in OUT.rglob('*') if p.is_file() and p.name not in ('handoff.json','operation.lock') and not p.name.endswith(('.tmp','.pyc')) and '__pycache__' not in p.parts}
    evidence.update({p.relative_to(PROJECT).as_posix():sha(p) for p in (PROJECT/'v6_mujoco/system_capture/estimation').glob('*.py')})
    save(OUT/'handoff.json',dict(phase='S02',baseline=BASE,branch=BRANCH,status='COMPLETE_WITH_RETAINED_LIMITATIONS',qualification=q,
        evidence_sha256=evidence,evidence_set_sha256=digest(evidence),frozen_model=read(PROJECT/'configs/system_capture/algorithms/s02_acceleration.json'),
        next_stage_started=False,pushed=False,merged=False,commit_identity='Read the local commit containing this handoff; full SHA delivered outside tracked content to avoid self-reference'))
    return all_replays and not violations

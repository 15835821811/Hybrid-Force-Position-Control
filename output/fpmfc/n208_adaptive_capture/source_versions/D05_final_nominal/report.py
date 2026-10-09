"""Evidence-derived qualification, prior dependence and delivery report."""
import argparse
import copy
import json
import time
import numpy as np
import mujoco
from .common import ROOT,read,save,config,digest,init_ledger
from .momentum_regressor import parameters,tensor,physical
from .inertial_estimator import InertialEstimator

def identification(name):
    out=ROOT/'runs'/name;c=read(out/'config.json');raw=read(out/'identification.json');f=raw['final'];pi=np.array(f['pi']);truth=c['truth_evaluation_only'];mass=pi[0];com=pi[1:4]/mass
    Ic=tensor(pi)-mass*((com@com)*np.eye(3)-np.outer(com,com));trueI=np.array(truth['inertia'])
    pred=raw['prediction_before_assimilation'];informative=[x for x in pred if x['rank']>0]
    result={'run':name,'status':f['status'],'rank':f['rank'],'physical':f['physical'],'singular_values':f['singular_values'],'right_singular_vectors_scaled':f['right_singular_vectors_scaled'],
        'truth_error_evaluation_only':{'mass_relative':abs(mass-truth['mass'])/truth['mass'],'com_m':float(np.linalg.norm(com-truth['com'])),'inertia_com_frobenius_relative':float(np.linalg.norm(Ic-trueI)/np.linalg.norm(trueI))},
        'future_blocks_before_assimilation':{'count':len(informative),'posterior_rms_whitened':float(np.sqrt(np.mean([x['posterior']**2 for x in informative]))) if informative else None,'prior_rms_whitened':float(np.sqrt(np.mean([x['prior']**2 for x in informative]))) if informative else None},
        'full_inertia_identification':False,'scope':'Natural task data; rank and physicality do not establish full identification. Errors are evaluation-only. Whitening is approximate and blocks are correlated.'}
    if name=='D01_ideal':result['scope']='Historical development diagnostic used mixed-unit whitening; excluded from final parameter-identification qualification.'
    metrics=read(out/'metrics.json');latch=metrics.get('latch_time_s');post=[x for x in pred if latch is not None and x['time']>=latch and x['rank']>0]
    result['future_postlatch_blocks_before_assimilation']={'count':len(post),'posterior_rms_whitened':float(np.sqrt(np.mean([x['posterior']**2 for x in post]))) if post else None,'prior_rms_whitened':float(np.sqrt(np.mean([x['prior']**2 for x in post]))) if post else None}
    cross=np.array([x['robot_vs_interface'] for x in raw['impulse_crosscheck']])
    if len(cross):result['independent_impulse_crosscheck_max']={'linear_kg_m_s':float(np.max(np.linalg.norm(cross[:,:3],axis=1))),'angular_kg_m2_s':float(np.max(np.linalg.norm(cross[:,3:],axis=1)))}
    blockfile=out/'regression_blocks.npz';alternates=[]
    if blockfile.exists():
        with np.load(blockfile) as z:A=np.vstack([np.linalg.solve(L,a) for a,L in zip(z['DY'],z['cholesky'])]);b=np.concatenate([np.linalg.solve(L,h) for h,L in zip(z['dh'],z['cholesky'])])
        for mass0 in [15.,32.]:
            pr=copy.deepcopy(c['prior']);pr['mass_kg']=mass0;est=InertialEstimator(pr,c['mission']);est.solve(A,b,f['absolute_excitation_detected']);alternates.append({'prior_mass_kg':mass0,'posterior':est.snapshot()})
        result['same_data_alternative_priors']=alternates;result['alternative_prior_scope']='Offline diagnostic only; identical recorded blocks and whitening, different initial mass. Never fed back into completed run.'
    return result

def locked_accounting(name):
    # Hypothetical candidate accounting at RECORDED true configurations is
    # evaluation-only; do not label it as an online prediction.
    from .known_model import compile_known
    from .governor import put_inertia
    from v6_mujoco.geometry_capture.physics import momenta
    out=ROOT/'runs'/name;c=read(out/'config.json');raw=read(out/'identification.json');prior=c['prior'];base=parameters(prior['mass_kg'],prior['com_m'],prior['inertia_com_kg_m2']);tr=c['truth_evaluation_only']
    candidates={'prior_low_scale':.5*base,'prior_center':base,'prior_high_scale':2*base,'final_posterior':np.array(raw['final']['pi']),'true_evaluation_only':parameters(tr['mass'],tr['com'],tr['inertia'])};result={}
    with np.load(out/'trace.npz') as a:
        for label,pi in candidates.items():
            m,d=compile_known(prior);put_inertia(m,pi);snapshots=[]
            for i in [0,-1]:
                d.qpos[:]=a['qpos'][i];d.qvel[:]=a['qvel'][i];mujoco.mj_forward(m,d);snapshots.append(momenta(m,d))
            initial,final=snapshots;H0=np.array(initial['angular_momentum_about_center_world_kg_m2_s']);I=np.array(final['locked_inertia_world_kg_m2']);w=np.linalg.solve(I,H0)
            result[label]={'initial_P_kg_m_s':initial['linear_momentum_world_kg_m_s'],'initial_H_center_kg_m2_s':H0.tolist(),'final_locked_inertia_kg_m2':I.tolist(),'conserved_initial_momentum_locked_rate_deg_s':float(np.rad2deg(np.linalg.norm(w))),'final_instantaneous_locked_rate_deg_s':float(np.rad2deg(np.linalg.norm(final['omega_locked_prediction_world_rad_s'])))}
    return {'scope':'Evaluation-only candidate accounting at recorded configurations; no all-configuration attainability proof and no true-state feedback','candidates':result}

def run(selected):
    ledger=init_ledger();rows=[];ids={};accounting={};state_quality={}
    for item in ledger['attempts']:
        out=ROOT/'runs'/item['name'];p=out/'metrics.json'
        if not p.exists():continue
        m=read(p);v=read(out/'validation.json') if (out/'validation.json').exists() else {};c=read(out/'config.json')
        rows.append({'name':item['name'],'category':item['category'],'sensor':c['sensor_mode'],'mode':c.get('mode','prior'),**{k:m.get(k) for k in ['status','reason','end_time_s','latch_time_s','continuous_task_completed','postgrasp_detumbling','world_window_max_deg_s','relative_window_max_deg_s','peak_contact_force_n','max_load_fraction','final_identification_rank','parameter_feedback_used','max_P_drift_kg_m_s','max_H_drift_kg_m2_s','cpu_s','wall_s']},'actuator_replay_passed':v.get('actuator_replay_passed',False),'decision_replay_passed':v.get('decision_replay_passed',False)})
        if (out/'identification.json').exists():ids[item['name']]=identification(item['name'])
        if (out/'trace.npz').exists():
            accounting[item['name']]=locked_accounting(item['name'])
            with np.load(out/'trace.npz') as z:
                valid=z['estimate_valid'].astype(bool)
                if 'estimate_current_tick' in z:valid &= z['estimate_current_tick'].astype(bool)
                state_quality[item['name']]={'sensor':c['sensors'],'valid_control_ticks':int(sum(valid)),'max_measurement_age_s':float(np.max(z['measurement_age_s'][valid])) if np.any(valid) else None,'minimum_covariance_eigenvalue':float(np.min(np.linalg.eigvalsh(z['estimate_covariance'][valid]))) if np.any(valid) else None,'innovation_rms_components':np.sqrt(np.mean(z['innovation'][valid]**2,axis=0)).tolist() if np.any(valid) else None,'state_error_rms_components':m.get('state_error_rms_components'),'state_error_max_components':m.get('state_error_max_components'),'empirical_marginal_3sigma_coverage':m.get('empirical_marginal_3sigma_coverage'),'scope':'World-expressed geometric-origin errors on current output ticks. Innovations can repeat between delivered poses; correlated samples, no calibrated probability guarantee.'}
    save(ROOT/'identifiability_report.json',{'runs':ids,'full_inertia_identification':False,'synthetic_evidence':'synthetic_newton_euler_validation.json','synthetic_scope':'Separate externally forced synthetic rigid-body validation; not robot-plant actuator input.'})
    save(ROOT/'momentum_feasibility.json',accounting)
    save(ROOT/'state_estimation_summary.json',state_quality)
    save(ROOT/'comparison.json',{'selected':selected,'runs':rows,'paired_scope':'prior and identified modes share sensors, damping and governor; mode only enables the bounded parameter information gate. Outcomes never imply benefit unless actual parameter use and paired differences support it.'})
    s=next(x for x in rows if x['name']==selected);audit=read(ROOT/'truth_access_audit.json');tests=[read(p) for p in sorted((ROOT/'selftests').glob('*.json'))];latest=tests[-1];unitpass=all(x['passed'] for x in latest.values())
    holdouts=[x for x in rows if x['category']=='holdout'];noise=[x for x in rows if x['sensor']=='noisy'];mismatch=[x for x in rows if x['name']=='S02_contact_mismatch']
    replay_all=all(x['actuator_replay_passed'] and x['decision_replay_passed'] for x in rows)
    q={'passive_target_dynamics_validated':bool(unitpass and s['actuator_replay_passed']),'controller_truth_isolation':bool(audit['static_access_passed'] and unitpass and replay_all),'state_estimation_validated':{'ideal_implementation':bool(unitpass and s['decision_replay_passed']),'noisy_task_success':bool(noise and all(x['continuous_task_completed'] for x in noise)),'scope':'Per-run state errors and empirical marginal coverage are in metrics.json; no calibrated probabilistic or real sensor claim.'},'physical_inertia_constraints':all(x['physical'] for x in ids.values()),'identifiable_subspace':{k:v['rank'] for k,v in ids.items()},'full_inertia_identification':False,'event_based_capture':s['latch_time_s'] is not None and s['continuous_task_completed'],'continuous_task_completed':s['continuous_task_completed'],'postgrasp_detumbling':s['postgrasp_detumbling'],'hidden_parameter_cases':{'executed':len(holdouts),'task_successes':sum(bool(x['continuous_task_completed'] and x['postgrasp_detumbling']) for x in holdouts),'planned':6},'sensor_uncertainty_cases':{'executed':len(noise),'completed':sum(bool(x['continuous_task_completed']) for x in noise)},'contact_model_mismatch_case':{'executed':len(mismatch),'completed':sum(bool(x['continuous_task_completed']) for x in mismatch)},'hardware_load_rating_validated':False,'physical_gripper_capture_validated':False,'real_time_ready':False,'selected_run':selected,'scope':'Finite simulation campaign, not universal robust capture or full hardware validation.'}
    save(ROOT/'qualification_matrix.json',q)
    lines=['# N208：被动目标估计、事件捕获与抓后消旋报告','',f'基线：`{config()["baseline_commit"]}`。选中名义运行：`{selected}`。所有数值来自本轮记录；N110—N206历史证据保持不变。','',f'名义连续任务：**{s["continuous_task_completed"]}**；固定末窗消旋：**{s["postgrasp_detumbling"]}**；自然任务全惯性辨识：**未验证**。这些判定互相独立。','','## 模型与信息边界','', '目标从 t=0 自由积分，无主动执行器或状态注入。仅七关节真实控制输入；接触反力和软件 weld 由双方动力学求解。局部微重力模型忽略轨道梯度、气动力和晃动。N206工具、硬件限位、载荷与捕获门保持原声明，50 N/2 N·m仍是设计假设。','', '在线只接收几何坐标系位姿、基座导航、编码器、执行器力矩和理想接口F/T。控制模型由已知机器人/工具和公开先验独立构建。SO(3)误差状态EKF按观测时间更新，再因果预测到控制时刻；接触期提高过程不确定性。参考随估计目标移动，四项估计捕获门及裕量连续成立40 ms后请求锁紧。真值评价不能批准锁紧。','', '目标先验质量10–40 kg、COM逐轴±20 mm、物理一致惯量；名义20 kg/0.3 I。固定阻尼来自离线名义设计；可关闭的20 ms三模型制动治理不构成全不确定性鲁棒证明。辨识使用固定世界原点动量差分，SVD秩先于约束计算，只更新数据支持方向。独立未来块预测、物理一致性和信息门成立后才允许参数渐变进入预测；否则保持先验。没有为了满秩实施额外激励。','','## 运行结果','', '|运行|模式/测量|终态|锁紧 s|连续完成|消旋|参数实际接入|','|---|---|---|---:|---|---|---|']
    for r in rows:lines.append(f'|{r["name"]}|{r["mode"]}/{r["sensor"]}|{r["status"]}|{r["latch_time_s"]}|{r["continuous_task_completed"]}|{r["postgrasp_detumbling"]}|{r["parameter_feedback_used"]}|')
    lines+=['','每条运行的失败原因、峰值载荷、独立P/H漂移与实际结束时间见 comparison.json。抓后固定20 s、最后2 s评价；目标世界≤0.1 deg/s、目标—基座≤0.02 deg/s。未完成全窗的运行不评价为消旋成功。','', '## 辨识与动量解释','', 'identifiability_report.json逐条列出数据秩、奇异方向、物理性、真值误差和先于同化的未来块预测。相同记录块以15/32 kg先验离线重估，显示先验依赖；这些结果不回灌控制。独立受迫合成Newton–Euler数据验证充分激励下的实现，不能冒称真实任务提供相同信息。D01采用早期混合量纲权重，只作历史开发诊断，不纳入最终辨识结论。','', 'momentum_feasibility.json记录先验低/中/高尺度、最终后验和评价真值下的锁定惯量与共同残余角速度。它使用记录构型作离线核算，不是全构型可达性证明。内部阻尼耗能而不消除系统总角动量。P/H、数值几何力偶、wrench功率及能量账本独立保留，未扣除残差制造通过。','', '## 核验、边界与资源','',f'完整机器人尝试 {len(ledger["attempts"])}/16；累计已入账CPU {ledger["cpu_s"]:.3f} s / 28800 s。开发最多6次，所有错误和早停保留；独立重放、孤立物理及合成辨识分开计数。冻结状态：{ledger["frozen"]}。','', '力矩输入重放从t=0积分，仅重施真实输入与事件；传感器输入重放另行重新决策。前者不证明控制决策或惯性识别正确。truth_access_audit.json是源代码及行为证据，并非操作系统能力隔离证明。协方差覆盖率仅为逐轨迹边际统计；噪声传感器和理想接口F/T仍是仿真假设。未验证硬件承载、真实夹爪、真实视觉或实时截止期。','', '## 证据与可视化','', '- [资格矩阵](../output/fpmfc/n208_adaptive_capture/qualification_matrix.json)','- [逐运行比较](../output/fpmfc/n208_adaptive_capture/comparison.json)','- [可辨识性](../output/fpmfc/n208_adaptive_capture/identifiability_report.json)','- [运行账本](../output/fpmfc/n208_adaptive_capture/run_ledger.json)','- [当前科学图与两条视频](../output/fpmfc/n208_adaptive_capture/visualizations/index.html)','- [文献实施映射](../output/fpmfc/n208_adaptive_capture/literature_to_implementation.md)','', '完整轨迹、SensorPacket、后验、参考进度、动作和实现身份在 runs/；历史源快照在 source_versions/。冻结后场景的失败不用于回调算法参数。未执行项应由完成审计显式列出，不能记为通过。']
    (ROOT.parents[2]/'paper/N208_PASSIVE_TARGET_ESTIMATION_CAPTURE_REPORT.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    return q

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('selected');a=p.parse_args();print(json.dumps(run(a.selected),ensure_ascii=False))

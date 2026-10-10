"""Read-only evidence synthesis; unsuccessful tasks stay in denominators."""
import time
from collections import Counter
from functools import lru_cache
import numpy as np
from scipy.spatial.transform import Rotation
from .contracts import *

def folder_for(name):return S02/'runs/E0_C2' if name=='B00' else OUT/'runs'/name

@lru_cache(maxsize=1)
def base_qpos_slice():
    from v6_mujoco.adaptive_capture.plant import Plant,TruthConfig
    from v6_mujoco.postgrasp.physics import joint_slices
    c=read(folder_for('B00')/'config.json')
    m=Plant(TruthConfig(**c['truth_evaluation_only']),c['dt'],c['mission']['solver_tolerance']).model
    return joint_slices(m,'base_free_joint')[0]

def measures(name,end=None):
    f=folder_for(name);m=read(f/'metrics.json')
    with np.load(f/'trace.npz') as z:
        t=z['time_s'];mask=np.ones(len(t),bool) if end is None else t<=end+1e-9;t=t[mask]
        w=z['base_omega_world_rad_s'][mask];norm=np.linalg.norm(w,axis=1)
        base=z['qpos'][mask][:,base_qpos_slice()];R=Rotation.from_quat(base[:,[4,5,6,3]]);drift=(R[0].inv()*R).magnitude()
        T=float(t[-1]-t[0]);rms=np.sqrt(np.trapz(norm**2,t)/T) if T>0 else 0.
        active=z['approach_reference_applicable'][mask].astype(bool)
        covers_full_run=end is None or end>=m['end_time_s']-1e-9
        return dict(end_time_s=float(t[-1]),base_peak_rad_s=float(max(norm)),base_rms_rad_s=float(rms),
            base_attitude_peak_rad=float(max(drift)),load_peak=float(max(z['load_fraction'][mask])),
            tracking_peak_m=float(max(z['approach_position_error_m'][mask][active])) if active.any() else None,
            min_original_clearance_m=float(min(z['minimum_noncontact_clearance_m'][mask])),
            max_abs_joint_velocity_rad_s=float(np.max(abs(z['joint_velocity_rad_s'][mask]))),
            actual_impulse_Ns=np.trapz(z['force_grasp_n'][mask],t,axis=0).tolist(),
            postgrasp_metrics_status=m['final_window_status'] if covers_full_run else 'NOT_EVALUATED',
            latch_time_s=m['latch_time_s'] if m['latch_time_s'] is not None and m['latch_time_s']<=t[-1]+1e-9 else None,
            full_task=m['continuous_task_completed'] and covers_full_run,
            detumbling=m['postgrasp_detumbling'] if covers_full_run else None,status=m['status'] if covers_full_run else 'TRUNCATED_EXECUTED_PREFIX',
            window_scope='FULL_EXECUTED_RUN' if covers_full_run else 'TRUNCATED_EXECUTED_PREFIX',
            run_outcome=dict(status=m['status'],full_task=m['continuous_task_completed'],detumbling=m['postgrasp_detumbling'],latch_time_s=m['latch_time_s'],end_time_s=m['end_time_s']))

def pair(a,b):
    ma=measures(a);mb=measures(b);qualified=ma['detumbling'] and mb['detumbling']
    if not qualified:
        end=min(ma['end_time_s'],mb['end_time_s']);ma=measures(a,end);mb=measures(b,end)
    pct={k:100*(ma[k]-mb[k])/ma[k] if ma[k] else None for k in ['base_peak_rad_s','base_rms_rad_s','base_attitude_peak_rad','load_peak']} if qualified else None
    return dict(baseline=a,comparison=b,status='QUALIFIED_FULL_TASK_PAIR' if qualified else 'COMMON_EXECUTED_PREFIX_ONLY',
        baseline_metrics=ma,comparison_metrics=mb,full_task_relative_reduction_percent=pct,
        caveat=None if qualified else 'failed runs remain in denominator; prefix values are not complete-task benefit or universal risk evidence')

def report(defer_ledger=False):
    cpu,wall=time.process_time(),time.perf_counter();book=read(OUT/'run_ledger.json');names=['B00']+[e['name'] for e in book['attempts']]
    rows={n:measures(n) for n in names if (folder_for(n)/'metrics.json').exists()}
    pairs={}
    for scene,baseline,b1,b2 in [('nominal','B00','P00','P01'),('H2','B01','P02','P03')]:
        if all(n in rows for n in [baseline,b1,b2]):pairs[scene]=dict(time_path=pair(baseline,b1),shape=pair(b1,b2),joint=pair(baseline,b2))
    save(OUT/'same_model_comparison.json',dict(runs=rows,pairs=pairs,measurement='same original ideal SensorPacket + frozen C2 CA18',
        task_denominator_includes_all_attempts=True,paired_success_count=sum(rows[n]['detumbling'] for n in ['P00','P01','P02','P03'] if n in rows),
        paired_attempt_count=sum(n in rows for n in ['P00','P01','P02','P03']),independent_unseen_validation=False))
    dual={}
    for n in names:
        dual[n]={}
        for k in ['actuator','decision']:
            p=OUT/'runs'/n/(k+'_replay.json');dual[n][k]=read(p)['passed'] if p.exists() else 'PENDING'
    budget=read(OUT/'candidate_budget.json');latency=[];control={};errors=[];selections={}
    for n in names:
        f=folder_for(n)
        if (f/'timing.json').exists():control[n]=read(f/'timing.json')
        if (f/'progress_governor.json').exists():
            g=read(f/'progress_governor.json');latency.extend(x['latency_s'] for x in g if 'latency_s' in x)
            selections[n]=[dict(time=x['time'],selected=x.get('selected',x.get('chosen_fraction')),failure=x.get('failure')) for x in g]
        if (f/'planner_cache_errors.json').exists():errors+=[dict(run=n,**e) for e in read(f/'planner_cache_errors.json')]
    save(OUT/'planner_vs_plant_errors.json',dict(cache_state_errors=errors,
        definition='CA18 public robot/target state versus independent-prior cached forecast at task ticks; truth never feeds planner',
        selections=selections,scope='cached prefix approximation, not global predictor accuracy'))
    compute=dict(cpu_s=used(),cpu_limit_s=LIMITS['cpu_s'],elapsed_wall_s=time.time()-book['started_epoch'],wall_limit_s=LIMITS['wall_s'],
        candidate_evaluations=budget['evaluations'],candidate_limit=LIMITS['candidates'],kinematic_steps=budget['kinematic_steps'],
        prior_physics_steps=budget['predictive_steps'],replay_prior_physics_steps=budget['replay_predictive_steps'],
        total_predictive_physics_steps=budget['predictive_steps']+budget['replay_predictive_steps'],
        named_evaluations_by_run=dict(Counter(e['run'] for e in budget['entries'])),
        named_evaluations_by_model=dict(Counter(e['model'] for e in budget['entries'])),
        actuator_replay_physics_steps=sum(e.get('physics_steps',0) for e in book['replays']),
        physical_step_scope='new full robot attempts only; strict archived B00 reuse is not a new sample',
        predictive_step_limit=LIMITS['predictive_steps'],plant_steps=sum(x['physical_steps'] for x in book['attempts']),
        actual_attempts=len(book['attempts']),reused=['B00'],latency_s_p50_p95_max=np.quantile(latency,[.5,.95,1.]).tolist() if latency else None,
        by_run=control,trial_cpu_s={r['name']:r['cpu_s'] for r in book['attempts']},
        timing_scope='B00 control timing is archived source-run timing; current B00 work is strict replays. Other run timing is measured this campaign. Aggregate latency is descriptive, not a synchronized hardware benchmark.',
        cpu_accounting_scope='instrumented process CPU plus separately declared conservative support allowances; video encoder children included',
        real_time='NON_REALTIME_SIMULATION',deadline='NOT_CERTIFIED; computation pauses simulation; no delay protocol simulated')
    save(OUT/'compute_profile.json',compute)
    nominal=rows.get('P01',{}).get('detumbling',False);h2=rows.get('P03',{}).get('detumbling',False)
    all_dual=all(v is True for r in dual.values() for v in r.values())
    q=dict(S02_retained='ACCURACY_IMPROVED_GUARD_NOT_COMPATIBLE',historical_source_gap='RETAINED',
        implementation='MODULE_TESTED' if read(OUT/'module_tests.json')['passed'] else 'FAILED',
        planning_authority=read(OUT/'candidate_authority.json')['status'] if (OUT/'candidate_authority.json').exists() else 'PENDING',
        nominal_continuous_task='PASS' if nominal else 'NOT_ADMITTED',H2_ideal_task='PASS' if h2 else 'NOT_ADMITTED',
        H1_seen_regression='PASS' if rows.get('R00',{}).get('detumbling',False) else ('FAILED' if 'R00' in rows else 'NOT_RUN_ADMISSION_NOT_MET'),
        time_path_benefit='SEE_QUALIFIED_PAIRS' if any(x['time_path']['status']=='QUALIFIED_FULL_TASK_PAIR' for x in pairs.values()) else 'NOT_DEMONSTRATED',
        shape_incremental_benefit='SEE_QUALIFIED_PAIRS' if any(x['shape']['status']=='QUALIFIED_FULL_TASK_PAIR' for x in pairs.values()) else 'NOT_DEMONSTRATED',
        base_disturbance_tradeoff='COMMON_PREFIX_ONLY_NO_FULL_TASK_BENEFIT' if not nominal or not h2 else 'SEE_QUALIFIED_PAIRS',noise_domain='NOT_ADMITTED',full_inertia_benefit='NOT_EVALUATED',
        dual_replay='PASS' if all_dual else 'PENDING_OR_FAILED',dual_replay_details=dual,real_time='NOT_CERTIFIED',
        next_stage_admission='IDEAL_TESTED_DOMAIN_ONLY_REQUIRES_NEW_AUTHORIZATION' if nominal and h2 else 'NOT_ADMITTED',
        scope='finite deterministic family and seen conditions; no general impossibility, robustness, hardware, or independent unseen-domain claim')
    save(OUT/'qualification.json',q)
    lines=['# S03 时间—路径—臂形规划报告','',
        f"B2 名义完整任务：{q['nominal_continuous_task']}；H2 理想任务：{q['H2_ideal_task']}。候选权限：{q['planning_authority']}；双重放：{q['dual_replay']}。",'',
        '## 冻结与范围','',
        '基线 b048891c57d3ec0df72ec53ab87e20f2fe9aa1ac；分支 codex/system-s03-pm-feasible-planning。冻结物理模型、20mm工具、七执行器、CA18 C2 参数、捕获四门/40ms确认、抓后20s阻尼与末2s评价。控制器只接收原理想 SensorPacket，真值仅用于独立评价和拒绝。',
        'S02 ACCURACY_IMPROVED_GUARD_NOT_COMPATIBLE、S01 PARTIAL、C1 原始记录缺失和 E0 PSD 原失败均保留。非理想测量域 NOT_ADMITTED；本轮没有噪声闭环或新传感器协议。','',
        '## 任务与配对','', '|运行|方法/条件|状态|实际终止(s)|完整消旋合格|','|---|---|---|---:|---|']
    for n,v in rows.items():
        row=read(OUT/'experiment_manifest.json')['run_matrix'].get(n,{})
        method='B0 / nominal (严格重放复用)' if n=='B00' else row.get('method','?')+' / '+row.get('scene','?')
        lines.append(f"|{n}|{method}|{v['status']}|{v['end_time_s']:.3f}|{v['detumbling']}|")
    lines+=['','B00 的原始身份经字节核对，并执行独立执行器与逐包决策重放后复用。B01 在同一 CA18 理想域 5.900s 停止，五候选均为预测任务残差拒绝，捕获门调用为0。已有 H2 硬集合证书属于另外的历史状态，不能与该停止原因混同。',
        '','B1/B2 的候选数、求解器、性能门和安全条件共享；B1 仅将臂形修正限制为0。失败仍进入任务分母。双方完整合格之前，same_model_comparison.json 只给共同已执行前缀，不给完整收益百分比。','',
        'B1/B2 每次规划最多筛查五个确定性成员、动态验证两个，资源上限相同，实际消耗逐次登记。B2 的两个成员耦合路径与臂形变化，并非独立穷举整个五维参数盒。每次参数、筛查理由、动态验证数量和选择序列见 search_coverage.json 与各运行 progress_governor.json。',
        'D00 原 V1 在6.440s遭遇原子预算文件替换的 Windows 共享错误，完整原始前缀保留。V2 仅重试同一原子写入，不改变科学参数；D00_R1 使用唯一实现错误重试预留。D00 决策重放显式注入原外部故障，证明科学前缀与故障路径可重现，不声称操作系统故障自然再现。两版实现额度已使用，仅一组规划参数。','',
        '## 候选与约束','',
        '五维参数、界限、评分、有效时间、缓存失效域和频率见 planning_contract.json。局部段从正在执行的参考 jets 拼接；终端 T_WF T_FE = T_WG 不变。s=u/8 的兼容接口明确区分虚拟进度和真实到达时间，运输项在终端仍然存在。',
        'candidate_authority.json 保存固定 H2 状态上每个变量的真实预测动作与命名裕量变化，diagnostics/current_sets.json 保存一级/二级、独立 Phase-I 和有单位的约束行。终端刚性 link7—目标关系不能由上游臂形消除。有限族失败不证明所有轨迹不存在。',
        '候选权限仅在四个预登记状态（0.04、3.0、4.6、5.2s），各11个候选、0.6s先验预测中验证。46是全部只读诊断状态数；12是B01当前集合检查状态数。五维最大动作/裕量变化可能来自不同状态和约束，并非可同时获得的改善。带正负号的完整有限距离表见 diagnostics/authority_signed_margins.csv；1e9缺失距离哨兵不作为物理裕量。',
        'B01前缀当前集合证书最晚为5.8s，没有单独认证5.9s终止当刻的硬集合。该次预测末端u约6.0583–6.0600，与历史u=8的路径失权不同。mixed-unit slack不转写为毫米；未进入捕获的零载荷不能作为低风险收益。',
        'S02 E1/E2 的90/490次实际捕获门调用、零单次通过和两类预测拒绝保持只读。启动峰值不被当作8s附近拒绝的唯一根因。SIM_ABORT没有被称为安全撤离。','',
        '## 计算与可重放性','',f"新增真实尝试 {len(book['attempts'])}/12；命名评价 {budget['evaluations']}/1000；已登记过程CPU {used():.3f}s/43200s。各次实际计算和预测步数见 compute_profile.json 与 ledger。",
        '原始测量、完整P18、提案/已施加力矩、完成物理步、参考系数与候选选择先分块持久化。重放从t=0施加力矩和锁紧事件，决策重放核对估计、参考系数、规划选择与终止。没有中间植物状态注入。',
        '模块检查19项通过，含独立参考导数、C2拼接、候选顺序隔离和原PSD缺陷回归；原18项结果和修复前诊断均保留。安全无越门仅适用于记录的已执行物理步，SIM_ABORT不是硬件安全撤离。H1仅在名义/H2 B2完整任务合格后准入；未达到该条件时跳过，不回填成功。',
        '计算期间植物暂停，明确为 NON_REALTIME_SIMULATION。未模拟计算时延，未认证硬实时。安全结论仅限已执行前缀；步数不是独立任务样本。','',
        '## 交接','',
        '精确来源与公式对应见 source_method_mapping.md。代码与证据只进入本独立分支；按用户目标末尾授权发布 GitHub 并刷新当前运行可视化。不会自动启动 S02R 或 S04–S08。',
        '[资格](qualification.json) · [同模型对照](same_model_comparison.json) · [原始运行](runs/) · [可视化](visualizations/index.html) · [交接](handoff.json)']
    if (OUT/'search_coverage.json').exists():
        lines+=['','## 终止候选与最小反例','',
            '|运行|实际终止(s)|候选记录时刻(s)|筛查拒绝|动态拒绝|筛查通过但未动态验证|实际捕获门调用|','|---|---:|---:|---|---|---:|---:|']
        for n,v in read(OUT/'search_coverage.json')['runs'].items():
            sr=Counter(c['screening'].get('reason') for c in v['last_candidates'] if c['screening'].get('reason'))
            pr=Counter(c['prediction'].get('reason') for c in v['last_candidates'] if c['prediction'].get('reason'))
            untested=sum(c['screening'].get('valid',False) and not c['dynamic_evaluated'] for c in v['last_candidates'])
            lines.append(f"|{n}|{v['actual_end_time_s']:.3f}|{v['last_candidate_record_time_s']:.3f}|{dict(sr)}|{dict(pr)}|{untested}|{v['actual_guard_calls']}|")
        lines+=['','上述终止来自当前有限策略；粗筛关节盒失败只是粗运动学模型的拒绝，不是实际当前硬集合空集证书。完整动态失败包含原任务残差/几何/跟踪/载荷门，不把性能门混称真实碰撞。精确参数、阈值、命名对与已验证时域见 search_coverage.json 和原始候选记录。']
        lines+=['P01 的 index 1/3 通过粗筛但未获两次动态验证名额，不能归为已经证明不可行；index 0/2 动态残差拒绝，index 4 粗筛拒绝。当前选择策略及名额限制下没有取得新验证计划，不代表这五个成员逐个都经过动力学否证。',
            'D00表中是故障前最后一组完整候选记录，不是故障时刻的动态否证；6.44s发生的是预算文件原子替换异常，失败事件与未提交预算文件另行保存。',
            '审查修正了汇总/绘图的基座姿态索引：现按 base_free_joint 名称取得 qpos[7:14]，不是位于 qpos[0:7] 的目标。该问题只涉及离线汇总/显示，不改变实际控制、轨迹或任务结论。历史 S02 已发布 base_motion 图的同类索引问题保留为媒体勘误，本轮未重生成历史媒体。']
        lines+=['','### 已观察到的到达时间变化','',
            '|配对运行|首次选定到达时刻(s)|最后选定到达时刻(s)|曾选定的最晚到达时刻(s)|','|---|---:|---:|---:|']
        for n in ['P00','P01','P02','P03']:
            sequence=read(OUT/'search_coverage.json')['runs'][n]['planning_sequence']
            arrivals=[x['arrival_s'] for x in sequence if x['selected'] is not None]
            lines.append(f"|{n}|{arrivals[0]:.2f}|{arrivals[-1]:.2f}|{max(arrivals):.2f}|")
        lines+=['','四条配对均多次延后计划到达时刻，且实际停止发生在最后选定到达时刻之前。这是记录中的行为，尚不能单独认定为失败原因。结合有限动态验证名额、粗筛与实际伺服差异，后续重新设计时应检查剩余时间代价和搜索覆盖；本轮未据此新增调参或运行。']
    if (OUT/'EXPERIMENT_RESULT_TO_CLAIM.md').exists():
        lines+=['','## 独立证据审查','',(OUT/'EXPERIMENT_RESULT_TO_CLAIM.md').read_text(encoding='utf-8')]
    (OUT/'report.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    if defer_ledger:save(OUT/'report_preview_receipt.json',dict(kind='report_preview',cpu_s=time.process_time()-cpu,wall_s=time.perf_counter()-wall))
    else:charge('report',cpu,wall)
    compute['cpu_s']=used();save(OUT/'compute_profile.json',compute)
    files={p.relative_to(PROJECT).as_posix():sha(p) for p in OUT.rglob('*') if p.is_file() and p.name not in ('handoff.json','operation.lock') and '.tmp' not in p.name}
    save(OUT/'handoff.json',dict(phase='S03',baseline=BASE,branch=BRANCH,qualification=q,actual_attempts=len(book['attempts']),
        reused='B00: S02/E0_C2 strict dual replay',unused_development='D02/D03 not run; two implementations consumed by V1 and IO-only V2 repair, one scientific parameter family',
        implementation_retry='D00_R1 consumed the single reserve; original D00 failure and fault-injected decision replay retained',
        H1_regression=read(OUT/'regression_skipped.json') if (OUT/'regression_skipped.json').exists() else 'see R00 result',
        S01='PARTIAL',C1='original raw gap retained',E0='original PSD implementation failure retained',
        terminal_hardset_certification='not newly certified at all stop times; current-set table ends at B01 5.8s',
        next_stage_started=False,S02R_todo=['causal startup readiness/history initialization','contact process covariance and innovation transients','separate sensing requirements with actual tracking reserve'],
        evidence_sha256=files,commit='containing Git commit, supplied at delivery to avoid self-reference',push_authorized=True))
    return q

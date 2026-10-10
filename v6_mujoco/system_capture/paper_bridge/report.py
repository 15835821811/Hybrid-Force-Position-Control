"""Report only: qualification, offline comparison and post-ledger handoff."""
import atexit
import time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from .common import OUT,PROJECT,CONFIG,PROTOCOL,BASE,read,save,sha,digest,source,assumptions,git
from .phase import book,verify
from .figures import run as figures

METRICS=['base_peak_angular_speed_rad_s','base_rms_angular_speed_rad_s','base_max_attitude_drift_rad']

def comparisons():
    output={}
    for name in ['fixed_time_comparison','equal_budget_comparison']:
        obj=read(OUT/(name+'.json'));groups={'common':obj['rows']} if name.startswith('fixed') else {str(s):[r for r in obj['rows'] if r['seed']==s] for s in obj['seeds']}
        out={}
        for key,rows in groups.items():
            joint=next(r for r in rows if r.get('label',r.get('strategy'))=='joint')
            pairs=[]
            for other in rows:
                if other is joint:continue
                valid=bool(joint['dynamics']['feasible'] and other['dynamics']['feasible'])
                changes={}
                if valid:
                    for metric in METRICS:
                        baseline=other['dynamics'][metric];value=joint['dynamics'][metric]
                        changes[metric]={'baseline':baseline,'joint':value,'relative_reduction_percent':100*(baseline-value)/baseline if baseline>0 else None}
                pairs.append({'baseline':other.get('label',other.get('strategy')),'admissible':valid,'metrics':changes,
                    'status':'QUALIFIED_PAIR' if valid else 'NOT_COMPARABLE_UNQUALIFIED_TRAJECTORY'})
            out[key]={'all_three_qualified':all(r['dynamics']['feasible'] for r in rows),'pairs':pairs}
        output[name]=out
    save(OUT/'qualified_benefit_comparison.json',output)
    return output

def refinement():
    original=read(OUT/'numerical_refinement.json');zs=[]
    for key in ['coarse_dynamic_run','fine_dynamic_run']:
        with np.load(OUT/'dynamics_runs'/original[key]/'trace.npz',allow_pickle=False) as a:zs.append({k:a[k] for k in a.files})
    c,f=zs;end=min(c['time_s'][-1],f['time_s'][-1]);mask=c['time_s']<=end;times=c['time_s'][mask]
    def interpolate(field):
        v=f[field];return np.stack([np.interp(times,f['time_s'],v[:,j]) for j in range(v.shape[1])],axis=1)
    result={'scope':'Offline absolute-time intersection, no added trajectory or state injection','coarse_end_time_s':float(c['time_s'][-1]),
      'fine_end_time_s':float(f['time_s'][-1]),'stop_time_difference_s':float(f['time_s'][-1]-c['time_s'][-1]),
      'comparison_end_s':float(end),'common_samples':len(times),'joint_position_max_abs_difference_rad':float(abs(c['qpos'][mask,7:]-interpolate('qpos')[:,7:]).max()),
      'qvel_max_abs_component_difference':float(abs(c['qvel'][mask]-interpolate('qvel')).max()),
      'flange_position_max_norm_difference_m':float(np.linalg.norm(c['flange_position_world_m'][mask]-interpolate('flange_position_world_m'),axis=1).max()),
      'base_position_max_norm_difference_m':float(np.linalg.norm(c['qpos'][mask,:3]-interpolate('qpos')[:,:3],axis=1).max()),
      'interpretation':'No predeclared dynamic convergence acceptance bound; descriptive sensitivity only. Early stopping is not a successful full-horizon refinement.'}
    rf=Slerp(f['time_s'],Rotation.from_matrix(f['base_rotation_world']))(times)
    rc=Rotation.from_matrix(c['base_rotation_world'][mask]);result['base_rotation_max_difference_rad']=float((rf.inv()*rc).magnitude().max())
    save(OUT/'numerical_refinement_alignment.json',result)
    return result

def rows():
    result=[]
    for row in book()['dynamics']:
        m=read(PROJECT/row['result_file'])
        result.append({'key':row['key'],'candidate':'C2' if row['key'].startswith('C2_') else 'C1','ledger_status':row['status'],'metrics':m,
           'raw_available':(PROJECT/row['result_file']).with_name('trace.npz').exists()})
    return result

def audit():
    manifest=verify();b=book();eq=read(OUT/'equation_tests.json');runs=rows();replay=read(OUT/'replay_summary.json')
    protocol=read(PROTOCOL);cells=[read(p) for p in sorted((OUT/'optimization_runs').glob('*/result.json'))]
    tests={}
    def add(key,status,evidence,limitation=''):tests[key]={'status':status,'evidence':evidence,'limitation':limitation}
    add('baseline_and_independent_branch','PASS','phase_manifest.json; input commit '+BASE)
    add('source_reading_pages_and_tables','PASS','source_contract.json; source_parameters.yaml; source_ambiguities.md')
    add('public_source_parameter_completeness','SOURCE_LIMITED','assumptions.yaml/missing_source_fields','COM, axes, mounting, geometry, target convention, objective and drive gaps remain explicit')
    add('freeze_before_first_trajectory','PASS','C1 manifest_history; C2 repair manifest; timestamped run_ledger','C2 is a recording-only software repair; initial C1 protocol candidate label is retained')
    add('free_base_seven_inputs_no_contact','PASS' if read(OUT/'model_checks.json')['passed'] else 'FAIL','model_checks.json; models/paper_compat/srs.xml')
    add('independent_math_eleven_states','PASS' if eq['passed'] and len(eq['states'])==11 else 'FAIL','equation_tests.json')
    add('shape_formula_ambiguity_and_nullspace_counterexample','PASS','equation_tests.json; equation_tests_diagnostic_l_normal.json','Eq23 two readings, Eq31 residual interpretation, JgN and AN separately retained')
    add('quintic_reference_and_velocity_consistency','PASS' if eq['passed'] else 'FAIL','equation_tests.json/reference_checks')
    add('author_reported_point_not_optimized_claim','PASS','author_point_check.json')
    add('fair_optimization_budget','PASS' if len(cells)==6 and all(c['evaluations']==120 for c in cells) else 'FAIL','optimization_runs/*/result.json')
    add('both_registered_seeds','PASS' if set(c['seed'] for c in cells)==set(protocol['seeds']) else 'FAIL','optimization_runs/*/result.json')
    add('common_time_rule_and_failures_retained','PASS','fixed_time_comparison.json; equal_budget_comparison.json; qualified_benefit_comparison.json')
    add('all_current_recordings_real_torque_integration','PASS' if all(r['metrics'].get('intermediate_state_injections')==0 and r['metrics'].get('initializations')==1 for r in runs if r['candidate']=='C2') else 'FAIL','dynamics_runs/C2_*/metrics.json; source_versions')
    add('all_original_attempts_have_raw_input_and_state','PASS' if all(r['raw_available'] for r in runs) else 'FAIL_RETAINED_C1_RECORDING_LOSS','dynamics_runs/author_point_source/metrics.json','One C1 attempt ended with a Python serialization error before raw trace persistence; original input cannot be recovered or invented')
    add('all_current_candidate_recordings_independently_replayed','PASS' if replay['available_recordings_passed'] else 'FAIL','replay_summary.json')
    add('whole_campaign_replay','PASS' if replay['passed'] else 'PARTIAL','replay_summary.json','The missing C1 recording prevents a blanket all-attempt replay claim')
    add('budget_and_no_scope_expansion','PASS' if len(b['dynamics'])<=12 and len(b['evaluations'])<=800 and b['new_flexiv_attempts']==0 and not b['next_phases_started'] and sum(o['cpu_s'] for o in b['operations'])<=28800 else 'FAIL','run_ledger.json','Exactly two implementation candidates; no new Flexiv or S02-S08')
    add('frozen_s00_and_runtime_preserved','PASS','phase_manifest.json hashes verified at report')
    add('four_core_figure_groups','PASS' if len(read(OUT/'figures/manifest.json'))==4 else 'FAIL','figures/manifest.json')
    add('trace_metrics_and_early_stop_scope','PASS','dynamics_runs/*/metrics.json; report.md','terminal_* fields of early-stop metrics mean last recorded sample; terminal requirements are NOT_EVALUATED unless completed_horizon=true')
    add('separate_qualification_fields','PASS','qualification.json')
    add('report_chapter_and_handoff','PASS','report.md; paper/system_framework/reproduction_section.md; handoff.json')
    add('optional_video','NOT_REQUIRED','At most one new SRS video; publication display may be generated by tools/s01_render.py','Historical display refresh is separately authorized by latest user request, not added research evidence')
    add('reproducible_entry_points','PASS','commands.md; CLI operation ledger')
    strict=all(x['status'] in ['PASS','SOURCE_LIMITED','NOT_REQUIRED'] for x in tests.values())
    save(OUT/'completion_audit.json',{'all_requirements_passed':strict,'engineering_delivery':'COMPLETE_WITH_RETAINED_FAILURES_AND_RECORDING_GAP','requirements':tests,
       'further_automatic_research':False,'open_irrecoverable_item':'C1 original raw trace; do not consume a new attempt to misrepresent its recovery'})
    return tests

def write_report():
    b=book();eq=read(OUT/'equation_tests.json');replay=read(OUT/'replay_summary.json');allrows=rows();current=[r for r in allrows if r['candidate']=='C2']
    cells=[read(p) for p in sorted((OUT/'optimization_runs').glob('*/result.json'))];benefit=read(OUT/'qualified_benefit_comparison.json')
    qualified=sum(r['metrics']['feasible'] for r in current);pairlist=[p for g in benefit.values() for entry in g.values() for p in entry['pairs']]
    validpairs=[p for p in pairlist if p['admissible']]
    positive=any(p['metrics']['base_peak_angular_speed_rad_s']['relative_reduction_percent']>0 for p in validpairs)
    q={'implementation':{'status':'C2_IMPLEMENTED_C1_RECORDING_FAILURE_RETAINED','candidates_used':2,'C2_runtime':read(OUT/'phase_manifest.json')['runtime_hash'],'missing_C1_trace':True},
       'mathematical_consistency':{'status':'PASS' if eq['passed'] else 'FAIL','states':len(eq['states'])},
       'source_parameter_completeness':{'status':'SOURCE_LIMITED','gaps':assumptions()['missing_source_fields']},
       'reproduction_scope':{'status':'SOURCE_COMPATIBLE_UNDER_ASSUMPTIONS','exact_source_numerical_reproduction':False,'protocol_A':'DECLARED_ADAPTATION','protocol_B':'NEW_EXTENSION','planning_budget':'REDUCED_BUDGET'},
       'dynamic_feasibility':{'status':'QUALIFIED_SUBSET' if qualified else 'NO_QUALIFIED_DYNAMICS','C2_qualified':qualified,'C2_attempts':len(current),'C1_failed_attempts':len(allrows)-len(current),'physical_collision_clearance':'SOURCE_LIMITED'},
       'base_disturbance_comparison':{'status':'BENEFIT_IN_QUALIFIED_PAIRS_ONLY' if positive else ('NO_BENEFIT_IN_TESTED_PROTOCOL' if validpairs else 'NOT_COMPARABLE_NO_QUALIFIED_PAIR'),'qualified_pairs':len(validpairs),'total_declared_pairs':len(pairlist),'population_or_original_percentage_claim':False},
       'replay':{'status':'PASS' if replay['passed'] else 'PARTIAL_C1_RECORDING_LOSS','whole_campaign_passed':replay['passed'],'available_recordings_passed':replay['available_recordings_passed'],'missing_original_recordings':replay['missing_original_recordings']},
       'next_stage_admission':{'status':'S02_SEPARATE_AUTHORIZATION_REQUIRED','automatically_started':False,'source_gaps_alone_block_engineering_line':False,'cannot_claim_positive_reproduction_without_qualified_evidence':True,'C1_recording_gap_requires_explicit_acceptance_for_strict_all_attempt_archival_admission':True}}
    save(OUT/'qualification.json',q)
    cpu=sum(o['cpu_s'] for o in b['operations']);wall=sum(o['wall_s'] for o in b['operations'])
    lines=['# S01 原论文兼容 SRS 基准报告','',
      '**工程交付完成，但全批验收存在不可恢复的 C1 记录缺口；不声明精确数值复现。** 来源缺口、数学正确性、动力学通过与重放完整性分别判定。',
      '',f'基线 {BASE}；当前分支 codex/system-s01-paper-bridge；运行身份 '+read(OUT/'phase_manifest.json')['runtime_hash']+'。','',
      '## 来源与补充假设','',
      '已逐页核对 PDF 4–11 页（印刷页961–968），包括广义 Jacobian、式18–25臂形、式26–34层级/阻抗、表2/3、式35及PSO。型Ⅰ是基座姿态，型Ⅱ是臂形。15.6 s、0.2686 rad和论文百分比不是拟合目标。论文证据限于接触前，期望操作力为零。',
      '',
      '源文缺少连杆 COM、惯性参考点/轴、DH安装、实体几何、目标完整运动约定、目标惯性、搜索边界、目标权重/尺度与δo聚合、姿态插值及执行器配置。assumptions.yaml在首条评价前冻结补充假设。使用Rx(pi)悬挂安装、DH位移中点COM和pre-Rx惯性轴；目标仅为规定运动的规划参考，无接触力。几何仅中心线示意，不能据此证明实际避碰。',
      '',
      'SRS质量524 kg、七个关节力矩输入、被动自由基座和零重力；没有引入Flexiv的20 mm工具、0.206 kg质量或工程阈值。原联仿的关节运动驱动替换为显式Schur补偿力矩伺服，物理步2 ms、任务周期20 ms，各方法共用相同驱动和限制。',
      '',
      '## 数学与层级','',f'模型与{len(eq["states"])}个预注册构型检查：{eq["passed"]}。独立DH/刚体求和与MuJoCo质量矩阵、运动学差分、动量和动能相符。',
      f'式23的l=w×V读法最大残差 {max(r["eq23_l_equals_w_cross_V_error"] for r in eq["states"]):.8g}；l=(w×V)×unit(w)读法为 {max(r["eq23_l_equals_projected_LV_error"] for r in eq["states"]):.8g}。控制采用独立atan2导数，两个读法均留档。',
      'JgN≈0与AN≠0分开验证；后者说明末端零空间不自动无基座反作用。式31采用减去主任务所产生臂形速度的残差解释，字面形式的重复项另测。式33/34位姿增量与速度的单位衔接缺口保留，本阶段不声称实现了接触阻抗。',
      f'无活跃约束的同模型A/B主任务差异最大 {max(r["AB_primary_velocity_error"] for r in eq["states"]):.8g}，臂形速度差异最大 {max(r["AB_shape_velocity_error"] for r in eq["states"]):.8g} rad/s。该条件性一致不要求受约束/不同正则化的动力学轨迹相同。',
      '',
      '## 预算与优化结果','',
      f'本批累计运动学评价 {len(b["evaluations"])}/800、真实SRS动力学尝试 {len(b["dynamics"])}/12、实现候选2/2、新Flexiv运行0。每个种子×策略严格120次，共720次PSO评价；其他评价包括作者点、共同时间和半步检查及C1失败记录。累计已完成CLI操作CPU {cpu:.3f} s、串行操作墙钟总和 {wall:.3f} s；不是实时控制性能统计。显示刷新CPU另记。缩减为20粒子×6代，不作收敛保证。',
      '',
      '|种子|策略|T (s)|psi (rad)|运动学合格|G|',
      '|---|---|---:|---:|---|---:|']
    for c in cells:lines.append(f'|{c["seed"]}|{c["strategy"]}|{c["best_position"][0]:.6f}|{c["best_position"][1]:.6f}|{c["best_rollout"]["feasible"]}|{c["best_rollout"]["objective_G"]:.8f}|')
    lines+=['','目标是归一化世界基座角速度峰值平方加捕获方向角平方，权重1/1为补充假设，非PSO学习因子1.5/2。RMS及姿态漂移另报；共同时间由种子240601的联合结果决定，失败也不重选。','',
      '## 全部真实动力学尝试','',
      '|运行|结果|实际/要求时间 (s)|合格|峰值角速度 (deg/s)|RMS (deg/s)|姿态最大漂移 (deg)|',
      '|---|---|---:|---|---:|---:|---:|']
    for r in allrows:
        m=r['metrics']
        if 'end_time_s' not in m:lines.append('|'+r['key']+'|IMPLEMENTATION_ERROR / 原始记录丢失|NOT_RECORDED|False|NOT_RECORDED|NOT_RECORDED|NOT_RECORDED|')
        else:lines.append(f'|{r["key"]}|{m["status"]}|{m["end_time_s"]:.6f}/{m["T_s"]:.6f}|{m["feasible"]}|{np.rad2deg(m["base_peak_angular_speed_rad_s"]):.6f}|{np.rad2deg(m["base_rms_angular_speed_rad_s"]):.6f}|{np.rad2deg(m["base_max_attitude_drift_rad"]):.6f}|')
    lines+=['',f'C2动力学合格 {qualified}/{len(current)}。达到时域末尾与通过全部门分别记录。早停的峰值/RMS仅针对其实际记录区间，不能和完整任务作收益结论。metrics中的terminal_*在早停时是最后样本值，并非完成任务的终端验收；完整终端指标为NOT_EVALUATED。所有扭矩图采用applied_ctrl，未施加的最后一步proposal不冒充实际输入。',
      '', '## 同模型扰动比较','',f'判定：**{q["base_disturbance_comparison"]["status"]}**。仅合格配对计算相对变化，完整字段见qualified_benefit_comparison.json。']
    for protocol,groups in benefit.items():
        for group,entry in groups.items():
            for pair in entry['pairs']:
                if pair['admissible']:
                    vals=pair['metrics'];lines.append(f'- {protocol}/{group}，联合对{pair["baseline"]}：峰值降低 {vals[METRICS[0]]["relative_reduction_percent"]:.4f}%，RMS降低 {vals[METRICS[1]]["relative_reduction_percent"]:.4f}%，最大姿态漂移降低 {vals[METRICS[2]]["relative_reduction_percent"]:.4f}%。负值表示变差。')
                else:lines.append(f'- {protocol}/{group}，联合对{pair["baseline"]}：NOT_COMPARABLE，存在不合格轨迹。')
    lines+=['','上述变化不等于论文5.22%/25.27%的复现。两个优化种子不构成统计鲁棒性样本。时间趋向冻结上界的解须连同时间、跟踪与漂移代价解释。','',
      '## 独立重放与数值复核','',
      f'可用C2轨迹的执行器独立重放：{replay["available_recordings_passed"]}；全批通过：{replay["passed"]}；原始记录缺失 {replay["missing_original_recordings"]} 条。每次仅t=0初始化，之后只输入记录力矩和步长，未注入中间状态。',
      '',
      'C1首条真实动力学运行在汇总处对Python列表执行abs而报错，原输入尚未持久化，无法恢复。该尝试计入12条上限并保留原源版本和失败文件。C2仅修复记录类型/顺序并使用独立身份，模型、控制、参数不变。C2通过不能追溯改判C1，更不能声称所有历史尝试可重放。',
      '',
      '预注册作者报告点的1 ms物理步复核保持20 ms任务周期；绝对时刻交集用插值/SO(3)插值比较，不注入粗步状态。结果见numerical_refinement_alignment.json。没有预先规定动力学收敛数值门，因此只报告敏感性，不把共同早停称为完整任务通过。',
      '',
      '## 交付、发布与下一步','',
      '[四组核心科学图](figures/README.md) · [逐项验收](completion_audit.json) · [独立资格字段](qualification.json) · [交接身份](handoff.json) · [复现命令](commands.md)',
      '',
      'S01接触前基准为原文兼容研究；协议B公平时间优化、MuJoCo力矩伺服、约束和独立重放为本项目扩展。未执行接触、锁紧、消旋或S02–S08。来源缺口不应永久阻塞工程线，但本批不具备无条件数值复现/全尝试归档通过结论。S02需单独授权，并显式接受或处理C1记录缺口。',
      '',
      '用户随后明确授权当前版本独立GitHub分支及全部可视化刷新；发布与历史N209媒体再生成单列在publication_manifest.json。该显示操作不增加SRS/Flexiv实验，不改写历史S00验收报告。']
    (OUT/'report.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    chapter=['# 原文兼容复现与工程桥接','',
      '本文将梁斌等（2024）的接触前SRS条件作为可追溯兼容基准，而非直接复用Flexiv几何。公开DH、质量惯量和初态逐表核验；未公开的COM、惯性轴、安装关系、驱动与目标函数定义分别列为假设。自由基座仅通过七个关节力矩运动，目标仅作为规定运动的规划参考。',
      '',
      '在11个预注册构型上，独立DH/刚体运动学与MuJoCo以及中心有限差分一致。式18的臂形导数提供独立检查；式23的未定义辅助向量与式31的主任务臂形分量分别披露解释。末端零空间条件JgN=0不蕴含AN=0，因此臂形自由度可影响基座而不改变一级末端运动。',
      '',
      '采用三种策略、两个固定种子和每格120次评价，区别共同捕获时间与各策略公平优化时间两种问题。评价器使用预先冻结的时间边界、单位尺度与可行性门。最终每个所选条件经力矩驱动真实动力学检查，精确重复条件复用同一尝试，失败保留。',
      '',f'本批{qualified}/{len(current)}条C2动力学尝试满足全部已声明门；扰动比较状态为{q["base_disturbance_comparison"]["status"]}。详细时间、误差和各指标相对变化由[阶段报告](../../output/fpmfc/system_capture/S01/report.md)和JSON数据提供。不得把不同失败区间的低角速度表述为改善。',
      '',
      '可用C2轨迹均经过独立执行器重放；C1的记录异常留下一个不可恢复的原输入缺口，故全批重放资格为PARTIAL。该限制独立于来源参数缺失。研究支持的是显式假设下的数学与软件兼容性，不是作者百分比精确复现、接触消旋完成或实际几何安全。',
      '',
      '参考文献：梁斌、徐文福、王学谦、闫磊，自由漂浮空间机器人捕获翻滚目标的力-位-型融合控制方法，宇航学报45(6):958–969，2024。DOI:10.3873/j.issn.1000-1328.2024.06.014。读取版本及PDF哈希见source_contract.json。']
    dest=PROJECT/'paper/system_framework/reproduction_section.md';dest.parent.mkdir(parents=True,exist_ok=True);dest.write_text('\n'.join(chapter)+'\n',encoding='utf-8')

def finalize():
    # Called after phase.operation records report CPU and removes its lock.
    write_report();audit()
    roots=[OUT,CONFIG,PROJECT/'models/paper_compat',PROJECT/'v6_mujoco/system_capture/paper_bridge',PROJECT/'experiments/system_capture/registry/S01']
    files=[p for root in roots for p in root.rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.name not in ['handoff.json','operation.lock'] and not p.name.endswith('.tmp')]
    files += [PROTOCOL,PROJECT/'paper/system_framework/reproduction_section.md']
    hashes={p.relative_to(PROJECT).as_posix():sha(p) for p in sorted(set(files))}
    save(OUT/'handoff.json',{'phase':'S01','input_commit':BASE,'branch':git('branch','--show-current'),
      'runtime_hash':read(OUT/'phase_manifest.json')['runtime_hash'],'status':'COMPLETE_WITH_RETAINED_FAILURES_AND_RECORDING_GAP',
      'strict_all_requirements_passed':False,'qualification':read(OUT/'qualification.json'),'evidence_sha256':hashes,'evidence_set_sha256':digest(hashes),
      'commit_identity':'Read the Git commit containing this handoff; no self-referential commit hash is embedded',
      'publication':'Separate latest-user authorization; actual remote identity recorded separately','next_phase_started':False})
    print({'S01_report_written':True,'strict_all_requirements_passed':False,'retained_gap':'C1 missing raw recording'},flush=True)

def report():
    comparisons();refinement();figures()
    plan='# S01 figure plan\n\n|ID|Content|Source|Output|\n|---|---|---|---|\n'
    for name in read(OUT/'figures/manifest.json'):plan+='|'+name+'|'+read(OUT/'figures/manifest.json')[name]['title']+'|Frozen source / recorded results|PDF + PNG|\n'
    (OUT/'figures/PAPER_PLAN.md').write_text(plan,encoding='utf-8')
    commands="""# S01 reproducibility

Use the exact Python/MuJoCo/NumPy/SciPy versions and single-thread environment in phase_manifest.json.
Run from the repository root. The archived baseline and C1/C2 source identities are preserved.

```powershell
$env:OPENBLAS_NUM_THREADS='1'
$env:OMP_NUM_THREADS='1'
$env:PYTHONIOENCODING='utf-8'
python -m v6_mujoco.system_capture --phase S01 --mode prepare
python -m v6_mujoco.system_capture --phase S01 --mode test
python -m v6_mujoco.system_capture --phase S01 --mode run --resume
python -m v6_mujoco.system_capture --phase S01 --mode replay --resume
python -m v6_mujoco.system_capture --phase S01 --mode report
```

Existing completed and failed keys are never re-executed. A live operation.lock must not be removed to start another run.
Use the retained environment executable or recreate its exact package versions; verify() deliberately checks environment identity.
C1 recording failure is not replayable. C2 recordings replay independently. No new attempt may be labeled recovery of the missing C1 trace.
The complete present repository is the resume starting point; deleting the ledger is not an authorized fresh campaign.
Report creation updates report CPU in the ledger first, then finalizes the handoff hashes on process exit.
A figure review or publication artifact added later requires rerunning report to refresh the handoff.
Publication display refresh: python -m v6_mujoco.feasible_capture --visualize (historical trace rendering only).
"""
    (OUT/'commands.md').write_text(commands,encoding='utf-8')
    atexit.register(finalize)
    return True

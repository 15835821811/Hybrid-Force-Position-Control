"""Build evidence-linked task tables and the stage report, including failures."""
import csv
import json
import subprocess
import sys
import time
from .common import ROOT,OLD,PAPER,PROJECT_ROOT,read,save,sha,ledger,identity,BASE
from .campaign import admission,freeze
from .statistics import run as summarize

def write(path,text):
    path.parent.mkdir(parents=True,exist_ok=True);path.write_text(text,encoding='utf-8')

def fmt(x):return 'NOT_EVALUATED' if x is None else f'{x:.6g}' if isinstance(x,float) else str(x)

def table(path,headers,rows):
    text='| '+' | '.join(headers)+' |\n|'+ '|'.join(['---']*len(headers))+'|\n'
    text+=''.join('| '+' | '.join(fmt(x) for x in row)+' |\n' for row in rows)
    write(path,text)
    with path.with_suffix('.csv').open('w',encoding='utf-8',newline='') as f:
        w=csv.writer(f);w.writerow(headers);w.writerows(rows)
    # Editable table fragment, no standalone LaTeX compiler dependency.
    esc=lambda x:fmt(x).replace('_',r'\_').replace('%',r'\%')
    tex='\\begin{tabular}{'+'l'*len(headers)+'}\n\\hline\n'+' & '.join(map(esc,headers))+r' \\'+'\n\\hline\n'
    tex+=''.join(' & '.join(map(esc,r))+r' \\'+'\n' for r in rows)+'\\hline\n\\end{tabular}\n'
    write(path.with_suffix('.tex'),tex)
    return text

def run():
    stats=summarize();a=admission();freeze();book=ledger();runs=[x for x in stats['runs'] if 'continuous_task_completed' in x]
    all_finished=len(runs)==len(book['attempts']) and all(x['status']!='NOT_EVALUATED' for x in a['regressions'].values())
    replays=all(x.get('validation',{}).get('actuator_replay_passed') and x.get('validation',{}).get('decision_replay_passed') for x in runs)
    testfiles=sorted(ROOT.glob('tests_*.json'));tests=read(testfiles[-1]) if testfiles else {};tests_pass=bool(tests) and all(x['passed'] for x in tests.values())
    status='PARTIAL_OPERATING_DOMAIN_MISMATCH' if all_finished else 'IN_PROGRESS'
    qual={'status':status,'engineering_delivery_complete':bool(all_finished and replays and tests_pass),
          'performance_promotion':False,'C1_improved_robust_capture':'NOT_SUPPORTED','C2_continuous_task':'SUPPORTED_ONLY_IN_RECORDED_IDEAL_SEEN_RUNS',
          'C3_identification_control_benefit':'CONTROL_BENEFIT_NOT_DEMONSTRATED','admission':a,'dual_replays_all_pass':replays,
          'tests_pass':tests_pass,'tests_file':testfiles[-1].name if testfiles else None,'new_holdouts':'NOT_EVALUATED_NOT_ADMITTED',
          'ablation':'NOT_EVALUATED_NOT_ADMITTED','fine_step':'NOT_EVALUATED_NOT_ADMITTED',
          'hardware_qualification':False,'recursive_safety_proof':False,'realtime_ready':False,
          'boundary':'Implemented trajectory/uncertainty/governor domain is insufficient; no proof all passive-base controllers or sensing methods are infeasible.'}
    save(ROOT/'qualification_matrix.json',qual)
    save(ROOT/'comparison.json',{'status':'NOT_EVALUATED_INDEPENDENT_PAIRED_COMPARISON','pairs':[],
         'historical_B0_context_only':{k:read(OLD/'runs'/k/'metrics.json') if (OLD/'runs'/k/'metrics.json').exists() else 'see archived result.json' for k in ['D06_final_gain3','H2_prior','S01_noise_delay']},
         'current_seen_attempts':[{'name':r['name'],'status':r['status']} for r in runs],
         'warning':'No frozen B0/B1 pairs or B2 mechanism ablations were run. Historical comparisons are context, not independent causal estimates.'})
    save(ROOT/'ablation_summary.json',{'status':'NOT_EVALUATED_NOT_ADMITTED','prespecified_clusters':[3,6],'seeds':[209201,209202,209203],'attempts':0})
    rows=[[r['name'],r['category'],r['status'],r['end_time_s'],r['latch_time_s'],r['world_window_max_deg_s'],r['max_load_fraction'],len(r['actual_safety_violations']),
           'PASS' if r.get('validation',{}).get('actuator_replay_passed') and r.get('validation',{}).get('decision_replay_passed') else 'PENDING_OR_FAIL'] for r in runs]
    task_table=table(PAPER/'generated_tables/task_outcomes.md',['Run','Use','Status','End (s)','Latch (s)','World last2s (deg/s)','Peak rho','Safety violations','Dual replay'],rows)
    diag=read(ROOT/'qp_failure_diagnosis.json');drows=[]
    for name,x in diag['runs'].items():
        d=read(ROOT/'diagnostics'/name/'diagnosis.json');w=x['box_witnesses'][0]
        drows.append([name,x['classification'],x['z'],w['row'],w['required_m_s'],w['maximum_over_joint_box_m_s'],w['shortfall_m_s']])
    diagnosis_table=table(PAPER/'generated_tables/phase_one.md',['Historical run','Classification','Dimensionless z','Conflict row','Required (m/s)','Box maximum (m/s)','Shortfall (m/s)'],drows)
    perf=table(PAPER/'generated_tables/compute_and_momentum.md',['Run','CPU (s)','p50 (s)','p99 (s)','Call max (s)','Predictor max (s)','P drift (kg m/s)','H drift (kg m2/s)'],
        [[r['name'],r['cpu_s'],r.get('control_latency_s',{}).get('p50'),r.get('control_latency_s',{}).get('p99'),r.get('control_latency_s',{}).get('max'),r['predictor_max_latency_s'],r['max_P_drift_kg_m_s'],r['max_H_drift_kg_m2_s']] for r in runs])
    sen=read(ROOT/'sensor_reference_audit.json');sensor_table=table(PAPER/'generated_tables/sensor_same_packets.md',['Component','Historical RMS','New RMS','Unit'],
        [[n,o,v,u] for n,o,v,u in zip(['px','py','pz','Rx','Ry','Rz','vx','vy','vz','wx','wy','wz'],sen['B0_rms_components'],sen['new_rms_components'],['m']*3+['rad']*3+['m/s']*3+['rad/s']*3)])
    newnoise=[r for r in runs if r['sensor']=='noisy'];gate_table=table(PAPER/'generated_tables/noisy_capture_gate.md',
        ['Run','Recorded gate ticks','Blocked by uncertainty alone','Minimum linear margin (m/s)','Minimum angular margin (rad/s)'],
        [[r['name'],r['capture_gate_samples'],r['capture_uncertainty_alone_blocks_samples'],r.get('minimum_capture_uncertainty_margin',[None]*4)[2],r.get('minimum_capture_uncertainty_margin',[None]*4)[3]] for r in newnoise])
    claims=[
        ['A1','Historical hard sets were empty, not merely a secondary Boolean failure','SUPPORTED_FROZEN_LINEAR_MODEL','output/fpmfc/n209_paper_system/qp_failure_diagnosis.json','runs.*.box_witnesses; z','failure tick and preceding0.5s','historical source manifest and exact packet torque reconstruction'],
        ['A2','Same-packet estimator velocity RMS decreases','SUPPORTED_OFFLINE_ONLY','output/fpmfc/n209_paper_system/sensor_reference_audit.json','B0_rms_components[6:12],new_rms_components[6:12]','absolute1–7s','estimator_adapter.py; historical packet and trace SHA in report'],
        ['C1','Improved safe capture across nonideal or multiaxis cases','NOT_SUPPORTED','output/fpmfc/n209_paper_system/qualification_matrix.json','admission.regressions','whole task','per-run config.identity and source_key'],
        ['C2','A recorded ideal nominal continuous task passes original gates','SUPPORTED_LIMITED','output/fpmfc/n209_paper_system/runs/D01_V1_nominal/metrics.json','continuous_task_completed,actual_safety_violations,world_window_max_deg_s','t0–27.992; post7.992–27.992; final25.992–27.992','config.identity; validation.validator_identity'],
        ['C3','Inertial feedback improves control performance','NOT_DEMONSTRATED','output/fpmfc/n209_paper_system/identifiability_supporting_audit.json','runs.*.pair and control_benefit','archived full H1/H3','blocks_sha and N208 archive'],
        ['T1','Reference transport and endpoint physical jets are consistent','PROVED_UNDER_ASSUMPTIONS_NUMERICALLY_CHECKED','paper/system_paper/theory_notes.md','A; selftest.reference_tests','isolated derivative fixtures','test artifact implementation manifest'],
        ['T2','Finite governor guarantees full-trajectory safety','OPEN_GAP','paper/system_paper/theory_notes.md','B2 assumptions and gaps','not established','no calibrated joint/model/intersample bound'],
        ['T3','Ideal internal damping has nonpositive power and nonzero H permits residual rotation','PROVED_UNDER_ASSUMPTIONS','paper/system_paper/theory_notes.md','C','ideal no-external-input continuous intervals','separate numerical energy/P/H evidence'],
        ['T4','Finite rank10 guarantees accurate COM and useful control','REJECTED','output/fpmfc/n209_paper_system/identifiability_supporting_audit.json','runs.H1_prior.historical_report','historical finite dataset','unregularized raw/white SVD and error histories'],
        ['R1','Every completed/early-stopped new attempt has reproducible physical and decision replay','SUPPORTED' if replays else 'PENDING','output/fpmfc/n209_paper_system/runs/*/validation.json','actuator_replay_passed,decision_replay_passed,max_intermediate_state_error','whole saved trajectory','per-run source_key and validator_identity']]
    with (PAPER/'claim_evidence_matrix.csv').open('w',encoding='utf-8',newline='') as f:
        w=csv.writer(f);w.writerow(['claim_id','claim','verdict','file','field','window','implementation_identity']);w.writerows(claims)
    elapsed=time.time()-book['started_epoch'];counts={k:sum(x['category']==k for x in book['attempts']) for k in book['limits']}
    resource={'measured_process_cpu_s':book['cpu_s'],'elapsed_campaign_wall_s':elapsed,'budget_cpu_s':28800,'budget_wall_s':28800,
              'attempt_counts':counts,'limits':book['limits'],'concurrency':'robot/replay serial under campaign.lock; figure review independent',
              'measurement_limit':'Early D01–D04 CPU snapshots precede artifact serialization. Their recorded finished-start wall times bound their total process CPU; document/plot/tool orchestration CPU not individually metered.',
              'early_run_wall_upper_bounds_s':{x['name']:x.get('finished_epoch',x['started_epoch'])-x['started_epoch'] for x in book['attempts'] if x['name'].startswith('D')},
              'budget_status':'WITHIN_RECORDED_BUDGET','unmetered_cpu_status':'DISCLOSED_NOT_ZERO'}
    save(ROOT/'resource_summary.json',resource)
    for script in sorted((ROOT/'figures').glob('gen_f*.py')):subprocess.run([sys.executable,str(script)],check=True)
    report=f'''# N209 可实现性与不确定性报告

本轮结论：**{status}**。工程交付完成状态见 qualification_matrix.json；该字段只表示限定工作闭环，不表示性能晋级。未通过的任务、盲测、消融及步长项保留失败或 NOT_EVALUATED。

基线 `{BASE}`，分支 `codex/n209-paper-ready-feasible-capture`。历史材料只读；独立工作树保存实现与证据。不推送、不合并，N210/N211仅给出计划。

## 1. 核心结论

H2/S01/D05 的历史最终约束集合均被独立 HiGHS Phase-I 与关节盒极值矛盾复核为空。历史控制包重建的力矩与归档一致；距离组和关节组单独可行、合并冲突。一级原本是最小二乘任务，不是末端速度硬等式。这里的 z 无量纲，不能解释为碰撞毫米数；失败时也不必已经发生实际碰撞。

{diagnosis_table}

新接口保留 SO(3) 滤波，将测量时间更新与当前时刻预测分开，保存0.25 s过程模式；乱序旧包和重复包拒绝，NIS仅新测量计算。独立参考状态按有限jerk更新；原捕获guard继续读原始估计和协方差。前瞻使用独立先验模型、0.20 s时域、实际2 ms伺服斜坡、20 ms任务tick和至多5个候选，不能读取真实未来。抓后结构未变，惯性估计为影子运行。

第一版名义通过，H2因启动软残差门在0.04 s停下。唯一第二版修订允许前0.20 s启动软残差过渡；硬件/几何/捕获门不变。第二版H2在5.90 s因预测任务残差拒绝全部候选，仍未完成捕获。没有第三版、未通过盲测选参。

## 2. 全部真实机器人尝试

{task_table}

以上全部为已见开发/回归。安全违规为零只表示已积分前缀上未触发评价门，不能把提前停止称为成功捕获或真实硬件安全恢复。失败的抓后末窗为NOT_EVALUATED。当前完成数{stats['complete_tasks']}/{stats['finished']}仅为工作记录计数，不估计独立成功概率。

## 3. 感知和可执行域

历史S01同包、同1–7 s窗的再估计支持速度误差下降；没有新植物，也没有B2闭环消融，不能独立归因于时序、过程噪声或整形中的某一模块。线速度每轴RMS由约2.08–2.17 mm/s降至0.218–0.228 mm/s，角速度每轴由约0.00878–0.00925 rad/s降至0.00122–0.00131 rad/s。

{gate_table}

原线/角捕获门为0.001 m/s和0.0034906585 rad/s。记录的噪声捕获窗口中，仅协方差速度裕量就能阻止批准；保留了该门。结论针对当前滤波/裕量/路径组合，**不证明所有传感器或被动基座方法都不可能完成任务**。H2的有限进度候选在既定带宽内没有足够预测控制余量；没有证明整条工作空间不可达。

## 4. 计算、物理与重放

{perf}

早期开发记录的c.timings漏掉抛异常的末次调用；上表Predictor max补充显示该次开销。最终回归已由runner测量每次调用，包括异常。完整2/20 ms超时比例、基座漂移/峰值/RMS、真实捕获四量、分阶段估计误差、功和能量见runs/*/analysis.json。每步P/H、实际力矩、接口载荷原量保留，未通过减去几何项制造通过。预测延迟显著超过20 ms，本实现不是实时控制器。

每条从t0一次初始化重放ctrl与锁紧事件；另一条从原SensorPacket重建估计、参考、guard、影子参数和力矩。验证身份逐运行保存，使用对应归档V1/V2源码。当前全部双重重放通过：{replays}。重放不是任务性能证据。

已记录进程CPU {book['cpu_s']:.3f} s；任务墙钟{elapsed:.1f} s，上限均28800 s。开发与最终回归计数{counts}。预测、独立诊断、测试和重放已计入账本。早期落盘/工具编排CPU未逐项计量，不能把账本数字称为精确总CPU；resource_summary.json披露范围与早期总墙钟上界。未启动后台补跑。

## 5. 辨识、理论与论文

历史H1/H3仅作支持性重分析，参数进入预测的时刻、白化前/后无正则SVD和误差时间序列已归档。H1数值秩10仍有约15.28 mm COM误差；prior/identified实际力矩和轨迹差为零，控制收益未被证明。C3不作为C1/C2前提。

theory_notes.md给出SE(3)输运/端点、冻结线性Phase-I、条件裕量、理想内部动量/耗能及孤立尺度歧义推导。全程递归可行性、模型误差界、联合概率校准和硬件恢复仍为OPEN_GAP。

paper/system_paper/manuscript_v1.md为规范草稿；八篇原始文献的版本与读取层级见related_work_matrix.csv。前三篇只读到摘要，未冒称公式复现；含推进器/反作用轮文献不作为本系统数值基线。图表来自JSON/NPZ，claim_evidence_matrix.csv逐项关联窗口和实现身份。

## 6. 停止与后续

选定版本未满足名义/H2/S01/H1全部通过的准入条件。冻结状态NOT_FROZEN，未生成新场景实际数值；36条配对、6条B2和2条细步长均NOT_EVALUATED。不是预算耗尽，也不是把已见重测包装成盲测。可执行域证据不足触发任务允许的阶段性停止。

N210只在接近/感知问题解决后考虑兼容执行器假设的抓后对照；N211再扩展独立连接实现、盲测、硬件误差预算和投稿规范。当前最优先缺口是更有依据的速度置信度与目标输运可执行域，而不是添加抓后复杂控制器。
'''
    write(PROJECT_ROOT/'paper/N209_FEASIBILITY_AND_UNCERTAINTY_REPORT.md',report)
    template=(PAPER/'manuscript_template.md').read_text(encoding='utf-8')
    manuscript=template.replace('{{TASK_TABLE}}',task_table).replace('{{PHASE_TABLE}}',diagnosis_table).replace('{{GATE_TABLE}}',gate_table)
    write(PAPER/'manuscript_v1.md',manuscript)
    save(ROOT/'report_identity.json',{'implementation':identity(),'inputs':{str(p.relative_to(PROJECT_ROOT)):sha(p) for p in [ROOT/'statistics.json',ROOT/'qualification_matrix.json',ROOT/'qp_failure_diagnosis.json',ROOT/'sensor_reference_audit.json',PAPER/'references.bib']},'generated_epoch':time.time()})
    print(json.dumps({'status':status,'delivery_complete':qual['engineering_delivery_complete'],'attempts':len(runs),'replays':replays}))

if __name__=='__main__':run()

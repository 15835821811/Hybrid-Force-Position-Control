"""Seal only completed, replay-verified evidence; never runs dynamics."""
import json
from pathlib import Path
import numpy as np
from v6_mujoco.model import PROJECT_ROOT
from v6_mujoco.postgrasp_campaign.io import read,save,identity,check_identity
from v6_mujoco.end_to_end_capture.common import arrays,digest
from .common import ROOT,verify_design,config

def run():
    manifest=verify_design();ledger=read(ROOT/'run_ledger.json');results={};replays={}
    for row in ledger['formal_runs']:
        scene=row['scenario'];r=read(ROOT/scene/'metrics.json');v=read(ROOT/scene/'validation.json');assert v['passed'] and r['trace_sha256']==v['trace_sha256']==digest(ROOT/scene/'trace.npz');check_identity(r['implementation_identity']);results[scene]=r;replays[scene]=v
    assert 0<len(results)<=4 and all(r['status']!='RUNNING' for r in ledger['formal_runs'])
    nominal=results['nominal'];a=arrays(ROOT/'nominal/trace.npz');screen=read(ROOT/'trajectory_screening.json');geom=read(ROOT/'geometry_audit.json');design=read(ROOT/'selected_design.json');unit=read(ROOT/'unit_tests.json');fixture=read(ROOT/'interface_tests/result.json');mass=read(ROOT/'changed_mass_inertia.json');cases=config()['scenario_order']
    matrix={'original_n205_end_to_end':'FAILED_GEOMETRY','distance_representation_validated':True,'original_terminal_geometry_feasible':False,
            'interface_geometry_modified':True,'hardware_design_assumption_changed':True,'terminal_geometry_qualified':True,'approach_reference_qualified':screen['selected']=='standard_C1',
            'continuous_18s_completed':{s:results[s]['continuous_18s_completed'] if s in results else False for s in cases},
            'final_detumbling_passed':{s:results[s]['performance']=='PASSED' if s in results else 'NOT_RUN_NOMINAL_FAILURE' for s in cases},
            'capture_gate_at_8s':{s:(results[s]['status']!='CAPTURE_GATE_FAILED' and bool(np.any(arrays(ROOT/s/'trace.npz')['eq_active']))) if s in results else 'NOT_RUN_NOMINAL_FAILURE' for s in cases},
            'terminal_approach_diagnostics':{s:read(ROOT/s/'terminal_approach_gates.json') for s in results if (ROOT/s/'terminal_approach_gates.json').exists()},
            'independent_cases':{s:{'status':results[s]['status'],'end_time_s':results[s]['end_time_s'],'replay_passed':replays[s]['passed']} if s in results else {'status':'NOT_RUN_NOMINAL_FAILURE','replay_passed':False} for s in cases},
            'real_gripper_and_hardware_validated':False,'new_controller_designed':False,'legacy_postgrasp_success_inherited':False,'new_restricted_fixture_passed':fixture['passed'],
            'classification':geom['classification'],'formal_attempt_count':len(results),'maximum_formal_attempts':4,
            'physical_system_accounting':'mocap visualization marker excluded from P/H/I_lock bookkeeping; all plant masses untouched',
            'distance_coverage':'24 original pairs + 19 original target pairs + 20 added spacer/other tool pairs in planning/HQP and execution',
            'scope':'supplied STL and stated rigid aluminium-equivalent tool assumption; no manufacturing geometry or hardware validation'}
    save(ROOT/'qualification_matrix.json',matrix)
    pad=read(ROOT/'terminal_mating_contract.json');end=nominal['end_time_s'];latched=bool(np.any(a['eq_active']));actualmin=float(a['extra_pair_distances_m'][:,16].min());tracking=float(np.max(a['approach_position_error_m'][a['time_s']<=8+1e-9]))
    table='\n'.join('| E_'+s+'_v2 | '+(f"{results[s]['end_time_s']:.3f} s | {results[s]['status']} | {results[s]['performance']} | 通过" if s in results else '未运行 | 名义失败后跳过 | NOT_EVALUATED | 未运行')+' |' for s in cases)
    terminal=', '.join(f"{x['label']}: {x['pairs'][-3]['signed_distance_m']*1000:.9f} mm" for x in geom['states'])
    final_summary=f"名义实际积分到 **{end:.3f} s**，状态 **{nominal['status']}**，原因 `{nominal['reason']}`。" if nominal['status']!='COMPLETED' else '名义连续积分至 **18 s**；最终消旋门状态为 **'+nominal['performance']+'**。'
    failure=read(ROOT/'capture_failure_analysis.json') if (ROOT/'capture_failure_analysis.json').exists() else None
    capture_table=('\n| t=8指标 | 实际值 | 原捕获门 |\n|---|---|---|\n'+ '\n'.join(f"| {name} | {failure['actual'][field]*scale:.9f} {unit_} | {failure['gates'][field]*scale:g} {unit_} |" for field,name,scale,unit_ in [('translation_m','接口平移',1000,'mm'),('rotation_deg','物理姿态',1,'deg'),('relative_linear_speed_m_s','相对线速度',1000,'mm/s'),('relative_angular_speed_deg_s','相对角速度',1,'deg/s')])) if failure else ''
    report=f'''# N206：终端几何兼容性修复与连续捕获—消旋复核

## 本轮结论

旧固定抓持关系在当前有效几何下不可行，分类为 **TERMINAL_MATING_GEOMETRY_INCOMPATIBLE**。本轮只采用路线 B：增加有实体、有质量的仿真工具组件，不修改原碰撞资产、机器人本体、目标、控制增益或阻尼。

{final_summary} 实际是否激活锁紧：**{latched}**；18 s 完成：**{nominal['continuous_18s_completed']}**；固定 [16,18] s：**{nominal['performance']}**。已使用 **{len(results)}/4** 次完整控制器动力学尝试。剩余额度不是改变模型/增益后自动重试的授权。

| 条件 | 实际结束时间 | 状态 | 固定最终窗 | 独立重放 |
|---|---|---|---|---|
{table}

## G0：来源、距离和独立相交证据

从完整提交 `f2c4cf5cc12dd299589dc4e0a3f32d52ed085403` 新建独立工作树和分支 `codex/n206-terminal-geometry-feasibility`。原工作树未跟踪资料、历史分支均未覆盖。LFS 的两个源 trace 从本地对象库展开后，通过 N205 全部 source/design 身份校验。G0 没有任何积分步。

四个状态分别保存，数值为：{terminal}。N205 7.362 s 仍为正距离，不能称已碰撞；历史 C1 的 8 s 状态从未注入 N206 正式植物。

link7 的原始碰撞 STL 有 46 个顶点、88 个三角面，边两两闭合且定向一致。历史/理想终端各有 8 个严格进入 box 的顶点和 29 个三角面—box 裁剪见证；同时独立凸多面体 SAT 的符号与 MuJoCo 一致。三角面裁剪覆盖边/面穿越，另有闭合网格实体包含检查和“无内部顶点但面穿越”的回归测试。因此 **−0.293 mm 已存在于所供 STL 表面几何，不能归因于仅凸包填充空洞**。所供 STL 本身是粗碰撞资产；缺少制造 CAD，不能宣称真实硬件实体的精度或认证。

全部八个原始 STL 用 body 变换进入世界坐标，与 compiled mesh 顶点经 geom 变换进入世界坐标比较；最大双向最近点误差 {geom['tests']['mesh_world_max_error_m']:.3e} m。mesh_pos/mesh_quat 的编译重定位已包含在 geom pose 中，不再重复应用。显示几何和碰撞 STL 同源，碰撞使用其凸包；不更换为更有利的检测器。

使用安装的 MuJoCo 3.3.2：nativeccd 开启、query distmax=2 m、CCD tolerance=1e-6、CCD iterations=50、动力学 solver tolerance=1e-10、iterations=50。每个 geom 的原 margin 在 geometry_audit.json 中逐项保存，未修改。legacy 只作对照；其 N205 7.362 s 距离约 0.959300 mm，未用于验收。共同刚体变换距离误差 {geom['tests']['rigid_transform_distance_error_m']:.3e} m；100 µm 量级退离扫描的距离斜率 {geom['tests']['retreat_distance_slope']:.12f}。

安装版本对 mesh–box 的 raw fromto 端点次序与 primitive–box 不同。审计保留原数组，再用 box 表面归属标注端点；正负距离本身没有改动。`distance_drift_raw_endpoint_regression.json` 保存直接套用端点顺序导致导数反号的失败诊断。在线新增约束直接使用隔离副本中心有限差分，并用正确端点的点速度 Jacobian 独立核对目标漂移。

## G1/G2：固定链证明与唯一修改路线

固定变换完整保存于 terminal_mating_contract.json：

```
T_target_link7 = T_target_grasp @ inv(T_flange_tool) @ inv(T_link7_flange)
T_world_flange(T) = T_world_grasp(T) @ inv(T_flange_tool)
```

flange 是 link7 的无关节刚性子体，site-based weld 固定全部六维相对位姿。没有机构证据允许自由绕法向旋转或滑移。因此上游臂形、共同世界姿态和抓持时间不能改变相同终端关系下的刚性交叠。本轮不做 psi/T 搜索。

预先冻结安装增量 [2,5,10,20] mm，工具沿法兰 +Z 前移，对齐后机器人沿目标面外方向退离。保守假设安装座面在法兰 z=0，不虚构可容纳工具垫的凹槽。前三个候选的工具垫后表面仍在座面以内，无法形成正长度外置支撑体，因而拒绝；20 mm 是此声明构造族内的最小合格候选，不是全局最小硬件设计。

选中实体为半径35 mm、长9.8 mm的支撑柱，与原半径35 mm、厚10 mm工具垫连续连接。新的工具面位于法兰 z=19.8 mm；工具接口与工具垫一同平移20 mm，保留原局部小偏置和姿态，不拟合历史末态。采用均质铝等效密度2700 kg/m³，新增质量 **{design['parameters']['mass_kg']:.12f} kg**；质心和正定圆柱惯量显式写入固定工具 body。旧每个 body 的质量、惯量及惯性 frame 逐体不变，nq/nv/nu=21/19/7。新增结构以整圆刚性座面连接法兰；没有螺栓/孔位/结构强度认证。**这是新工具安装仿真假设，真实硬件尚未批准。**

link7—cube 理想距离约19.707063 mm；12个有限姿态/平移扰动样本的所有非预期目标对最小距离 {unit['findings']['terminal_perturbations']['minimum_target_clearance_m']*1000:.6f} mm。扰动按原捕获容差和接口允许形变设置，不宣称覆盖连续全部组合姿态。

## 几何一致参考、全时域规划与约束

初态取新模型真实 home。T=8 s 和原终端臂形固定。终端 p/R/v/ω/a 来自同一固定变换链，线速度含 ω×r，线加速度含 α×r 和 ω×(ω×r)；新工具偏置不沿用旧速度。边界值、有限差分与 C2 末端连续性已通过定向测试。

三条预定义参考均进行0–8 s运动学评估，标准 C1 按预定顺序选用；其规划终端法兰误差0.093677 mm。另两条是6–8 s沿运动目标外法向的 C2 离面 bump，不改变终端关系。trajectory_screening 的 qualified 表示原C1末端门/几何/HQP筛查；补充的 candidate_terminal_capture_audit 单列更严的锁紧门：标准 C1通过，40/80 mm模板不通过。该补查没有产生新候选、未改变选择或读取抓后表现。

原24对保留45 mm规划/40 mm执行口径；新增19目标对和20个新工具相关对进入规划与在线HQP。非预期目标对采用2 mm规划裕量、1 mm执行门及原10 µm容差；工具与其他机械臂部件采用45/40 mm。工具垫—cube保留原有意接触面−2 mm门；pad—plate有原物理响应。link7、支撑体与目标没有豁免，也未冒称距离监测对具有物理碰撞响应。刚性支撑体—link7的安装连接及支撑体—pad内部面是逐项声明的结构连接，未扩大目标接触范围。

距离约束形式为 grad(d)·qdot >= −k(d−d_min)−d_drift，目标六维速度参与 d_drift，保留转动点速度。新增梯度和漂移均由同一步状态的独立副本计算，主数据不被写回。硬QP残差、关节边界冲突单列，无放宽约束。

全域2 ms采样；不确定的工具垫步内界对相同已存路径自适应细分至标准C1的0.5 ms、其余0.25 ms。原粗上界未通过的文件保留为 *_coarse_bound.json，后续仅加密，没有改路径或生成第四候选。机器人3 m点运动力臂界大于模型所需约1.724119 m，目标转动半径界0.27 m。此界只覆盖所定义的分段广义速度插值，不覆盖真实跟踪误差或任意扰动，真实积分另行逐步验收。

## 变更影响与局部测试

新质心和全惯量张量改变所有方向的局部相对运动响应，所以只重做受限夹具的16方向与原3个fine对照：新夹具为真实 link7+新增工具的刚性复合惯量、原20 kg目标和移位后的接口点，排除上游机械臂。19项通过，最大实际ρ={max(x['max_interface_load_fraction'] for x in fixture['results']):.9f}。保持原候选2 solref/solimp/torquescale、50 N/2 Nm实际包络；输入仍是原0.8受限曲线。旧满边界资格依旧FAILED，本次通过只适用于这组受限局部输入。

定向测试覆盖三角面相交、变换不变性、固定链、实体支撑/质量/七驱动、接口与工具面关系、终端导数、目标漂移、实际t=8坏状态拒绝锁紧、主状态隔离、物理wrench及功率、P/H/I_lock虚拟整体刚体运动与质量矩阵能量一致性。两个完整成功的短测试批次各9物理步和9重放步，最长单条6 ms；另一个初始化派生状态问题在3物理步后的单元重放构造阶段发现并修复，未进入完整动力学。静态检查中的失败及修正记入 tests.log；没有隐藏完整动力学调试。

新增物体后重新计算全系统P0/H0/I_lock、实际接口与关节载荷。单元测试发现旧动量记账含一个7.238 g的 prescribed mocap可视标记；新观察器明确计算自由机器人+自由目标，排除该标记，植物本身的质量数组、动力学与原控制器不改变。此为记账修正，不是 −0.293 mm 的来源，不追溯改写N205。t=0 P0={mass['home_system']['linear_momentum_world_kg_m_s']}，H0={mass['home_system']['angular_momentum_about_center_world_kg_m2_s']}；完整锁定惯量矩阵保存于 changed_mass_inertia.json。

## 实际连续动力学与重放

正式植物从本次t=0初始化一次，目标从0自由运动，七关节原控制、20 ms HQP和2 ms伺服保持。t=8捕获门失败即拒绝锁紧；锁紧仅改eq_active、声明的pad接触掩码及原阻尼控制，不重写qpos/qvel、site、时钟或warmstart。没有1 s恒力阶段，没有重新计算阻尼。

名义真实时域0–{end:.3f} s；link7—cube实际最小间距 **{actualmin*1000:.6f} mm**。最大线动量漂移 {nominal['max_P_drift']:.3e} kg·m/s，角动量漂移 {nominal['max_H_drift']:.3e} kg·m²/s，最大ρ **{nominal['max_rho']:.9f}**。接口/关节/碰撞安全门全程按冻结配置检查。最终评价窗不会因失败移动。

{capture_table}

本轮7.718 s首次检测到预定pad接触，峰值约0.284131 N；接触前目标位置/姿态与自由预测一致（最大姿态偏差约5.31e-14 rad），t=8时目标中心偏离解析预测约0.200796 mm、姿态约0.06399 deg。几何距离已通过，但自由目标接触后的实际状态与冻结运动匹配参考分离，四个实际捕获量均越门。这里记录时间先后与观测结果，不以单条轨迹作排他因果证明。没有为“过门”等待、延迟锁紧、再规划或新增控制器。

每条已执行轨迹的重放从本次t=0 integration state开始，仅使用记录ctrl和显式锁紧事件驱动mj_step；无控制器重算、未来状态修正或旧终态。视频渲染才读取逐帧记录用于显示，不能混同记录输入独立重放。正式运行计算包含全部目标对有限差分和安全查询，不构成硬实时性能证据。

## 七项问题回答与结论边界

1. **0.293 mm来源**：所供原始碰撞STL的表面相交，独立三角面和SAT验证；不是编译坐标/单位错误，也不是仅凭“凸包可能保守”宣称误报。真实制造CAD尚不可知。
2. **固定终端是否可行**：旧关系不可行；同一固定关系下换臂形、共同世界姿态不能消除。新声明工具关系静态可行。
3. **改了什么**：路线B的实体工具安装假设；没有改碰撞表示或控制增益。参考由新安装链重生。另有显式端点归属与物理系统记账修正，不改变标量距离或植物。
4. **质量、力臂、资格**：新增约0.205738 kg，接口沿法兰轴前移20 mm；复合惯量、P0/H0/I_lock、受限接口测试和实际关节载荷重新检查，不能继承旧抓后性能。
5. **距离是否进入规划**：是；原24对、新目标19对、新工具20对均进入全域运动学筛查/HQP，执行阶段继续监测，目标漂移未省略。
6. **是否真实到8/18 s**：实际结束{end:.3f} s，锁紧={latched}，连续18 s={nominal['continuous_18s_completed']}，最终消旋={nominal['performance']}；静态/规划图不冒充动力学。
7. **旧成功不能继承**：历史分段C1→锁紧→消旋、旧fine/light/heavy和旧局部夹具结果不能作为新工具与新增几何覆盖的端到端成功；硬件、真实夹爪、感知和硬实时均未验证。

## 可视化与复现

当前入口：output/fpmfc/n206_geometry_capture/visualizations/index.html。两张核心几何图、尺寸图、全域规划/分类距离、真实末端轨迹误差、角速度/载荷、接口保持、动量功率、七关节/力矩和实际安全距离均刷新。总览视频合并五个固定视角与连续基座体侧视角，另一个接口近景，共2个视频；只覆盖本轮真实且已重放时域。历史目录保持原身份，README明确历史边界。

技术语义核对：[MuJoCo 3.3.2 XML Reference](https://mujoco.readthedocs.io/en/3.3.2/XMLreference.html#asset-mesh)、[site-based weld](https://mujoco.readthedocs.io/en/3.3.2/XMLreference.html#equality-weld)、[mj_geomDistance](https://mujoco.readthedocs.io/en/3.3.2/APIreference/APIfunctions.html#mj-geomdistance)、[Computation](https://mujoco.readthedocs.io/en/3.3.2/computation/)。文档解释模型/API语义；所有试验数值来自本分支实际文件，候选尺寸与材料均为本任务新假设。

遵循附件最后补充要求上传独立GitHub分支，不合并、不移动历史分支。复现命令和完整尝试范围见 commands.md、run_ledger.json、tests.log。
'''
    (PROJECT_ROOT/'paper/N206_GEOMETRY_COMPATIBILITY_REPORT.md').write_text(report,encoding='utf-8',newline='\n')
    captions=read(ROOT/'figures/captions.json');videos=read(ROOT/'visualizations/manifest.json');n=videos['frames'];cards=''.join(f'<section><h2>{key}</h2><video controls preload="none" poster="{key}_{n-1:03d}.png" src="{key}.mp4"></video></section>' for key in videos['videos'])
    figures=''.join(f'<figure><img loading="lazy" src="../figures/{name}.png"><figcaption>{caption}</figcaption><a href="../figures/{name}.pdf">PDF</a></figure>' for name,caption in captions.items())
    html=f'''<!doctype html><html lang="zh"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>N206 几何兼容性与连续复核</title><style>body{{font:17px system-ui;background:#edf3f5;color:#163246;max-width:1400px;margin:25px auto;padding:20px}}section,figure{{background:white;padding:20px;border-radius:10px;margin:24px 0}}img,video{{width:100%}}p,figcaption{{line-height:1.65}}a{{color:#0072b2}}</style><h1>N206：几何兼容性与连续复核</h1><p>原固定终端不兼容；新增20 mm安装增量的有质量工具候选属于仿真设计假设，未经硬件批准。名义实际到{end:.3f} s；状态 {nominal['status']}；最终窗口 {nominal['performance']}。本页不拼接历史抓后成功。</p><p><a href="../../../../paper/N206_GEOMETRY_COMPATIBILITY_REPORT.md">完整报告</a> · <a href="../qualification_matrix.json">资格矩阵</a> · <a href="../run_ledger.json">运行账本</a> · <a href="../geometry_audit.json">几何原始证据</a></p><p>总览含五视角及连续随基座体侧视角；红线为实际法兰轨迹，蓝线为本轮参考。视频来自本轮通过独立重放的记录，仅用于显示。</p>{cards}{figures}</html>'''
    (ROOT/'visualizations/index.html').write_text(html,encoding='utf-8',newline='\n')
    files=[p for p in ROOT.rglob('*') if p.is_file() and p.name not in ['delivery_manifest.json','completion_audit.json']]
    save(ROOT/'delivery_manifest.json',{'files':identity(files),'report_identity':identity([PROJECT_ROOT/'paper/N206_GEOMETRY_COMPATIBILITY_REPORT.md']),'formal_attempts':len(results),'videos':2})
    print({'outcome':nominal['status'],'end_time_s':end,'formal_attempts':len(results)})

if __name__=='__main__':run()

# N209 可实现性与不确定性报告

本轮结论：**PARTIAL_OPERATING_DOMAIN_MISMATCH**。工程交付完成状态见 qualification_matrix.json；该字段只表示限定工作闭环，不表示性能晋级。未通过的任务、盲测、消融及步长项保留失败或 NOT_EVALUATED。

基线 `b1af09f89226fa6f2ae1b362f3d6225b3363cc17`，分支 `codex/n209-paper-ready-feasible-capture`。历史材料只读；独立工作树保存实现与证据。不推送、不合并，N210/N211仅给出计划。

## 1. 核心结论

H2/S01/D05 的历史最终约束集合均被独立 HiGHS Phase-I 与关节盒极值矛盾复核为空。历史控制包重建的力矩与归档一致；距离组和关节组单独可行、合并冲突。一级原本是最小二乘任务，不是末端速度硬等式。这里的 z 无量纲，不能解释为碰撞毫米数；失败时也不必已经发生实际碰撞。

| Historical run | Classification | Dimensionless z | Conflict row | Required (m/s) | Box maximum (m/s) | Shortfall (m/s) |
|---|---|---|---|---|---|---|
| H2_prior | LINEAR_HARD_SET_INFEASIBLE | 0.0195548 | gripper_contact_pad__tumbling_target_geom | -0.127798 | -0.161947 | 0.0341494 |
| S01_noise_delay | LINEAR_HARD_SET_INFEASIBLE | 0.00550255 | gripper_contact_pad__tumbling_target_geom | -0.0372463 | -0.0467254 | 0.00947903 |
| D05_final_nominal | LINEAR_HARD_SET_INFEASIBLE | 0.000983 | gripper_contact_pad__tumbling_target_geom | -0.027238 | -0.0288792 | 0.00164122 |


新接口保留 SO(3) 滤波，将测量时间更新与当前时刻预测分开，保存0.25 s过程模式；乱序旧包和重复包拒绝，NIS仅新测量计算。独立参考状态按有限jerk更新；原捕获guard继续读原始估计和协方差。前瞻使用独立先验模型、0.20 s时域、实际2 ms伺服斜坡、20 ms任务tick和至多5个候选，不能读取真实未来。抓后结构未变，惯性估计为影子运行。

第一版名义通过，H2因启动软残差门在0.04 s停下。唯一第二版修订允许前0.20 s启动软残差过渡；硬件/几何/捕获门不变。第二版H2在5.90 s因预测任务残差拒绝全部候选，仍未完成捕获。没有第三版、未通过盲测选参。

## 2. 全部真实机器人尝试

| Run | Use | Status | End (s) | Latch (s) | World last2s (deg/s) | Peak rho | Safety violations | Dual replay |
|---|---|---|---|---|---|---|---|---|
| D01_V1_nominal | development | COMPLETED | 27.992 | 7.992 | 0.0235542 | 0.363223 | 0 | PASS |
| D02_V1_H2 | development | NO_VERIFIED_CONTROL | 0.04 | NOT_EVALUATED | NOT_EVALUATED | 0 | 0 | PASS |
| D03_V1_S01 | development | NO_VERIFIED_CONTROL | 7.84 | NOT_EVALUATED | NOT_EVALUATED | 0.0103905 | 0 | PASS_TS_AUDIT |
| D04_V2_H2 | development | NO_VERIFIED_CONTROL | 5.9 | NOT_EVALUATED | NOT_EVALUATED | 0 | 0 | PASS |
| R01_V2_nominal | regression | COMPLETED | 27.992 | 7.992 | 0.0235542 | 0.363223 | 0 | PASS |
| R02_V2_H2 | regression | NO_VERIFIED_CONTROL | 5.9 | NOT_EVALUATED | NOT_EVALUATED | 0 | 0 | PASS |
| R03_V2_S01 | regression | NO_VERIFIED_CONTROL | 7.84 | NOT_EVALUATED | NOT_EVALUATED | 0.0103905 | 0 | PASS_TS_AUDIT |
| R04_V2_H1 | regression | COMPLETED | 27.932 | 7.932 | 0.0128079 | 0.597206 | 0 | PASS |


以上全部为已见开发/回归。安全违规为零只表示已积分前缀上未触发评价门，不能把提前停止称为成功捕获或真实硬件安全恢复。失败的抓后末窗为NOT_EVALUATED。当前完成数3/8仅为工作记录计数，不估计独立成功概率。

## 3. 感知和可执行域

历史S01同包、同1–7 s窗的再估计支持速度误差下降；没有新植物，也没有B2闭环消融，不能独立归因于时序、过程噪声或整形中的某一模块。线速度每轴RMS由约2.08–2.17 mm/s降至0.218–0.228 mm/s，角速度每轴由约0.00878–0.00925 rad/s降至0.00122–0.00131 rad/s。

| Run | Actual gate ticks | Audited1–7s samples | Uncertainty precludes gate | Minimum linear3sigma lower bound (m/s) | Minimum angular3sigma (rad/s) |
|---|---|---|---|---|---|
| D03_V1_S01 | 0 | 3001 | 3001 | 0.0016097 | 0.00964336 |
| R03_V2_S01 | 0 | 3001 | 3001 | 0.0016097 | 0.00964336 |


原线/角捕获门为0.001 m/s和0.0034906585 rad/s。噪声运行在进入捕获窗口前已因预测残差停止，实际guard调用数为0，不能称为锁紧申请被拒。上表是对已记录1–7 s原始协方差的必要条件反事实审计：仅速度裕量下界就会阻止捕获批准。结论针对当前滤波/裕量/路径组合，**不证明所有传感器或被动基座方法都不可能完成任务**。H2的有限进度候选在既定带宽内没有足够预测控制余量；没有证明整条工作空间不可达。

进一步的条件性分析见uncertainty_floor.json与theory_notes.md E：在当前平移常速度协方差模型、6 ms采样、至少12 ms延迟及不小于0.003 m/s^(3/2)过程噪声密度下，Riccati固定点与协方差序关系给出有效估计的线速度3sigma裕量下限1.609697 mm/s，超过未改动的1 mm/s门。独立固定点求解与10000次迭代一致。这是**当前滤波器计算裕量的下界**，不是实际误差下界，也不是传感器精度的基本极限；接触过程噪声只会增大本模型的平移协方差。该限制说明当前配置无法批准捕获，与实际轨迹更早发生的预测残差停机分别报告。

## 4. 计算、物理与重放

| Run | CPU (s) | p50 (s) | p99 (s) | Call max (s) | Predictor max (s) | P drift (kg m/s) | H drift (kg m2/s) |
|---|---|---|---|---|---|---|---|
| D01_V1_nominal | 279.078 | 0.0010205 | 0.885892 | 0.954045 | 0.877906 | 2.63924e-08 | 4.32438e-07 |
| D02_V1_H2 | 0.234375 | 0.0009455 | 0.00124829 | 0.001275 | 0.0634562 | 3.36055e-37 | 5.05352e-15 |
| D03_V1_S01 | 208.688 | 0.0012854 | 0.898961 | 1.03027 | 4.42006 | 8.11414e-10 | 5.85661e-11 |
| D04_V2_H2 | 122.828 | 0.0010532 | 0.954086 | 1.06788 | 4.50248 | 4.12197e-10 | 1.89482e-11 |
| R01_V2_nominal | 286.938 | 0.0010324 | 0.885949 | 0.937136 | 0.862583 | 2.63924e-08 | 4.32438e-07 |
| R02_V2_H2 | 119.25 | 0.0010655 | 0.897432 | 4.18577 | 4.1848 | 4.12197e-10 | 1.89482e-11 |
| R03_V2_S01 | 210.5 | 0.0012984 | 0.898508 | 4.1804 | 4.17897 | 8.11414e-10 | 5.85661e-11 |
| R04_V2_H1 | 281.812 | 0.0010078 | 0.874536 | 0.915231 | 0.835853 | 1.26511e-08 | 6.86938e-07 |


早期开发记录的c.timings漏掉抛异常的末次调用；上表Predictor max补充显示该次开销。最终回归已由runner测量每次调用，包括异常。完整2/20 ms超时比例、基座漂移/峰值/RMS、真实捕获四量、分阶段估计误差、功和能量见runs/*/analysis.json。每步P/H、实际力矩、接口载荷原量保留，未通过减去几何项制造通过。预测延迟显著超过20 ms，本实现不是实时控制器。

每条从t0一次初始化重放ctrl与锁紧事件；另一条从原SensorPacket重建估计、参考、guard、影子参数和力矩。验证身份逐运行保存，使用对应归档V1/V2源码。当前全部双重重放通过：True。最终统一版本为validation_timestamp_audited.json。初次S01验证发现t=0无效估计快照协方差的可变引用问题；原失败validation.json和原control_states.json保留不改。logging_integrity_audit.json核对全部8条，只在两条噪声运行的t=0存在差异，独立拷贝的同刻trace正确。统一重验仅以该同刻trace核验此无效协方差字段，其余状态/guard/动作全量比较。PASS_TS_AUDIT明确标识这项记录修正，物理任务结论不变。重放不是任务性能证据。

已记录进程CPU 3578.312 s；任务墙钟5502.5 s，上限均28800 s。开发与最终回归计数{'development': 4, 'regression': 4, 'holdout': 0, 'ablation': 0, 'step_sensitivity': 0, 'implementation_reserve': 0}。预测、独立诊断、测试和重放已计入账本。早期落盘/工具编排CPU未逐项计量，不能把账本数字称为精确总CPU；resource_summary.json披露范围与早期总墙钟上界。未启动后台补跑。

## 5. 辨识、理论与论文

历史H1/H3仅作支持性重分析，参数进入预测的时刻、白化前/后无正则SVD和误差时间序列已归档。H1数值秩10仍有约15.28 mm COM误差；prior/identified实际力矩和轨迹差为零，控制收益未被证明。C3不作为C1/C2前提。

theory_notes.md给出SE(3)输运/端点、冻结线性Phase-I、条件裕量、理想内部动量/耗能、孤立尺度歧义及当前感知配置的Riccati裕量下界推导。全程递归可行性、模型误差界、联合概率校准和硬件恢复仍为OPEN_GAP。

paper/system_paper/manuscript_v1.md为规范草稿；八篇原始文献的版本与读取层级见related_work_matrix.csv。前三篇只读到摘要，未冒称公式复现；含推进器/反作用轮文献不作为本系统数值基线。图表来自JSON/NPZ，claim_evidence_matrix.csv逐项关联窗口和实现身份。

论文数据审计状态为WARN，详细逐项结果见paper/PAPER_CLAIM_AUDIT.md和对应新上下文审查记录。WARN表示仍有无法由所提供原始数字材料完整验证的主张，需要投稿前补证；不能据此称为投稿就绪。

术语补充：沿用的“20 mm工具”指N206相对旧设计的+20 mm轴向延伸，不是法兰至物理前端面的距离；旧前端面为-0.2 mm，延伸后为19.8 mm。已在notation_and_assumptions.md区分该设计名称、实际前端面和原标定接口site，并链接原配置。此补充不属于最终数据审计的159项输入，审查中的几何映射WARN保留，未更改受审正文、原始trace或物理模型。

## 6. 停止与后续

选定版本未满足名义/H2/S01/H1全部通过的准入条件。冻结状态NOT_FROZEN，未生成新场景实际数值；36条配对、6条B2和2条细步长均NOT_EVALUATED。不是预算耗尽，也不是把已见重测包装成盲测。可执行域证据不足触发任务允许的阶段性停止。

N210只在接近/感知问题解决后考虑兼容执行器假设的抓后对照；N211再扩展独立连接实现、盲测、硬件误差预算和投稿规范。当前最优先缺口是更有依据的速度置信度与目标输运可执行域，而不是添加抓后复杂控制器。

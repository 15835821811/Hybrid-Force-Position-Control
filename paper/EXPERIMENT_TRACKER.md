# Experiment Tracker

> **历史运行说明（2026-09-16）**：下表 R000–R105 的已完成/部分完成数值对应旧目标几何与旧抓捕姿态，只用于追溯，不得复用为新配置的实验结论。新配置从 `N000` 系列重新计数，并要求重新执行 PSO、500 Hz 动力学复核和可视化。

## Material Passport

- Origin Skill: experiment-agent + experiment-plan
- Origin Mode: plan
- Origin Date: 2026-09-24T11:06:29+08:00
- Verification Status: PARTIALLY_VERIFIED
- Version Label: experiment_tracker_v12

## New-configuration Runs (authoritative after 2026-09-16)

> **实现世代说明（2026-09-21）**：N020--N041 的优化、动力学和媒体来自平滑/jerk 约束加入前的实现，只作为 `SUPERSEDED_BASELINE`；N051--N054 验证平滑控制器的单轨迹；N055P--N055 使用 `implicitfast` 身份并保留为失败诊断；N060M 以后冻结 RK4，新身份的跨种子统计由 N055P4/N055R 提供。

| Run ID | Milestone | Purpose | System / Variant | Split / Scenario | Metrics | Priority | Status | Notes |
|---|---|---|---|---|---|---|---|---|
| N000 | M0 | 冻结新位置、尺寸和端面姿态 | config + MuJoCo scene | `C=[0.95,0.14,0.95] m`, side `0.30 m` | geometry contract | MUST | DONE | `G=[0.80,0.14,0.95] m`；初始法兰到 G 为 0.532529446 m；法向相反 |
| N010 | M0 | 新配置回归与几何审计 | unit + integration tests | foundations | pass count, pose checks | MUST | DONE | 33/33 通过；配置、抓捕点、距离和端面法向均有测试覆盖 |
| N020 | M2 | 新配置正式 PSO | joint optimization, 20x1000 | seeds 0--9 | feasibility, objective, `T_c`, `psi_f` | MUST | SUPERSEDED_BASELINE | 修改前实现：9/10 top-1 规划可行；最佳 seed 0：41.208648，8.0 s，-1.531855 rad |
| N021 | M2 | top-5/seed 500 Hz 动力学复核 | torque-level replay | 50 candidates | joint qualification, rank | MUST | SUPERSEDED_BASELINE | 修改前实现：14/50 合格；3/10 seeds 至少一条合格；不得外推到当前控制器 |
| N030 | M3 | 新配置主轨迹 | seed 0 / rank 1 | selected pre-contact capture | terminal, base, safety metrics | MUST | SUPERSEDED_BASELINE | 修改前实现：位置 0.049826 mm；姿态 0.003446 deg；最小间距 131.792 mm；端面误差 0.003441 deg |
| N040 | M3 | 全过程可视化 | selected trace + all-seed PSO | GIF/MP4 + figures | hash, dimensions, frames | MUST | SUPERSEDED_BASELINE | 修改前实现的 GIF、MP4、故事板和统计图，仅作追溯 |
| N041 | M3 | 五视角与关节运动学同步视频 | selected trace | 5 views + `q/dq/ddq` | frame/time synchronization | MUST | SUPERSEDED_BASELINE | 修改前实现的 5 个独立 MP4 与合成视频，仅作追溯 |
| N050 | M3 | 新配置复现报告 | evidence synthesis | authoritative new results | claim/evidence alignment | MUST | DONE | `paper/FPMFC_NEW_TARGET_REPRODUCTION_REPORT.md`；结论保持 PARTIALLY_VERIFIED |
| N051 | M3 | 六秒关节折点诊断与文献检索 | hard angular cap + 50 Hz update | 5.5--7.2 s | command/actual accel, jerk | MUST | DONE | 定位为硬角速度范数限幅跨界与无 jerk 更新；采用 RSS 2021 三级运动学约束与光滑饱和原则 |
| N052 | M3 | C² 饱和、jerk 和提前制动实现 | smooth shoulder + HQP jerk bounds | J=40/60/80/100/120 sweep | feasibility, jerk, conflicts | MUST | DONE | 选定 J=80 rad/s³；新增速度上限与关节位置 jerk-aware viability bounds；回归测试覆盖 |
| N053 | M3 | 平滑控制器主轨迹动力学复核 | T=8.0 s, psi=-1.50 rad | 500 Hz torque replay | 11 acceptance gates | MUST | DONE | 11/11 通过；命令 jerk 80；边界冲突 0；独立重放误差 0 |
| N054 | M3 | 平滑轨迹全过程可视化 | selected smooth trace | GIF + five-view MP4 + q/dq/ddq | media manifest | MUST | DONE | 96 帧 GIF；5 独立视角 + 1920x1080 合成视频，241 帧、30 fps、8.033 s；两份 manifest 通过 |
| N055P | M2 | 平滑控制器正式预检 | joint PSO 20x50 + top-5 replay | seed 0, planning/acceptance 45/40 mm | hash, feasibility, 11 gates, replay, warmed p99 `<20 ms` | MUST | FAILED_GATE | 920 次评估/46 代；top-5 的 p99 为 2.43--2.62 ms，除终端臂形外均通过；5/5 动力学臂形误差 1.10--1.20°，故 0/5 联合合格 |
| N055P2 | M2 | N060 修复后的隔离预检 | feasible-first joint PSO 20x50 + top-5 replay | seed 0, planning shape 0.75°, dynamic shape 1°, clearance 45/40 mm | hash, feasible ranking, 11 gates, replay, warmed p99 `<20 ms` | MUST | DONE | 41 代/820 次评估；5/5 规划可行，3/5 联合合格，top-1 保持；top-1 动态臂形 0.788°、p99 2.438 ms、独立重放通过 |
| N055P3 | M2 | NaN 防护后的最终身份预检 | finite-safe feasible-first PSO 20x50 + top-5 replay | seed 0, same thresholds as N055P2 | final implementation hash, 11 gates, replay | MUST | DONE | optimizer payload 与 v2 完全一致；3/5 联合合格、top-1 保持、独立重放通过；最终实现 hash `1a19f776…0e6b` |
| N055 | M2 | `implicitfast` 身份的正式 PSO 与动力学复核 | joint PSO 20x1000 + top-5/seed | seeds 0--9, planning/acceptance 45/40 mm | seed qualification, rank, 11 gates, warmed p99 `<20 ms` | MUST | FAILED_GATE | PSO 10/10 规划可行且 top-5 全部可行；50/50 动力学完成，但仅 seed 0 合格（3/5），故仅 1/10 seeds 联合合格。seeds 1--9 的 45 条候选均只失败于 `maximum_momentum_delta>0.05`；旧输出冻结，不以新身份重解释 |
| N060 | M2 | 规划--动力学终端臂形一致性 | 0.75° planning margin + finite-safe feasible-first PSO | N055P failed candidates + seed-0 reruns | 1° acceptance preserved, qualification, rank | MUST | DONE | v3 端到端门通过，46/46 回归通过；随后正式组暴露独立的积分器动量问题，转入 N060M |
| N060M | M2 | 动量门失败根因与积分器契约 | same candidate under `implicitfast` vs RK4 | seed 1 / rank 1 diagnostic + former-failure regression | momentum delta, 11 gates, model identity | MUST | DONE | 同一候选最大动量差 `5.576878e-2 -> 9.408724e-8`；RK4 11/11 通过、p99 `4.604 ms`。冻结 2 ms RK4，保持动量门 `0.05`，47/47 回归通过；新实现 hash `8e09ba74…b89fa` |
| N055P4 | M2 | RK4 最终身份预检 | finite-safe feasible-first joint PSO 20x50 + top-5 replay | seed 0, planning shape 0.75°, dynamic shape 1°, clearance 45/40 mm | identity closure, 11 gates, replay, warmed p99 `<20 ms` | MUST | DONE | 41 代/820 次评估；optimizer payload 与 v3 精确一致，top-5 全部规划可行；RK4 动力学 3/5 联合合格、top-1 保持，合格候选动量差 `1.496e-8`、p99 `2.450--2.527 ms`，SHA/有限值/独立重放通过 |
| N055R | M2 | RK4 身份正式重跑 | joint PSO 20x1000 + top-5/seed | seeds 0--9, same frozen gates | seed qualification, rank, 11 gates, replay | MUST | DONE | PSO 10/10、top-5 50/50 规划可行；RK4 动力学每 seed 合格数 `[3,5,5,5,5,5,5,5,5,5]`，即 10/10 seeds、48/50 candidates 联合合格，top-1 保持 4/10；p99 `4.409--14.490 ms`，动量 `1.496e-8--9.550e-8`，50/50 fresh replay 与完整 SHA/有限值审计通过 |
| N061 | M3 | 归一化目标权重敏感性 | `w_base:w_align=1:1,1:2,2:1` | seeds 0--9 + top-5/seed | feasibility, unweighted base/alignment costs, common dynamic metrics | MUST | DONE | 三组均为 PSO 10/10 seeds、top-5 50/50 规划可行，RK4 动力学 10/10 seeds、48/50 candidates 联合合格；top-1 保持数 `3/4/3`。三组共同 top-1 为 6/10 seeds，共同动态选择为 5/10；`1:1` 与 `2:1` 动态选择 10/10 相同。SHA/身份/50 条重放/NPZ 有限值审计闭合；结论见 `paper/N061_WEIGHT_SENSITIVITY_REPORT.md` |
| N070 | M3 | 同捕获时间固定臂形诊断 | full optimized `ψ_f` vs fixed `0,π/2` | `T_c=8.0 s`, same target/config | 11 gates, base/EE/safety metrics, replay | SUPPLEMENTARY | DONE | 三条轨迹身份/配置/目标/时间/重放契约闭合；full 通过 11/11，fixed0 与 fixedpi2 均通过 7/11。该结果只说明当前 Flexiv 场景的构型可行性差异，不作为复现成败门槛 |
| N071 | M3 | 固定 `ψ_f=0` 参数化诊断 | fixed0 time-only PSO + deterministic diagnostic | 5 complete + 3 partial seeds; 171-point 100 ms grid | feasibility, failure mechanism, search value | SUPPLEMENTARY | STOPPED_DIAGNOSTIC | 约 5,620 次随机评价与 0/171 全域诊断点均无可行解；joint5 达 0.04 rad 停止裕量、任务雅可比条件数约 286--302。不补完整 2 ms 穷举；不是连续域不可行证明 |
| N072 | M3 | 固定 `ψ_f=π/2` 参数化诊断 | deterministic time-only diagnostic | 171-point 100 ms + 46-point 20 ms + two 2 ms local grids | feasibility, joint-limit/singularity mechanism | SUPPLEMENTARY | STOPPED_DIAGNOSTIC | 四组扫描去重 324 点均不可行；最优 `T=9.408 s` 的臂形误差 23.986°（31.982 倍规划门）。不补完整 2 ms 穷举；不是连续域不可行证明 |
| N073 | M3 | 控制器结构消融 | `full/no-shape/no-base-reaction` | first qualified row in original `candidates`, 10x3 | qualification, paired base/EE metrics, replay | MUST | DONE | 10 行不可变 manifest 与 30/30 轨迹完成；10/10 seed 公平性契约、有限值与确定性重放全部通过。full 10/10 合格；no-shape 0/10（臂形门 10、位置门 9）；no-base-reaction 10/10。后者相对 full 的峰值基座角速度中位差仅 0.0067%，Wilcoxon `p=0.6953`，不支持当前设置下的正向基座反作用收益主张 |
| N100 | M5 | 接触 wrench 符号与参考点预检 | analytic 2 kg box-on-plane contact | 4 contact points, both geom selections, two reference points | support force, action-reaction, wrench transport | MUST | DONE | 合力 `19.62 N`，重力平衡误差 `3.55e-15 N`；作用反作用误差 0；参考点力矩搬移误差 `1.40e-15 N·m`；独立接触配置与实现哈希已保存 |
| N101 | M5 | 法向导纳离散阶跃预检 | `M=1 kg, B=100 N·s/m, K=2500 N/m`, 2 ms | 3 N no-contact step, 5 s | continuous-match, overshoot, steady state, limits | MUST | DONE | 临界阻尼；解析稳态 1.2 mm；无过冲；2% 调节时间 0.124 s；离散/连续最大误差 `2.91e-5 m`；全部限幅门通过 |
| N102 | M5 | 物理自由目标刚性接触基线 | 1.2 mm rigid normal offset + FPMFC | seed-00 full handoff, 1 s unilateral transient | peak force, impulse, penetration, loss, replay | MUST | DONE | 公共门全部通过：峰值 `4.0117 N`、冲量 `2.4132 N·s`、最大活动接触穿透 `0.2856 mm`、最长丢失 `52 ms`；独立力矩重放通过 |
| N103 | M5 | 力—位—型接触扩展 | 3 N normal admittance + full FPMFC | 与 N102 相同初态/模型/接口 | force RMSE, impulse, base/angular impulse, replay | MUST | DONE_FAILED_GATE | 公共安全/接触门通过且重放通过；冲量较 N102 降 `18.42%`、接触角冲量降 `39.47%`、基座峰值角速度降 `18.51%`，但峰值力增加 `27.60%`，稳态 RMSE `1.2383 N=41.28% F_d`，未过 10% 力跟踪门 |
| N104 | M5 | 无臂形的导纳接触对照 | 3 N normal admittance + no-shape | 与 N102/N103 相同初态/模型/接口 | same metrics, replay | MUST | DONE_FAILED_GATE | 公共门与重放通过；稳态 RMSE `1.5934 N=53.11% F_d`、接触丢失 6 次，未过力门。N103 相比本组降低峰值力 `13.17%` 和 RMSE `22.29%`，但并非所有指标更优，不能宣称整体优势 |

## Historical Runs (old geometry; traceability only)

| Run ID | Milestone | Purpose | System / Variant | Split / Scenario | Metrics | Priority | Status | Notes |
|---|---|---|---|---|---|---|---|---|
| R000 | M0 | 确认现有迁移基线 | current `v6_mujoco` | 6 项回归测试 | test pass | MUST | DONE | 2026-09-15：6/6 通过 |
| R001 | M0 | 冻结论文/用户/标定参数来源 | config audit | paper main | provenance completeness | MUST | DONE | `configs/fpmfc_paper.yaml` 每个叶参数均有来源，单测通过 |
| R002 | M0 | 验证反作用映射 | MuJoCo `A=-Mbb^-1Mbq` | home + 100 随机可行状态 | momentum residual | MUST | DONE | 100 状态均满足 `≤1e-8` |
| R003 | M0 | 交叉验证广义雅可比 | MuJoCo vs finite difference vs Simscape | home + 20 状态 | relative error | MUST | DONE | Simscape home 导出已核验：位置差 0.075 mm，`J_g` 相对误差 0.187%，`J_bm` 2.146%；MuJoCo 随机状态有限差分已通过 |
| R004 | M0 | 验证臂形角与 `J_ψ` | S–E–W geometric | home + 零空间 sweep | derivative error, singularity margin | MUST | DONE | 基座不变性、步长收敛和零空间敏感性单测通过 |
| R010 | M1 | 验证目标刚体传播 | prescribed spin | 5°/s, 25 s | pose/velocity analytic error | MUST | DONE | 抓捕点解析速度与中心差分一致 |
| R011 | M1 | 验证五次轨迹边界 | quintic SE(3)+ψ | 多个 `T_c` | endpoint p/v/a, R/ω | MUST | DONE | 位置、姿态、臂形 C2 边界单测通过 |
| R012 | M1 | 论文抓捕半径可达性 | offset 0.25 m | `T_c∈[8,25] s` 稀疏网格 | feasible rate, clearance | MUST | DONE | 找到多条可行分支；19.016 s/1.075 通过最终动力学门槛；`ψ=π/2` 整秒网格 0/18 可行 |
| R013 | M1 | 几何缩放备选 | offset 0.15 m | 同 R012 | feasible rate, clearance | NICE | NOT_NEEDED | 0.25 m 忠实半径已有可行解，不触发缩放 |
| R018 | M2 | 固定臂形优化管线冒烟 | 4 particles × 2 generations | `ψ=0,π/2`, seed 0 | fixed-coordinate integrity, feasibility | MUST | DONE | `ψ=0` 找到可行时间；`ψ=π/2` 小样本未找到可行解；两者均严格保持固定维度 |
| R019 | M2 | PSO 管线冒烟 | 4 particles × 2 generations | seed 0 | objective descent, serialization | MUST | DONE | 8 次评估；43.5677 → 7.45053；仅验证管线，不作收敛声明 |
| R020 | M2 | PSO 收敛预检 | 20 particles × 50 generations | seed 0 | objective descent, feasibility | MUST | DONE | 49 代按 20 代停滞规则收敛，980 次评估，目标 6.64216→3.79693；top-1 动力学仍为第 1 并通过重放；最小间距仅 40.000807 mm |
| R020b | M2 | 双阈值 PSO 收敛预检 | 20 particles × 50 generations | seed 0, planning 45 mm / acceptance 40 mm | objective descent, feasibility | MUST | DONE | 跑满 50 代/1000 次评估，目标 6.64216→3.84665；top-5 中 4 条联合合格，动力学 top-1 保持，最小间距 45.259 mm |
| R021 | M2 | 论文配置 PSO | 20 × 1000 | 10 fixed seeds | best cost, `T_c*`, `ψ_f*`, dispersion | MUST | PARTIAL | 第一批 seeds 0–2 完成：3/3 可行，3440 次评估；全批最佳 seed 2 为 2.19831。第二批 seeds 3–9 的断点/资源预检已通过，等待显式确认 |
| R022 | M2 | 权重敏感性 | `w1:w2=1:1,1:2,2:1` | 5°/s | Pareto trade-off | MUST | TODO | 目标先归一化 |
| R023 | M2 | 完整动力学复核 | top-5/seed | 500 Hz MuJoCo | rank agreement, true metrics | MUST | PARTIAL | R021 seeds 0–2：15/15 通过动力学验收与重放，14/15 联合合格，3/3 top-1 保持；全批最佳动力学目标 2.19163；seeds 3–9 未运行 |
| R024 | M2 | 规划/验收安全裕量决策 | 40 mm vs 45/40 mm 双阈值 | R021 前 | clearance margin, ranking change | MUST | DONE | 冻结为规划 45 mm / 动力学验收 40 mm；相对 40 mm 解增加 5.258 mm 间距，动力学目标代价 +1.32% |
| R025 | M2 | 正式运行实现身份冻结 | 核心源码 + MuJoCo XML | R021 前 | composite SHA, cross-stage match, resume rejection | MUST | DONE | 组合实现指纹 `6763e400…5ae7`；seeds 103/104 契约冒烟验证优化/动力学一致性与身份感知断点跳过 |
| R030 | M3 | A级主方法 | optimized `T_c*,ψ_f*` | paper main | base/EE/safety metrics | MUST | TODO | 主结果 |
| R031 | M3 | 论文固定臂形对照 | `ψ=0`, fixed `T_c*` | same as R030 | same metrics | MUST | PARTIAL | smoke 同时间运行因 6.55 mm 终点误差未通过；正式最优时间待定 |
| R032 | M3 | 论文固定臂形对照 | `ψ=π/2`, fixed `T_c*` | same as R030 | same metrics | MUST | PARTIAL | smoke 同时间运行因 93.74 mm 终点误差未通过；正式最优时间待定 |
| R033 | M3 | 强固定臂形基线 | `ψ=0`, re-optimize `T_c` | same target | same metrics | MUST | PARTIAL | 4×2 smoke 动力学合格；联合方法峰值角速度低 14.08%，不得作为正式统计 |
| R034 | M3 | 强固定臂形基线 | `ψ=π/2`, re-optimize `T_c` | same target | same metrics | MUST | PARTIAL | 8–25 s 整秒网格 0/18 可行；正式 PSO 仍需运行并报告不可行率 |
| R035 | M3 | 控制结构对照 | current weighted QP | same trajectory | EE error vs base disturbance | MUST | TODO | 不得改变硬约束 |
| R036 | M3 | 严格优先级主方法 | two-level HQP | same trajectory | Level-1/2 residuals | MUST | PARTIAL | 单元测试与两个动力学 smoke 的层级退化均约 `1e-16`；正式主对照未运行 |
| R040 | M4 | 去臂形消融 | no `J_ψ` objective | 5°/s | same metrics | MUST | PARTIAL | 同一 `T_c,ψ_f` smoke：轨迹可零误差重放，但终端臂形误差 2.008 rad，验收失败；正式多种子结果未运行 |
| R041 | M4 | 去反作用消融 | no base reaction objective | 5°/s | same metrics | MUST | PARTIAL | 同一 `T_c,ψ_f` smoke：通过验收，但峰值基座角速度比完整方法低 2.38%；当前不支持正向机制主张，须由正式统计与权重敏感性判断 |
| R042 | M4 | 自旋稳健性 | optimized vs fixed | 2/5/8°/s | improvement distribution | NICE | TODO | 主论文可放 5°/s，其余附录 |
| R043 | M4 | 初态稳健性 | optimized vs fixed | ±2 cm/±2° | success rate, metrics | NICE | TODO | 固定采样表 |
| R044 | M4 | 参数稳健性 | mass/inertia perturbation | ±10% robot params | improvement distribution | NICE | TODO | 每次记录参数哈希 |
| R100 | M5 | 接触力符号/参考点测试 | contact wrench unit test | 单点已知法向 | force/torque sign error | MUST | TODO | 失败不得调控制器 |
| R101 | M5 | 阻抗离散阶跃测试 | no-contact filter | scalar normal axis | poles, overshoot, settling | MUST | TODO | 对照连续系统 |
| R102 | M5 | 刚性接触基线 | rigid position tracking | physical target | peak force, impulse, penetration | MUST | TODO | B级基线 |
| R103 | M5 | 力-位-型扩展 | normal admittance + FPMFC | physical target | force RMSE, impulse, base angular impulse | MUST | TODO | 论文框架扩展，不冒充原结果 |
| R104 | M5 | 无位-型的阻抗对照 | normal admittance, no shape optimization | physical target | same metrics | MUST | TODO | 隔离位-型贡献 |
| R105 | M5 | 目标参数敏感性 | FPMFC contact | target mass/inertia ±20% | stability and force metrics | NICE | TODO | 参数论文未公开 |
| R200 | M6 | 捕获后保持/消旋 | TBD | physical target | relative motion, angular rate | NICE | BLOCKED_BY_DESIGN | 需先冻结抓捕机构与约束模型 |

## Immediate Next Runs

1. `N110`（若继续 B 级主张）：把 A 级终端速度改为目标抓捕点 live twist，重新规划并冻结接触切换相对速度/能量门；不得在当前 seed-00 轨迹上继续事后搜索导纳参数。
2. `N111`（N110 通过后）：在训练/验证场景分离的有限参数网格上重新标定导纳，再复核峰值力和 `≤10% F_d` 稳态 RMSE。
3. 长期 `FORCE_HOLD` 前先定义双侧夹持或抓捕约束；单个单向接触垫对自由目标的 2 s 恒力诊断只作为结构边界，不作为失败后放宽门槛的理由。
4. 可行固定臂形性能对照与 N080–N082 稳健性仍按论文论证需要选做，不阻塞已经闭合的 A 级结论。

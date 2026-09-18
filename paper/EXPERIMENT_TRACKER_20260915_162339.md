# Experiment Tracker

## Material Passport

- Origin Skill: experiment-agent + experiment-plan
- Origin Mode: plan
- Origin Date: 2026-09-15T16:23:39+08:00
- Verification Status: UNVERIFIED
- Version Label: experiment_tracker_v1

| Run ID | Milestone | Purpose | System / Variant | Split / Scenario | Metrics | Priority | Status | Notes |
|---|---|---|---|---|---|---|---|---|
| R000 | M0 | 确认现有迁移基线 | current `v6_mujoco` | 6 项回归测试 | test pass | MUST | DONE | 2026-09-15：6/6 通过 |
| R001 | M0 | 冻结论文/用户/标定参数来源 | config audit | paper main | provenance completeness | MUST | TODO | 生成 `configs/fpmfc_paper.yaml` |
| R002 | M0 | 验证反作用映射 | MuJoCo `A=-Mbb^-1Mbq` | home + 100 随机可行状态 | momentum residual | MUST | TODO | 目标 `≤1e-8` |
| R003 | M0 | 交叉验证广义雅可比 | MuJoCo vs finite difference vs Simscape | home + 20 状态 | relative error | MUST | TODO | 先统一坐标系和速度排列 |
| R004 | M0 | 验证臂形角与 `J_ψ` | S–E–W geometric | home + 零空间 sweep | derivative error, singularity margin | MUST | TODO | 步长收敛 1e-5/1e-6/1e-7 |
| R010 | M1 | 验证目标刚体传播 | prescribed spin | 5°/s, 25 s | pose/velocity analytic error | MUST | TODO | A级无物理接触 |
| R011 | M1 | 验证五次轨迹边界 | quintic SE(3)+ψ | 多个 `T_c` | endpoint p/v/a, R/ω | MUST | TODO | 起止速度/加速度为 0 |
| R012 | M1 | 论文抓捕半径可达性 | offset 0.25 m | `T_c∈[8,25] s` 稀疏网格 | feasible rate, clearance | MUST | TODO | 不可达时先报告 |
| R013 | M1 | 几何缩放备选 | offset 0.15 m | 同 R012 | feasible rate, clearance | NICE | TODO | 与忠实场景分开报告 |
| R020 | M2 | PSO 冒烟 | 20 particles × 50 generations | seed 0 | objective descent, feasibility | MUST | TODO | 只检验管线 |
| R021 | M2 | 论文配置 PSO | 20 × 1000 | 10 fixed seeds | best cost, `T_c*`, `ψ_f*`, dispersion | MUST | TODO | `w=0.9, c1=1.5, c2=2` |
| R022 | M2 | 权重敏感性 | `w1:w2=1:1,1:2,2:1` | 5°/s | Pareto trade-off | MUST | TODO | 目标先归一化 |
| R023 | M2 | 完整动力学复核 | top-5/seed | 500 Hz MuJoCo | rank agreement, true metrics | MUST | TODO | 排序变化必须记录 |
| R030 | M3 | A级主方法 | optimized `T_c*,ψ_f*` | paper main | base/EE/safety metrics | MUST | TODO | 主结果 |
| R031 | M3 | 论文固定臂形对照 | `ψ=0`, fixed `T_c*` | same as R030 | same metrics | MUST | TODO | 只隔离臂形 |
| R032 | M3 | 论文固定臂形对照 | `ψ=π/2`, fixed `T_c*` | same as R030 | same metrics | MUST | TODO | 只隔离臂形 |
| R033 | M3 | 强固定臂形基线 | `ψ=0`, re-optimize `T_c` | same target | same metrics | MUST | TODO | 排除时间优势 |
| R034 | M3 | 强固定臂形基线 | `ψ=π/2`, re-optimize `T_c` | same target | same metrics | MUST | TODO | 排除时间优势 |
| R035 | M3 | 控制结构对照 | current weighted QP | same trajectory | EE error vs base disturbance | MUST | TODO | 不得改变硬约束 |
| R036 | M3 | 严格优先级主方法 | two-level HQP | same trajectory | Level-1/2 residuals | MUST | TODO | 验证不放宽末端任务 |
| R040 | M4 | 去臂形消融 | no `J_ψ` objective | 5°/s | same metrics | MUST | TODO | 机制隔离 |
| R041 | M4 | 去反作用消融 | no base reaction objective | 5°/s | same metrics | MUST | TODO | 机制隔离 |
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

1. `R001`：建立每个参数的来源表。
2. `R002`：把现有反作用映射测试扩展到随机状态。
3. `R004`：实现并验证 Flexiv 的连续臂形角，这是后续 PSO 和位-型控制的关键路径。


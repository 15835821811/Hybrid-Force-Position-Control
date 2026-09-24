# N073 控制器结构消融报告

## Material Passport

- Origin Skill: experiment-plan + experiment-result-to-claim
- Origin Mode: run + review
- Origin Date: 2026-09-24T10:35:00+08:00
- Verification Status: PARTIALLY_VERIFIED
- Version Label: n073_controller_ablation_v1

## 1. 问题与证据范围

N073 检验完整控制器二级目标的在线作用。每个 seed 从 N055R `candidate_replay_summary.json` 的原始 `candidates` 顺序中选择首个 `planning_and_dynamic_qualified=true` 的候选，冻结其捕获时间与终端臂形，再分别运行：

- `full`：保留臂形与基座反作用目标；
- `no-shape`：仅把二级臂形权重置零；
- `no-base-reaction`：仅把二级基座反作用权重置零。

该设计回答的是“删除在线二级目标对完整方法预先规划轨迹的影响”，不是各消融方法独立重规划后的端到端优劣。

## 2. 公平性与完整性

- 不可变候选 manifest 含 10 行，seeds 0--9 均为 kinematic rank 1；选择规则不读取 `dynamic_ranking[0]`。
- 30/30 轨迹均包含 `metrics.json`、`trace.npz` 与 `validation.json`。
- 10/10 seed 的候选、目标、模型、实现身份、dynamic run config、物理步数/任务 tick、有限值和控制器配置差异契约全部通过。
- 30/30 轨迹的 trace 哈希和确定性 qpos/qvel/法兰位置/间距重放全部通过；消融验收失败仍完整保留。

## 3. 结果

| Variant | 合格数 | 主要失败门 | 可支持结论 |
|---|---:|---|---|
| `full` | 10/10 | 无 | 完整方法在冻结候选上稳定通过终端、安全、力矩、动量与重放门 |
| `no-shape` | 0/10 | 臂形 10/10；位置 9/10；线速度 4/10；角速度 4/10；姿态 1/10 | 在线臂形目标对这些冻结轨迹实现规定终端臂形并通过捕获验收是必要的 |
| `no-base-reaction` | 10/10 | 无 | 删除该项不破坏当前 10 条轨迹的终端验收 |

`full` 与 `no-base-reaction` 的 10 个合格配对结果如下：

| Metric | Full mean | No-base mean | Full improvement mean | Median | Wilcoxon p |
|---|---:|---:|---:|---:|---:|
| 峰值基座角速度 (rad/s) | 0.0782922 | 0.0781554 | -0.1761% | 0.0067% | 0.6953 |
| RMS 基座角速度 (rad/s) | 0.0232663 | 0.0232902 | 0.1104% | -0.0204% | 0.8457 |
| 最大基座姿态漂移 (rad) | 0.0650057 | 0.0650615 | 0.0841% | -0.0074% | 1.0000 |

三项差异均接近零、方向不稳定且无统计显著性。因此 N073 不支持“当前权重和场景下，在线基座反作用二级项显著降低基座扰动”的主张。

## 4. Result-to-Claim 判定

- **Verdict**：`partial`
- **Confidence**：`high`
- **支持**：在当前自定义 Flexiv 场景、由完整方法预先规划并冻结的 10 条轨迹上，在线臂形目标对实现规定终端臂形和通过终端捕获验收是必要且有效的。
- **不支持**：两个二级目标都必要/有效；基座反作用目标显著降低扰动；消融方法独立重规划后的端到端优劣；向其他目标初态、自旋和参数外推。

完整方法的终端验收全部通过，但全过程位置 RMS 和最大误差在 10 条轨迹中的最坏值分别为约 `0.0843 m` 和 `0.2888 m`。这些数值必须作为瞬态跟踪指标单独报告，不能用终端门替代全过程一级精度证明。层级锁定本身记录的最大线/角退化约为 `7.47e-5 m/s` 和 `1.63e-4 rad/s`；若论文要强化“严格层级全过程保持”的表述，仍需逐任务周期审计一级最优值锁定前后残差。

## 5. 后续路由

主线进入 N100+ 接触力与力—位—型验证。以下仅作为按论文论证需要选做的补充，不阻塞接触主线：

1. `shape_weight=0, base_reaction_weight=0` 的纯一级控制对照；
2. 逐任务周期一级残差与二级 KKT/活跃度审计；
3. 若仍要保留基座反作用有效性主张，预注册权重扫描并在多个初态/自旋场景做配对检验；若仍无效，删除该机制收益主张；
4. 只有需要端到端消融结论时，才让各变体独立重规划。

## 6. 权威产物

- `output/fpmfc/precontact/n073_controller_ablation_rk4_formal_planning45/candidate_manifest.json`
- `output/fpmfc/comparison/n073_controller_ablation_rk4_formal_planning45/controller_ablation_summary.json`
- `output/fpmfc/comparison/n073_controller_ablation_rk4_formal_planning45/controller_ablation_runs.csv`
- `paper/review-traces/experiment-result-to-claim/2026-09-24_run01/`

由于当前没有 `paper/PAPER_CLAIM_AUDIT.json`，上述论文主张判定仍标记为 provisional；这不影响已完成的数值与重放审计。

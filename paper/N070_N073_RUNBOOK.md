# N070--N073 对照与消融收口记录

## Material Passport

- Origin Skill: experiment-plan
- Origin Mode: run
- Origin Date: 2026-09-24T10:35:00+08:00
- Verification Status: VERIFIED
- Version Label: n070_n073_runbook_v4

## 1. 前置门与公平性契约

- N055R 和 N061 均已通过：主组及两个权重敏感性组全部为 10/10 seeds 含联合合格候选。
- 所有对照使用 RK4、2 ms 物理步、20 ms 任务步、同一目标初态、同一 45/40 mm 规划/动力学间距门、同一 11 项动力学验收与实现组合 `8e09ba7485f96c549791041a9170fa08459f6031b5a8f8100dd0929d1deb89fa`。
- `run_capture` 不接收 `--planning-clearance`。为避免其回退到源配置的 40/40 mm 语义，手动动力学对照必须使用 `configs/fpmfc_paper_planning45_effective.yaml`。
- 上述载体文件 SHA-256 为 `5698170e1c44a68fada8aab158c346d59912004e84c6a4b4d7c3717a294c145c`；加载后的语义配置与 N055R `effective_config` 精确相等，语义 SHA-256 必须为 `07876267e8f44bb4bff75bb1c4b6a666d3b0e248d78396c66159305eb8589b0c`。源配置保持 `bc8dbafc49bdcef6ba9fb80c22a03a35dbcd643e48885bfcd6f80cf72bc85ee6`。
- 验收失败是实验结果，不是运行错误；所有失败轨迹仍必须完成哈希与确定性重放。只有哈希/重放/身份失败才中止后续比较。

## 2. 编号冻结

- `N070`：主方法 `ψ_f=-1.492877399060744` 与固定 `ψ_f=0, π/2` 在相同 `T_c=8.0 s` 下的三组公平对照。
- `N071`：固定 `ψ_f=0`，仅重新优化捕获时间；10 seeds、top-5/seed。
- `N072`：固定 `ψ_f=π/2`，仅重新优化捕获时间；10 seeds、top-5/seed。
- `N073`：按 N055R `candidate_replay_summary.json` 的原始 `candidates` 顺序，为每个 seed 选择第一个 `planning_and_dynamic_qualified=true` 的候选；禁止使用按 full-controller 动态目标重排的 `dynamic_ranking[0]`。对冻结的 10 条轨迹运行 `full`、`no-shape`、`no-base-reaction` 三种控制器，共 30 条轨迹。

## 3. N070 同捕获时间对照

输出根：

- `output/fpmfc/precontact/n070_fixed_time_rk4_formal_planning45/`
- `output/fpmfc/comparison/n070_fixed_time_rk4_formal_planning45/`

三条轨迹必须有相同有效配置、模型、目标和执行捕获时间；均完成 validation。固定臂形若未通过 11 项门槛，保留失败结果且不得计算相对改善率。

执行结果：三条轨迹的配置、模型、目标、时间和确定性重放契约均闭合。主方法通过 11/11 门；fixed0 与 fixedpi2 均只通过 7/11，失败门同为终端位置、臂形、线速度和角速度。因此同时间对照已完成，但 `relative_improvement_claim_allowed=false`，只能报告可行性差异，不能计算性能改善率。

## 4. N071/N072 固定臂形强基线

原协议要求两组均使用 20 粒子、最多 1000 代、seeds 0--9、规划间距 45 mm、top-5/seed RK4 动力学复核。输出根分别为：

- `output/fpmfc/optimization/n071_fixed0_rk4_formal_planning45/`
- `output/fpmfc/precontact/n071_fixed0_rk4_formal_planning45/`
- `output/fpmfc/optimization/n072_fixedpi2_rk4_formal_planning45/`
- `output/fpmfc/precontact/n072_fixedpi2_rk4_formal_planning45/`

N071 已于 2026-09-22 主动停止：五个完整 seed 和三个部分 seed 累计约 5,620 次评价均未产生可行解；171 点、100 ms 全域诊断网格也为 0/171 可行。fixed0 只有一个自由变量，继续增加随机 seed 不能有效区分漏搜与结构冲突。诊断显示 joint5 到达 0.04 rad 停止裕量、肘平面投影显著下降且任务雅可比条件数恶化到约 286--302。完整结论与证据边界见 `paper/N071_FIXED0_DIAGNOSTIC_REPORT.md`。

N072 也已诊断性停止：100 ms 全域筛查 0/171，20 ms 局部网格 0/46，分支切换与最优门槛盆地的 2 ms 网格分别 0/81、0/51。最优点 `T_c=9.408 s` 的位置已通过，但臂形误差仍为 23.986°，是规划门的 31.982 倍；分支切换区最小臂形误差 7.436° 时位置误差约 75.6 mm。joint5/joint7 停止裕量与近任务奇异是主要机制，详见 `paper/N072_FIXEDPI2_DIAGNOSTIC_REPORT.md`。

因此 N071/N072 均标记为 `STOPPED_DIAGNOSTIC`，定位为补充参数化诊断，不是方法复现成败门槛。按当前论文定位不再恢复随机 PSO或补完整 2 ms 穷举；若后续确需性能型固定臂形对照，应先用不读取动态扰动结果的多解 IK/零空间扫描选择可行角度，并使用新 run ID。

## 5. N073 控制器消融

输出根：

- `output/fpmfc/precontact/n073_controller_ablation_rk4_formal_planning45/`
- `output/fpmfc/comparison/n073_controller_ablation_rk4_formal_planning45/`

每个 seed 固定 N055R 原始 `candidates` 列表中首个联合合格候选的 `T_c,ψ_f`，只切换 `full`、`no-shape`、`no-base-reaction`。当前 10/10 均为 kinematic rank 1。不得使用 `dynamic_ranking[0]`，因为它在 6/10 seeds 中按 full-controller 结果换轨迹，形成处理结果选样偏差。

正式启动前必须先生成不可变的 10 行候选 manifest，并冻结源 summary/metrics/validation SHA、seed、kinematic rank、requested/executed `T_c`、`ψ_f`、配置和实现身份。30/30 轨迹必须有 metrics、trace、validation；每个 seed 必须恰好包含三个声明变体。比较契约还必须核对精确候选、目标、模型、实现身份、dynamic run config、physics steps/task ticks、有限值及除单一二级权重外无其他配置差异。消融失败仍计入分母；仅对双方都联合合格的配对计算性能差。

N073 的结论范围仅为“在 full 方法预先规划并冻结的轨迹上，删除在线二级目标的影响”，不能外推为消融方法各自重新规划后的端到端优劣。

### 5.1 执行结果

- 候选 manifest：`candidate_manifest.json`，10/10 seeds 均按原始 `candidates` 顺序选中首个联合合格候选，且均为 kinematic rank 1；SHA-256 为 `14c49ec37190d966ff253dbea6bdefa87700a37aef4c17fb48c475849abbe3f9`。
- 正式输出：30/30 轨迹、metrics、trace 与 validation 齐全；10/10 seed 的候选、目标、模型、实现身份、dynamic run config、步数/tick、有限值、声明权重差异与确定性重放契约全部通过。
- `full`：10/10 联合合格。
- `no-shape`：0/10 联合合格；终端臂形门失败 10/10、位置门失败 9/10、线速度门 4/10、角速度门 4/10、姿态门 1/10。因此支持“在完整方法预先规划的轨迹上，在线臂形目标对维持位—型可行性是必要的”；由于不存在双方合格配对，不计算扰动改善率。
- `no-base-reaction`：10/10 联合合格。相对 full，峰值基座角速度的中位改善仅 `0.0067%`、均值 `-0.1761%`、双侧 Wilcoxon `p=0.6953`；RMS 角速度与最大姿态漂移同样无显著差异。因此当前 N073 不支持“在线基座反作用二级项在冻结轨迹上显著抑制基座扰动”的正向主张。
- 聚合报告：`controller_ablation_summary.json`，SHA-256 为 `97f34b3bcef59fe0e8e4c32f8244f9bdcf9a775639c31d3a6b251d6706b62dd6`。

## 6. 固定运行顺序

本段实验顺序已闭合为 `N070 DONE -> N071/N072 STOPPED_DIAGNOSTIC -> N073 DONE`。不得因基线或消融失败而调门槛、删除失败证据或计算未合格方法的相对改善率。

后续不再设置固定 0°/90°基线决策门，主线进入 N100+ 接触力与力—位—型验证。固定臂形相关工作仅在论文确需性能基线时选做：

1. 先由多解 IK/零空间扫描冻结当前 Flexiv 几何下的可行固定臂形，不读取动态扰动结果。
2. 负、正可行分支各保留一个角度，再分别优化捕获时间并使用共同验收门；任意新角度必须使用新 run ID，不得覆盖 N071/N072。

# N055 当前平滑控制器正式运行手册

## Material Passport

- Origin Skill: experiment-plan
- Origin Mode: run
- Origin Date: 2026-09-21T15:45:02+08:00
- Verification Status: VERIFIED
- Version Label: n055_runbook_v5

## 1. 冻结契约

- 主方法：严格两级 HQP，C2 角速度饱和，关节命令 jerk 上限 `80 rad/s^3`。
- PSO：20 粒子、最多 1000 代、固定 seeds 0--9、连续 20 代改善不超过 `1e-5` 时提前停止。
- 决策变量：捕获时间 `T_c` 与终端臂形角 `psi_f`。
- 主权重：`w1:w2=1:2`，来源为本项目 `calibrated`，不是论文公开数值。
- 安全阈值：运动学规划最小间距 45 mm，500 Hz 动力学验收最小间距 40 mm。
- 终端臂形阈值：50 Hz 规划使用 0.75° 安全裕量，500 Hz 动力学验收仍保持 1°；两者均为本项目标定值，不冒充论文公开数值。
- PSO 选择策略：先按规划可行性分区，再在分区内按惩罚目标升序；若当前没有可行候选，仍按目标值排序。该规则同时用于个体最优、全局最优、社会引导和 top-5。
- 动量门槛：`maximum_momentum_delta=0.05`，失败时不得放宽。
- 模型积分契约：MuJoCo `RK4`，物理步长 `0.002 s`；XML、源配置、Python 模型契约和运行时枚举必须四者一致。
- 所有阶段必须保存并核对有效配置 SHA-256、实现组合 SHA-256、候选来源哈希和独立重放结果。
- 本次源配置 SHA-256：`bc8dbafc49bdcef6ba9fb80c22a03a35dbcd643e48885bfcd6f80cf72bc85ee6`。
- 本次 45/40 mm 有效配置 SHA-256：`07876267e8f44bb4bff75bb1c4b6a666d3b0e248d78396c66159305eb8589b0c`。
- 本次实现组合 SHA-256：`8e09ba7485f96c549791041a9170fa08459f6031b5a8f8100dd0929d1deb89fa`。
- 本次模型源包 SHA-256：`a6084bc9a331168290eea1402ad0546301962bfe562a8183abd02fff81640cde`。
- 本次模型运行时契约 SHA-256：`d524e1a17b40b38379d71be95aea6a6b2360c766f0f88448ebbaf2edd38a90cf`。

## 2. 旧身份结果与 N060M 诊断

首轮预检保留在不带 `_v2` 的输出根中：920 次评估、46 代，top-5 的任务层 p99 为 2.43--2.62 ms，五条均仅因动力学终端臂形误差 1.10--1.20° 而失败，联合合格为 0/5。该结果不得被新实现重解释或覆盖。

N060 已加入 0.75° 规划裕量和规划可行优先选择。v2 在 41 代/820 次评估后得到 5/5 规划可行、3/5 动力学联合合格，top-1 保持，终端臂形 0.788°，任务层 p99 2.438 ms。随后 N055P3 加入首个非有限目标拒绝保护，optimizer payload 与 v2 完全一致，仍为 3/5 联合合格。

旧 `implicitfast` 正式 N055 已完整运行并冻结在不带 `rk4` 的正式输出根：PSO 10/10 seeds 规划可行，50/50 top-5 候选完成动力学与验证，但只有 seed 0 的 3 条候选联合合格，故项目门仅为 1/10。seeds 1--9 的 45 条候选均只失败于 `maximum_momentum_delta>0.05`；50 条任务层 p99 均小于 20 ms。该 suite 在记录的旧 identity 下，目录、哈希链、有限值、门槛重算和确定性重放均通过审计；当前工作树切换到 RK4 后，它属于 `STALE_FOR_CURRENT_IDENTITY`，不得直接复用或覆盖。

N060M 对 seed 1 / rank 1 做同候选、同门槛的积分器对照：`implicitfast` 最大动量差为 `0.0557687803`，只失败动量门；RK4 为 `9.4087241e-8`、p99 `4.604142 ms`，11/11 通过。因而保持 `maximum_momentum_delta=0.05` 不变，并把 RK4 纳入配置、模型和实现身份。

## 3. N055P4：RK4 最终身份预检

使用全新隔离根；虽然运动学优化数值预期与 N055P3 一致，也必须重新生成以闭合新的配置/实现/模型身份：

```powershell
C:\Users\admin\.conda\envs\rltoorch\python.exe -m v6_mujoco.fpmfc.pso_suite --config configs/fpmfc_paper.yaml --variants joint --seeds 0 --population 20 --generations 50 --planning-clearance 0.045 --parallel 1 --output-root output/fpmfc/optimization/n055_smooth_jerk80_rk4_preflight

C:\Users\admin\.conda\envs\rltoorch\python.exe -m v6_mujoco.fpmfc.run_suite_candidates --suite-summary output/fpmfc/optimization/n055_smooth_jerk80_rk4_preflight/pso_suite_summary.json --config configs/fpmfc_paper.yaml --top-k 5 --parallel 1 --output-root output/fpmfc/precontact/n055_smooth_jerk80_rk4_preflight
```

Go 条件：

1. 当前测试通过；配置和实现哈希在优化、动力学和独立重放之间一致。
2. PSO、序列化、恢复和 top-5 动力学入口无异常。
3. 至少一个候选同时满足规划门槛和 11 项动力学验收，并通过独立重放。
4. 无 NaN、持续 QP 失败、速度/位置边界冲突或力矩饱和。
5. 预热后的重复计时应报告任务层 p99 并满足 `<20 ms`；单次操作系统调度抖动只作为实时性风险记录，不单独作为算法失败依据。
6. 序列化结果必须声明 `selection_policy=planning_feasible_then_objective_v1`，并保存与最终可行类别一致的 `history_feasible`。
7. `model_identity.integrator=RK4`，配置、有效配置、实现组合、模型源包和运行时契约 SHA 必须与本手册冻结值一致。

任一条件失败即停止，不启动正式十种子运行；先记录失败类型并修正根因。

## 4. N055R：RK4 身份正式重跑

```powershell
C:\Users\admin\.conda\envs\rltoorch\python.exe -m v6_mujoco.fpmfc.pso_suite --config configs/fpmfc_paper.yaml --variants joint --seeds 0 1 2 3 4 5 6 7 8 9 --population 20 --generations 1000 --planning-clearance 0.045 --parallel 3 --output-root output/fpmfc/optimization/n055_smooth_jerk80_rk4_formal_planning45

C:\Users\admin\.conda\envs\rltoorch\python.exe -m v6_mujoco.fpmfc.run_suite_candidates --suite-summary output/fpmfc/optimization/n055_smooth_jerk80_rk4_formal_planning45/pso_suite_summary.json --config configs/fpmfc_paper.yaml --top-k 5 --parallel 3 --output-root output/fpmfc/precontact/n055_smooth_jerk80_rk4_formal_planning45
```

正式完成条件：10/10 seeds 均产生可审计输出，50/50 候选均有动力学结果或明确失败记录，最终选择只来自 11/11 验收且独立重放通过的候选。

恢复时必须同时人工检查每个优化 JSONL 的末条事件为 `complete`。PSO 只支持按 seed 恢复，不支持从中断代继续；不得使用 `--overwrite`，不得让两个进程写入同一输出根，优化完成后不得移动目录。

项目 go/no-go 门槛：若少于 8/10 seeds 至少包含一条联合合格候选，不进入 N061/N070 论文对照，而按实际失败门继续诊断；该 8/10 门槛是本项目稳健性门槛，不是论文公开标准。

实际结果：10/10 seeds 含联合合格候选，48/50 candidates 联合合格，正式门通过。仅 seed0 rank4/5 因终端臂形误差失败；50 条 fresh torque replay 的 qpos/qvel/法兰/间距误差均为 0，任务层 p99 `4.409--14.490 ms`，最大动量差 `1.496e-8--9.550e-8`。完整身份、SHA、有限值和 11 门独立审计均通过，N061 已解锁。

## 5. 后续顺序

1. N055P4：RK4 seed-0 预检，未通过则停止。
2. N055R：RK4 10-seed PSO 与 50 条动力学复核，达到至少 8/10 seeds 联合合格才继续。
3. N061：权重 `1:1、1:2、2:1` 敏感性，每组隔离配置哈希和输出根。
4. N070--N073：联合方法、固定 `psi=0`、固定 `psi=pi/2` 和机制消融；只比较联合验收通过的方法。
5. N080--N082：自旋、初态和质量/惯量稳健性。
6. N100+：接触力扩展最后执行，不得替代预接触论文证据。

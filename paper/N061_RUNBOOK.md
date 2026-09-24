# N061 目标权重敏感性运行手册

## Material Passport

- Origin Skill: experiment-plan
- Origin Mode: run
- Origin Date: 2026-09-21T20:12:00+08:00
- Verification Status: VERIFIED
- Version Label: n061_runbook_v3

## 1. 前置门与冻结契约

- N055R 的 RK4 正式组已经完成 10 seeds 与 50 条候选；10/10 seeds 含联合合格候选，超过项目 `8/10` go/no-go 门。
- 只改变归一化优化目标权重 `w_base:w_align`；模型、目标、控制器、PSO、规划/验收阈值、seeds 和动力学门槛全部不变。
- 权重来源为本项目 `calibrated`，不是论文公开数值；主组为 `1:2`，敏感性组为 `1:1` 和 `2:1`。
- PSO：20 粒子、最多 1000 代、seeds 0--9、连续 20 代改善不超过 `1e-5` 时提前停止；每个 seed 的 top-5 做 500 Hz RK4 动力学与独立重放。
- 规划/动力学间距为 45/40 mm；规划/动力学终端臂形门为 0.75°/1°；任务层预热 p99 `<20 ms`；动量门保持 `0.05`。
- 源配置 SHA-256：`bc8dbafc49bdcef6ba9fb80c22a03a35dbcd643e48885bfcd6f80cf72bc85ee6`。
- 实现组合 SHA-256：`8e09ba7485f96c549791041a9170fa08459f6031b5a8f8100dd0929d1deb89fa`。
- 模型源包/运行时契约 SHA-256：`a6084bc9a331168290eea1402ad0546301962bfe562a8183abd02fff81640cde` / `d524e1a17b40b38379d71be95aea6a6b2360c766f0f88448ebbaf2edd38a90cf`。
- 45/40 mm 有效配置 SHA-256：`1:1 = eb7e3091d00869b8c8125b1baf4123e4aed61aa49c5e995ba59183dceb8f1547`；`1:2 = 07876267e8f44bb4bff75bb1c4b6a666d3b0e248d78396c66159305eb8589b0c`；`2:1 = 4e9e01823f2726953b42496c08d886e6302a2919156c989363dbf494bcd7d3d1`。

## 2. `1:2` 主组复用边界

`1:2` 直接引用以下 N055R 证据，不重复计算、不复制目录：

- optimizer：`output/fpmfc/optimization/n055_smooth_jerk80_rk4_formal_planning45/`
- dynamics：`output/fpmfc/precontact/n055_smooth_jerk80_rk4_formal_planning45/`

复用条件是上述有效配置 hash 恰为 `07876267…9b0c`，且模型/实现身份与本手册一致。复用只避免重复运行，不得把 `1:2` 的目标值与其他权重的目标值直接比较大小；应比较原始 `base_cost`、`alignment_cost`、可行率和共同动力学指标。

## 3. `1:1` 正式运行

```powershell
C:\Users\admin\.conda\envs\rltoorch\python.exe -m v6_mujoco.fpmfc.pso_suite --config configs/fpmfc_paper.yaml --variants joint --seeds 0 1 2 3 4 5 6 7 8 9 --population 20 --generations 1000 --objective-weights 1 1 --planning-clearance 0.045 --parallel 3 --output-root output/fpmfc/optimization/n061_weights_1_1_rk4_formal_planning45

C:\Users\admin\.conda\envs\rltoorch\python.exe -m v6_mujoco.fpmfc.run_suite_candidates --suite-summary output/fpmfc/optimization/n061_weights_1_1_rk4_formal_planning45/pso_suite_summary.json --config configs/fpmfc_paper.yaml --top-k 5 --parallel 3 --output-root output/fpmfc/precontact/n061_weights_1_1_rk4_formal_planning45
```

## 4. `2:1` 正式运行

```powershell
C:\Users\admin\.conda\envs\rltoorch\python.exe -m v6_mujoco.fpmfc.pso_suite --config configs/fpmfc_paper.yaml --variants joint --seeds 0 1 2 3 4 5 6 7 8 9 --population 20 --generations 1000 --objective-weights 2 1 --planning-clearance 0.045 --parallel 3 --output-root output/fpmfc/optimization/n061_weights_2_1_rk4_formal_planning45

C:\Users\admin\.conda\envs\rltoorch\python.exe -m v6_mujoco.fpmfc.run_suite_candidates --suite-summary output/fpmfc/optimization/n061_weights_2_1_rk4_formal_planning45/pso_suite_summary.json --config configs/fpmfc_paper.yaml --top-k 5 --parallel 3 --output-root output/fpmfc/precontact/n061_weights_2_1_rk4_formal_planning45
```

## 5. 完成条件与报告规则

1. 三组各有 10/10 seeds 的可审计 optimizer 结果；新增两组各有 50/50 候选动力学结果或明确失败记录。
2. 配置、有效配置、实现、模型、候选来源和 validation SHA 链闭合；JSON/NPZ 无 NaN/Inf。
3. 逐组报告规划可行 seed 数、联合合格 seed 数、联合合格候选数、top-1 保持数和 warmed p99；任何失败 seed/候选都保留在分母中。
4. 跨权重比较使用未加权的 `base_cost`、`alignment_cost`、基座角速度/姿态、末端误差与安全指标；不同权重下的加权 objective 不能直接当性能排名。
5. 若某敏感性组少于 8/10 seeds 联合合格，仍运行完另一组并如实报告其鲁棒性下降；不得调门槛或丢弃失败 seed。N070 是否启动以主组 N055R 已通过和 N061 证据完整为条件。
6. 运行顺序固定为 `1:1 optimizer -> 1:1 dynamics -> 2:1 optimizer -> 2:1 dynamics`；不得并发写入同一根，不使用 `--overwrite`。

## 6. 当前执行记录

- `1:1` optimizer 已完成：10/10 seeds 最优解规划可行，50/50 top-5 规划可行；代数为 `[41,58,61,55,68,59,48,51,40,41]`，评价次数严格为粒子数 20 乘以代数。suite SHA-256 为 `159d9c585c96a715f7ce0879ba7b592c3d93d7a68e028f03dd61f8fc3ce7bed6`。
- `1:1` dynamics 已完成：每 seed 联合合格数 `[3,5,5,5,5,5,5,5,5,5]`，即 10/10 seeds、48/50 candidates 联合合格，top-1 保持 3/10。仅 seed 0 的两条候选失败于终端臂形门；失败项保留在分母中。
- `1:1` 的 50/50 候选均通过确定性 qpos/qvel/法兰/间距重放；50 个 NPZ 共 2250 个数组、41,186,222 个值无 NaN/Inf。任务层 p99 为 `4.492725--15.555505 ms`，最大动量差为 `1.495829e-8--9.550324e-8`。dynamic summary SHA-256 为 `3bef785c23c2105cda8bc65c51e7eea7bac1c56327164f2143cbe2cc9aa5ea73`。
- `2:1` optimizer 已完成：10/10 seeds 最优解规划可行，50/50 top-5 规划可行；代数和评价次数与 `1:1` 相同。suite SHA-256 为 `f1fb4751973a4b182b5f80f5fe660d1dcbf8e782e1e5806fd3c757afe634db1f`。
- `2:1` dynamics 已完成：每 seed 联合合格数 `[3,5,5,5,5,5,5,5,5,5]`，即 10/10 seeds、48/50 candidates 联合合格，top-1 保持 3/10；50/50 确定性重放通过，2250 个数组、41,186,222 个数值无 NaN/Inf。p99 为 `4.452000--14.077735 ms`，最大动量差为 `1.495829e-8--9.550324e-8`。dynamic summary SHA-256 为 `58e8d07c33e3cf54955807762aa3ea9db72a9686abc2c2526d44485ab645209e`。
- 三组均为 10/10 seeds、48/50 candidates 联合合格；跨权重描述性分析见 `paper/N061_WEIGHT_SENSITIVITY_REPORT.md`。N061 已完成并解锁 N070--N073。

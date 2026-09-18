# R021 正式多种子 PSO 运行手册

## Material Passport

- Origin Skill: academic-research-suite + experiment-agent
- Origin Mode: plan + preflight
- Origin Date: 2026-09-15
- Verification Status: BATCH_03_09_RUNNING
- Version Label: r021_runbook_v1

## 1. 冻结实验契约

- 模型：500 kg 自由基座 + 7-DOF Flexiv Rizon 4s
- 目标翻滚角速度：5°/s
- 规划/动力学验收间距：45/40 mm
- 有效配置 SHA-256：`18f42a25d89a6ffa4e80d392288c1ee5a88522479bde16c46ed62cb8b37ffd4f`
- 模型运行契约 SHA-256：`e64027710bd07974fa9c4e0e15b2dbe9e7383e395e6fbd8007801dacb7fd59c6`
- 实现指纹 SHA-256：`6763e400f685cc6a46f8392187aeb8bf1543c67a58985b38c244322c7a8f5ae7`
- 粒子数：20
- 最大代数：1000
- 停滞规则：连续 20 代改善不超过 `1e-5`
- PSO 参数：`w=0.9, c1=1.5, c2=2.0`，速度上限为各维跨度 10%
- 正式随机种子：0–9
- 每种子保存 top-5

## 2. 本机并行预检

- CPU：Intel Core i9-14900KF，24 物理核 / 32 逻辑线程
- 内存：31.76 GB；预检时可用 12.42 GB
- 活动 Python 实验：无
- 目标输出目录：`output/fpmfc/optimization/formal_planning45/`
- 输出冲突：无，目录尚未创建
- 推荐并行度：3 个种子

R020b 的 50 代耗时为 23.6 分钟。每个正式种子可能因停滞规则提前结束，也可能继续到 1000 代；因此第一批 seeds 0–2 预计通常为 0.5–2 小时墙钟时间，单种子硬上限按 8 小时管理。不得依据预计时间提前宣称停止或收敛。

## 3. 第一批：seeds 0–2

推荐由套件用一条命令并行启动三个独立子进程，各自写独立 JSON/JSONL：

```powershell
C:\Users\admin\.conda\envs\rltoorch\python.exe -m v6_mujoco.fpmfc.pso_suite --config configs/fpmfc_paper.yaml --variants joint --seeds 0 1 2 --population 20 --generations 1000 --planning-clearance 0.045 --parallel 3 --output-root output/fpmfc/optimization/formal_planning45
```

套件实际派发的三个等价子命令为：

```powershell
C:\Users\admin\.conda\envs\rltoorch\python.exe -m v6_mujoco.fpmfc.optimizer --config configs/fpmfc_paper.yaml --seed 0 --population 20 --generations 1000 --planning-clearance 0.045 --output output/fpmfc/optimization/formal_planning45/joint/pso_seed_00.json

C:\Users\admin\.conda\envs\rltoorch\python.exe -m v6_mujoco.fpmfc.optimizer --config configs/fpmfc_paper.yaml --seed 1 --population 20 --generations 1000 --planning-clearance 0.045 --output output/fpmfc/optimization/formal_planning45/joint/pso_seed_01.json

C:\Users\admin\.conda\envs\rltoorch\python.exe -m v6_mujoco.fpmfc.optimizer --config configs/fpmfc_paper.yaml --seed 2 --population 20 --generations 1000 --planning-clearance 0.045 --output output/fpmfc/optimization/formal_planning45/joint/pso_seed_02.json
```

第一批完成后，对每个种子的 top-5 运行 500 Hz 动力学复核。只有同时满足规划可行、动力学验收和确定性重放的候选才进入正式排名。

```powershell
C:\Users\admin\.conda\envs\rltoorch\python.exe -m v6_mujoco.fpmfc.run_suite_candidates --suite-summary output/fpmfc/optimization/formal_planning45/pso_suite_summary.json --config configs/fpmfc_paper.yaml --top-k 5 --parallel 3 --output-root output/fpmfc/precontact/formal_planning45_batch_00_02
```

该入口按优化文件和配置哈希断点跳过，并输出 `dynamic_suite_summary.json`。

## 4. 恢复与失败策略

- 不自动覆盖已有 JSON。
- 每个优化结果保存完整有效配置快照及哈希。
- 每个优化与动力学结果同时保存核心源码和 MuJoCo XML 的组合实现指纹；代码指纹不一致时拒绝断点跳过、套件聚合或跨阶段复核。
- JSONL 是逐代进度证据，但只有最终 JSON 和 `complete` 事件同时存在才算完成。
- 中断后使用 `pso_suite` 汇总时，配置、种子、粒子数、代数和固定臂型全部匹配的结果才会跳过。
- 不可行种子保留原始结果，不自动换种子、不改变罚函数、不静默重试。
- 第一批若出现共同失败模式，先诊断再决定 seeds 3–9，不在运行中修改配置。
- `--parallel 3` 已用两个 2×1 临时种子完成真实并行和二次运行跳过检查。
- 批量 top-k 动力学入口已用两个临时种子完成并行、联合资格筛选和二次运行跳过检查。
- 实现指纹已用 seeds 103/104 的 2×1 契约冒烟验证：优化/动力学身份一致，二次运行正确跳过，不可行样本原样保留。

## 5. 第一批通过条件

1. 三个种子均生成完整 JSON/JSONL，哈希与冻结契约一致；
2. 至少存在可行种子，但不以此替换正式 10 种子要求；
3. 每个种子 top-5 完成动力学与独立重放；
4. 汇报可行率、终止代数、目标分布、`T_c/ψ_f` 分散度和动态重排；
5. 若结果正常，再以相同契约运行 seeds 3–9。

## 6. 第二批：完成 seeds 3–9

第二批必须在同一输出根上请求完整 seeds 0–9。套件会逐个核验配置、实现指纹和 PSO 参数，跳过已有的 seeds 0–2，只派发 seeds 3–9；全部结束后，`pso_suite_summary.json` 会覆盖完整 10 个固定种子。若仅请求 3–9，汇总文件将只含 7 个新种子，因此禁止使用该缩窄命令。

```powershell
C:\Users\admin\.conda\envs\rltoorch\python.exe -m v6_mujoco.fpmfc.pso_suite --config configs/fpmfc_paper.yaml --variants joint --seeds 0 1 2 3 4 5 6 7 8 9 --population 20 --generations 1000 --planning-clearance 0.045 --parallel 3 --output-root output/fpmfc/optimization/formal_planning45
```

预计通常需要约 1–3 小时墙钟时间；每个种子的硬上限仍为 8 小时。监控 `pso_seed_03.jsonl` 至 `pso_seed_09.jsonl`，30 秒检查一次进程存活和文件增长。任何崩溃、不可行或超时均保留原始证据，不自动换种子或重试。

优化完成后对完整 10 种子汇总运行：

```powershell
C:\Users\admin\.conda\envs\rltoorch\python.exe -m v6_mujoco.fpmfc.run_suite_candidates --suite-summary output/fpmfc/optimization/formal_planning45/pso_suite_summary.json --config configs/fpmfc_paper.yaml --top-k 5 --parallel 3 --output-root output/fpmfc/precontact/formal_planning45_batch_00_02
```

动力学输出根的 `batch_00_02` 名称为第一批历史路径，故不移动或重命名；入口会跳过已验证的 seeds 0–2，只新增 3–9，并把其中的 `dynamic_suite_summary.json` 更新为覆盖 seeds 0–9 的累计汇总。

第二批启动前预检记录：`output/fpmfc/optimization/r021_batch_03_09_preflight.json`。当前状态为 `READY_AWAITING_CONFIRMATION`。

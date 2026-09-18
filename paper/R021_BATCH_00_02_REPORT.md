# R021 第一批（seeds 0–2）正式运行报告

## Material Passport

- Origin Skill: academic-research-suite + experiment-agent + visualize
- Origin Mode: formal run + dynamic verification
- Origin Date: 2026-09-15
- Verification Status: VERIFIED_FIRST_BATCH
- Version Label: r021_batch_00_02_v1

## 1. 证据范围

本批次属于论文范围内的 A 级接触前复现：翻滚目标为规定运动，期望接触力为零，轨迹在接触前结束。结果验证 500 kg 自由基座 + 7-DOF Flexiv Rizon 4s 的时间–臂型联合优化与力矩级跟踪，不代表真实碰撞、抓持或捕获后消旋。

冻结契约为：规划间距 45 mm、动力学验收间距 40 mm、20 粒子、最多 1000 代、连续 20 代改善不超过 `1e-5` 时提前终止。有效配置 SHA-256 为 `18f42a25d89a6ffa4e80d392288c1ee5a88522479bde16c46ed62cb8b37ffd4f`，模型运行契约为 `e64027710bd07974fa9c4e0e15b2dbe9e7383e395e6fbd8007801dacb7fd59c6`，组合实现指纹为 `6763e400f685cc6a46f8392187aeb8bf1543c67a58985b38c244322c7a8f5ae7`。

## 2. 已确认命令

用户明确确认“R021 第一批”后执行：

```powershell
C:\Users\admin\.conda\envs\rltoorch\python.exe -m v6_mujoco.fpmfc.pso_suite --config configs/fpmfc_paper.yaml --variants joint --seeds 0 1 2 --population 20 --generations 1000 --planning-clearance 0.045 --parallel 3 --output-root output/fpmfc/optimization/formal_planning45
```

三个种子完成后，按同一批次契约执行：

```powershell
C:\Users\admin\.conda\envs\rltoorch\python.exe -m v6_mujoco.fpmfc.run_suite_candidates --suite-summary output/fpmfc/optimization/formal_planning45/pso_suite_summary.json --config configs/fpmfc_paper.yaml --top-k 5 --parallel 3 --output-root output/fpmfc/precontact/formal_planning45_batch_00_02
```

优化墙钟时间约 32 分 28 秒；三个种子共 3440 次运动学 rollout。动力学阶段对 15 个候选执行 500 Hz 力矩仿真和独立重放。

## 3. 优化与动力学结果

| Seed | 终止代数 | 评估数 | 运动学目标 | 动力学目标 | `T_c` (s) | `ψ_f` (rad) | 动力学间距 (mm) | 联合合格 top-5 | top-1 保持 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|:---:|
| 0 | 54 | 1080 | 3.846650 | 3.853054 | 16.996750 | -0.770298 | 45.259 | 4/5 | 是 |
| 1 | 43 | 860 | 3.898125 | 3.904276 | 16.912086 | -0.777754 | 47.206 | 5/5 | 是 |
| 2 | 75 | 1500 | **2.198309** | **2.191632** | 17.144869 | 1.017933 | **111.166** | 5/5 | 是 |

三个种子均得到运动学可行解，可行种子率为 100%。运动学最优目标的均值为 3.314361、标准差 0.966872、中位数 3.846650。15/15 候选通过动力学验收与独立重放；14/15 同时满足 45 mm 规划门槛。唯一未联合合格的是 seed 0/rank 4：动力学间距 44.912 mm，高于 40 mm 验收线但低于 45 mm 规划线，因此按预先冻结的规则排除。

三个 seed 的运动学 top-1 均保持为各自的动力学 top-1。第一批的正式选择是 seed 2/candidate 1；它位于正臂形角分支，且同时降低目标与增大间距。负臂形角分支仍存在并在 seeds 0–1 稳定复现，说明当前目标存在至少两个明显的可行吸引域。

## 4. 选定候选的完整动力学指标

| 指标 | 数值 | 门槛 | 结果 |
|---|---:|---:|:---:|
| 终端位置误差 | 0.002762 mm | ≤ 1 mm | 通过 |
| 终端姿态误差 | 0.000257° | ≤ 0.2° | 通过 |
| 终端臂形误差 | 0.001232° | ≤ 1° | 通过 |
| 终端线速度 | `1.309×10^-5` m/s | ≤ 0.001 m/s | 通过 |
| 终端角速度 | `3.630×10^-5` rad/s | ≤ 0.003491 rad/s | 通过 |
| 最小间距 | 111.166 mm | ≥ 40 mm | 通过 |
| 峰值 / RMS 基座角速度 | 0.014015 / 0.008050 rad/s | 报告指标 | — |
| 最大基座姿态漂移 | 0.110916 rad | 报告指标 | — |
| 最大动量变化 | 0.0002525 | ≤ 0.05 | 通过 |
| 最大关节力矩 | 0.7891 Nm | 各关节额定限幅 | 通过 |
| 力矩饱和率 | 0 | ≤ 0.001 | 通过 |
| 任务成功率 | 1.0 | ≥ 1.0 | 通过 |
| 任务层 p99 延迟 | 7.848 ms | 20 ms 周期内 | 通过 |

严格层级的最大线/角任务退化分别为 `3.04×10^-17 m/s` 和 `6.94×10^-17 rad/s`；最大动量映射残差为 `1.92×10^-15`。初始化后 `qpos/qvel` 写入计数均为 0。独立重放的 `qpos`、`qvel`、末端位置和最小间距最大误差均为 0。

## 5. 完整性与哈希

- 优化汇总：`output/fpmfc/optimization/formal_planning45/pso_suite_summary.json`，SHA-256 `d60f7dc1135fee8c8d8b994f91226fa31e10879c0f720756b02cab843b8fa039`。
- 动力学汇总：`output/fpmfc/precontact/formal_planning45_batch_00_02/dynamic_suite_summary.json`，SHA-256 `d022e4056f0a2fbff8d9e5ef4dd89d5f3ecb40bde984a0ca91db4322115f6ae1`。
- 选定候选指标：SHA-256 `666e0107e534341959a94eec6e8d829d177f42b48a5705228b100fbe28415c67`。
- 选定候选验证：SHA-256 `44fb682db05d13a7c92ff28cf31f45cae12b62b61de0ca2f2e6dd50f1b7992d4`。
- 选定候选完整轨迹：SHA-256 `83244db5fd525b8db04d1cfb03a30d2364ffcde766662881dc6a936d40671ac6`。

## 6. 当前可主张与不可主张

本批次支持：正式入口能在三个独立种子上找到可行动作；top-5 运动学排序经力矩级动力学复核后稳定；seed 2 找到一个比负臂形角分支目标更低、间距更大的候选。

本批次仍不支持最终多种子统计结论，因为 R021 预先规定 10 个固定种子，目前只完成 seeds 0–2。也不能据此主张真实接触力控制有效；物理接触属于单独的 B 级扩展。

## 7. 可视化交付

- 交互式全流程面板：参数冻结、三种子收敛、15 个候选的动力学目标–间距分布，以及选定轨迹的位置/姿态误差、基座角速度和间距时序。
- 3D MuJoCo 回放：`output/fpmfc/visualization/r021_batch_00_02/selected_capture_isometric.mp4`，640×480、30 fps、515 帧、17.1667 s，SHA-256 `00219130f3a1b2db3b0e8059791e31ba49e25059dd4a06b8e5c344cdc33fda82`。
- 三联画：`output/fpmfc/visualization/r021_batch_00_02/selected_capture_storyboard.png`，显示起点/中点/终点，SHA-256 `71b686e1a7c7659bd3ca1f3f65f1ddb3764ddd25470480e2c738c2df898d74e5`。
- 可视化清单：SHA-256 `47c4aa35a5f2e2f13aa84467886eb4931f4900e14e28166ca2903d39debae94a`；源轨迹哈希、视频帧数、时长、分辨率及三联画完整性全部通过。

首次请求 960×720 时，MuJoCo 在创建 Renderer 前因 XML 离屏缓冲上限 640×480 而拒绝，未生成任何帧；随后按模型原生上限重跑成功。该渲染失败不涉及优化或动力学实验重跑。

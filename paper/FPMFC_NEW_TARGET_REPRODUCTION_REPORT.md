# FPMFC 新目标位置与端面姿态复现报告

> **控制器更新说明（2026-09-18）**：第 3--5 节记录的是硬角速度限幅、无 jerk 约束的 v2 控制器结果，现仅用于修改前对照。当前控制器已改为 C² 角速度饱和、80 rad/s³ 关节 jerk 限制及 jerk 感知的速度/位置提前制动边界。当前主轨迹与视频见第 7 节；由于尚未用新控制器重跑 10-seed PSO，第 3 节的跨种子统计不应外推到新控制器。

## Material Passport

- Origin Skill: experiment-agent + paper-figure
- Origin Mode: run
- Origin Date: 2026-09-18T10:00:00+08:00
- Verification Status: PARTIALLY_VERIFIED
- Version Label: new_target_reproduction_report_v2

## 1. 结论与复现进度

基于新目标中心 `C=[0.95, 0.14, 0.95] m`、边长 `0.30 m`、抓捕点 `G=[0.80, 0.14, 0.95] m` 以及端面正对约束，预接触复现主链路已经完成：

1. 新几何与姿态契约已写入配置和 MuJoCo 场景，并通过静态几何审计；初始法兰到抓捕点距离为 `0.532529446 m`。
2. 端面约束已改为“末端接触面外法向与目标抓捕面外法向反向”，终态点积为 `-0.9999999982`，面面相向误差为 `0.003440821 deg`。
3. 已在新配置上重新运行 10 个固定随机种子的正式 PSO（20 粒子、最多 1000 代），没有复用旧位置的优化数值、断点或候选。
4. 已对每个种子的前 5 个候选进行 500 Hz 动力学重放，并对最终选中轨迹执行独立验证。
5. 已生成新配置的全过程 GIF、MP4、优化过程图，以及追踪误差和基座漂移图。

当前复现结论为 **PARTIALLY_VERIFIED**：新配置下存在通过全部终态、动力学和安全验收的完整轨迹；但只有 3/10 个种子在各自前 5 个候选中找到联合合格解，尚不能声称优化对随机种子具有充分稳健性。接触力控制、对照实验、消融和扰动稳健性也尚未在新配置上重跑。

本项目没有需要沿用的机器学习训练 checkpoint；此前所谓“训练/优化结果”均属于旧几何条件，现已从新结论中排除。复用的是经过测试的算法实现，而不是旧参数相关的实验输出。

## 2. 冻结的新实验契约

| 项目 | 值 |
|---|---:|
| 目标中心 | `[0.95, 0.14, 0.95] m` |
| 目标边长 | `0.30 m` |
| 抓捕点 | `[0.80, 0.14, 0.95] m` |
| 抓捕点体坐标偏移 | `[0.00, -0.15, 0.00] m` |
| 抓捕坐标系 RPY | `[-90, 0, 0] deg` |
| 初始法兰到抓捕点距离 | `0.532529446 m` |
| PSO 变量 | 捕获时间 `T_c`、终端臂形角 `psi_f` |
| PSO 规模 | 20 粒子，最多 1000 代，种子 0--9 |
| 规划/验收最小间距 | `45/40 mm` |
| 动力学步长/任务周期 | `2/20 ms`（500/50 Hz） |

有效配置 SHA-256 为 `aeb47ca99f7ba99efc831d2428373495322c21261c884f808bd020b80ab1e887`，正式实现组合 SHA-256 为 `9977d3e00f301e065f42483b493bde14b9660e1200ed67e2dc967d2334c52a5b`。

## 3. 新 PSO 结果

10 个种子中 9 个种子的 top-1 规划解可行，可行率为 `90%`。各种子最终目标值的均值为 `58.4485`，样本标准差为 `10.3834`，中位数为 `60.4443`，范围为 `[41.2086, 76.8096]`。

| Seed | 规划可行 | 目标值 | `T_c` (s) | `psi_f` (rad) | 收敛代数 |
|---:|:---:|---:|---:|---:|---:|
| 0 | 是 | 41.208648 | 8.000000 | -1.531855 | 49 |
| 1 | 是 | 60.834375 | 11.419516 | 2.352792 | 49 |
| 2 | 是 | 60.285807 | 11.518171 | 2.362878 | 51 |
| 3 | 否 | 41.209502 | 8.000000 | -1.533018 | 39 |
| 4 | 是 | 60.333337 | 11.456725 | 2.354952 | 57 |
| 5 | 是 | 60.541319 | 11.322185 | 2.351732 | 56 |
| 6 | 是 | 76.809590 | 12.765171 | 3.141593 | 33 |
| 7 | 是 | 60.347297 | 11.442664 | 2.358622 | 127 |
| 8 | 是 | 61.684729 | 11.537881 | 2.335962 | 42 |
| 9 | 是 | 61.230505 | 12.073148 | 2.382283 | 35 |

选中 seed 0 的最优规划解为 `T_c=8.0 s`、`psi_f=-1.531854548 rad`（`-87.7688 deg`）。边界审计表明，同一臂形分支在 `T_c<8 s` 时无法同时满足终态位置、臂形或动力学约束，因此 `8 s` 是该分支的可行边界，不应被解释为搜索提前截断后的任意边界解。

## 4. 500 Hz 动力学复核

对 10 个种子的 top-5 共 50 条候选进行了重放，其中 14/50 条通过全部动态验收；3/10 个种子至少有一条联合合格候选。35 条失败仅由动量差超过 `0.05` 引起，另 1 条仅由终端臂形误差超过 `1 deg` 引起。位置、姿态、间距和力矩饱和没有造成这些分支的失败。

最终选中 seed 0 / rank 1。其规划目标为 `41.208648`，动态目标为 `41.648514`，规划可行、动态验收和独立重放均通过。

| 指标 | 结果 | 验收值/说明 |
|---|---:|---:|
| 任务成功率 | 1.000 | 要求 1.000 |
| 终端位置误差 | 0.049826 mm | <= 1 mm |
| 终端姿态误差 | 0.003446 deg | <= 0.2 deg |
| 终端臂形误差 | 0.980550 deg | <= 1 deg |
| 终端线速度 | 0.120376 mm/s | <= 1 mm/s |
| 终端角速度 | 0.000493 rad/s | <= 0.003491 rad/s |
| 位置追踪 RMS | 1.010537 mm | 全过程统计 |
| 最大瞬态位置误差 | 3.165182 mm | 全过程峰值 |
| 最大瞬态姿态误差 | 7.080193 deg | 全过程峰值 |
| 最大基座平移漂移 | 5.161962 mm | 全过程峰值 |
| 最大基座姿态漂移 | 6.550914 deg | 全过程峰值 |
| 最大基座角速度 | 0.062783 rad/s | 全过程峰值 |
| 最小间距 | 131.791960 mm | >= 40 mm |
| 最大动量差 | 0.006435 | <= 0.05 |
| 最大关节力矩 | 3.742550 Nm | 无饱和 |
| 力矩饱和比例 | 0 | <= 0.001 |

终点精度通过并不等于全过程误差始终低于终点阈值。轨迹中间段的瞬态位置和姿态峰值分别为 `3.165 mm` 和 `7.080 deg`，图中因此将虚线明确标注为 **Terminal limit**。

## 5. 可视化与证据文件

| 产物 | 路径 | 验证 |
|---|---|---|
| 全过程 GIF | `output/fpmfc/visualization/adaptive_cube_c095_014_095_face_aligned_v2_selected/selected_capture_isometric.gif` | 720x540，96 帧，8.0 s，12 fps |
| 全过程 MP4 | `output/fpmfc/visualization/adaptive_cube_c095_014_095_face_aligned_v2_selected/selected_capture_isometric.mp4` | 960x720，241 帧，8.033 s，30 fps |
| 五视角+关节运动学合成 MP4 | `output/fpmfc/visualization/adaptive_cube_c095_014_095_face_aligned_v2_selected/five_view_joint_kinematics/selected_capture_five_view_joint_kinematics.mp4` | 1920x1080，241 帧，8.033 s，30 fps |
| 五个独立视角 MP4 | `output/fpmfc/visualization/adaptive_cube_c095_014_095_face_aligned_v2_selected/five_view_joint_kinematics/selected_capture_{isometric,front,right,top,rear}.mp4` | 各 640x360，同帧数、帧率和时长 |
| 关节位置/速度/加速度曲线 | `output/fpmfc/visualization/adaptive_cube_c095_014_095_face_aligned_v2_selected/five_view_joint_kinematics/joint_position_velocity_acceleration.pdf` | 7 关节、500 Hz 全轨迹；加速度为速度中心差分 |
| 故事板 | `output/fpmfc/visualization/adaptive_cube_c095_014_095_face_aligned_v2_selected/selected_capture_storyboard.png` | 已目视检查 |
| 优化流程图 | `output/fpmfc/visualization/adaptive_cube_c095_014_095_face_aligned_v2_selected/fig_optimization_process.pdf` | 收敛、种子结果、`T_c-psi_f` 分布 |
| 追踪/误差/漂移图 | `output/fpmfc/visualization/adaptive_cube_c095_014_095_face_aligned_v2_selected/fig_tracking_error_base_drift.pdf` | 8 面板全过程指标 |
| PSO 汇总 | `output/fpmfc/optimization/adaptive_cube_c095_014_095_face_aligned_v2/formal_planning45/pso_suite_summary.json` | 10 种子正式结果 |
| 动力学汇总 | `output/fpmfc/precontact/adaptive_cube_c095_014_095_face_aligned_v2/formal_planning45/dynamic_suite_summary.json` | top-5/seed 重放结果 |
| 最终轨迹 | `output/fpmfc/precontact/adaptive_cube_c095_014_095_face_aligned_v2/formal_planning45/joint/seed_00/candidate_01/trace.npz` | SHA-256 `802d06b2...cb430` |

全过程图清单 `full_process_visualization_manifest.json`、视频/GIF 清单 `visualization_manifest.json` 和五视角清单 `five_view_joint_kinematics/five_view_joint_kinematics_manifest.json` 的 `passed` 均为 `true`。这些清单记录输入文件哈希、输出尺寸、帧数和持续时间，用于避免把旧配置的可视化误当成新结果。

## 6. 尚未完成的论文级证据

- 新配置的固定臂形、去臂形目标、去基座反作用目标等对照与消融尚未重跑。
- 权重敏感性、目标自旋速度、初态扰动和质量/惯量扰动尚未重跑。
- 当前范围是预接触零期望力复现；接触阶段的法向力控制、捕获后保持和消旋仍未实现。
- 跨种子动力学联合合格率为 30%，需要进一步分析动量差门槛附近的失效机理，或在不改变验收标准的前提下改进规划—动力学一致性。

因此，当前能够支持的严谨表述是：**新目标位置和正确端面姿态下，已经找到并独立验证了一条安全、终态准确、面面相向的预接触轨迹，并完成了可复现的正式 PSO、动力学重放和全过程可视化；跨种子稳健性及接触阶段结论仍待补充。**

## 7. 角速度光滑饱和与 jerk 限制更新（当前主轨迹）

约 6 s 的“关节跳变”不是位置不连续，而是末端角速度硬限幅边界与 50 Hz 加速度盒约束共同造成的速度斜率突变。依据 jerk-limited online trajectory generation 文献，控制器现采用 C² 径向 `tanh` 肩部饱和，并在 HQP 速度更新中显式约束离散 jerk；另外加入 jerk 感知的速度上限和关节位置停止距离，以避免到硬边界才紧急制动。完整推导和文献来源见 `paper/FPMFC_SMOOTHING_METHOD_REVIEW.md`。

参数扫描后冻结 `angular_saturation_transition_ratio=0.90`、`joint_jerk_limit_rad_s3=80.0`。当前选中轨迹为 `T_c=8.0 s, psi_f=-1.50 rad`。它在运动学规划中可行，并通过 500 Hz 动力学与独立力矩重放。

| 指标 | 当前结果 | 验收 |
|---|---:|---:|
| 终端位置误差 | 0.031682 mm | <= 1 mm |
| 终端姿态误差 | 0.002521 deg | <= 0.2 deg |
| 终端臂形误差 | 0.72899 deg | <= 1 deg |
| 终端线速度 | 0.21023 mm/s | <= 1 mm/s |
| 终端角速度 | 0.000414 rad/s | <= 0.003491 rad/s |
| 最小间距 | 134.237 mm | >= 40 mm |
| 最大动量差 | 0.006878 | <= 0.05 |
| 最大命令 jerk | 80.0000 rad/s³ | <= 80 rad/s³（舍入容差） |
| 速度/位置边界冲突 | 0 | 必须为 0 |

在 5.5--7.2 s 诊断窗口内，命令 jerk 峰值从 `229.5572` 降至 `80.0000 rad/s³`（降低 65.15%），实测关节 jerk 峰值从 `1292.2202` 降至 `455.3914 rad/s³`（降低 64.76%）。新五视角合成视频位于 `output/fpmfc/visualization/adaptive_cube_c095_014_095_face_aligned_v6_smooth_jerk80/five_view_joint_kinematics/selected_capture_five_view_joint_kinematics.mp4`，241 帧、30 fps、8.033 s；全过程 GIF 位于同级 `single_view/selected_capture_isometric.gif`，96 帧、12 fps、8.0 s。两份媒体 manifest 均通过。

本节只验证“六秒折点消除”和一条完整轨迹的动态可行性；正式 10-seed 优化、对照、消融与鲁棒性仍需在新控制器实现身份上重跑。因此整份报告的总状态仍为 **PARTIALLY_VERIFIED**。

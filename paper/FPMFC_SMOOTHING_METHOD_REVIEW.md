# FPMFC 角速度限幅与关节 jerk 平滑方法说明

## 材料护照

- 任务：解释并消除约 6 s 处的关节运动折点。
- 证据范围：同行评议论文原文或出版社/会议官方页面，以及本项目可复现实验数据。
- 方法状态：已实现并通过 500 Hz 动力学重放、独立力矩重放及五视角媒体校验。

## 问题定位

原控制器先计算末端角速度命令，再用欧氏范数硬裁剪到
`0.45 rad/s`。原始命令在约 6.28 s 穿越该边界，硬裁剪映射的一阶导数在边界处不连续；下一次 50 Hz HQP 更新又只约束关节加速度幅值，没有约束相邻周期的加速度变化。因此关节位置保持连续，但速度斜率会突然变化，表现为加速度台阶和很大的离散 jerk。

## 文献依据

1. Berscheid 与 Kröger 的 Ruckig 工作把每个自由度的速度、加速度和 jerk 同时列为在线轨迹生成约束，并指出三级约束产生线性加速度、二次速度和三次位置段。该结果直接支持“在 50 Hz 更新层显式限制 jerk”，而不只是事后平滑曲线。[RSS 2021 官方论文](https://www.roboticsproceedings.org/rss17/p015.pdf)，DOI: 10.15607/RSS.2021.XVII.015。
2. Kröger 与 Wahl 将在线轨迹生成定义为机器人对未预见事件进行即时、连续反应的中间层。这支持在实时命令更新处实施平滑约束，而不是只修改离线绘图。[IEEE Transactions on Robotics 官方页面](https://ieeexplore.ieee.org/document/5350749/)，DOI: 10.1109/TRO.2010.2060467。
3. Sussmann、Sontag 与 Yang 讨论有界控制并明确以 `tanh` 作为光滑饱和函数示例。该论文研究的是更一般的有界反馈稳定化，而非本项目的 HQP；这里仅采用其“光滑有界映射”原则。[IEEE Transactions on Automatic Control 论文](https://www.sontaglab.org/FTPDIR/saturated-stab.pdf)，39(12):2411–2425, 1994。

## 采用的方法

### 1. C² 径向角速度饱和

令原始角速度向量为 \(\boldsymbol\omega\)，\(r=\|\boldsymbol\omega\|\)，上限为 \(L\)，平滑肩点为 \(r_0=\alpha L\)，\(0<\alpha<1\)，\(\Delta=L-r_0\)。输出幅值取

\[
s(r)=
\begin{cases}
r, & r\le r_0,\\
r_0+\Delta\tanh\!\left(\dfrac{r-r_0}{\Delta}\right), & r>r_0.
\end{cases}
\]

输出向量为 \(\boldsymbol\omega_s=s(r)\boldsymbol\omega/r\)。在肩点处，两段的函数值、一阶导数和二阶导数分别相等，因此该映射为 C²；它在低速区完全保持原命令，在高速度区单调、保方向且渐近不超过 \(L\)。本实验使用 \(L=0.45\,\mathrm{rad/s}\)、\(\alpha=0.90\)。该具体“恒等区 + tanh 肩部”是针对当前控制器构造的工程实现，不应表述为上述论文的原公式。

### 2. 50 Hz HQP 的离散 jerk 约束

对关节 \(i\)，已知上一周期速度 \(v_{k-1,i}\) 与加速度 \(a_{k-1,i}\)。周期 \(T=0.02\,\mathrm{s}\)，jerk 上限 \(J\)，加速度上限 \(A_i\)。先构造

\[
a^-_{k,i}=\max(-A_i,a_{k-1,i}-JT),\qquad
a^+_{k,i}=\min(A_i,a_{k-1,i}+JT),
\]

再把 HQP 的速度盒约束收紧为

\[
v_{k-1,i}+Ta^-_{k,i}\le v_{k,i}\le
v_{k-1,i}+Ta^+_{k,i}.
\]

由此直接保证离散关系
\(|a_{k,i}-a_{k-1,i}|/T\le J\)，同时保留原有关节速度、加速度、位置屏障和碰撞距离约束。参数扫描比较了 \(J=40,60,80,100,120\,\mathrm{rad/s^3}\)：40 对原 8 s 轨迹过于保守，120 虽可通过但平滑收益较小，最终取 \(J=80\,\mathrm{rad/s^3}\)。因此每个 20 ms 周期的加速度最多变化 \(1.6\,\mathrm{rad/s^2}\)，从零爬升到原 \(4\,\mathrm{rad/s^2}\) 上限至少需要 0.05 s。

仅增加 jerk 盒约束还不够：如果加速度在速度硬上限前没有提前回落，或仍使用只看当前位置的一阶位置屏障，控制器会到边界才急刹车，造成速度/位置约束与 jerk 约束冲突。因此实现增加两层可行性边界：(1) 依据 \(v+a_+^2/(2J)\le v_{\max}\) 在速度上限前把正加速度平滑降到零；(2) 计算“以最大反向 jerk 降至最大制动加速度、再保持最大制动加速度直至停止”所需的距离，并通过单调二分得到当前剩余行程允许的最大速度。两者都使制动提前发生，对应 Ruckig 所述的越界前 brake pre-trajectory 原则。

500 Hz 伺服参考仍在两个相邻 HQP 速度之间线性插值；由于每段斜率就是受 jerk 约束的 HQP 加速度，相邻段的斜率跳变量现在有明确上界。

## 验收条件

- 光滑饱和保持方向，范数严格小于或等于 `0.45 rad/s`，并通过肩点 C² 数值回归测试。
- 50 Hz 命令 jerk 不超过配置值（计及求解容差）。
- 6 s 邻域不再出现原先约 `0 → 4 rad/s²` 的单周期加速度突变。
- 动态仿真的末端位置、姿态、形位、末端速度、最小间隙、动量和转矩饱和仍满足既有验收阈值。

## 边界与说明

这次修改限制的是运动学命令的离散 jerk，不等价于对真实执行器力矩导数做硬约束。真实关节加速度/jerk 仍会受到 500 Hz 伺服、模型惯性及数值微分的影响，因此必须用 MuJoCo 动力学重放验证，不能只凭运动学规划结果下结论。

## 验证结果（2026-09-18）

最终采用 `T=8.0 s`、`psi_f=-1.50 rad`、`J=80 rad/s^3`。50 Hz 运动学规划可行，500 Hz 动力学验收的 11 个分项全部通过，独立重放的 qpos、qvel、法兰位置和间距误差均为 0。

| 指标 | 修改前 | 修改后 | 变化 |
|---|---:|---:|---:|
| 5.5--7.2 s 最大命令 jerk | 229.5572 rad/s³ | 80.0000 rad/s³ | -65.15% |
| 5.5--7.2 s 最大实测关节 jerk | 1292.2202 rad/s³ | 455.3914 rad/s³ | -64.76% |
| 5.5--7.2 s 最大实测关节加速度 | 4.5211 rad/s² | 4.2045 rad/s² | -7.00% |
| 速度/位置可行性边界冲突 | 未记录 | 0 | 通过 |

修改后终端位置误差为 `0.031682 mm`，姿态误差为 `0.002521 deg`，臂形误差为 `0.72899 deg`，线速度为 `0.21023 mm/s`，角速度为 `0.000414 rad/s`，最小间距为 `134.237 mm`，最大动量差为 `0.006878`，均满足既有阈值。

证据文件：

- 动力学轨迹：`output/fpmfc/precontact/adaptive_cube_c095_014_095_face_aligned_v6_smooth_jerk80/selected_candidate/trace.npz`
- 独立验证：同目录 `validation.json`
- 六秒事件对比：`output/fpmfc/analysis/adaptive_cube_c095_014_095_face_aligned_v6_smooth_jerk80/six_second_smoothing_comparison.{png,pdf,json}`
- 五视角 MP4 与关节曲线：`output/fpmfc/visualization/adaptive_cube_c095_014_095_face_aligned_v6_smooth_jerk80/five_view_joint_kinematics/`
- 全过程 GIF、单视角 MP4 与故事板：`output/fpmfc/visualization/adaptive_cube_c095_014_095_face_aligned_v6_smooth_jerk80/single_view/`

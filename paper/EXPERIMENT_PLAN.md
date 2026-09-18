# 基于 Flexiv Rizon 4s 的力-位-型融合控制复现设计

## Material Passport

- Origin Skill: experiment-agent + experiment-plan
- Origin Mode: plan
- Origin Date: 2026-09-15T16:23:39+08:00
- Verification Status: UNVERIFIED
- Version Label: code_plan_v1

## Experiment Overview

- **Title**：用自由漂浮 Flexiv Rizon 4s 复现梁斌等（2024）的力-位-型融合控制方法
- **Objective**：保留用户现有 500 kg 自由基座、7-DOF Flexiv Rizon 4s 和 MuJoCo 动力学模型，复现论文的广义雅可比、捕获时间/臂形联合优化、多优先级位-型控制与三组对照实验；在此基础上单独扩展真实接触力控制。
- **Primary hypothesis**：在末端捕获任务、约束与捕获时刻公平一致时，优化捕获时刻与臂形角能比固定臂形显著降低自由基座角速度/姿态扰动，且不牺牲末端跟踪精度与安全性。
- **Supporting hypothesis**：加入真实接触后，沿接触法向的基于位置的阻抗/导纳修正能比刚性位置跟踪降低峰值接触力与基座角冲量。
- **Anti-claim**：改善不能来自放宽末端误差、延长运动时间、越过关节/碰撞约束、改变目标初态或人为选择更有利的评价时间窗。
- **Type**：simulation

## 1. 复现边界与结论

本项目应分成两个严格分开的层级：

1. **A级：论文忠实复现（必须先完成）**。复现论文实际验证的接触前捕获过程。目标按给定刚体运动传播，末端期望力设为 0；比较优化臂形与 `ψ=0`、`ψ=π/2`。该层级对应论文第 4 节真正报告的仿真证据。
2. **B级：真实接触扩展（A级通过后再做）**。增加带自由关节的物理目标、抓捕接口、接触力估计、阻抗修正和稳定保持。论文结论明确说明接触碰撞与捕获后消旋尚未被其仿真验证，因此 B 级结果必须标注为“基于论文框架的扩展”，不能称为论文原结果复现。

由于机器人模型改为 Flexiv，合理的复现目标是**机制复现和相对结论复现**，不是强求论文的最优时间 `15.6 s`、最优臂形 `0.2686 rad` 或扰动曲线数值完全相同。

## 2. 输入与当前模型审计

| Input | Path | Description |
|---|---|---|
| 参考论文 | `reference_papers/自由漂浮空间机器人捕获翻滚...标的力-位-型融合控制方法_梁斌.pdf` | 方法、公式、PSO 参数与对照实验定义 |
| MuJoCo 模型 | `models/flexiv_rizon4s_scene.xml` | 500 kg 自由基座 + 7-DOF Rizon 4s，零重力，2 ms 步长 |
| 模型契约 | `v6_mujoco/model.py` | `nq=14, nv=13, nu=7`，关节/力矩/速度限值与模型加载 |
| 现有任务控制 | `v6_mujoco/hierarchical_qp.py` | 50 Hz 反作用感知速度 QP、碰撞距离约束 |
| 现有低层执行 | `v6_mujoco/run.py` | 500 Hz 逆动力学关节力矩伺服、日志与动量计算 |
| Simscape 对照 | `diagnostics/source_model_inventory.json` | 原模型广义雅可比、质心与 `J_bm` 的 MATLAB Function 源码 |

当前模型的可复用事实：

- 自由基座质量为 `500 kg`，但对角惯量为 `20.8333 kg·m²`；论文基座惯量为 `50 kg·m²`。保留用户模型值，不替换成论文值。
- 机器人总质量为 `520.8 kg`；机械臂质量、几何偏置和关节限位均按用户模型保留。
- 当前 `reaction_velocity_map()` 已从 MuJoCo 完整质量矩阵计算自由基座反作用映射；`task_jacobians()` 已形成自由漂浮广义末端雅可比。
- 当前场景中的 `reference_marker` 是无碰撞的 mocap 标记，不是物理翻滚目标；`run.py` 还会把所有接触全部关闭。
- 现有随机路点 5 场景的 QP 成功率为 100%，位置 RMS 最坏约 `0.243 mm`，动量残差最坏约 `5.56e-4`。这些只作为平台健康基线，不是论文结果。

### 2.1 参数迁移原则

| 项目 | 论文设置 | 本项目设置 | 来源标签 |
|---|---|---|---|
| 机械臂 | 理想 SRS 7-DOF，论文表 2/3 | 偏置 SRS-like Flexiv Rizon 4s | user |
| 基座质量 | `500 kg` | `500 kg` | paper + user |
| 基座对角惯量 | `[50,50,50] kg·m²` | `[20.8333,20.8333,20.8333] kg·m²` | user |
| 初始关节角 | `[0,45°,0,90°,0,45°,0]` | `[0,-40°,0,90°,0,40°,0]` | user home |
| 目标主场景自旋 | `5°/s` | `5°/s` | paper |
| 抓捕点偏置半径 | `0.25 m` | 先试 `0.25 m`，不可达时另做 `0.15 m` 缩放版 | paper / calibrated |
| PSO | 20 粒子、1000 代、`w=0.9,c1=1.5,c2=2` | 原样保留并增加 10 种子 | paper |
| A级末端期望力 | `0` | `0` | paper |
| B级目标质量/惯量与阻抗 | 未公开 | 显式配置并做敏感性 | calibrated |

## 3. 论文方法到用户模型的数学映射

### 3.1 自由漂浮反作用映射与广义雅可比

令基座速度为 `ν_b ∈ R^6`，关节速度为 `q̇ ∈ R^7`。把 MuJoCo 质量矩阵按自由基座与机械臂分块：

```text
M(q) = [M_bb  M_bq]
       [M_qb  M_qq]
```

接触前、零初始动量时，基座动量约束给出：

```text
M_bb ν_b + M_bq q̇ = 0
ν_b = A(q) q̇,  A(q) = -M_bb^{-1} M_bq
T(q) = [A(q); I_7]
```

若 MuJoCo 末端 site 的完整雅可比为 `J_site ∈ R^{6×13}`，则：

```text
J_g(q) = J_site(q) T(q) ∈ R^{6×7}
ẋ_e = J_g q̇
```

这与论文式 (10)–(14) 等价，但直接使用引擎质量矩阵可避免手工重写 Flexiv 每个偏置连杆的惯量变换。实现上保留现有 `reaction_velocity_map()` 和 `task_jacobians()`，新增独立单元测试：

- `||M_bb A + M_bq||` 小于 `1e-8`（离线双精度测试）；
- `J_g q̇` 与一个小步长自由漂浮前向差分一致；
- 与 Simscape 中已有 `J_bm`/`J_g` 在相同状态下交叉比较，记录坐标系变换后误差。

### 3.2 Flexiv 的型Ⅱ变量（连续臂形角）

论文式 (18)–(23) 针对理想 SRS 机械臂。用户的 Rizon 4s 为带几何偏置的 SRS-like 结构，不能直接照搬解析式。采用如下数值稳健定义：

- 所有点先变换到基座坐标系，排除基座运动对“臂形”的污染；
- `S = joint2` 锚点、`E = joint4` 锚点、`W = joint6` 锚点；
- 当前臂平面法向 `n = normalize((E-S) × (W-S))`；
- 以 `w = normalize(W-S)` 为旋转轴；只在初始化时选择与 `w(0)` 最不平行的基座坐标轴并固定为参考矢量，把它投影到 `w` 的法平面形成 `n_ref`。运行中不得切换参考轴；若投影退化则报告奇异；
- `ψ = atan2(w·(n_ref×n), n_ref·n)`，并沿时间连续 unwrap 到 `R` 而不是每步截断在 `[-π,π]`；
- `J_ψ = ∂ψ/∂q ∈ R^{1×7}` 用中心差分求取，步长从 `1e-6 rad` 开始，并做 `1e-5/1e-6/1e-7` 收敛检查。

退化条件 `||(E-S)×(W-S)|| < ε` 或参考投影过小时必须标记为臂形奇异，不允许优化器穿越而不加惩罚。若该定义在 Flexiv 工作区频繁退化，备选定义是对 `null(J_g)` 做符号连续化后的积分型零空间坐标；备选结果不能与几何臂形角混称。

### 3.3 型Ⅰ变量（基座状态）

不使用论文的 ZXZ 欧拉角作为内部状态，以避免奇异性。MuJoCo 中基座姿态保留四元数；姿态扰动采用：

```text
δ_R(t) = ||Log(R_b(0)^T R_b(t))||_2
```

同时记录论文图 9 可对照的 `v_b(t)` 和 `ω_b(t)` 三轴曲线。论文文本把“位姿扰动”以 `mm/s` 和 `(°)/s` 报告，实际对应速度曲线；本项目必须把“速度峰值、姿态角漂移、时间积分”分开命名，不能混用。

## 4. 目标与场景设计

### 4.1 A级运动学翻滚目标

新增一个可精确传播的目标状态：

- 初始自旋速度沿目标主轴设为 `5°/s`，对应论文主场景；
- A级使用 prescribed/mocap 刚体，按无外力矩欧拉方程或解析主轴自旋传播；
- 目标质心与抓捕点均有独立 site，抓捕点体坐标偏置优先尝试论文的 `0.25 m`；
- 坐标系方向应适配 Flexiv 的实际可达方向。首个标定建议把目标质心放在 `[0.65, 0.14, 0.55] m` 附近、抓捕偏置设为 `[0, -0.25, 0] m`，使初始抓捕点落在现有已验证工作区 `[0.65, -0.11, 0.55] m` 附近；最终位置必须经过可达性扫描后冻结。

若 `0.25 m` 旋转半径导致部分轨迹不可达，先报告失败，再把半径缩放到 `0.15 m` 做“几何缩放适配实验”。缩放实验不能与完全忠实的场景混在同一表格里。

### 4.2 B级物理目标

新增带 `freejoint` 的目标本体、抓捕接口和接触几何。目标质量/惯量论文未给出，因此：

- 质量、主惯量、接口尺寸、摩擦和接触顺应性全部作为显式配置；
- 主结果必须报告这些参数，另做质量/惯量 `±20%` 敏感性；
- 接触前按初始角速度自由运动，发生接触后完全由 MuJoCo 动力学演化，禁止继续用 mocap 强行覆盖位姿；
- 只开启夹具–抓捕接口的接触；自碰撞/障碍物仍由有符号距离约束处理。必须删除当前 `model.geom_contype[:] = 0` 的全局禁用逻辑在 B 级路径中的使用。

## 5. 捕获时间–臂形联合优化

### 5.1 决策变量

```text
z = [T_c, ψ_f]
T_c ∈ [T_min, T_max], ψ_f ∈ 可行连续臂形区间
```

初始搜索窗建议为 `T_c ∈ [8, 25] s`；先通过稀疏可达性扫描缩紧。`ψ_f` 不直接假设完整 `[-π,π]` 可行，而从末端捕获位姿的多解 IK/零空间扫描得到实际可行区间。

### 5.2 轨迹生成

对位置、SO(3) 姿态和臂形分别使用同一个五次时间标度：

```text
τ=t/T_c
s(τ)=10τ³-15τ⁴+6τ⁵
```

- 位置：`p_d(t)=p_0+s(p_f-p_0)`；
- 姿态：`R_d(t)=R_0 Exp(s Log(R_0^T R_f))`；
- 臂形：`ψ_d(t)=ψ_0+s(ψ_f-ψ_0)`；
- 起止速度、加速度为零；捕获末端姿态由目标抓捕接口姿态与固定抓取变换共同确定。

### 5.3 目标函数与约束

论文目标为 `G=w1 δo²+w2 α²`。为避免量纲和权重掩盖结论，采用归一化实现：

```text
C_base = (max_t ||ω_b(t)|| / ω_ref)^2
α = acos(clamp(r_hat(T_c) · vrel_hat(T_c), -1, 1))
C_align = (α / α_ref)^2
G = w1 C_base + w2 C_align + P_constraints
```

其中 `r` 是机器人系统质心到目标抓捕点的向量；A级按论文假设使末端终端速度为零，因此 `v_rel` 为抓捕点惯性速度。这里保留论文 `argmax cosα` 的有向夹角语义，不把反平行误当成同样优；同时输出 `||r×v_rel||` 对应的冲量力矩代理。`P_constraints` 包含：

- 终端位置/姿态不可达；
- 关节位置、速度、加速度或力矩越限；
- 最小自碰/障碍距离小于 `0.04 m`；
- `J_g` 或臂形定义接近奇异；
- 末端终端速度不满足捕获边界条件。

主权重先采用论文的相对比例 `w1:w2 = 1:2`，但由于归一化方式为本项目新增，必须做 `1:1、1:2、2:1` 敏感性。论文 PSO 设置作为第一版忠实配置：群体 20、最大 1000 代、惯性权重 0.9、速度上下限为搜索区间的 `±10%`、个体/全局学习因子 1.5/2、相邻最优值阈值 `1e-5`。至少运行 10 个固定随机种子并报告最优值离散度，不能只展示一次最漂亮的收敛曲线。

### 5.4 优化计算策略

- 内层候选评估使用 50 Hz 自由漂浮运动学/HQP rollout，不在 20,000 个候选上跑完整 500 Hz 接触动力学；
- 对每个 PSO 种子的前 5 个候选再做 500 Hz MuJoCo 动力学验证；
- 最终 `T_c*、ψ_f*` 必须在完整动力学中重新计算真实基座扰动和末端误差；若排序变化，使用完整动力学结果选择最终候选并记录差异。

## 6. 多优先级位-型控制器

现有控制器是单个加权 QP，不能直接宣称已经实现论文的零空间多优先级。新增 `FPMFCHQP`，保留 50 Hz 任务层和 500 Hz 力矩层：

### Level 1：末端捕获任务（硬优先级）

```text
v_e* = ẋ_d + K_x e_x
J_g q̇ ≈ v_e*
```

将末端位置/姿态残差限制为硬容差或先求 Level-1 最优值并锁定，避免低优先级通过放宽末端跟踪换取更小基座扰动。

### Level 2：型与反作用（末端任务零空间）

```text
min  ||W_ψ (J_ψ q̇ - ψ̇_d*)||²
   + ||W_b (A(q)q̇ - ν_b,d)||²
   + λ||q̇-q̇_prev||²
```

A级默认 `ν_b,d=0`。由于 7 关节在 6D 末端任务后通常只剩 1 个零空间自由度，臂形跟踪和六维基座反作用不能同时独立精确满足；它们必须在同一低优先级层显式权衡，并报告二者残差。实现可采用严格两级 HQP，也可用解析主任务解：

```text
q̇ = J_g# v_e* + N_g z
N_g = I - J_g#J_g
z = argmin 低优先级目标与约束
```

与论文式 (31) 相比，形状命令应使用残差 `ψ̇_d - J_ψ J_g#v_e*`，否则主任务本身引起的臂形变化会被重复计算。

### 硬约束

沿用现有：关节位置、速度、加速度、有符号距离 barrier；增加：力矩可行性近似、臂形奇异 barrier、捕获接口视线/接近方向约束。低层继续使用现有逆动力学力矩伺服，但记录饱和率，确保优化收益不是由力矩裁剪造成。

## 7. B级力/位混合与接触状态机

### 7.1 状态机

```text
APPROACH -> CONTACT_TRANSIENT -> FORCE_HOLD -> (可选) DETUMBLE
```

- `APPROACH`：运行 A 级位-型控制；
- `CONTACT_TRANSIENT`：距离与法向力双阈值触发，期望力平滑上升；
- `FORCE_HOLD`：法向力控制，切向位置和姿态保持；
- `DETUMBLE`：论文未验证，单独作为后续扩展，不纳入首轮成功门槛。

### 7.2 接触力与阻抗滤波

从所有“夹具–目标接口”接触调用 `mj_contactForce`，变换并合成为末端 6D wrench，检查符号和力矩参考点。沿接触法向采用论文式 (32) 的位置型阻抗/导纳修正：

```text
M_d Δẍ + B_d Δẋ + K_d Δx = F_d - F_e
```

以 500 Hz 半隐式或 Tustin 离散，法向以外维度由选择矩阵保持位置控制。必须添加 `Δx`、`Δẋ` 限幅和接触丢失复位。论文式 (34) 的文本提取存在位移/速度量纲混合，代码中必须使用 `Δẋ` 修正速度命令，或先将 `Δx` 叠加到位置参考后再由位置环生成速度，不能把米直接加到米/秒。

第一轮工程初值可取临界阻尼关系 `B_d=2ζ√(M_dK_d), ζ≈1`，期望法向力从 `2–5 N` 平滑爬升；这些都属于本项目标定值，必须通过无接触阶跃、刚性墙接触和目标质量敏感性逐级调参，不能标为论文参数。

## 8. Implementation Blueprint

建议新增而不破坏现有迁移基线：

| 模块 | 责任 | 主要接口 |
|---|---|---|
| `v6_mujoco/fpmfc/target.py` | A/B 两类翻滚目标传播、抓捕点 pose/twist | `sample(t)`, `initialize_free_body()` |
| `v6_mujoco/fpmfc/shape.py` | S–E–W 臂形角、unwrap、`J_ψ`、奇异检测 | `arm_angle(data)`, `arm_angle_jacobian(data)` |
| `v6_mujoco/fpmfc/dynamics.py` | `A(q)`、`J_g`、SO(3) 基座扰动、Simscape 对照 | `reaction_map()`, `generalized_jacobian()` |
| `v6_mujoco/fpmfc/trajectory.py` | 五次位置/姿态/臂形轨迹 | `sample(t, T_c)` |
| `v6_mujoco/fpmfc/optimizer.py` | PSO、约束惩罚、候选缓存与种子复现 | `optimize_capture(config)` |
| `v6_mujoco/fpmfc/controller.py` | 严格两级 HQP 与 B 级选择矩阵 | `solve_precontact()`, `solve_contact()` |
| `v6_mujoco/fpmfc/contact.py` | 接触 wrench、阻抗滤波、状态机 | `wrench()`, `admittance_step()` |
| `v6_mujoco/fpmfc/run_capture.py` | 场景运行、日志、检查点和 CLI | `python -m ...` |
| `v6_mujoco/fpmfc/metrics.py` | 论文指标、工程指标与对照汇总 | `summarize_run()` |
| `tests/test_fpmfc_*.py` | 数学、优化、控制、接触回归测试 | `unittest` |

原 `v6_mujoco/hierarchical_qp.py` 和 `run.py` 保持原行为，用作迁移前后回归基准。

## 9. Claim Map

| Claim | Why It Matters | Minimum Convincing Evidence | Linked Blocks |
|---|---|---|---|
| C1：位-型联合优化在 Flexiv 上降低基座扰动 | 是论文核心机制 | 同一目标/时刻/轨迹约束下，优化臂形优于 `ψ=0, π/2`；强基线还允许固定臂形各自重优化时刻 | B1, B2, B3 |
| C2：力控制扩展降低接触冲击 | 证明框架可落到真实接触 | 与刚性位置控制相比，峰值力、冲量、基座角冲量下降且捕获不丢失 | B4, B5 |
| Anti-claim：收益来自放宽任务或不安全运动 | 保证公平性 | 所有方法共享末端误差门槛、时限、目标初态、关节/碰撞/力矩约束 | 全部 |

## 10. Experiment Blocks

### Block B1：数学与仿真一致性

- **Claim tested**：Flexiv 的 `A(q)`、`J_g` 和 `J_ψ` 实现正确。
- **Compared systems**：MuJoCo 质量矩阵结果、有限差分、原 Simscape MATLAB Function。
- **Metrics**：动量方程残差、雅可比相对误差、臂形角导数误差、SO(3) 积分误差。
- **Success criterion**：解析/引擎矩阵残差 `≤1e-8`；有限差分相对误差 `≤1e-4`；跨平台误差若超限必须能由坐标/惯量差异解释。
- **Priority**：MUST-RUN。

### Block B2：论文三组主对照

- **Systems**：`FPMFC optimized`、`ψ=0`、`ψ=π/2`。
- **Fair isolation**：三组固定同一个 `T_c*`，只比较臂形效果。
- **Strong baseline**：固定 `ψ` 的两组各自重新优化 `T_c`，排除“只是选了更有利时间”的反驳。
- **Metrics**：`max||ω_b||`、`RMS||ω_b||`、`max δ_R`、末端位置/姿态误差、最小距离、力矩峰值/饱和率、动量残差。
- **Success criterion**：优化方法在主扰动指标上优于两个固定臂形；鲁棒场景中相对最佳固定臂形的中位改善建议达到 `≥5%`，同时末端位置最大误差 `≤1 mm`、姿态最大误差 `≤0.2°`、最小距离 `≥0.04 m`、无力矩持续饱和。
- **Priority**：MUST-RUN。

### Block B3：机制隔离与稳健性

- **Ablations**：去掉臂形目标、去掉基座反作用目标、现有单层加权 QP、严格两级 HQP。
- **Robustness**：目标自旋 `2/5/8°/s`，初始位姿扰动 `±2 cm/±2°`，机械臂/基座惯量 `±10%`，PSO 10 种子。
- **Metrics**：与 B2 相同，加 PSO 最优值方差和不可行率。
- **Success criterion**：主结论不是单一种子、单一自旋速度或单一权重产生；严格优先级不以末端误差换收益。
- **Priority**：MUST-RUN（5°/s、权重消融）；其余 NICE-TO-HAVE。

### Block B4：接触力基线与阻抗扩展

- **Systems**：刚性位置跟踪、法向位置型阻抗/导纳、无臂形优化的阻抗控制。
- **Metrics**：峰值法向力、力冲量、稳态力 RMSE、最大穿透、接触丢失次数、基座角冲量、捕获后目标相对运动。
- **Success criterion**：与刚性位置控制相比，阻抗版本峰值力和角冲量均下降；建议目标为峰值力下降 `≥30%`、稳态力 RMSE `≤10% F_d`、最大穿透 `≤2 mm`、无持续接触丢失。
- **Priority**：A级后 MUST-RUN，属于扩展结论。

### Block B5：捕获后稳定/消旋

- **Claim tested**：完整任务是否可从接触过渡到稳定保持。
- **Metrics**：目标角速度衰减、相对姿态误差、夹具力/力矩、系统总动量守恒。
- **Success criterion**：需在选定抓捕机构与保持约束后另行冻结；当前不作为论文复现完成条件。
- **Priority**：NICE-TO-HAVE。

## 11. Run Order and Milestones

| Milestone | Goal | Runs | Decision Gate | Cost | Risk |
|---|---|---|---|---|---|
| M0 | 冻结模型契约与论文参数表 | R000–R003 | 质量矩阵、坐标系、臂形导数测试全过 | <1 CPU-hour | 坐标系符号不一致 |
| M1 | 目标与轨迹 sanity | R010–R013 | 目标 5°/s 传播、终端零速、工作区可达 | 1–2 CPU-hours | 0.25 m 抓捕半径不可达 |
| M2 | PSO 与候选验证 | R020–R023 | 10 种子收敛，完整动力学复核排序稳定 | 2–6 CPU-hours | 内层 rollout 成本高、局部最优 |
| M3 | A级主结果 | R030–R038 | B2 成功标准满足，主结论可复查 | 1–3 CPU-hours | 权重尺度或公平性错误 |
| M4 | A级稳健性/消融 | R040–R049 | C1 在核心扰动下保持 | 3–8 CPU-hours | 场景组合膨胀 |
| M5 | B级接触扩展 | R100–R109 | 力符号、阻抗阶跃、接触稳定性全过 | 4–12 CPU-hours | 接触刚度/采样导致振荡 |
| M6 | 可选消旋 | R200+ | 先冻结抓捕约束与目标惯量 | 待定 | 超出原论文证据范围 |

不需要 GPU。首轮只跑 MUST-RUN；NICE-TO-HAVE 不得阻塞 A级结论。

## 12. Planned Setup and Commands

- **Language/Framework**：Python（已验证环境 `C:\Users\admin\.conda\envs\rltoorch\python.exe`）、MuJoCo、NumPy、SciPy。
- **Working Directory**：`E:\桌面\力位形混合控制`
- **Dependencies**：见 `requirements.txt`。
- **Planned entry commands**（实现后启用）：

```powershell
C:\Users\admin\.conda\envs\rltoorch\python.exe -m v6_mujoco.fpmfc.optimizer --config configs/fpmfc_paper.yaml --seed 0 --output output/fpmfc/optimization/pso_seed_00.json
C:\Users\admin\.conda\envs\rltoorch\python.exe -m v6_mujoco.fpmfc.run_capture --config configs/fpmfc_paper.yaml --capture-time <T> --arm-angle <psi> --output-dir output/fpmfc/precontact/<candidate>
C:\Users\admin\.conda\envs\rltoorch\python.exe -m v6_mujoco.fpmfc.validate_capture --output-dir output/fpmfc/precontact/<candidate>
C:\Users\admin\.conda\envs\rltoorch\python.exe -m unittest discover -s tests -p "test_fpmfc_*.py" -v
```

前三个入口已经创建并通过 smoke 验证。接触扩展仍是 B 级后续工作，不属于论文原仿真的完成门槛。

## 13. Expected Outputs

| Output | Planned Path | Format | Success Criterion |
|---|---|---|---|
| 冻结参数 | `configs/fpmfc_paper.yaml` | YAML | 每个参数标注 `paper/user/calibrated` 来源 |
| PSO 结果 | `output/fpmfc/optimization/*.json` | JSON | 种子、边界、最优值、约束、版本齐全 |
| 逐步轨迹 | `output/fpmfc/traces/*.npz` | NPZ | 可独立回放，含目标/基座/关节/末端/力/约束 |
| A级汇总 | `output/fpmfc/precontact_metrics.json` | JSON | 三组主对照和强基线完整 |
| B级汇总 | `output/fpmfc/contact_metrics.json` | JSON | 接触峰值、冲量、稳态误差和穿透完整 |
| 图表 | `output/fpmfc/figures/*.png` | PNG | PSO、关节速度、基座扰动、对照曲线可读 |
| 复现报告 | `paper/FPMFC_REPRODUCTION_REPORT.md` | Markdown | 明确 A/B 证据边界，无伪造结果 |

## 14. Monitoring Configuration

- **Per-run timeout**：A级单场景 30 min；PSO 总任务 8 h；B级单场景 60 min。
- **Monitor files**：优化 JSONL 进度、逐场景 trace、汇总 JSON。
- **Process checks**：进程存活、输出增长、QP 不可行率、NaN、力矩饱和率、接触穿透、动量突变。
- **Stop/go gates**：任一数学单元测试失败时不得进入 PSO；A级主结果失败时不得用 B级接触结果掩盖；接触力符号/参考点测试失败时不得调阻抗参数。

## 15. Risks and Mitigations

- **论文信息不足**：目标质量/惯量、阻抗参数、选择矩阵和若干实现细节未公开。A级只复现有证据部分；B级参数全部显式标为本项目标定。
- **机器人不同导致数值不可比**：使用相对改善、无量纲归一化和同场景公平对照；不把 `15.6 s/0.2686 rad` 当验收阈值。
- **Flexiv 不是理想无偏置 SRS**：使用基座系 S–E–W 几何量和有限差分 `J_ψ`，并建立奇异性门槛。
- **单层权重伪装优先级**：新控制器使用两级锁定或末端硬容差；现有加权 QP 只作为消融基线。
- **PSO 通过更慢轨迹降低扰动**：增加固定 `T_c*` 的臂形隔离对照，以及固定臂形各自重优化时间的强基线。
- **接触阶段破坏零动量假设**：接触后使用完整联合动力学，不再使用接触前的零动量消元作为精确模型；总系统（机器人+目标）动量单独核验。
- **数值接触调参过拟合**：先做刚性墙阶跃，再做自由目标；对接触 stiffness/damping、步长和目标惯量做敏感性。

## 16. Final Checklist

- [x] 论文真实验证范围与框架扩展范围已分开
- [x] 论文公式已映射到用户的 7-DOF 自由漂浮 MuJoCo 模型
- [x] 当前可复用代码和关键缺口已识别
- [x] A(q)、J_g、J_ψ 的 MuJoCo/有限差分数学测试全部通过
- [x] 目标场景与参数来源表冻结
- [ ] PSO 10 种子和完整动力学候选复核完成
- [ ] 论文三组主对照及强基线完成
- [ ] 末端精度、安全性、力矩与动量约束同时达标
- [x] B级接触结果与 A级论文复现结果分开定义；B级尚未执行
- [ ] 所有主图可由保存的 trace 独立重建

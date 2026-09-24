# 基于 Flexiv Rizon 4s 的力-位-型融合控制方法复现与场景验证

## Material Passport

- Origin Skill: experiment-agent + experiment-plan
- Origin Mode: plan
- Origin Date: 2026-09-24T11:06:29+08:00
- Verification Status: PARTIALLY_VERIFIED
- Version Label: code_plan_v8

## Experiment Overview

- **Title**：基于原方法思想的自由漂浮 Flexiv Rizon 4s 力-位-型控制实现与用户场景验证
- **Objective**：保留用户现有 500 kg 自由基座、7-DOF Flexiv Rizon 4s、目标几何和 MuJoCo 数值设置，重新实现论文的位—型联合建模、捕获时间/臂形联合优化、零空间控制、基座扰动抑制与力—位—型框架，并在当前用户场景中验证方法机制与适用边界；不追求原论文数值复现。
- **Primary hypothesis**：在当前 Flexiv 捕获任务和统一安全/精度门槛下，捕获时间—臂形联合优化与严格位—型层级能够稳定产生动力学可执行的捕获轨迹；若需性能型固定臂形比较，只与预先确定的可行固定臂形作公平对照。
- **Supporting hypothesis**：加入真实接触后，沿接触法向的基于位置的阻抗/导纳修正能比刚性位置跟踪降低峰值接触力与基座角冲量。
- **Anti-claim**：改善不能来自放宽末端误差、延长运动时间、越过关节/碰撞约束、改变目标初态或人为选择更有利的评价时间窗。
- **Type**：simulation

## 1. 复现边界与结论

本项目分成两个证据层级，但二者共同服务于“方法复现 + 用户场景验证”：

1. **A级：接触前方法复现与场景验证**。在用户的 Flexiv 模型、目标尺寸与初态、抓捕点、质量惯量、约束阈值和 2 ms RK4 设置下，验证广义雅可比、捕获时间—臂形联合优化、严格零空间位—型控制和基座扰动指标。`ψ=0` 与 `ψ=π/2` 仅作为补充构型诊断，不是复现成败门槛。
2. **B级：真实接触下的力—位—型场景验证**。增加自由目标、抓捕接口、接触力估计、阻抗/导纳修正与稳定保持。该层级验证框架在用户场景中的扩展能力，不冒充原论文已经报告的接触结果。

N071/N072 已证明原论文的两个固定角在当前 Flexiv 几何下落入不利构型；不再补做完整 2 ms 穷举，也不为使其可行而修改模型。若论文需要性能型固定臂形对照，应先用不读取动态扰动结果的 IK/零空间扫描冻结一到两个可行角度，再分别公平优化捕获时间。

本项目不以论文的最优时间 `15.6 s`、最优臂形 `0.2686 rad`、5.22% 或 25.27% 等数值为验收目标。最终表述统一为“基于原方法思想，在自定义机器人与任务场景中重新实现并验证”。

## 2. 输入与当前模型审计

| Input | Path | Description |
|---|---|---|
| 参考论文 | `reference_papers/自由漂浮空间机器人捕获翻滚...标的力-位-型融合控制方法_梁斌.pdf` | 方法、公式、PSO 参数与对照实验定义 |
| MuJoCo 模型 | `models/flexiv_rizon4s_scene.xml` | 500 kg 自由基座 + 7-DOF Rizon 4s，零重力，2 ms 步长，RK4 积分 |
| 模型契约 | `v6_mujoco/model.py` | `nq=14, nv=13, nu=7`，关节/力矩/速度/积分器限值与模型加载 |
| 现有任务控制 | `v6_mujoco/hierarchical_qp.py` | 50 Hz 反作用感知速度 QP、碰撞距离约束 |
| 现有低层执行 | `v6_mujoco/run.py` | 500 Hz 逆动力学关节力矩伺服、日志与动量计算 |
| Simscape 对照 | `diagnostics/source_model_inventory.json` | 原模型广义雅可比、质心与 `J_bm` 的 MATLAB Function 源码 |

当前模型的可复用事实：

- 自由基座质量为 `500 kg`，但对角惯量为 `20.8333 kg·m²`；论文基座惯量为 `50 kg·m²`。保留用户模型值，不替换成论文值。
- 机器人总质量为 `520.8 kg`；机械臂质量、几何偏置和关节限位均按用户模型保留。
- 当前 `reaction_velocity_map()` 已从 MuJoCo 完整质量矩阵计算自由基座反作用映射；`task_jacobians()` 已形成自由漂浮广义末端雅可比。
- 2 ms 动力学积分器冻结为 `RK4`。旧 N055 的 `implicitfast` 身份仅 1/10 seeds 联合合格，45 条失败候选均只触发动量门；同一 seed-1 候选的对照重放把最大动量差由 `5.576878e-2` 降至 `9.408724e-8`，同时保持 11/11 门槛通过。因此不放宽 `maximum_momentum_delta=0.05`，而把积分器作为模型身份的一部分。
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
| 抓捕点偏置半径 | `0.25 m` | 当前新场景固定为 `0.15 m` | user / adaptive；旧 `0.25 m` 结果仅作历史追溯 |
| 数值积分器 | 未公开 | MuJoCo `RK4`，2 ms；旧 `implicitfast` 结果只作失败诊断 | calibrated |
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
- 目标质心与抓捕点均有独立 site；当前用户场景已冻结为目标中心 `[0.95, 0.14, 0.95] m`、边长 `0.30 m`、抓捕点体坐标偏置 `[0, -0.15, 0] m`，世界系初始抓捕点为 `[0.80, 0.14, 0.95] m`；
- 坐标系方向按 Flexiv 的实际可达方向与端面相向约束设置，终态要求末端接触面外法向与目标抓捕面外法向反向；
- 论文 `0.25 m` 偏置的旧 R 系列结果只用于历史追溯。当前 N 系列属于机制/相对结论的适配复现，不得与论文原场景数值混在同一统计结论中。

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
- 运动学规划最小自碰/障碍距离小于 `0.045 m`；500 Hz 动力学验收仍使用 `0.04 m`；
- `J_g` 或臂形定义接近奇异；
- 末端终端速度不满足捕获边界条件。

论文只给出 `G=w1 δo²+w2 α²` 并说明权重可按重要性设置，没有给出 `w1:w2` 的数值。主设置暂采用本项目标定比例 `w1:w2 = 1:2`，不得表述为论文公开参数；由于归一化和数值比例均为本项目新增，必须做 `1:1、1:2、2:1` 敏感性。论文 PSO 设置作为第一版忠实配置：群体 20、最大 1000 代、惯性权重 0.9、速度上下限为搜索区间的 `±10%`、个体/全局学习因子 1.5/2、相邻最优值阈值 `1e-5`。至少运行 10 个固定随机种子并报告最优值离散度，不能只展示一次最漂亮的收敛曲线。

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

沿用现有：关节位置、速度、加速度、有符号距离 barrier；增加：力矩可行性近似、臂形奇异 barrier、捕获接口视线/接近方向约束。低层继续使用现有逆动力学力矩伺服，但记录饱和率，确保优化收益不是由力矩裁剪造成。当前 N060 冻结 45 mm 规划/40 mm 动力学间距双阈值，并将终端臂形规划门槛设为 0.75°、动力学验收保持 1°；PSO 的个体最优、全局最优、社会引导和 top-5 均采用“规划可行优先、分区内目标值优先”，避免微小越界候选凭平方罚项压过可行解。N060M 进一步把 2 ms 动力学积分器冻结为 RK4，模型、配置与运行时契约均须显式一致；动量门仍为 `0.05`，不得以积分误差为由放宽。

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
| C1：位—型联合建模与控制在当前 Flexiv 场景中产生可执行捕获轨迹 | 是方法复现的核心 | 数学契约通过；10-seed 联合优化和完整动力学验收稳定；删除臂形目标会破坏冻结轨迹的任务/臂形可行性 | B1, B2, B3 |
| C2：力控制扩展降低接触冲击 | 证明框架可落到真实接触 | 与刚性位置控制相比，峰值力、冲量、明确定义的基座扰动指标下降且捕获不丢失 | B4, B5；N102–N104 仅部分支持，完整主张未通过 |
| Anti-claim：在当前场景中已经复现原论文数值，或 `ψ=0,π/2` 必须可行 | 防止越界表述 | 明确使用用户系统参数；N071/N072 只作补充诊断；只在双方合格时计算性能差 | 全部 |

## 10. Experiment Blocks

### Block B1：数学与仿真一致性

- **Claim tested**：Flexiv 的 `A(q)`、`J_g` 和 `J_ψ` 实现正确。
- **Compared systems**：MuJoCo 质量矩阵结果、有限差分、原 Simscape MATLAB Function。
- **Metrics**：动量方程残差、雅可比相对误差、臂形角导数误差、SO(3) 积分误差。
- **Success criterion**：解析/引擎矩阵残差 `≤1e-8`；有限差分相对误差 `≤1e-4`；跨平台误差若超限必须能由坐标/惯量差异解释。
- **Priority**：MUST-RUN。

### Block B2：接触前主方法与固定构型诊断

- **Main system**：`FPMFC optimized`，以 N055R 的 10-seed 联合优化和 top-5 完整动力学复核为主证据。
- **Supplementary diagnostics**：N070/N071/N072 的 `ψ=0` 与 `ψ=π/2` 只用于说明当前 Flexiv 场景中的不利构型和可行分支选择，不计算相对性能改善率，不要求完整 2 ms 穷举。
- **Optional performance baseline**：只有论文确需固定臂形性能比较时，才由预注册的 IK/零空间可行性扫描冻结负、正分支各一个固定角，并各自重新优化捕获时间；基线选择不得读取后续动态扰动结果。
- **Metrics**：11 项动力学门槛、`max||ω_b||`、`RMS||ω_b||`、`max δ_R`、最小距离、力矩峰值/饱和率和动量残差。
- **Success criterion**：主方法跨种子稳定通过规划、动力学和独立重放；任何性能差只在双方均通过共同门槛时计算。
- **Priority**：主方法 MUST-RUN（已完成）；0°/90° 为 SUPPLEMENTARY；可行固定臂形性能对照为 OPTIONAL。

### Block B3：机制隔离与稳健性

- **Ablations**：去掉臂形目标、去掉基座反作用目标；现有单层加权 QP 仅在论文需要隔离“严格优先级”时补充。
- **Robustness**：目标自旋 `2/5/8°/s`，初始位姿扰动 `±2 cm/±2°`，机械臂/基座惯量 `±10%`，PSO 10 种子。
- **Metrics**：与 B2 相同，加 PSO 最优值方差和不可行率。
- **Current result**：N073 在 10 条由完整方法预先规划并冻结的轨迹上完成 30/30 次运行。完整方法 10/10 合格；去臂形 0/10 合格；去基座反作用 10/10 合格，且三项基座扰动指标与完整方法无显著差异。因此现有证据支持臂形在线目标的必要性，但不支持在当前权重/轨迹上宣称在线基座反作用项带来可测收益。
- **Success criterion**：主结论不是单一种子或单一权重产生；严格优先级不以放宽末端误差换收益；任何未获支持的机制主张必须收窄或另做专门敏感性实验。
- **Priority**：MUST-RUN（5°/s、权重消融）；其余 NICE-TO-HAVE。

### Block B4：接触力基线与阻抗扩展

- **Systems**：刚性位置跟踪、法向位置型阻抗/导纳、无臂形优化的阻抗控制。
- **Metrics**：峰值法向力、力冲量、稳态力 RMSE、最大穿透、接触丢失次数、基座角冲量、捕获后目标相对运动。
- **Success criterion**：与刚性位置控制相比，阻抗版本峰值力和角冲量均下降；建议目标为峰值力下降 `≥30%`、稳态力 RMSE `≤10% F_d`、最大穿透 `≤2 mm`、无持续接触丢失。
- **Current result**：N102–N104 在相同 seed-00 交接状态、物理目标、接触接口与 1 s 瞬态下完成且独立重放通过。N103 相对 N102 的力冲量、机器人侧接触角冲量代理和基座峰值角速度分别下降 `18.42%/39.47%/18.51%`，但峰值力增加 `27.60%`，稳态力 RMSE 为 `41.28% F_d`；公共安全/有界丢失门通过，力跟踪门失败。N104 同样未过力门。C2 复合主张不成立，只保留场景内权衡结论。
- **Priority**：N073 完成后 MUST-RUN，属于用户场景下的框架扩展结论，不再由 N071/N072 阻塞。

### Block B5：捕获后稳定/消旋

- **Claim tested**：完整任务是否可从接触过渡到稳定保持。
- **Metrics**：目标角速度衰减、相对姿态误差、夹具力/力矩、系统总动量守恒。
- **Success criterion**：需在选定抓捕机构与保持约束后另行冻结；当前不作为论文复现完成条件。
- **Priority**：NICE-TO-HAVE。

## 11. Run Order and Milestones

| Milestone | Goal | Runs | Decision Gate | Cost | Risk |
|---|---|---|---|---|---|
| M0 | 冻结新场景、模型与数学契约 | N000–N010 | 新几何与全部基础测试通过 | 已完成 | 坐标系符号不一致 |
| M1 | 目标、轨迹与平滑控制器 sanity | N051–N054 | 单轨迹 11/11 门槛、独立重放与可视化通过 | 已完成 | jerk/边界冲突 |
| M2 | 当前实现身份的 PSO 与候选验证 | N055P–P3、N055、N060M、N055P4、N055R | RK4 预检至少 1 条通过 11/11 与独立重放；RK4 正式重跑 10/10 seeds 有可审计输出、50/50 候选有结果或明确失败；项目门槛为至少 8/10 seeds 含联合合格候选 | 2–6 CPU-hours | 规划--动力学不一致、积分器身份漂移、局部最优 |
| M3 | 接触前方法验证与控制器消融 | N061、N070–N073 | 主方法、权重敏感性与 N073 审计闭合；固定角诊断不作为门槛 | 已完成 | 权重尺度或公平性错误 |
| M4 | 选择性稳健性/可行固定臂形对照 | N080–N082 或新编号 | 只在论文论证确有需要时运行 | 3–8 CPU-hours | 场景组合膨胀 |
| M5 | 用户场景接触力与力—位—型验证 | N100–N104 | N100/N101 与重放通过；公共接触门通过但 10% 力跟踪门失败 | 已完成，结论 PARTIAL | 接触切换速度不匹配、单边接触无长期保持约束 |
| M6 | 可选消旋 | N200+ | 先冻结抓捕约束与目标惯量 | 待定 | 超出原论文证据范围 |

不需要 GPU。首轮只跑 MUST-RUN；NICE-TO-HAVE 不得阻塞 A级结论。

N055R 已在 RK4 身份下通过 M2 项目门：10/10 seeds 含联合合格候选，48/50 top-5 候选通过全部动力学门，50/50 fresh replay 的 qpos/qvel/法兰位置/间距误差均为 0。N061 三组权重敏感性均为 10/10 seeds、48/50 candidates 联合合格。N070 的 0°/90° 同时间对照仅支持可行性差异；N071/N072 已停止为补充诊断，不再扩展为完整穷举。N073 已完成 30/30 轨迹并通过候选、身份、配置、有限值和确定性重放审计：完整方法 10/10 合格、去臂形 0/10 合格、去基座反作用 10/10 合格。接触前主线至此闭合。N100/N101 预检和 N102–N104 用户场景接触实验也已完成：三组公共安全/接触门与独立重放通过，但两组导纳均未过 10% 稳态力误差门，且 N103 峰值力高于刚性基线。B 级结论因此收窄为场景内冲量/基座扰动权衡；若继续，应新建 N110 live-twist 切换实验，而不是继续在当前轨迹上调参。

## 12. Planned Setup and Commands

- **Language/Framework**：Python（已验证环境 `C:\Users\admin\.conda\envs\rltoorch\python.exe`）、MuJoCo、NumPy、SciPy。
- **Working Directory**：`E:\桌面\力位形混合控制`
- **Dependencies**：见 `requirements.txt`。
- **Planned entry commands**（实现后启用）：

```powershell
C:\Users\admin\.conda\envs\rltoorch\python.exe -m v6_mujoco.fpmfc.optimizer --config configs/fpmfc_paper.yaml --seed 0 --output output/fpmfc/optimization/pso_seed_00.json
C:\Users\admin\.conda\envs\rltoorch\python.exe -m v6_mujoco.fpmfc.run_capture --config configs/fpmfc_paper.yaml --capture-time <T> --arm-angle <psi> --output-dir output/fpmfc/precontact/<candidate>
C:\Users\admin\.conda\envs\rltoorch\python.exe -m v6_mujoco.fpmfc.validate_capture --output-dir output/fpmfc/precontact/<candidate>
C:\Users\admin\.conda\envs\rltoorch\python.exe -m v6_mujoco.fpmfc.run_contact --variant rigid --output-dir output/fpmfc/contact/n102_rigid_authoritative
C:\Users\admin\.conda\envs\rltoorch\python.exe -m v6_mujoco.fpmfc.run_contact --variant admittance --output-dir output/fpmfc/contact/n103_admittance_authoritative
C:\Users\admin\.conda\envs\rltoorch\python.exe -m v6_mujoco.fpmfc.run_contact --variant admittance-no-shape --output-dir output/fpmfc/contact/n104_admittance_no_shape_authoritative
C:\Users\admin\.conda\envs\rltoorch\python.exe -m v6_mujoco.fpmfc.validate_contact --output-dir <run-dir>
C:\Users\admin\.conda\envs\rltoorch\python.exe -m v6_mujoco.fpmfc.contact_comparison
C:\Users\admin\.conda\envs\rltoorch\python.exe -m unittest discover -s tests -p "test_fpmfc_*.py" -v
```

以上入口均已创建并执行。B 级结果不属于论文原仿真的完成门槛，且当前力跟踪主门失败。

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
- **Process checks**：进程存活、输出增长、JSONL `complete` 事件、配置/实现哈希、QP 不可行率、NaN、力矩饱和率、接触穿透、动量突变。
- **Stop/go gates**：任一数学单元测试失败时不得进入 PSO；N055P4 必须在预热后的重复测量中报告任务层 p99 时延并满足 `<20 ms`，且至少一条候选通过全部 11 项动力学门槛和独立重放；配置、实现和模型运行时身份必须在优化、动力学、验证之间闭合。不得用单次主机调度抖动冒充硬实时结论；N055R 少于 8/10 seeds 联合合格时不得进入 N061/N070；固定臂形退化为一维时间搜索且多个 seed 重复落入同一不可行盆地时，应停止随机加种子，改用预注册确定性网格或可行臂形扫描，并明确“不等于连续域不可行证明”；未通过联合验收的固定基线不得用于相对性能改善率；A级主结果失败时不得用 B级接触结果掩盖；接触力符号/参考点测试失败时不得调阻抗参数。

## 15. Risks and Mitigations

- **论文信息不足**：目标质量/惯量、阻抗参数、选择矩阵和若干实现细节未公开。A级只复现有证据部分；B级参数全部显式标为本项目标定。
- **机器人不同导致数值不可比**：使用相对改善、无量纲归一化和同场景公平对照；不把 `15.6 s/0.2686 rad` 当验收阈值。
- **Flexiv 不是理想无偏置 SRS**：使用基座系 S–E–W 几何量和有限差分 `J_ψ`，并建立奇异性门槛。
- **单层权重伪装优先级**：新控制器使用两级锁定或末端硬容差；现有加权 QP 只作为消融基线。
- **PSO 通过更慢轨迹降低扰动**：增加固定 `T_c*` 的臂形隔离对照，以及固定臂形各自重优化时间的强基线。
- **数值积分器破坏动量守恒**：旧 `implicitfast` 正式组已触发该风险；固定 RK4、把积分器写入配置和运行时身份，并以同一候选的双积分器对照和动量门回归测试防止复发。
- **接触阶段破坏零动量假设**：接触后使用完整联合动力学，不再使用接触前的零动量消元作为精确模型；总系统（机器人+目标）动量单独核验。
- **数值接触调参过拟合**：先做刚性墙阶跃，再做自由目标；对接触 stiffness/damping、步长和目标惯量做敏感性。

## 16. Final Checklist

- [x] 论文真实验证范围与框架扩展范围已分开
- [x] 论文公式已映射到用户的 7-DOF 自由漂浮 MuJoCo 模型
- [x] 当前可复用代码和关键缺口已识别
- [x] A(q)、J_g、J_ψ 的 MuJoCo/有限差分数学测试全部通过
- [x] 目标场景与参数来源表冻结
- [x] RK4 身份的 PSO 10 种子和完整动力学候选复核完成；10/10 seeds、48/50 candidates 联合合格并通过独立审计
- [x] N071/N072 已按补充构型诊断收口，不再作为复现成败门槛
- [x] N073 控制器消融 30/30 完成并通过公平性与重放审计
- [x] 主方法末端精度、安全性、力矩与动量约束同时达标
- [x] N100 接触 wrench 符号/参考点与 N101 导纳离散阶跃预检通过
- [x] B级接触结果与 A级论文复现结果分开定义；N102–N104 已执行并如实记录力跟踪门失败
- [ ] 所有主图可由保存的 trace 独立重建

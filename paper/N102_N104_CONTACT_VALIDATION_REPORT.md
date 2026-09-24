# N102–N104 用户场景接触力与力—位—型验证报告

## Material Passport

- Origin Skill: experiment-plan + experiment-result-to-claim
- Origin Mode: run
- Origin Date: 2026-09-24T11:06:29+08:00
- Verification Status: VERIFIED_RESULTS / FAILED_FORCE_TRACKING_GATE
- Version Label: n102_n104_contact_validation_v1

## 1. 结论先行

N102–N104 已在同一物理目标、同一 N073 seed-00 完整控制器终态、同一接触接口和同一 2 ms RK4 动力学下完成，并全部通过独立力矩重放。结果支持一个收窄结论：当前法向导纳相对刚性位置基线降低了法向力冲量、机器人侧接触角冲量代理和基座峰值角速度，但没有降低峰值法向力，也没有达到预注册的稳态力 RMSE `≤10% F_d` 门槛。因此，预注册的复合主张“导纳同时降低峰值力和角冲量并稳定跟踪 3 N”不成立。

这仍属于用户场景中的有效负结果，不影响 A 级“方法复现 + 场景验证”已经闭合的结论，也不能写成原论文接触数值复现。

## 2. 冻结场景与公平性

- 物理目标：`20 kg`，主惯量 `[0.30, 0.30, 0.30] kg·m²`，自由 6-DOF；
- 目标本体：边长 `0.30 m` 的用户立方体；
- 接触板：`100 × 100 × 10 mm`，外表面与 `target_grasp_site` 共面，零附加质量；
- 工具垫：直径 `70 mm`、厚 `10 mm`，接触面相对法兰缩进 `0.2 mm`；
- 接触 margin：`0`；MuJoCo `solref=[0.05,1]`；摩擦 `[0.8,0.005,0.0001]`；
- 接触阶段：`1.0 s`，期望力在前 `0.5 s` 五次平滑爬升到 `3 N`，末 `0.2 s` 计算稳态 RMSE；
- 导纳：`M_d=1 kg, B_d=100 N·s/m, K_d=2500 N/m`，位移/速度限幅 `3 mm / 20 mm·s⁻¹`；
- 公共门槛：峰值力 `≤20 N`、最大活动接触穿透 `≤2 mm`、持续接触丢失 `≤0.1 s`、任务成功率 `100%`、力矩饱和率 `≤0.1%`、总系统动量漂移 `≤0.05`；
- 力跟踪门槛：导纳组稳态 RMSE `≤0.3 N`。

三组均由 `output/fpmfc/precontact/n073_controller_ablation_rk4_formal_planning45/seed_00/full/trace.npz` 初始化。状态通过关节名称搬移；自由目标角速度先由世界系旋转到 MuJoCo 自由关节的本体系。接触阶段开始后没有直接写 `qpos/qvel`。

## 3. 场景审计与预检排除

正式运行前发现并修正了三个场景契约问题：

1. 接触几何曾继承默认 `1 mm` margin，使求解力与几何穿透口径不一致；正式接口显式冻结为零 margin。
2. 配置中的目标接触板最初没有独立进入 XML，随后一版又把板向目标外侧布置；正式模型把接触板向目标内部布置，使外表面与抓捕点共面。
3. 通用 `mj_geomDistance` 在两个有限凸体充分重叠时会返回完全分离所需距离，不等价于表面压缩；正式穿透指标只取活动夹具—接触板 `mjContact.dist` 的负值。

这些预检输出保留在 `output/fpmfc/contact/` 中作为工程诊断，但不进入 N102–N104 权威比较。另一个 2 s 单边恒力保持诊断也被排除：对 `20 kg` 自由目标施加 `3 N`，理想平移量级已达 `0.3 m`，单个单向接触垫不是抓持约束，不能用它验证长期保持。

## 4. 权威结果

| Metric | N102 rigid | N103 admittance + shape | N104 admittance, no shape |
|---|---:|---:|---:|
| Peak normal force (N) | 4.0117 | 5.1189 | 5.8952 |
| Normal-force impulse (N·s) | 2.4132 | 1.9688 | 1.5766 |
| Steady force RMSE (N) | 1.0673* | 1.2383 | 1.5934 |
| Steady RMSE / 3 N | 35.58%* | 41.28% | 53.11% |
| Maximum active-contact penetration (mm) | 0.2856 | 0.2790 | 0.1910 |
| Contact-loss events | 3 | 3 | 6 |
| Maximum sustained loss (ms) | 52 | 86 | 86 |
| Maximum base angular speed (rad/s) | 0.010264 | 0.008364 | 0.011614 |
| Robot contact angular impulse (N·m·s) | 0.22940 | 0.13885 | 0.13313 |
| Common safety/contact gate | PASS | PASS | PASS |
| 10% force-tracking gate | N/A | **FAIL** | **FAIL** |
| Independent torque replay | PASS | PASS | PASS |

`*` 刚性位置基线不以 3 N 力跟踪为控制目标，因此该 RMSE 只作描述，不作为 N102 验收门。

N103 相对 N102：

- 峰值法向力增加 `27.60%`，不支持峰值抑制主张；
- 法向力冲量降低 `18.42%`；
- 机器人侧、关于初始机器人质心的接触角冲量代理降低 `39.47%`；
- 基座峰值角速度降低 `18.51%`；
- 最大活动接触穿透降低 `2.32%`；
- 稳态力 RMSE 为 `1.2383 N = 41.28% F_d`，超过 `0.3 N` 门槛。

N103 相对 N104：带臂形目标的导纳将峰值力降低 `13.17%`、力 RMSE 降低 `22.29%`、基座峰值角速度降低 `27.98%`，且接触丢失事件由 6 次减为 3 次；但其力冲量高 `24.88%`、穿透高 `46.10%`、接触角冲量高 `4.29%`。两组均未通过力跟踪门，故不能据此声称位—型目标在接触阶段整体优于无臂形版本。

## 5. 可写与不可写的论文主张

可以写：

> 在自定义 Flexiv—自由目标的 1 s 单边接触瞬态中，法向导纳与刚性位置基线均满足共同安全和有界接触丢失门槛。导纳把法向力冲量、机器人侧接触角冲量代理和基座峰值角速度分别降低 18.42%、39.47% 和 18.51%，但峰值力增加 27.60%，稳态 3 N 力跟踪 RMSE 为 41.28%，未达到预注册门槛。因此当前实现只支持部分累积冲量/基座扰动缓解，不支持稳定精确力跟踪或峰值力抑制的完整主张。

不可写：

- “复现了原论文的接触力数值结果”；
- “导纳降低了峰值接触力”；
- “力—位—型控制已经稳定跟踪 3 N”；
- “带臂形的接触控制在所有指标上优于无臂形控制”；
- 从 seed-00 单场景外推到目标质量、惯量、初态或接口刚度的普遍鲁棒性。

独立零上下文评审给出的 verdict 为 `partial`、置信度 `high`。评审同时指出：N104 复用了由完整方法规划的接触前终态，所以它只能隔离接触阶段在线臂形项，不能替代各方法独立重规划的端到端消融；当前角冲量字段是机器人侧接触角冲量代理，不应无条件改写成“基座角冲量”。

## 6. 失败机制与下一步

A 级轨迹把末端终端速度设为世界系零，而旋转目标抓捕点在捕获时仍有约 `13 mm/s` 的运动。这一切换条件会把剩余相对速度交给被动接触和导纳吸收，是当前峰值与力振荡的重要来源。继续在同一轨迹上搜索 `M_d/B_d/K_d` 容易变成事后调参；本轮已按“先通过 N101、再通过公共接触门、最后才看 RMSE”的顺序停止。

下一轮若要支持完整 C2，应建立新的、预注册的 N110 系列：

1. 把 A 级终端速度从世界系零改为目标抓捕点 live twist，并重新规划/验证接触前轨迹；
2. 冻结接近速度或能量门槛，再做有限网格的导纳参数选择，使用独立验证场景评估；
3. 若论文需要 `FORCE_HOLD` 而非 1 s 瞬态，先定义双侧夹持或抓捕约束；单个单向接触垫不能对自由目标提供长期静态 3 N；
4. 在机制修正后再运行目标质量/惯量 `±20%` 和多个 N073 合格初态，不用当前 seed-00 结果宣称鲁棒性。
5. 直接记录基座角动量变化或明确定义的基座角冲量，避免用机器人接触角冲量代理替代目标构念。

## 7. 权威产物

- `configs/fpmfc_contact.yaml`
- `models/flexiv_rizon4s_contact_scene.xml`
- `v6_mujoco/fpmfc/contact.py`
- `v6_mujoco/fpmfc/contact_model.py`
- `v6_mujoco/fpmfc/run_contact.py`
- `v6_mujoco/fpmfc/validate_contact.py`
- `v6_mujoco/fpmfc/contact_comparison.py`
- `output/fpmfc/contact/n100_n101_precheck.json`
- `output/fpmfc/contact/n102_rigid_authoritative/`
- `output/fpmfc/contact/n103_admittance_authoritative/`
- `output/fpmfc/contact/n104_admittance_no_shape_authoritative/`
- `output/fpmfc/contact/n102_n104_authoritative_comparison.json`

全量回归测试：`59/59 PASS`。

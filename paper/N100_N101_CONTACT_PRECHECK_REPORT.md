# N100/N101 接触力与法向导纳预检报告

## Material Passport

- Origin Skill: experiment-plan
- Origin Mode: run
- Origin Date: 2026-09-24T11:12:36+08:00
- Verification Status: VERIFIED
- Version Label: n100_n101_contact_precheck_v2

## 1. 配置边界

接触扩展使用独立配置 `configs/fpmfc_contact.yaml`，不修改已冻结的接触前配置或 N055R/N073 实现身份。首轮标定值均属于本项目设置：

- 期望法向力：`3 N`；
- 虚拟质量：`1 kg`；
- 刚度：`2500 N/m`；
- 阻尼比：`1.0`，对应 `B=100 N·s/m`；
- 采样周期：`2 ms`；
- 位移/速度限幅：`3 mm` / `20 mm/s`。

物理目标的 `20 kg` 质量、`[0.30,0.30,0.30] kg·m²` 主惯量和接口几何也已显式记录为 calibrated/user 参数；这些参数已进入 N102–N104 的单场景动力学验证，但尚未执行 `±20%` 敏感性验证，因此不得外推为鲁棒性结论。它们不是论文公开参数。

## 2. N100：接触 wrench 符号与参考点

预检模型为零初速的 `2 kg` 立方体在重力下静置于平面，稳定后形成 4 个接触点。聚合器逐接触调用 `mj_contactForce`，把接触坐标系 wrench 变换到世界坐标，对选中 geom 的方向作符号统一，并把力矩从接触点搬移到指定参考点。

| Check | Result | Gate |
|---|---:|---:|
| 立方体竖直支撑力 | `19.620000000000005 N` | `2×9.81 N` |
| 重力平衡误差 | `3.55e-15 N` | `≤1e-8 N` |
| 作用—反作用合力误差 | `0 N` | `≤1e-10 N` |
| 参考点力矩搬移误差 | `1.40e-15 N·m` | `≤1e-10 N·m` |

四项全部通过。N102–N104 已使用同一接口读取夹具—目标接触的世界系净 wrench，并始终限定 `gripper_contact_pad` / `target_contact_plate` geom 对，没有汇总无关自碰或障碍接触。

## 3. N101：法向导纳离散阶跃

实现采用半隐式离散：

```text
M_d Δẍ + B_d Δẋ + K_d Δx = F_d - F_e
```

在 `F_d-F_e=3 N` 的无接触阶跃下，连续临界阻尼解析解的稳态位移为 `3/2500=1.2 mm`。

| Metric | Result | Gate |
|---|---:|---:|
| 离散最终位移 | `1.2 mm` | 与解析稳态一致 |
| 离散/连续最大绝对误差 | `0.0291 mm` | `≤0.03 mm` |
| 过冲 | `0 mm` | `≤1e-6 mm` |
| 稳态误差 | `1.08e-15 mm` | `≤0.0001 mm` |
| 2% 调节时间 | `0.124 s` | 记录值 |
| 最大速度 | `20 mm/s` | 等于冻结限幅 |

全部门槛通过。这里仅证明滤波器离散实现与连续标称系统一致，不单独证明接触闭环稳定。后续 N102–N104 已在物理自由目标上检查峰值力、穿透、接触丢失与独立重放：公共安全/接触门通过，但两组导纳均未通过 `10% F_d` 稳态力 RMSE 门，详见 `paper/N102_N104_CONTACT_VALIDATION_REPORT.md`。

## 4. 权威产物

- `configs/fpmfc_contact.yaml`
- `v6_mujoco/fpmfc/contact.py`
- `v6_mujoco/fpmfc/contact_config.py`
- `v6_mujoco/fpmfc/contact_model.py`
- `v6_mujoco/fpmfc/contact_precheck.py`
- `output/fpmfc/contact/n100_n101_precheck.json`
- `tests/test_fpmfc_contact.py`

本报告只验证 N100/N101 前置契约；N102–N104 的权威场景结果与结论边界由 `paper/N102_N104_CONTACT_VALIDATION_REPORT.md` 给出。

# N208：被动目标估计、事件捕获与抓后消旋报告

基线：`877446208453c441c226b5b5a613856f00c4b3fd`。选中名义运行：`D06_final_gain3`。所有数值来自本轮记录；N110—N206历史证据保持不变。

名义连续任务：**True**；固定末窗消旋：**True**；自然任务全惯性辨识：**未验证**。这些判定互相独立。

## 模型与信息边界

目标从 t=0 自由积分，无主动执行器或状态注入。仅七关节真实控制输入；接触反力和软件 weld 由双方动力学求解。局部微重力模型忽略轨道梯度、气动力和晃动。N206工具、硬件限位、载荷与捕获门保持原声明，50 N/2 N·m仍是设计假设。

在线只接收几何坐标系位姿、基座导航、编码器、执行器力矩和理想接口F/T。控制模型由已知机器人/工具和公开先验独立构建。SO(3)误差状态EKF按观测时间更新，再因果预测到控制时刻；接触期提高过程不确定性。参考随估计目标移动，四项估计捕获门及裕量连续成立40 ms后请求锁紧。真值评价不能批准锁紧。

目标先验质量10–40 kg、COM逐轴±20 mm、物理一致惯量；名义20 kg/0.3 I。固定阻尼来自离线名义设计；可关闭的20 ms三模型制动治理不构成全不确定性鲁棒证明。辨识使用固定世界原点动量差分，SVD秩先于约束计算，只更新数据支持方向。独立未来块预测、物理一致性和信息门成立后才允许参数渐变进入预测；否则保持先验。没有为了满秩实施额外激励。

## 运行结果

|运行|模式/测量|终态|锁紧 s|连续完成|消旋|参数实际接入|
|---|---|---|---:|---|---|---|
|D01_ideal|prior/ideal|COMPLETED|8.1779999999994|True|True|False|
|D02_ideal_governor|prior/ideal|COMPLETED|8.1779999999994|True|True|False|
|D03_slower_approach|prior/ideal|POSTGRASP_SAFETY_FAILED|8.427999999999484|False|False|False|
|D04_noisy_development|prior/noisy|MOMENTUM_OR_NUMERICS_FAILED|None|False|False|False|
|D05_final_nominal|prior/ideal|HQP_FAILED|None|False|False|False|
|D06_final_gain3|prior/ideal|COMPLETED|7.991999999999342|True|True|False|
|H1_prior|prior/ideal|COMPLETED|7.9319999999993485|True|True|False|
|H1_identified|identified/ideal|COMPLETED|7.9319999999993485|True|True|True|
|H2_prior|prior/ideal|HQP_FAILED|None|False|False|False|
|H2_identified|identified/ideal|HQP_FAILED|None|False|False|False|
|H3_prior|prior/ideal|COMPLETED|7.9319999999993485|True|True|False|
|H3_identified|identified/ideal|COMPLETED|7.9319999999993485|True|True|True|
|F01_fine|prior/ideal|COMPLETED|7.992000000001004|True|True|False|
|S01_noise_delay|identified/noisy|HQP_FAILED|None|False|False|False|
|S02_contact_mismatch|identified/ideal|COMPLETED|7.997999999999341|True|True|False|

每条运行的失败原因、峰值载荷、独立P/H漂移与实际结束时间见 comparison.json。抓后固定20 s、最后2 s评价；目标世界≤0.1 deg/s、目标—基座≤0.02 deg/s。未完成全窗的运行不评价为消旋成功。

## 辨识与动量解释

identifiability_report.json逐条列出数据秩、奇异方向、物理性、真值误差和先于同化的未来块预测。相同记录块以15/32 kg先验离线重估，显示先验依赖；这些结果不回灌控制。独立受迫合成Newton–Euler数据验证充分激励下的实现，不能冒称真实任务提供相同信息。D01采用早期混合量纲权重，只作历史开发诊断，不纳入最终辨识结论。

momentum_feasibility.json记录先验低/中/高尺度、最终后验和评价真值下的锁定惯量与共同残余角速度。它使用记录构型作离线核算，不是全构型可达性证明。内部阻尼耗能而不消除系统总角动量。P/H、数值几何力偶、wrench功率及能量账本独立保留，未扣除残差制造通过。

## 核验、边界与资源

完整机器人尝试 15/16；累计实验/测试/重放CPU 4115.643 s / 28800 s。开发最多6次，所有错误和早停保留；独立重放、孤立物理及合成辨识分开计数。冻结状态：True。另为未逐项计时的辅助处理预留3600 s预算，合计仍低于8小时；该预留不是实测CPU，详见resource_accounting.json。

力矩输入重放从t=0积分，仅重施真实输入与事件；传感器输入重放另行重新决策。前者不证明控制决策或惯性识别正确。truth_access_audit.json是源代码及行为证据，并非操作系统能力隔离证明。协方差覆盖率仅为逐轨迹边际统计；噪声传感器和理想接口F/T仍是仿真假设。未验证硬件承载、真实夹爪、真实视觉或实时截止期。

## 证据与可视化

- [资格矩阵](../output/fpmfc/n208_adaptive_capture/qualification_matrix.json)
- [逐运行比较](../output/fpmfc/n208_adaptive_capture/comparison.json)
- [可辨识性](../output/fpmfc/n208_adaptive_capture/identifiability_report.json)
- [运行账本](../output/fpmfc/n208_adaptive_capture/run_ledger.json)
- [当前科学图与两条视频](../output/fpmfc/n208_adaptive_capture/visualizations/index.html)
- [文献实施映射](../output/fpmfc/n208_adaptive_capture/literature_to_implementation.md)

完整轨迹、SensorPacket、后验、参考进度、动作和实现身份在 runs/；历史源快照在 source_versions/。冻结后场景的失败不用于回调算法参数。未执行项应由完成审计显式列出，不能记为通过。

## 关键数值与归因边界

名义锁紧时刻 7.991999999999342 s，实际结束 27.992000 s；最后固定窗口的目标世界/相对角速度最大值分别为 0.02355415326165176 / 7.779798527468356e-05 deg/s。全程最大真实载荷比 0.36322051052286997。

最终位置/姿态外环增益比例为 3.0，数值求解容差为 1e-12；仍使用原机器人/工具、原接口模型、原物理安全门。开发修改与失败按时间保存在 requirements_and_parameter_policy.md 和 run_ledger.json。

相对N206，变化包含估计目标驱动的实时参考、测量漂移处理和事件捕获时刻。没有逐因素完整消融，不能把任务成功单独归因于惯性辨识。D03的较慢接近改变了接触峰值，却在锁紧后载荷超门；不能把“减慢接近”当成必然安全规律。

|未知场景/模式|质量误差 %|COM误差 mm|惯量Frobenius误差 %|秩|全部精度门|
|---|---:|---:|---:|---:|---|
|H1_prior|0.7235|15.2758|8.5448|10|False|
|H1_identified|0.7235|15.2758|8.5448|10|False|
|H2_prior|42.9418|17.5477|306.9575|4|False|
|H2_identified|42.9418|17.5477|306.9575|4|False|
|H3_prior|0.4976|0.1981|0.1488|10|True|
|H3_identified|0.4976|0.1981|0.1488|10|True|

预注册精度门为质量5%、COM5 mm、惯量张量10%。数值满秩与这些精度门不同；达到点估计目标的个别轨迹不构成整个未知参数范围的全辨识保证。球对称主轴自由运动阶段不声称绝对尺度可观；接触后新增的测得作用量可以提高数据秩。

H1配对：辨识模式实际参数进入预测=True；共同时间段最大实际力矩差=0 N·m。参数用于预测而力矩相同的情况，不记为控制性能改善。
H2配对：辨识模式实际参数进入预测=False；共同时间段最大实际力矩差=0 N·m。参数用于预测而力矩相同的情况，不记为控制性能改善。
H3配对：辨识模式实际参数进入预测=True；共同时间段最大实际力矩差=0 N·m。参数用于预测而力矩相同的情况，不记为控制性能改善。

噪声、数值、预测不可行、几何/规划失败分别按原始终态报告。D03落盘错误恢复没有重写轨迹；D04原始数值失败没有追溯改成成功。H2等冻结后失败不再回调参数。保留最多16次总预算中的未用兼容性预留，不为了消耗预算追加试验。

state_estimation_summary.json列出信息陈旧度、创新、协方差正定性和状态误差；momentum_feasibility.json列出各参数候选下的锁定惯量及共同残余转速。两者均不向控制器回灌真值。

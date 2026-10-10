# Experiment Result-to-Claim

## N073 Controller Ablation

- **Result source**: `output/fpmfc/comparison/n073_controller_ablation_rk4_formal_planning45/controller_ablation_summary.json`
- **Intended claim**: 在自定义 Flexiv 接触前捕获场景中，严格层级的臂形与基座反作用二级目标均为必要且有效的机制，同时保持一级末端任务。
- **Verdict**: `partial`
- **Confidence**: `high`
- **Paper claim audit**: `unavailable`；当前判定为 provisional。
- **What the data supports**: 对完整方法预规划并冻结的 10 条轨迹，full 10/10 合格而 no-shape 0/10；在线臂形目标对实现规定终端臂形和通过终端捕获验收是必要的。所有 30 条轨迹均通过确定性重放与公平性契约。
- **What the data does not support**: no-base-reaction 仍为 10/10 合格，三项基座扰动差异接近零且无显著性；因此不能声称当前基座反作用项有效或必要。该执行级消融也不能代表各变体独立重规划后的端到端优劣，终端合格不能替代全过程跟踪精度证明。
- **Missing evidence**: 逐任务周期一级锁定残差、纯一级双删除对照、基座反作用残差/活跃度与预注册权重敏感性、多场景配对验证；端到端主张还需各变体独立重规划。
- **Revised claim**: 在当前 Flexiv 场景和冻结轨迹上，在线臂形目标对终端位—型捕获合格是必要的；删除基座反作用目标未产生可检测的基座扰动退化。
- **Routing action**: 收窄基座反作用收益主张；N073 收口，主线转入 N100+ 接触力与力—位—型场景验证。残差审计、纯一级基线和基座权重扫描为按论文需要选做的补充。
- **Trace**: `paper/review-traces/experiment-result-to-claim/2026-09-24_run01/`

## N102–N104 Contact Validation

- **Result source**: `output/fpmfc/contact/n102_n104_authoritative_comparison.json`
- **Intended claim**: 在自定义自由目标接触场景中，法向导纳 + 位—型控制相对刚性位置控制同时降低峰值力、力冲量和基座角冲量，并稳定跟踪 3 N；在线臂形项优于无臂形版本。
- **Verdict**: `partial`
- **Confidence**: `high`
- **Paper claim audit**: `unavailable`；当前判定为 provisional。
- **What the data supports**: 三组公共安全/有界接触丢失门和独立力矩重放均通过。N103 相对 N102 将力冲量降低 18.42%、机器人侧接触角冲量代理降低 39.47%、基座峰值角速度降低 18.51%。N103 相对 N104 在峰值力、RMSE、基座峰值角速度和接触丢失次数上更好。
- **What the data does not support**: N103 峰值力增加 27.60%，稳态 RMSE 为 41.28% Fd，未通过 10% 门；因此不支持“导纳降低峰值冲击”或“稳定精确力跟踪”。N103 也并非所有指标优于 N104。单一 seed-00 冻结交接只隔离接触阶段在线臂形项，不能代表端到端 no-shape 方法或普遍鲁棒性。
- **Missing evidence**: live-twist 接触切换、多初态/参数/步长敏感性、独立验证场景、更长保持及抓持约束、直接定义的基座角动量/角冲量、各变体独立重规划的端到端消融。
- **Revised claim**: 当前 1 s 单边接触瞬态只表明累积冲量/基座扰动与峰值/力跟踪之间的场景内权衡；完整 C2/B4 主张未通过。
- **Routing action**: 停止在当前 seed-00 轨迹上继续事后调导纳；如继续 B 级主张，新建 N110 live-twist 切换并在独立场景验证。A 级结论不受该 B 级负结果影响。
- **Trace**: `paper/review-traces/experiment-result-to-claim/2026-09-24_run02/`

## S03 时间—路径—臂形有限策略

主任务收益主张：`claim_supported = no`，`confidence = high`。独立审查范围为冻结理想 SensorPacket + C2 CA18、两种已见场景与当前有限候选策略。四条冻结 B1/B2 配对均未捕获，不支持“解决 H2、保留名义连续捕获—消旋能力、建立臂形增量收益”。

局部权限主张单独为 `yes`：四个预登记 B01 状态、每状态11个候选、统一0.6s独立先验预测中，五类变量均使动作和部分命名裕量产生大于冻结阈值的变化。变化可以改善也可以恶化，不能汇总为可同时获得的联合收益。

该负结果只否定已执行策略的任务收益，不证明五维参数盒或所有轨迹都无解。P01在终止时有两个通过粗筛但未动态验证的候选；其他拒绝也要区分粗筛关节盒、动态性能残差与真实物理安全门。已执行前缀的安全复核不等于已验证硬件恢复。B00为归档结果的严格复用，开发与配对的确定性重复也不作为独立统计样本。

路由：`narrow_claim_and_negative_result_handoff`。本轮不追加调参或试验，不启动 S04；下一次若继续 S03，应重新规划并冻结比较，检查粗筛与实际伺服的一致性、到达时间持续延后的行为及未验证候选的覆盖。S02R感知问题保留为独立待办，非理想域仍未准入。

[PAPER CLAIM AUDIT: WARN]：现有审计属于历史 N209 稿件，不是本轮 S03 的完整论文数字审计。正式论文写作不能据此自动获得审计通过。

审查痕迹：`paper/review-traces/experiment-result-to-claim/2026-10-10_run01/` 与 `2026-10-10_run02/`。原始发现与修正回执分别保存，不覆盖原审查。双重放和媒体的最终交付状态以 `qualification.json`、`completion_audit.json` 及审查追加确认文件为准。

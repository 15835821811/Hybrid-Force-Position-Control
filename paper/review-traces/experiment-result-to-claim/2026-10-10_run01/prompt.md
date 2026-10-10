# Bounded reviewer prompt

prompt_fidelity: exact
review_role: paper architect/reviewer
reviewer_agent: /root/s03_evidence_review
review_mode: independent delegated local review; not pending external review

以下为主执行者实际发出的原始任务，逐字保存：

> 按已调用的 experiment-result-to-claim 与 paper-figure 技能执行 paper architect/reviewer 独立审查。工作区 C:/Users/admin/.codex/worktrees/system-s03-pm-planning/力位形混合控制。只读，不运行机器人、不改参数或代码。先读 experiments/system_capture/manifests/CODEX_S03_PM_PLANNING_AFTER_S02.md，以及 output/fpmfc/system_capture/S03/EXPERIMENT_PLAN.md、candidate_authority.json、diagnostics/window_coverage.json、implementation_failure.json、runs/B00 两类重放、runs/B01/metrics.json、runs/D00 两类重放、figure_plan.md。当前 V2 的 D00_R1/D01 正在运行，不能把未完成结果当结论。审查：候选权限证据能支持什么；B0 H2 的归因；失败/非理想域/原记录缺口是否准确保留；图表公平性及终止窗口是否清楚；需要在最终报告补齐的关键证据。返回具体、可执行的发现，尤其标出阻碍科学交付的证据或表述问题。先给阶段性 verdict，后续会提供最终配对结果供最终 judgment。技能要求保存 review trace；本轮由主执行者保存你的原始回复与输入身份，你不要编辑共享文件。

另外遵照继承的用户要求先读取：C:/Users/admin/.codex/attachments/7763d53a-696c-45f8-a800-1a98ec6b16a7/goal-objective.md。

实际读取的主要证据、支撑性代码与技能路径列于 inputs.json。这里只记录该有界任务，不声称复制整个系统/开发者上下文。最终配对结果不在本轮完成证据范围内。

trace 保存授权：主执行者后续明确允许仅写入本目录；未授权修改实验代码、证据或原报告。


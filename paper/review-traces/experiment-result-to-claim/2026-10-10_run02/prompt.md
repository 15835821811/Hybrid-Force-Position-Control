# Final bounded reviewer prompt

prompt_fidelity: exact
reviewer_agent: /root/s03_evidence_review
review_role: paper architect/reviewer
pending_external_review: false

以下为主执行者实际发出的任务，逐字保存：

> 请开始最终科学结论与图注审查（只读实验数据，可写 paper/review-traces/experiment-result-to-claim/2026-10-10_run02/），输入仍为同一S03目录，现全部8次新增尝试完成：P00/P01/P02/P03分别8.04/7.24/6.44/6.44s，均PREDICTION_REJECTED、未捕获。读 search_coverage.json、same_model_comparison.json、planner_vs_plant_errors.json、diagnostics/authority_signed_margins.csv、diagnostics/safety_*.json、report.md、qualification.json、source_recoverability.json、visualizations/figures/scientific_manifest.json，以及3张 scientific PNG（可自行view_image）。19模块测试通过，全部已完成步重新安全复核PASS；B00/D00原双重放通过，其他逐包重放还在顺序运行，当前dual状态PENDING，绝不将其写成已通过。请重点审：有限策略与5维盒区分、P01两个screen通过但未获动态名额的候选是否准确表述；当前/未来可行性归因；延后到达与粗筛模型局限；合格完整配对缺失；带符号图表公平、单位和哨兵排除。判断任务收益主张yes/partial/no及局部权限独立结论，给后续建议但不运行新实验。保存prompt/response/inputs/verdict/handoff完整trace，将重放与最终媒体核验明确列为待根代理完成的交付门；不要把negative科学结论变成建议立即S04。完成后我会通知最终重放状态供你追加简短结论确认。

输入完整路径和审查时身份见 inputs.json；只读复算与原始 trace 哈希见 reviewer_checks.json。沿用 run01 已阅读技能与任务目标，不重新解释历史失败为新成功。


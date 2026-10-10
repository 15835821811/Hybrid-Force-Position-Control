# Reviewer handoff

完成独立最终科学结论/图注审查并保存 prompt.md、response.md、inputs.json、verdict.json、reviewer_checks.json 与本文件。已阅读三张 scientific PNG；已逐项复核带符号 CSV、恢复源文件 SHA、全部 safety 摘要，并只读重算基座姿态字段。

已向根代理报告 F1–F3：基座四元数索引错误、截断窗口完成标签污染、P01 两个未动态验证候选需显式列出。审查者未修代码或报告，根代理负责修复与重生，原审查输入身份保留，不事后改写为已修版本。

主收益 no / high；局部权限 yes / high。pending_external_review=false，因为独立 reviewer 已完成受托范围。交付未完成：双重放 PENDING、最终媒体核验 PENDING，另有汇总修正。科学负结果不授权新实验、S04 或非理想域准入。

已验证：CSV 2200 行、4 状态、55 有限对、无哨兵、差值算术误差 0；11 源码恢复条目 SHA 全匹配；9 组 safety 已完成步数全部匹配且 PASS。纠正的完整轨迹基座漂移另见 reviewer_checks.json；共同前缀仍应由根代理统一按相应窗口重算。

只写本目录；没有物理步或实验变更。待根代理提供修正与最终重放结果后，在独立 addendum 中作简短确认，不能覆盖原始发现。

保存期间收到根代理修正通知：已按 base_free_joint 修复姿态提取，分离 run_outcome 与截断窗口状态，增加 P01 未动态验证成员列，修订 forecast 图注，并加入姿态路径积分与截断标签回归检查。该通知仅记为执行者报告；最终文件重建、双重放与媒体核验尚待最终输入后独立确认，不在本 trace 中追溯改写为已通过。


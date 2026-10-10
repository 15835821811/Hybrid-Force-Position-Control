# S03 执行与证据导航

起点 `b048891c57d3ec0df72ec53ab87e20f2fe9aa1ac`，独立分支 `codex/system-s03-pm-feasible-planning`。实际环境以 `entry_acceptance.json` 为准：Python 3.10.19、NumPy 1.26.4、SciPy 1.11.2、MuJoCo 3.3.2。OpenBLAS/OMP 均单线程。

本目录是已执行的不可覆盖证据集。下列是实际使用的入口；原始尝试不得删除、覆盖后冒充首次运行，`--resume` 会跳过已有成功验证的记录。完整机器人尝试从 home 开始。诊断、显示和重放不新增任务成功样本。

```powershell
$env:OPENBLAS_NUM_THREADS='1'
$env:OMP_NUM_THREADS='1'
$env:PYTHONIOENCODING='utf-8'
python -m v6_mujoco.system_capture --phase S03 --mode prepare
python tools/s03_baselines.py
python tools/s03_authority.py
python -m v6_mujoco.system_capture --phase S03 --mode run --resume
python tools/s03_replay_failure.py
python -m v6_mujoco.system_capture --phase S03 --mode replay --resume
python tools/s03_evidence.py
python tools/s03_close_accounting.py
python -m v6_mujoco.system_capture --phase S03 --mode report
python tools/s03_visualization.py --plots --videos
python tools/s03_completion_audit.py
python -m v6_mujoco.system_capture --phase S03 --mode report
```

这不是重新执行所有入口的要求。第一次权限诊断、其修正、原 V1 运行和 V2 的单次实现重试有独立日志与预算；不会由上面的普通 `run` 命令隐式补跑。`implementation_failure.json` 登记唯一重试 D00_R1。`preflight_original/` 保留原诊断失败，修复不是删除原失败。两版源码和冻结合约在 `versions/V1`、`versions/V2`；更早的确切源码按已记录 SHA256 保存在 `versions/source_by_sha`。来源恢复只接受与原 config 字节哈希相等的文件。

账本收口必须等待重放进程结束、`operation.lock` 释放；`s03_close_accounting.py` 对延后登记的 B01 预测步和分析收据只计一次。最终报告在媒体与完整性核验之后生成，交接清单覆盖最终文件。图表只读保存轨迹；视频刷新不执行新的 `mj_step`。

运行证据：

- `run_ledger.json`、`candidate_budget.json`：真实尝试、命名候选、预测步、重放与计算账本。
- `runs/*/raw/index.json`：分块原始 SensorPacket、完整 P18、参考系数、提案、施加力矩、已完成物理步及失败事件。
- `runs/*/config.json`、`completion.json`：冻结身份和真实终止；B00 的 `reused_result.json` 精确指向 S02/E0_C2，不伪造一条新轨迹。
- `runs/*/actuator_replay.json`：从 t=0 施加原始力矩/锁紧事件，比较逐步状态。
- `runs/*/decision_replay.json`：逐包重算估计、候选、参考及终止；D00 原外部共享错误明确采用故障注入，不声称操作系统错误自然再现。
- `search_coverage.json`：每次有限候选覆盖与最后拒绝原因；没有穷举五维参数盒。
- `diagnostics/safety_*.json`：对每个已完成物理步重新应用原安全评价器，含原实现失败的末步；没有执行安全撤离。
- `same_model_comparison.json`：完整合格配对或共同执行前缀，两者不可互换。
- `visualizations/index.html`：九个证据条目的五视角、连续体侧、近景、跟踪及其他诊断；保存状态仅用于显示，未推进物理。

非理想域 `NOT_ADMITTED`。未自动开展 S02R、S04—S08；S02/S01/C1/E0 历史状态和旧媒体来源保持不变。用户目标末段授权把本分支上传 GitHub，优先于目标前文的“默认仅本地”描述。

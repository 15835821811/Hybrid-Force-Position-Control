# N201R 执行与复查命令

工作树：`C:\Users\admin\.codex\worktrees\n200-postgrasp-baseline\力位形混合控制`。
Python：`.venv310\Scripts\python.exe`。起点 724c658，当前独立分支 `codex/n201-grasp-interface-calibration`。

## 已完成执行顺序

```powershell
git status --short
git rev-parse HEAD
git switch -c codex/n201-grasp-interface-calibration
.\.venv310\Scripts\python.exe -m v6_mujoco.postgrasp_calibration.audit
```

独立加载最初均以 `load_tests` 模块名执行；当时的源码现分别冻结为 `load_tests_v1.py`、`load_tests_v2.py`、`load_tests_v3.py`，账本记录真实字节哈希。阶段顺序如下：

| 当时源码 | 执行阶段 | 输出 |
|---|---|---|
| v1 | `original`, `derive`, `candidate` | 原参数与候选 1；载荷搬移实现无效 |
| v2 | `derive2`, `candidate2` | 候选 2；非零初始 anchor，参考无效 |
| v3 | `derive2corrected`, `corrected` | 相同候选 2 改正参考；载荷搬移仍无效 |
| current | `balanced_original`, `balanced_candidate1`, `balanced_candidate2`, `fine` | 四个输入有效套件 |

最终有效套件的执行命令为：

```powershell
.\.venv310\Scripts\python.exe -m v6_mujoco.postgrasp_calibration.load_tests balanced_original
.\.venv310\Scripts\python.exe -m v6_mujoco.postgrasp_calibration.load_tests balanced_candidate1
.\.venv310\Scripts\python.exe -m v6_mujoco.postgrasp_calibration.load_tests balanced_candidate2
.\.venv310\Scripts\python.exe -m v6_mujoco.postgrasp_calibration.load_tests fine
.\.venv310\Scripts\python.exe -m v6_mujoco.postgrasp_calibration.finalize
.\.venv310\Scripts\python.exe -m pytest -q tests/test_n201_interface_calibration.py
```

`finalize` 已执行一次，包含 16 个另计的夹具独立重放。加载套件拒绝覆盖已存在目录；不能删除现有结果后重复运行。复现实验应在独立副本中使用版本对应源码，并保留新的账本。不能使用当前夹具代码假装重现已标为无效的历史实现。

## 完成验证与身份冻结

```powershell
.\.venv310\Scripts\python.exe -m v6_mujoco.postgrasp_calibration.verify --seal
.\.venv310\Scripts\python.exe -m v6_mujoco.postgrasp_calibration.verify
git diff --check
```

`--seal` 仅保存静态验证、文档和代码身份，不重新加载测试轨迹。无参数 `verify` 核查 sealed 输出字节、历史身份和新模型的静态已知 wrench，新增动力学为 0。不要为重新计算文档哈希而重跑 `finalize`。

## 正式轨迹及 Git 边界

本轮 Z_new/D_new/D_new_fine 全部有 skipped.json；没有执行正式运行命令。判定为 CONNECTION_MODEL_NOT_QUALIFIED，实际载荷峰值 1.0002697363241817。

仅新增本轮文件及新属性规则进入本地提交。没有执行推送、合并、破坏性 reset，未修改历史结果。新增 NPZ 由本地 Git LFS 保存；离开此工作树搬运结果时须同时保留 LFS 对象。

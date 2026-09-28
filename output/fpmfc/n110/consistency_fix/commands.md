# N110B 修复批次实际执行命令

工作目录：`C:\Users\admin\.codex\worktrees\n110-contact-audit\力位形混合控制`。解释器均为 `.venv\Scripts\python.exe`。每个 `run.log` 和 `validation.log` 保存相应命令的标准输出；`replay_cadence_probe.log` 保存重放调用顺序诊断。

```powershell
.venv\Scripts\python.exe -m pytest -q

.venv\Scripts\python.exe -m v6_mujoco.fpmfc.reevaluate_contact --variant rigid --output-dir output/fpmfc/n110/consistency_fix/R/rigid --overwrite
.venv\Scripts\python.exe -m v6_mujoco.fpmfc.reevaluate_contact --variant admittance --output-dir output/fpmfc/n110/consistency_fix/R/admittance --overwrite
.venv\Scripts\python.exe -m v6_mujoco.fpmfc.reevaluate_contact --variant admittance-no-shape --output-dir output/fpmfc/n110/consistency_fix/R/admittance-no-shape --overwrite

.venv\Scripts\python.exe -m v6_mujoco.fpmfc.run_contact_consistent --variant rigid --mapping-mode homogeneous --output-dir output/fpmfc/n110/consistency_fix/S/rigid
.venv\Scripts\python.exe -m v6_mujoco.fpmfc.run_contact_consistent --variant admittance --mapping-mode homogeneous --output-dir output/fpmfc/n110/consistency_fix/S/admittance
.venv\Scripts\python.exe -m v6_mujoco.fpmfc.run_contact_consistent --variant admittance-no-shape --mapping-mode homogeneous --output-dir output/fpmfc/n110/consistency_fix/S/admittance-no-shape

.venv\Scripts\python.exe -m v6_mujoco.fpmfc.run_contact_consistent --variant rigid --mapping-mode affine --output-dir output/fpmfc/n110/consistency_fix/A/rigid
.venv\Scripts\python.exe -m v6_mujoco.fpmfc.run_contact_consistent --variant admittance --mapping-mode affine --output-dir output/fpmfc/n110/consistency_fix/A/admittance
.venv\Scripts\python.exe -m v6_mujoco.fpmfc.run_contact_consistent --variant admittance-no-shape --mapping-mode affine --output-dir output/fpmfc/n110/consistency_fix/A/admittance-no-shape

.venv\Scripts\python.exe -m v6_mujoco.fpmfc.validate_contact_consistent --output-dir output/fpmfc/n110/consistency_fix/S/rigid
.venv\Scripts\python.exe -m v6_mujoco.fpmfc.validate_contact_consistent --output-dir output/fpmfc/n110/consistency_fix/S/admittance
.venv\Scripts\python.exe -m v6_mujoco.fpmfc.validate_contact_consistent --output-dir output/fpmfc/n110/consistency_fix/S/admittance-no-shape
.venv\Scripts\python.exe -m v6_mujoco.fpmfc.validate_contact_consistent --output-dir output/fpmfc/n110/consistency_fix/A/rigid
.venv\Scripts\python.exe -m v6_mujoco.fpmfc.validate_contact_consistent --output-dir output/fpmfc/n110/consistency_fix/A/admittance
.venv\Scripts\python.exe -m v6_mujoco.fpmfc.validate_contact_consistent --output-dir output/fpmfc/n110/consistency_fix/A/admittance-no-shape

.venv\Scripts\python.exe output/fpmfc/n110/consistency_fix/replay_cadence_probe.py
```

六条闭环轨迹各运行一次，均为 500 个 2 ms RK4 步、50 个 20 ms 任务周期。R 组三条只重放归档力矩；重放调用顺序诊断与六次独立重放不计为新闭环轨迹。首次 A/无臂形独立重放使用直接 `mj_step` 时超出原 qvel 容差；探针证明补上在线伺服已有的前向计算节奏后通过。验证器修正后，六次独立重放全部重新执行，未改任何闭环 trace 或容差。

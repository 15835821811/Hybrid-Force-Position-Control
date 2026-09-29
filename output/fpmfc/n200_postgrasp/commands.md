# N200/N201 execution commands

Repository branch: `codex/n200-postgrasp-baseline` at baseline `685ac6bd7bb5bd9f2da6f844375d0485d68c7cdb`.

The worktree used Python 3.10.19, MuJoCo 3.3.2, NumPy 1.26.4, SciPy 1.11.2 and PyYAML 6.0.1. The repository's NumPy 1.21.6 pin produced a binary ABI error with SciPy 1.11.2 on this Windows interpreter; this new environment change is recorded here and did not alter the historical requirements file.

```powershell
git lfs fetch hybrid --include='output/fpmfc/n110/live_twist_handoff/precontact/C1/trace.npz' --exclude='*'
git lfs checkout 'output/fpmfc/n110/live_twist_handoff/precontact/C1/trace.npz'
$env:PYTHONPATH='.'
& '.\.venv310\Scripts\python.exe' -m pytest -q tests/test_n200_postgrasp.py
& '.\.venv310\Scripts\python.exe' -m v6_mujoco.postgrasp.run --phase prepare
& '.\.venv310\Scripts\python.exe' -m v6_mujoco.postgrasp.run --phase Z
& '.\.venv310\Scripts\python.exe' -m v6_mujoco.postgrasp.run --phase D
& '.\.venv310\Scripts\python.exe' -m v6_mujoco.postgrasp.run --phase D_fine
& '.\.venv310\Scripts\python.exe' -m v6_mujoco.postgrasp.run --phase compare
& '.\.venv310\Scripts\python.exe' -m v6_mujoco.postgrasp.validate
```

Formal dynamics budget: exactly Z, D and D_fine. The small model test and three same-step deterministic replays are counted separately and do not extend the formal trajectories.

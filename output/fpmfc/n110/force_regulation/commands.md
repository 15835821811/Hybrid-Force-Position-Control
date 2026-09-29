# N110D execution record

Working branch: `codex/n110-force-regulation-ablation`, created from `dd80807948778e8a77a0d7277553a5f6501d8067`. Working directory: repository root on Windows PowerShell. Commands use the checked-out `.venv\Scripts\python.exe`.

## Read-only baseline diagnosis, then freeze

```powershell
.\.venv\Scripts\python.exe -m v6_mujoco.fpmfc.force_regulation_analysis --trace output/fpmfc/n110/live_twist_handoff/contact/C1/admittance/trace.npz --output output/fpmfc/n110/force_regulation/baseline_error_decomposition.json --reset
.\.venv\Scripts\python.exe -m pytest -q tests/test_n110_force_regulation.py
.\.venv\Scripts\python.exe -m v6_mujoco.fpmfc.force_regulation_contract
```

The baseline command also saved `baseline_outer_diagnostics.npz`. The frozen manifest was written after the baseline reconstruction passed and before any D01/D10/D11 result existed. The manifest records the D00 equivalence checks, parent C1 source hash, code identities, selection rule, and closed-loop budget. An additional temporary copy of the archived D00 trace/metrics/config was independently replayed with the experimental D00 outer loop; it passed without modifying historical files. The actual replay output is retained as `D00_replay.log`.

## Exactly three new 1 s S contact runs

```powershell
.\.venv\Scripts\python.exe -m v6_mujoco.fpmfc.run_force_regulation --cell D01
.\.venv\Scripts\python.exe -m v6_mujoco.fpmfc.run_force_regulation --cell D10
.\.venv\Scripts\python.exe -m v6_mujoco.fpmfc.run_force_regulation --cell D11
```

Actual stdout is saved in each cell's `run.log`; D10 and D11 were independent processes. D00 reused the N110C archived C1 admittance trace and was **not** run again. The optional no-shape confirmation was not run because no full-shape cell passed the 0.3 N force RMSE gate.

## Independent validation and summary

The first D01 verification invocation of `validate_force_regulation` failed before replay: the wrapper passed a required path argument to a zero-argument frozen validator call. Its actual traceback is retained at `D01_validation.log`. The manifest and dynamics code remained frozen. The separate source file `validate_force_regulation_v2.py` repairs the wrapper signature; each resulting validation records this file's SHA-256 in addition to the pre-run verification identity.

```powershell
.\.venv\Scripts\python.exe -m v6_mujoco.fpmfc.validate_force_regulation_v2 --cell D01
.\.venv\Scripts\python.exe -m v6_mujoco.fpmfc.validate_force_regulation_v2 --cell D10
.\.venv\Scripts\python.exe -m v6_mujoco.fpmfc.validate_force_regulation_v2 --cell D11
.\.venv\Scripts\python.exe -m v6_mujoco.fpmfc.derive_force_regulation_tracking --cell D00
.\.venv\Scripts\python.exe -m v6_mujoco.fpmfc.derive_force_regulation_tracking --cell D01
.\.venv\Scripts\python.exe -m v6_mujoco.fpmfc.derive_force_regulation_tracking --cell D10
.\.venv\Scripts\python.exe -m v6_mujoco.fpmfc.derive_force_regulation_tracking --cell D11
.\.venv\Scripts\python.exe -m v6_mujoco.fpmfc.summarize_force_regulation
.\.venv\Scripts\python.exe -m pytest -q
```

Successful validator stdout is in `D01/validation.log`, `D10/validation.log`, and `D11/validation.log`. Full-suite actual output is `tests.log` (92 passed). Independent replays do not consume new closed-loop runs. The final source and archive preservation check used `git diff --name-only dd80807948778e8a77a0d7277553a5f6501d8067 -- configs/fpmfc_contact.yaml configs/fpmfc_n110_handoff.yaml v6_mujoco/fpmfc/contact.py v6_mujoco/fpmfc/run_contact_consistent.py v6_mujoco/fpmfc/handoff_trajectory.py output/fpmfc/n110/live_twist_handoff` and returned no paths.

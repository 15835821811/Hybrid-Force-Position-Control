# N110C execution record

Working directory: the repository root on `codex/n110-live-twist-handoff` at baseline `31377e8a1f95d1ac0b7eeed11397e594c702522f`. Commands below use the checkout's `.venv\Scripts\python.exe` on Windows PowerShell. The pairing manifest was created before any N110C dynamics run and not rewritten after contact results existed.

## Frozen contract and two precontact runs

```powershell
.\.venv\Scripts\python.exe -m v6_mujoco.fpmfc.handoff_contract
.\.venv\Scripts\python.exe -m v6_mujoco.fpmfc.run_handoff_precontact --condition C0
.\.venv\Scripts\python.exe -m v6_mujoco.fpmfc.run_handoff_precontact --condition C1
.\.venv\Scripts\python.exe -m v6_mujoco.fpmfc.validate_handoff_precontact --condition C0
.\.venv\Scripts\python.exe -m v6_mujoco.fpmfc.validate_handoff_precontact --condition C1
```

Each run and validator's actual stdout is saved as `precontact/<condition>/run.log` or `validation.log`. Both source handoffs and strict torque replays passed before the contact runs.

## Contact preflight and exactly six S runs

```powershell
.\.venv\Scripts\python.exe -m v6_mujoco.fpmfc.run_handoff_contact --condition C0 --preflight-only
.\.venv\Scripts\python.exe -m v6_mujoco.fpmfc.run_handoff_contact --condition C1 --preflight-only
.\.venv\Scripts\python.exe -m v6_mujoco.fpmfc.run_handoff_contact --condition C0 --variant rigid
.\.venv\Scripts\python.exe -m v6_mujoco.fpmfc.run_handoff_contact --condition C0 --variant admittance
.\.venv\Scripts\python.exe -m v6_mujoco.fpmfc.run_handoff_contact --condition C0 --variant admittance-no-shape
.\.venv\Scripts\python.exe -m v6_mujoco.fpmfc.run_handoff_contact --condition C1 --variant rigid
.\.venv\Scripts\python.exe -m v6_mujoco.fpmfc.run_handoff_contact --condition C1 --variant admittance
.\.venv\Scripts\python.exe -m v6_mujoco.fpmfc.run_handoff_contact --condition C1 --variant admittance-no-shape
```

The preflight records are `contact/<condition>/preflight.json`. Every contact run's stdout is `contact/<condition>/<variant>/run.log`. All six used `mapping_mode=homogeneous`, with 500 physics steps and 50 HQP task ticks. No additional N110C closed-loop trajectories were run.

## Independent validation, summary and tests

```powershell
.\.venv\Scripts\python.exe -m v6_mujoco.fpmfc.validate_handoff_contact --condition C0 --variant rigid
.\.venv\Scripts\python.exe -m v6_mujoco.fpmfc.validate_handoff_contact --condition C0 --variant admittance
.\.venv\Scripts\python.exe -m v6_mujoco.fpmfc.validate_handoff_contact --condition C0 --variant admittance-no-shape
.\.venv\Scripts\python.exe -m v6_mujoco.fpmfc.validate_handoff_contact --condition C1 --variant rigid
.\.venv\Scripts\python.exe -m v6_mujoco.fpmfc.validate_handoff_contact --condition C1 --variant admittance
.\.venv\Scripts\python.exe -m v6_mujoco.fpmfc.validate_handoff_contact --condition C1 --variant admittance-no-shape
.\.venv\Scripts\python.exe -m v6_mujoco.fpmfc.summarize_handoff
.\.venv\Scripts\python.exe -m pytest -q
```

Each validator's actual stdout is saved as `contact/<condition>/<variant>/validation.log`. All eight source/contact torque replays passed. The read-only summary is `paired_summary.json`, and the actual complete test stdout is `tests.log` (84 passed). Independent replay and summary generation do not count against the closed-loop run budget.

The baseline-frozen paths were checked with `git diff 31377e8a1f95d1ac0b7eeed11397e594c702522f --name-only -- configs/fpmfc_contact.yaml configs/fpmfc_paper_planning45_effective.yaml v6_mujoco/fpmfc/trajectory.py v6_mujoco/fpmfc/controller.py v6_mujoco/fpmfc/run_capture.py v6_mujoco/fpmfc/run_contact_consistent.py output/fpmfc/contact output/fpmfc/precontact`; this returned no paths.

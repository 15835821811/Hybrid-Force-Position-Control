# S01 reproducibility

Use the exact Python/MuJoCo/NumPy/SciPy versions and single-thread environment in phase_manifest.json.
Run from the repository root. The archived baseline and C1/C2 source identities are preserved.

```powershell
$env:OPENBLAS_NUM_THREADS='1'
$env:OMP_NUM_THREADS='1'
$env:PYTHONIOENCODING='utf-8'
python -m v6_mujoco.system_capture --phase S01 --mode prepare
python -m v6_mujoco.system_capture --phase S01 --mode test
python -m v6_mujoco.system_capture --phase S01 --mode run --resume
python -m v6_mujoco.system_capture --phase S01 --mode replay --resume
python -m v6_mujoco.system_capture --phase S01 --mode report
```

Existing completed and failed keys are never re-executed. A live operation.lock must not be removed to start another run.
Use the retained environment executable or recreate its exact package versions; verify() deliberately checks environment identity.
C1 recording failure is not replayable. C2 recordings replay independently. No new attempt may be labeled recovery of the missing C1 trace.
The complete present repository is the resume starting point; deleting the ledger is not an authorized fresh campaign.
Report creation updates report CPU in the ledger first, then finalizes the handoff hashes on process exit.
A figure review or publication artifact added later requires rerunning report to refresh the handoff.
Publication display refresh: python -m v6_mujoco.feasible_capture --visualize (historical trace rendering only).

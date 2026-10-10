# S02 execution and reproduction

The working directory is the managed checkout on
`codex/system-s02-estimation-capture-contract`, created from
`71caa4149176f9becd76f95020cb43e97dabc851`. No push or merge is authorized.
S01 PARTIAL and the original missing C1 trace are unchanged.

## Environment

Actual executable:
`C:\Users\admin\.codex\worktrees\n200-postgrasp-baseline\力位形混合控制\.venv310\Scripts\python.exe`.
Python 3.10.19, NumPy 1.26.4, SciPy 1.11.2, MuJoCo 3.3.2.
The full version/platform identity is in `estimation_protocol.json`.
Each execution shell sets:

```powershell
$env:OPENBLAS_NUM_THREADS='1'
$env:OMP_NUM_THREADS='1'
$env:PYTHONIOENCODING='utf-8'
$s02Python='C:\Users\admin\.codex\worktrees\n200-postgrasp-baseline\力位形混合控制\.venv310\Scripts\python.exe'
```

Only the 16 required N209 trace/packet/control-state/estimator-event LFS files
were checked out from the existing local LFS store. No historical video batch
was downloaded or rerendered.

## Actual execution path

1. Entry source, backing-record and SHA audit:
   `python -m v6_mujoco.system_capture --phase S02 --mode prepare`.
2. Pre-registered split, two-candidate development selection, six signal
   validations, four archive regressions, unit/property/fault tests and freeze:
   `python -m v6_mujoco.system_capture --phase S02 --mode test`.
   Early readonly-R test failure is retained in
   `tests_preflight_readonly_failure.log`. Undefined autocorrelation for constant
   H2 innovations was changed to null after JSON rejected NaN; already-saved
   comparison arrays were retained. This unmetered preflight call is explicitly
   accounted as a conservative budget charge, not an exact CPU measurement.
3. Original E0: `python -m v6_mujoco.system_capture --phase S02 --mode run`,
   logged in `run.log`. Its covariance implementation error at 7.814 s is
   retained with the raw committed prefix and original source archive.
4. Estimator-only replay isolated the angular process-covariance defect.
   `tools/s02_repair_validation.py` applied the repaired estimator to all
   3908 recorded public packets, including the failing packet. No physical
   integration and no parameter reselection occurred. See
   `covariance_failure_reproduction.json`, `covariance_repair_tests.json`
   and `implementation_failure.json`.
5. Repaired properties and the same validation sequences were rerun as
   implementation regressions, using the saved candidate selection.
   Outputs are `tests.log` and `repair_offline.log`. Earlier outputs are
   preserved in `pre_repair/`; they are not replaced by repaired evidence.
6. Original E0 dual replay used `tools/s02_failed_replay.py` with the exact
   original archived runtime as its working directory and PYTHONPATH.
   `S02_EVIDENCE_ROOT` pointed to this S02 output directory.
   The actuator replay and expected-failure decision replay are in
   `E0_original_replay.log` and `runs/E0/*_replay.json`.
7. After repair freeze, the remaining order was E0_C2 → E1 → E2, conditional
   on the unchanged safety and ideal-task admission rules. E0_C2 consumes the
   second allowed attempt, replacing E3 in the four-attempt budget.
   `python -m v6_mujoco.system_capture --phase S02 --mode run` produces
   `C2_run.log`. The ledger, not the planned matrix, defines which runs
   actually happened. Failed admission leaves no empty result directory.
8. Current-runtime replays:
   `python -m v6_mujoco.system_capture --phase S02 --mode replay --resume`.
   Existing successful original E0 replay results are reused; they are never
   replayed with the repaired runtime.
9. Read-only evidence synthesis:
   `python -m v6_mujoco.system_capture --phase S02 --mode report`.
   This also audits every completed physical observation with the unchanged
   evaluator, generates reference-point budgets and the four scientific
   figure classes, and writes the paper supplement.
   `tools/s02_timing.py` separately records CV/CA update and 12 ms blind-forecast
   timing from the archived public packet stream. Its first diagnostic loader
   omitted the outer packet envelope and stopped before any estimator update;
   the schema correction and conservative 2 s budget charge are in the ledger.
   No robot integration occurred in either timing invocation.
   `tools/s02_sensor_requirements.py` adds eight offline stationary covariance
   scenarios with fixed estimator parameters, unchanged bias bounds and varied
   hypothetical sampling/noise/delay. It verifies agreement with the original
   contact-mode floor and records its own timing. These are sensing requirements,
   not additional estimator candidates, robot attempts, or sensor deployments.
10. Closure audit: `python tools/s02_completion_audit.py` with repository root
    on PYTHONPATH; then refresh the handoff evidence hashes to include the
    audit and final ledger. Verify `git diff --check`, stage only S02 additions
    and the explicit CLI/registry/attributes changes, and commit locally.

Steps 8–10 have machine-readable completion evidence in their output files;
the recipe itself is not proof they passed. Replays are independent validations,
not new control samples. The two E0 implementations are never pooled as
independent method trials.

## Reproduction and original-failure replay

The runtime checks frozen checkout byte hashes and environment identity before
running. Do not overwrite this evidence directory to start another trial.
Use a separate checkout/output version and a newly authorized protocol for
future work. The CLI's no-duplicate-attempt guard intentionally rejects implicit
retries.

For the original failure, select the directory named by
`implementation_failure.json.source_key` under `source_versions/` as cwd,
set PYTHONPATH to that directory, set S02_EVIDENCE_ROOT to the absolute S02 output
directory, and invoke the repository's `tools/s02_failed_replay.py`.
The expected terminal PSD error is part of that reproduction.

Raw inputs are `runs/<actual ID>/raw/index.json` plus hash-verified compressed
blocks. Only input_applied / step_completed pairs form the replay prefix.
Derived arrays and figures never replace those raw records.

Figure scripts are `v6_mujoco/system_capture/estimation/fig1_errors.py`
through `fig4_outcomes.py`; invoke them as Python modules. Copies of these
scripts and their shared style are delivered beside the figures as a source
record. The package versions are the executable entry points.

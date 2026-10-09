# N209 commands and recovery

Working tree: `codex/n209-paper-ready-feasible-capture`, based on
`b1af09f89226fa6f2ae1b362f3d6225b3363cc17`. No push or merge.

Python used: the pre-existing Python3.10 environment in the N200 worktree,
MuJoCo3.3.2, NumPy1.26.4 and SciPy1.11.2. Set OPENBLAS_NUM_THREADS=1 and
OMP_NUM_THREADS=1. No new environment or global package installation was needed.

```powershell
$env:OPENBLAS_NUM_THREADS='1'
$env:OMP_NUM_THREADS='1'
python -m v6_mujoco.feasible_capture --prepare
python -m v6_mujoco.feasible_capture --audit-existing --resume
python -m v6_mujoco.feasible_capture --test
python -m v6_mujoco.feasible_capture --all --resume
```

The final command verifies historical hashes, runs the current isolated tests,
completes only missing selected seen regression names, replays saved attempts
with their archived source in separate processes, performs supporting historical
analysis, evaluates admission and rebuilds tables/figures/report/manuscript.
It does not retry any recorded physical failure. A present but incomplete attempt
is retained for inspection, not silently restarted. No N210/N211 or background
automation is invoked. Actual physical runs are serial under campaign.lock.

Executed development sequence (source/config identities were saved before each
model construction):

```text
--develop --name D01_V1_nominal --scenario nominal --sensor ideal
--develop --name D02_V1_H2 --scenario H2_prior --sensor ideal
--develop --name D03_V1_S01 --scenario nominal --sensor noisy
```

After the single documented V2 startup revision:

```text
--develop --name D04_V2_H2 --scenario H2_prior --sensor ideal
--regression --name R01_V2_nominal --scenario nominal --sensor ideal
--regression --name R02_V2_H2 --scenario H2_prior --sensor ideal
--regression --name R03_V2_S01 --scenario nominal --sensor noisy
--regression --name R04_V2_H1 --scenario H1_prior --sensor ideal
```

V1 cannot be recreated with the current V2 defaults; its configuration and online
modules are archived in runs/*/config.json and source_versions/<key>. Decision
replay selects those exact modules. `--replay-only --name NAME` refuses to replace
an existing validation; `--replay-only --resume` handles only missing final
`validation_timestamp_audited.json` files in separate processes. To make that
explicit per run, use `--replay-only --name NAME --validation-tag timestamp_audited`.
`--regression --resume` handles missing final seen runs. `--develop` without a name
reports the closed development status; it does not create a new tuning loop.

`--freeze` and `--validate-holdout --resume` evaluate the actual regression gate.
For this failed candidate they write NOT_FROZEN/NOT_EVALUATED artifacts and
generate no new scenario values. The planned generator has not been implemented
or run because this stage was not admitted; these commands do not pretend to
provide independent validation. A future algorithm requires a new task/protocol.

`python -m v6_mujoco.feasible_capture --paper` rebuilds all data-derived material.
`python -m v6_mujoco.feasible_capture.supporting_audit` performs same-packet
estimator and archived H1/H3 identification analysis, with zero new robot trials.
Scientific figure scripts are independently executable under figures/.
Paper source is Markdown; `.tex` files are optional editable include/table
fragments, not a standalone LaTeX document or a compiled submission PDF.

Artifacts use checkout-byte SHA256. The historical manifest is never overwritten
by prepare; expected algorithm modifications are identified by each new run.
Large traces and packet streams are stored through Git LFS. The local release
audit checks all required files, citations, figure links, validation status and
original safety definitions. Git commit is local only.

One initial D01 replay was interrupted before producing validation: its loop
repeatedly decoded NPZ members rather than caching arrays. The validation-only
implementation now materializes five needed arrays once. Consumed process CPU
and start/termination times are retained in replay_interruption.json and charged
as interrupted_dual_replay. The failed invocation log is preserved. This was no
new robot attempt and did not alter any saved state, control, result or candidate.

The first noisy decision replay then found an invalid-t0 covariance alias in
control_states.json, while the same-tick independently copied trace was correct.
All dependent validators were paused (CPU retained in replay_integrity_pause.json).
`python -m v6_mujoco.feasible_capture.log_integrity` audits the discrepancy across
all8 runs without changing their original files. Future snapshots use deep copy.
All8 replays use the same tagged validator: only the known invalid t0 covariance
is checked against the same-tick trace; every other field remains compared to
the original record. Original failed validation.json remains readable. The final
table labels the two affected runs PASS_TS_AUDIT; this does not change task failure.

`python -m v6_mujoco.feasible_capture.uncertainty_floor` independently solves and
iterates the translation Riccati recursion for the recorded sensing configuration.
It writes the conditional computed-margin floor and assumptions, with no robot
trajectory or change to process noise. `--all --resume` includes this calculation.
`python tools/n209_release_audit.py` checks the final release and rehashes the exact
declared inputs of the fresh paper claim audit. Regenerating paper/data after that
audit may require a new audit identity; a stale hash is never silently accepted.

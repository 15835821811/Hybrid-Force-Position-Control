# Actual execution and reproduction

Environment: Python 3.10.19, MuJoCo 3.3.2, NumPy 1.26.4, SciPy 1.11.2; `.venv310/Scripts/python.exe` in the managed worktree. Base `79c1df133a6d3330f88a594020964d8eecbc3e0c`. Branch `codex/n201s-to-n204-grasp-detumbling`.

Executed sequence (all paths relative to repository):

1. `python -m v6_mujoco.postgrasp_campaign --prepare` — froze all conditional stages, parameters, identities and four-hour budget before first new physical run.
2. `python -m v6_mujoco.postgrasp_campaign --stage S --resume` — 19 restricted fixture cases.
3. Preflight `selftest.run()` — first short unit batch.
4. `python -m v6_mujoco.postgrasp_campaign --stage all --resume` — preserved P_Z diagnostic stop at 0.006 s.
5. Archived v1 sources; corrected geometry-aware diagnostic; second `selftest.run()`; `python -m v6_mujoco.postgrasp_campaign.correction_audit` — read-only audit and replay of P_Z, no closed-loop restart.
6. `python -m v6_mujoco.postgrasp_campaign --stage all --resume` — P_D, P_D_fine, L_nominal, L_fine, V_light, V_heavy; strict recorded-input replays; G skipped as unnecessary.
7. `python -m v6_mujoco.postgrasp_campaign --report` — recomputed metrics from saved arrays and generated figures/report. Repeated only for layout correction, no dynamics.
8. `python -m v6_mujoco.postgrasp_campaign.render --preview` — read-only rendering preview; corrected phase field and camera framing.
9. `python -m v6_mujoco.postgrasp_campaign.render` — six complete conditions × seven videos, 300 frames each, no mj_step.
10. `python -m v6_mujoco.postgrasp_campaign --resume` — verified completed identities and skipped all physical runs, counts unchanged.
11. `python -m v6_mujoco.postgrasp_campaign.delivery` — read-only completion checks and delivery hashes.

`python -m v6_mujoco.postgrasp_campaign --validate-only` is the supported additional recorded-input replay entry point; it adds replay ledger entries, never new closed-loop evidence. Existing strict replay results are already included; no redundant final replay batch was added.

The original resource start is frozen. After the budget expires, archived result reporting/rendering and recorded-input validation remain available. Starting a new physics campaign requires a separate preregistration/output identity; do not edit the old manifest to extend this campaign.

Git delivery: contract/S and initial P_Z baseline committed separately; corrected baseline/latch/scenario implementation and evidence followed by derived report/visual delivery. Upload uses `git push -u hybrid codex/n201s-to-n204-grasp-detumbling`, as explicitly requested at the end of the current user attachment. No merge and no edits to historical branches.

# N205 actual execution

Base: `7c521550c39e7cea97c83634e21504c888f5f8af`.
New branch: `codex/n205-end-to-end-capture-detumbling`.
Python: existing `.venv310/Scripts/python.exe`, Python3.10.19 / MuJoCo3.3.2.

1. Inspected clean worktree, baseline and historical source identities; read current user attachment.
2. `git switch -c codex/n205-end-to-end-capture-detumbling`.
3. Added a new unified model, named adapter, isolated-copy C1 controller, state machine, observer/gates, independent replay and CLI. No old runner was invoked to generate physics.
4. `python -m v6_mujoco.end_to_end_capture --prepare --test` — froze experiment/design/source/model/initial/contact contracts before formal execution; first short unit batch passed.
5. Added explicit physical tool-face and original C1 terminal gate checks before formal execution. Preserved first unit artifact as `unit_tests_v1.json`; second `--test` passed. Two batches each contain 69 physics steps and 9 replay steps, plus static checks. Neither is an integrated performance scenario.
6. `python -m v6_mujoco.end_to_end_capture --all --resume` — one E_nominal physical attempt, stopping at7.362s for the frozen target distance gate. Independent replay passed. Fine/light/heavy automatically not admitted. No physical rerun.
7. `python -m v6_mujoco.end_to_end_capture --report` — metrics verified, six scientific figure groups, report and qualification matrix. Added isolated static historical C1 t=8 geometry diagnostic; zero dynamics and no write to actual plant.
8. `python -m v6_mujoco.end_to_end_capture.render` — four actual-horizon videos. First renderer invocation detected insufficient framebuffer width before any frame; adjusted display-only framebuffer. Refined close-up camera and translucent target for visibility. No dynamics in any renderer invocation.
9. `python -m v6_mujoco.end_to_end_capture --all --resume` — same failed trace returned without new physics or replay; counts unchanged.
10. `python -m v6_mujoco.end_to_end_capture.audit` — final read-only source/data/coverage/delivery audit.

Supported entry points: `--prepare`, `--scenario nominal`, `--all --resume`, `--validate-only`, `--report`, `--test`. For this completed bounded attempt use `--resume`; omitting it refuses existing evidence. Additional `--validate-only` performs recorded-input replay and adds a ledger entry, never a new end-to-end sample.

The four-video delivery contains overview, interface close-up, five-view composite, and body-following side view. Actual physical horizon is0–7.362s,222 frames at30fps encode7.4s. No fake18s video and no old postgrasp clip appended.

User attachment section1 initially requested local-only, but its final paragraph explicitly requested separate-branch GitHub upload and refreshed media. The latter instruction is followed. No merge or old-branch pointer movement.

# N206 commands and evidence accounting

Base: `f2c4cf5cc12dd299589dc4e0a3f32d52ed085403`.
Worktree: `C:/Users/admin/.codex/worktrees/n206-terminal-geometry/力位形混合控制`.
Branch: `codex/n206-terminal-geometry-feasibility`.
Interpreter: existing `C:/Users/admin/.codex/worktrees/n200-postgrasp-baseline/力位形混合控制/.venv310/Scripts/python.exe`, Python 3.10.19 / MuJoCo 3.3.2.

1. Inspected original working tree, full SHA, branch pointers, source model and result identities. Original untracked files were preserved. Created separate managed worktree from the exact SHA, then `git switch -c codex/n206-terminal-geometry-feasibility`.
2. `git lfs checkout output/fpmfc/n110/live_twist_handoff/precontact/C1/trace.npz output/fpmfc/n205_end_to_end/E_nominal/trace.npz`. Verified all N205 source/design identities after expanding local LFS objects.
3. Read N205 report/config/contracts/qualification/trace/historical diagnosis and installed-version official XML/API/Computation documentation.
4. `python -m v6_mujoco.geometry_capture.audit`: G0/G1 saved-state and ideal rigid-chain geometry only; zero physics. Repeated after adding independently labelled closest-point endpoints; original scalar distances unchanged.
5. `python -m v6_mujoco.geometry_capture.design`: frozen 2/5/10/20 mm route-B geometry candidates. Initial static old-inertia-frame assertion exposed compiler bookkeeping for the massless flange; explicitly preserved its old inertial frame. No dynamics. Final selection uses a 20 mm increment and positive-mass contiguous tool.
6. `python -m v6_mujoco.geometry_capture.planning`: exactly three predeclared 0–8 s kinematic integrations, no torque servo or `mj_step`. All coarse-grid hard QPs passed; coarse step bounds were conservative near intentional pad contact.
7. `python -c "from v6_mujoco.geometry_capture.planning import refine_saved; refine_saved()"`: adaptive checks of the same three saved paths, no extra candidate. Preserved original coarse-bound results. Standard C1 selected first; no postgrasp metrics read.
8. `python -m v6_mujoco.geometry_capture.selftest`: static diagnostics and short units, with failures/corrections fully described in tests.log. Two successful batches each used 9 robot physics steps plus 9 independent replay steps; one failed unit used 3 robot steps before an uninitialized derived-frame replay constructor failed. No complete controller trial occurred during these units.
9. `python -m v6_mujoco.geometry_capture.interface_tests`: frozen affected scope, 16 restricted direction tests and 3 original fine comparisons, each 0.9 s in a two-free-body distal-composite fixture. All passed with unchanged candidate2 parameters and load envelope. Not full-robot controller attempts.
10. `python -m v6_mujoco.geometry_capture --prepare --all --resume`: freeze unique model/reference/control version, then exactly one nominal full-controller attempt. Actual integration reached 8 s and failed the original capture gate; no latch, no postgrasp integration. Fine/light/heavy are skipped. Recorded-control independent replay follows without controller update calls. No retry or control/connection tuning.
11. Supplemental read-only audits of the already saved planning endpoints confirmed standard C1 satisfies stricter capture gates, while the two unselected normal-offset templates do not. Independent link-chain radius accounting confirmed the conservative 3 m bound. These audits did not generate a new trajectory or alter the selected reference.
12. `python -m v6_mujoco.geometry_capture.figures`: geometry, planning and actual-horizon scientific figures. Actual figures require replay validation.
13. `python -m v6_mujoco.geometry_capture.render`: replay-verified actual saved states only. Exactly two videos: five fixed views plus continuous base-body side in one composite; interface close-up. No historical splice or fabricated 18 s segment.
14. `python -m v6_mujoco.geometry_capture.delivery`: report, qualification matrix, visualization index and delivery identities, requiring successful independent replay.
15. `python -m v6_mujoco.geometry_capture.completion_audit`: 25 static source/result/budget/media/link checks; no physics. Figure review followed by plotting-layer label/margin fixes and resealed figure/report manifests.
16. `git add .gitattributes README.md configs/n206_geometry_capture.yaml models/flexiv_rizon4s_n206_tool_scene.xml paper/N206_GEOMETRY_COMPATIBILITY_REPORT.md v6_mujoco/geometry_capture output/fpmfc/n206_geometry_capture`; `git diff --cached --check`; local commit; `git push -u hybrid codex/n206-terminal-geometry-feasibility`. No merge, force push or historical branch pointer changes. Remote SHA checked against local HEAD after push.

The final user instruction explicitly asks for GitHub upload and refreshed visuals, superseding the attachment's earlier local-only sentence. Historical directories keep their original provenance. Current branch README routes to N206 evidence; it does not relabel old segmented success as N206.

Completed formal evidence must be accessed with `--resume`. Omitting it refuses overwrite. `--validate-only` adds independent replay work, never a controller attempt. This campaign does not authorize more than four formal attempts, and a physical nominal failure blocks dependent scenarios regardless of remaining budget.

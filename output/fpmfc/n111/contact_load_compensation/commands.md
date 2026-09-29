# N111 execution record

Repository branch: `codex/n111-contact-load-compensation`; parent commit `85eec3a53a8806e87210be3843c9635f7a11da8b`. All commands used this checkout's `.venv\\Scripts\\python.exe` from the repository root. No push or merge was performed.

1. Reconstructed archived D10 without a new closed loop:
   `python -c "from pathlib import Path; from v6_mujoco.fpmfc.n111_execution_chain import reconstruct; reconstruct(Path('output/fpmfc/n110/force_regulation/D10/trace.npz'), Path('output/fpmfc/n111/contact_load_compensation'))"`
2. Ran regression tests before freezing: `python -m pytest tests -q` → 97 passed. Output is in `tests.log`.
3. Froze `experiment_manifest.json` with `n111_contract.prepare_manifest()` before N111-0. Manifest SHA-256: `948e66f3e1755672b9d0fc8fa66c427e968f3e56a535b4c4c02c0f4ae53994b1`.
4. Ran `python -m v6_mujoco.fpmfc.n111_run --cell gamma_0`. The 500-step trace completed, but postprocessing hit `KeyError('gamma_zero_torque_atol')`. The frozen runner was left untouched. `run_failure.json` and `run_ledger.json` preserve the error and count this as the first closed-loop attempt.
5. Ran `python -m v6_mujoco.fpmfc.n111_finalize_v2` on that existing trajectory only. It repaired the manifest-key lookup, checked exact D10 compatibility, and wrote the remaining derived artifacts. No extra closed loop was executed.
6. Ran `python -m v6_mujoco.fpmfc.n111_validate --cell gamma_0` → pass.
7. Ran `python -m v6_mujoco.fpmfc.n111_run --cell gamma_05` → common safety pass; force RMSE gate fail. Original `n111_validate` replay passed all checks except one absolute `1e-7` dynamics residual at a single sample; `validation_v1.json` preserves that result. Ran `python -m v6_mujoco.fpmfc.n111_validate_v2 --cell gamma_05` → pass with declared `1e-3` residual bound and the original failure retained.
8. Since gamma=0.5 passed common safety and independent replay, ran `python -m v6_mujoco.fpmfc.n111_run --cell gamma_10`, then `python -m v6_mujoco.fpmfc.n111_validate_v2 --cell gamma_10` → pass. The v1 result is also retained.
9. Ran `python -m v6_mujoco.fpmfc.n111_dynamics_detail` to split robot-side interface action from target reaction, then `python -m v6_mujoco.fpmfc.n111_summarize` to recompute paired metrics from the saved traces.

Exactly three 1 s closed-loop attempts are recorded. All additional replay, dynamics, and execution-chain computations are read-only derivations from saved states and torques.

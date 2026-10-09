# N208 commands and provenance

Base 877446208453c441c226b5b5a613856f00c4b3fd; branch
codex/n208-passive-target-estimation-capture. Worktree:
C:/Users/admin/.codex/worktrees/n208-passive-target-estimation/力位形混合控制.
Interpreter: C:/Users/admin/.codex/worktrees/n200-postgrasp-baseline/力位形混合控制/.venv310/Scripts/python.exe
(existing Python 3.10 / MuJoCo 3.3.2 / NumPy 1.26.4 / SciPy 1.11.2).

1. Read user objective, N206 report, reference/approach/planning/runner/observer,
   target XML, physical interface observer, servo/HQP, damping and source config.
   Original E: working tree has untracked historical files; left untouched.
2. Created and attached managed worktree from exact N206 SHA; switched to new
   N208 branch. No N207 branch existed in inspected local/remote refs.
3. Registered parameter policy, asynchronous sensor contract, independent known
   model, SO(3) estimator, physical momentum fit and event control implementation.
4. `python -m v6_mujoco.adaptive_capture.selftest` -> tests_initial.log and unique
   selftests timestamp. Six test groups passed, zero full-robot physics steps.
   Isolated physics and static controller calls are separately labelled.
5. Registered pre-run ideal EKF process-noise design change in run_ledger.json;
   no change to hardware, capture, load or performance gates.
6. `python -m v6_mujoco.adaptive_capture.runner D01_ideal` with
   OPENBLAS_NUM_THREADS=1 and OMP_NUM_THREADS=1 -> D01.log. First full attempt.

Further commands/results are appended as executed. A command appearing here is
not itself proof of a passed test; inspect corresponding ledger and output.
7. Repeated isolated/self-contained validation after recorded development changes:
   `python -m v6_mujoco.adaptive_capture.selftest` -> tests_second.log,
   tests_pre_D02.log, tests_final_pre_D02.log; all groups passed. No additional
   full robot integration. Analytic reference/state-Jacobian checks and physical
   nonspherical/nonzero-COM mated predictive fixture included.
8. `python -m v6_mujoco.adaptive_capture.synthetic_validation` -> synthetic.log
   and synthetic_newton_euler_validation.json. Forced synthetic ODE only.
9. `python -m v6_mujoco.adaptive_capture.audit` -> static truth-access audit
   passed. `python -m compileall -q v6_mujoco/adaptive_capture` passed.
10. Profiled current observer+safety with ten static observations: about 0.052 s;
    no plant step. D01 remains on its archived original implementation, whose
    repeated YAML parsing made it much slower. No interruption or substituted
    trajectory. Reporting and figure tools developed while D01 completes.
11. Paper-figure skill Step7 read-only reviewer checked figure plans/code. Added
    capture twist, spatial trajectory projections, input-work accounting,
    singular values, exact error-tick masking, and fixed world camera framing.
    Final rendered visual QA remains required.
12. D01 completed at 28.178 s; latch 8.178 s. Fixed final-window world/relative
    maxima 0.0235693 / 0.000217928 deg/s. This is task performance only.
13. `python -m v6_mujoco.adaptive_capture.validate D01_ideal` ->
    D01_validation.log: both replays passed with zero state/torque error, using
    verified archived original online sources. Sampled physical residuals passed.
14. `python -m v6_mujoco.adaptive_capture.runner D02_ideal_governor` -> D02.log;
    second development attempt, current physical whitening/governor/logging.
15. `python -m v6_mujoco.adaptive_capture.validate D02_ideal_governor`:
    both replays passed with exact zero state/torque error.
16. `python -m v6_mujoco.adaptive_capture.runner D03_slower_approach --duration 8.4`:
    actual latch-load failure at 8.428 s, rho 1.04634145. This is an unsuccessful
    duration ablation, not a replacement nominal candidate. Serialization of a
    NumPy boolean then failed. Original trace/packets/events/source survived.
17. Recovered missing D03 summary only from preserved original evidence under
    separately recorded recovery identity; original partial metrics kept as
    metrics_serialization_failure.original.txt. CPU was unavailable after the
    crash, so original elapsed wall time was conservatively charged to CPU.
18. `python -m v6_mujoco.adaptive_capture.validate D03_slower_approach`:
    both original-source replays passed with zero state/torque error.
19. Existing finite-prior brake predictor moved to prospective latch admission;
    current-controller offline D03 packet-prefix replay rejected the offending
    event (predicted rho about 1.04554). D03_prospective_latch_rejection.json and
    D03_rejection_test.log preserve test and identity. No additional task attempt.
20. `python -m v6_mujoco.adaptive_capture.runner D04_noisy_development --sensor noisy`
    -> D04.log; fourth development attempt under predeclared noise/delay/seed.
21. D04 stopped at 8.788 s on target Newton residual; no latch. Original actuator
    and archived-controller decision replays passed exactly (D04_validation.log).
    The initial sampled audit omitted its final non-grid point; its failed final
    state is preserved in trace and D04_numerical_diagnostic.json. Validator now
    always samples the final point for subsequent runs.
22. Offline same-state diagnostic: original tolerance 1e-10 produced EOM residual
    6.42e-4 N at 2 iterations; 1e-12 produced 2.29e-16 N at 3 iterations. No plant
    integration. Tightened numerical tolerance, without changing hardware,
    contact model, noise, covariance, capture or physical acceptance thresholds.
23. `python -m v6_mujoco.adaptive_capture.selftest` -> tests_pre_D05.log, all nine
    groups passed. `python -m v6_mujoco.adaptive_capture.runner D05_final_nominal`
    -> D05.log; fifth development attempt, final proposed executable.
24. D05 stopped safely before latch: prospective prior forecasts rejected the
    candidate and HQP became infeasible at 9.420 s. Both original replays passed.
    D05_latch_prior_ensemble_diagnostic.json: at first proposed latch, central
    prior rho0.901 / low0.403 / high1.685, so nominal physical feasibility alone
    did not provide the registered finite-prior safety margin.
25. Final development candidate: constant kinematic position/orientation gains
    scaled by3, with all hardware limits and capture/load/predictive gates intact.
    Motivation and exact gains recorded before launch in run_ledger.json. Static
    truth-isolation check passed. No further development attempts permitted.
26. `python -m v6_mujoco.adaptive_capture.runner D06_final_gain3` -> D06.log.
27. D06 completed at27.992 s, latch7.992 s. Fixed-window world/relative maxima
    0.0235542 / 0.000077798 deg/s; max true rho0.363221.
28. `python -m v6_mujoco.adaptive_capture.validate D06_final_gain3` ->
    D06_validation.log: both replays passed exactly, final-state audit included.
29. `python -m v6_mujoco.adaptive_capture.campaign freeze --selected D06_final_gain3`
    succeeded once. Independent geometric mass-distribution cases constructed
    afterward; holdout SHA is in holdout_identity.json. No subsequent online
    algorithm/config/model changes are permitted.
30. Sequential frozen campaign queued: H1/H2/H3 each prior+identified, F01_fine,
    S01_noise_delay, S02_contact_mismatch. Each has an individual log and separate
    actuator/decision replay log; failures retain their actual truncated horizon.

31. Frozen campaign finished: H1/H3 pairs completed; H2 pair failed at7.820 s;
    F01 fine and S02 contact mismatch completed; S01 noise/delay failed7.540 s.
    All nine actuator and decision replays passed exactly. No online retuning.
32. `python -m v6_mujoco.adaptive_capture.report D06_final_gain3` generated
    comparison, qualification, identifiability, momentum and state summaries.
33. `python -m v6_mujoco.adaptive_capture.figures D06_final_gain3` generated
    all ten PNG/PDF figures. Caption/layout-only corrections were regenerated.
34. `python -m v6_mujoco.adaptive_capture.render D06_final_gain3` generated
    exactly two videos, 841 frames each. Renderer and ffmpeg exited zero.
35. Rendered figure review and six saved video frames checked; see
    visual_quality_review.md. Resource allowance and prediction counts saved
    separately in resource_accounting.json (allowance is not measured CPU).
36. Completion audit passed all15 checks, including frozen execution, all trace replays, media hashes, full validation schedule and budget. Both videos fully decoded without errors. Final Git scope review and independent-branch push follow.

37. Content commit 12ecd6faf54793d9d72e4f387831c73e6de7c335 pushed to hybrid/codex/n208-passive-target-estimation-capture. Git LFS27/27 objects (442 MB) uploaded successfully. git ls-remote matched content HEAD. A metadata-only follow-up stores push_receipt.json and the remote audit flag.

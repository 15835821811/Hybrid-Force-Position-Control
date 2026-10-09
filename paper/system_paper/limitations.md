# Limitations retained in the V1

1. C1 robust capture is unsupported: selected H2 and noisy S01 completion is not established. A predictive stop is a failed task, not a hardware recovery guarantee.
2. All new trials are development or already-seen regression cases. No independent clusters, paired B0/B1 effect, B2 ablation, fine-step result or population success rate exists.
3. Local three-sigma covariance margins and empirical marginal coverage are neither calibrated joint probability nor hard uncertainty bounds. A counterfactual audit of recorded 1–7 s approach covariance exceeds the noisy capture velocity thresholds; the run stops before any actual capture-guard call. A conditional Riccati analysis gives a 1.609697 mm/s floor for the current filter's computed allowance, above the 1 mm/s gate. This concerns the implemented covariance recursion, not a fundamental sensor error limit.
4. The scalar reference family and finite progress bandwidth cannot command arbitrary target transport. No claim establishes global unreachable geometry or impossibility for every controller.
5. Single-prior, sampled short-horizon prediction has no certified model-error or intersample bound, no recursive feasibility proof and no proven physical stop action. Future measurement corrections and contact-mode changes are absent from covariance prediction. It holds the last elapsed-interval process mode, which can lag a newly detected contact transition by one2 ms servo interval; no all-contact uncertainty certificate is claimed.
6. Translation shaper substeps are exact for constant jerk; rotation integration is approximate. The declared rejection policy for older out-of-order packets is not a general delayed smoother.
7. Finite-difference geometry and repeated optimization cost far more than the20 ms deadline. Legacy development timing excludes terminal exceptions; predictor maxima and final external timing disclose the difference.
   Estimator, reference, HQP and logging costs were not individually profiled on
   the original runs; only total controller calls and candidate prediction costs
   support the present latency statements. Detailed module profiling is future work.
8. Earlier measured CPU omits some serialization/orchestration overhead. Wall times and the precise resource scope are preserved; exact all-process CPU is not claimed.
9. Soft weld/contact simulations do not qualify a real clamp. The prior full-boundary interface qualification failure and hardware-load uncertainty remain in force.
10. Momentum idealizations do not absorb soft connection energy, switching work or numerical residuals. P/H and real interface quantities remain separately logged.
11. Finite regressor rank is not structural observability, parameter accuracy or control utility. H1 rank10 with COM error above5 mm and unchanged prior/identified actions prevent a control-benefit claim.
12. Three literature seeds are ABSTRACT_ONLY. No external method has been numerically reproduced. Actuated-base theories and rigid-environment-contact assumptions cannot silently replace this system's assumptions.
13. The two noisy control-state logs contain an invalid-t0 covariance alias. Original bytes and the initial failed validation remain preserved. The uniform final replay checks only that field against the independently copied same-tick trace; PASS_TS_AUDIT identifies the affected runs. This disclosed recording defect is not a changed physical trajectory or a retroactive task pass.

All limitations are reflected in the abstract, results, conclusion and machine-readable qualification matrix. V1 completion is not submission readiness.

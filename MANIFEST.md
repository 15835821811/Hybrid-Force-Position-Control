# Research Output Manifest

> Tracks generated artifacts produced by paper skills.

| Timestamp | Skill | File | Stage | Description |
|-----------|-------|------|-------|-------------|
| 2026-09-15 16:23 | /experiment-plan | paper/EXPERIMENT_PLAN_20260915_162339.md | experiment | timestamped Flexiv FPMFC reproduction design |
| 2026-09-15 16:23 | /experiment-plan | paper/EXPERIMENT_PLAN.md | experiment | latest Flexiv FPMFC reproduction design |
| 2026-09-15 16:23 | /experiment-plan | paper/EXPERIMENT_TRACKER_20260915_162339.md | experiment | timestamped execution tracker |
| 2026-09-15 16:23 | /experiment-plan | paper/EXPERIMENT_TRACKER.md | experiment | latest execution tracker |
| 2026-09-15 17:09 | /academic-research-suite | configs/fpmfc_paper.yaml | implementation | source-labelled adaptive Flexiv reproduction configuration |
| 2026-09-15 17:09 | /academic-research-suite | v6_mujoco/fpmfc/ | implementation | zero-momentum kinematics, arm shape, strict HQP, PSO, torque runner, and replay validator |
| 2026-09-15 17:09 | /academic-research-suite | output/fpmfc/optimization/smoke_seed0.json | experiment-smoke | 4-particle × 2-generation PSO pipeline check; not a converged paper result |
| 2026-09-15 17:09 | /academic-research-suite | output/fpmfc/precontact/smoke_t8_psi_m055/ | experiment-smoke | verified 8 s feasible candidate and deterministic torque replay |
| 2026-09-15 17:09 | /academic-research-suite | output/fpmfc/precontact/smoke_pso_seed0_best/ | experiment-smoke | verified dynamic replay of smoke PSO best candidate |
| 2026-09-15 17:15 | /academic-research-suite | output/fpmfc/optimization/fixed_psi_0_smoke.json | experiment-smoke | fixed-shape time-only PSO pipeline check for psi=0 |
| 2026-09-15 17:15 | /academic-research-suite | output/fpmfc/optimization/fixed_psi_pi2_smoke.json | experiment-smoke | fixed-shape time-only PSO pipeline check for psi=pi/2; no feasible candidate in the tiny smoke budget |
| 2026-09-15 17:15 | /academic-research-suite | output/fpmfc/precontact/smoke_seed0_candidates/ | experiment-smoke | top-candidate batch dynamic reranking and independent replay check |
| 2026-09-15 17:30 | /academic-research-suite | paper/FPMFC_REPRODUCTION_REPORT.md | report | adaptive Flexiv reproduction status, evidence boundary, smoke results, and formal gates |
| 2026-09-15 17:30 | /paper-figure | paper/figures/ | figures | reproducible vector-figure scripts, LaTeX snippets, and visually checked smoke PDFs |
| 2026-09-15 17:30 | /academic-research-suite | output/fpmfc/reachability/fixed_psipi2_1s_grid.json | experiment-smoke | 18-point fixed-pi/2 feasibility scan over 8–25 s |
| 2026-09-15 17:30 | /academic-research-suite | output/fpmfc/comparison/smoke_fixed_time_v3/ | experiment-smoke | gate-aware fixed-time comparison; fixed baselines unqualified |
| 2026-09-15 17:30 | /academic-research-suite | output/fpmfc/comparison/smoke_reoptimized_time_v3/ | experiment-smoke | verified joint-vs-fixed0 reoptimized-time smoke comparison |
| 2026-09-15 17:38 | /academic-research-suite | diagnostics/simscape_jacobian_home.mat | experiment | source Simscape home-state J, J_bm, pose, and geometry export |
| 2026-09-15 17:38 | /academic-research-suite | diagnostics/fpmfc_simscape_cross_validation.json | verification | passing Simscape-to-MuJoCo Jacobian and reaction-map audit |
| 2026-09-15 17:34 | /academic-research-suite | output/fpmfc/optimization/weights_1_1_smoke/ | experiment-smoke | runtime objective-weight override and effective-config-hash check |
| 2026-09-15 17:44 | /academic-research-suite | output/fpmfc/precontact/smoke_controller_ablation_v1/ | experiment-smoke | replay-audited full/no-shape/no-base-reaction controller ablation at one fixed candidate |
| 2026-09-15 17:44 | /academic-research-suite | output/fpmfc/comparison/smoke_controller_ablation_v1/ | experiment-smoke | configuration-difference-audited ablation comparison; no-shape unqualified and no positive base-reaction claim allowed |
| 2026-09-15 18:24 | /academic-research-suite | output/fpmfc/optimization/precheck_seed0.json | experiment-precheck | R020 20-particle, 50-generation-cap seed-0 PSO; converged at generation 49 after 980 evaluations |
| 2026-09-15 18:25 | /academic-research-suite | output/fpmfc/precontact/precheck_seed0_top5/ | verification | 500 Hz dynamic reranking of R020 top-5; 4/5 accepted, top-1 preserved, deterministic replay checked |
| 2026-09-15 18:27 | /academic-research-suite | paper/R020_PRECHECK_REPORT.md | report | audited R020 command, hashes, convergence, dynamic reranking, and clearance-boundary risk |
| 2026-09-15 18:27 | /paper-figure | paper/figures/fig_r020_pso_convergence.pdf | figure | visually verified vector convergence curve for R020 |
| 2026-09-15 18:32 | /academic-research-suite | output/fpmfc/optimization/planning45_accept40_smoke.json | experiment-smoke | 4×2 verification of a 45 mm planning / 40 mm acceptance runtime override |
| 2026-09-15 18:32 | /academic-research-suite | output/fpmfc/precontact/planning45_accept40_smoke_top1/ | verification | matching-effective-hash 500 Hz replay of the dual-threshold smoke candidate |
| 2026-09-15 18:36 | /academic-research-suite | output/fpmfc/optimization/precheck_seed0_recheck_planning45.json | diagnostic | fixed-candidate 45 mm counterfactual; only original rank-5 remains feasible, not a substitute for rerunning PSO |
| 2026-09-15 18:37 | /academic-research-suite | output/fpmfc/optimization/planning45_snapshot_contract_smoke.json | contract-smoke | deliberately minimal 2×1 run proving full effective-config snapshot serialization; sampled candidate infeasible and retained |
| 2026-09-15 18:37 | /academic-research-suite | output/fpmfc/precontact/planning45_snapshot_contract_smoke_top1/ | contract-smoke | optimizer/dynamics hash equality and deterministic replay verified; candidate acceptance remains failed |
| 2026-09-15 18:38 | /academic-research-suite | output/fpmfc/optimization/suite_planning45_contract_smoke/ | contract-smoke | suite propagation, aggregation hash, and resume/skip behavior verified; 2×1 sampled candidate retained as infeasible |
| 2026-09-15 19:02 | /academic-research-suite | output/fpmfc/optimization/precheck_seed0_planning45.json | experiment-precheck | R020b 20×50 planning-45/acceptance-40 PSO; 1000 evaluations, feasible best 3.84665 |
| 2026-09-15 19:04 | /academic-research-suite | output/fpmfc/precontact/precheck_seed0_planning45_top5_v2/ | verification | corrected joint planning/dynamic qualification; 4/5 qualified, all torque traces replayed deterministically |
| 2026-09-15 19:05 | /academic-research-suite | paper/R020B_PLANNING45_PRECHECK_REPORT.md | report | audited robust precheck, top-5 reranking, and 40-vs-45 mm tradeoff |
| 2026-09-15 19:05 | /paper-figure | paper/figures/fig_r020_clearance_comparison.pdf | figure | visually verified 40 mm versus 45 mm PSO convergence comparison |
| 2026-09-15 19:08 | /academic-research-suite | paper/R021_RUNBOOK.md | plan | frozen first-batch commands, hashes, hardware preflight, timeouts, recovery, and pass conditions |
| 2026-09-15 19:08 | /academic-research-suite | output/fpmfc/optimization/r021_preflight.json | preflight | machine-readable seeds 0–2 launch contract; output root confirmed absent |
| 2026-09-15 19:10 | /academic-research-suite | output/fpmfc/optimization/suite_parallel_contract_smoke/ | contract-smoke | real two-process dispatch, common effective hash, aggregation, and resume/skip verified; infeasible samples retained |
| 2026-09-15 19:12 | /academic-research-suite | v6_mujoco/fpmfc/run_suite_candidates.py | implementation | parallel, resumable top-k torque-level replay and joint planning/dynamic qualification aggregation |
| 2026-09-15 19:12 | /academic-research-suite | output/fpmfc/precontact/suite_parallel_contract_smoke_dynamics/ | contract-smoke | two-seed parallel dynamic replay, failure preservation, qualification filtering, and resume/skip verified |
| 2026-09-15 19:16 | /academic-research-suite | v6_mujoco/fpmfc/provenance.py | implementation | composite SHA-256 identity over core control/optimization/replay code and MuJoCo XML |
| 2026-09-15 19:16 | /academic-research-suite | output/fpmfc/optimization/suite_identity_contract_smoke/ | contract-smoke | identity-bearing two-seed parallel optimization; infeasible random samples retained |
| 2026-09-15 19:16 | /academic-research-suite | output/fpmfc/precontact/suite_identity_contract_smoke_dynamics/ | contract-smoke | optimization/dynamics implementation-identity match and identity-aware resume skip verified |
| 2026-09-15 19:50 | /academic-research-suite | output/fpmfc/optimization/formal_planning45/ | experiment-formal-partial | R021 seeds 0–2, 20×1000 cap, 3/3 feasible, 3440 evaluations, implementation identity frozen |
| 2026-09-15 19:52 | /academic-research-suite | output/fpmfc/precontact/formal_planning45_batch_00_02/ | verification | 15/15 dynamic acceptance and replay, 14/15 joint qualification, top-1 preserved for all three seeds |
| 2026-09-15 19:55 | /academic-research-suite | paper/R021_BATCH_00_02_REPORT.md | report | commands, hashes, per-seed optimization/dynamic results, selected trajectory gates, evidence boundary |
| 2026-09-15 20:00 | /visualize | v6_mujoco/fpmfc/visualize_capture.py | implementation | verified-trace MuJoCo video renderer with tumbling grasp point, frames, metrics, storyboard, and manifest |
| 2026-09-15 20:00 | /visualize | output/fpmfc/visualization/r021_batch_00_02/ | visualization | 515-frame selected-capture MP4, three-state storyboard, source-hash and media validation manifest |
| 2026-09-15 20:05 | /academic-research-suite | output/fpmfc/optimization/r021_batch_03_09_preflight.json | preflight | hash-verified cumulative seeds 0–9 command; seeds 0–2 reusable, 3–9 absent, no active experiment, awaiting confirmation |
| 2026-09-18 10:00 | /academic-research-suite | paper/FPMFC_SMOOTHING_METHOD_REVIEW.md | method-review | primary-source rationale, C2 radial saturation, discrete jerk constraint, and jerk-aware braking derivation |
| 2026-09-18 10:00 | /academic-research-suite | v6_mujoco/hierarchical_qp.py | implementation | smooth angular saturation, 80 rad/s3 joint jerk bounds, and velocity/position viability bounds |
| 2026-09-18 10:00 | /academic-research-suite | output/fpmfc/precontact/adaptive_cube_c095_014_095_face_aligned_v6_smooth_jerk80/selected_candidate/ | verification | 500 Hz selected replay with all acceptance gates and deterministic independent replay passed |
| 2026-09-18 10:00 | /visualize | output/fpmfc/analysis/adaptive_cube_c095_014_095_face_aligned_v6_smooth_jerk80/ | figure | visually checked six-second pre/post acceleration and jerk comparison |
| 2026-09-18 10:00 | /visualize | output/fpmfc/visualization/adaptive_cube_c095_014_095_face_aligned_v6_smooth_jerk80/five_view_joint_kinematics/ | visualization | five independent MP4s, synchronized composite MP4, and q/dq/ddq figures; manifest passed |

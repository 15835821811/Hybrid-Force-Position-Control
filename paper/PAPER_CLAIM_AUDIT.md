# Experiment Claim Audit Report

**Overall verdict: WARN**

WARN: 260 claim records; 99 exact, 123 rounding-consistent, 38 explicitly unverified/ambiguous; 0 material numerical/configuration/aggregation mismatches. All 159 declared hashes match.

**Auditor:** paper architect/reviewer, fresh zero-context reviewer `/root/claim_audit_final`.
**Generated:** 2026-10-09T09:09:21.110299+00:00

**Scope:** canonical Markdown manuscript, five generated tables, five figure images and their standalone captions, and exactly the 159 declared inputs. All input hashes were verified before analysis and again at emission. No simulations were rerun.

## Principal findings

- Core empirical numerical claims reconcile: three terminal hard-set conflicts; all eight attempts (including five failures); 3001 samples in each 1–7 s noisy audit; complete two-second windows only for the three completed runs; historical H1/H3 exact torque/state identities; and the conditional covariance fixed-point number.
- No material number, configuration-comparison, denominator, aggregation, missing-failure or table/figure mismatch was found. Figures were inspected visually. Dense samples and repeated seen runs are not independent trials.
- WARN remains because this bounded evidence set cannot establish all implementation, historical-provenance, mathematical and preserved-contract assertions. Saved validator PASS fields were not promoted into source-level assurance.
- A frame definition is needed for “20 mm”: the recorded physical face/flange distance is 19.8 mm. This may be a distinct reference interface; it is an ambiguous mapping, not a proven wrong number.

## Counts

| Status | Count |
|---|---:|
| ambiguous_mapping | 2 |
| exact_match | 99 |
| missing_evidence | 7 |
| rounding_ok | 123 |
| unsupported_claim | 29 |

## Remaining issues

### Claim 178: ambiguous_mapping

**Location:** system_paper/manuscript_v1.md: §3

> the fixed 20 mm tool/interface geometry

Physical tool-face to flange distance is 19.8 mm, whereas the text may mean a distinct 20 mm reference/interface frame. No model geometry or frame-definition source is declared. This is not classified as a wrong number without that mapping.

**Action:** Define which frame/offset is 20 mm and distinguish it from the recorded physical face, or add its geometry definition to the audit inputs.

### Claim 180: unsupported_claim

**Location:** system_paper/manuscript_v1.md: §1; §3; §4.3

> The controller uses its declared nominal prior, never the true target mass matrix.

Fixed-prior configuration and isolation-test results support intended architecture. A universal no-truth-access/data-flow assertion requires controller and model source, which are outside the manifest.

**Action:** Treat as unverified here; audit the archived controller/model interfaces before claiming source-level isolation.

### Claim 182: missing_evidence

**Location:** system_paper/manuscript_v1.md: §3

> After locking, position and attitude limits are 0.5 mm and 0.1 degree.

These can be declared task requirements, but their original/frozen definition and enforcement are not independently established by the supplied configuration set.

**Action:** Supply the post-lock safety contract/configuration or explicitly label them as the manuscript task definition.

### Claim 184: unsupported_claim

**Location:** system_paper/manuscript_v1.md: §3

> All original noncontact clearance, permitted interface penetration, joint, torque and separate linear/angular momentum checks remain active.

No recorded violation does not independently prove that every original check remained active. Original safety contracts and implementing source are not in the declared set.

**Action:** Preserve the recorded-safety scope and separately verify all safety predicates against the archived source/contracts.

### Claim 188: unsupported_claim

**Location:** system_paper/manuscript_v1.md: §3

> The approach servo applies its actual bounded ramp between task commands. The primary QP minimizes tracking error subject to distance and joint feasibility inequalities. Tracking is already a soft least-squares objective.

Frozen historical inequalities are inspectable; servo ramp and full objective construction in the new implementation are not provable from output arrays.

**Action:** Audit the exact archived servo/HQP source for this implementation assertion.

### Claim 189: unsupported_claim

**Location:** system_paper/manuscript_v1.md: §4.1

> A 0.25 s history stores the contact/process mode used over each past interval.

A passing mode_history test result is supplied, but no history-capacity configuration, timestamp algorithm or test code is declared.

**Action:** Verify the 0.25 s bound and mode-history logic against archived source.

### Claim 190: unsupported_claim

**Location:** system_paper/manuscript_v1.md: §4.1

> Batches are sorted by measurement time; duplicate and older out-of-order samples are rejected rather than retrospectively assimilated.

Observed event stamps are increasing and duplicate/stale tests report success. The complete rejection/sorting policy and SO(3) update ordering cannot be established without implementation/test inputs.

**Action:** Limit evidence to observed timestamp behavior or provide source for policy verification.

### Claim 192: unsupported_claim

**Location:** system_paper/manuscript_v1.md: §4.1

> Invalid or excessively old estimates produce an explicit `ESTIMATE_UNRELIABLE` stop.

Tests report explicit abort-mode verification, but no physical run here takes ESTIMATE_UNRELIABLE and the enforcing source is undeclared.

**Action:** Retain as intended behavior pending source-level verification.

### Claim 196: unsupported_claim

**Location:** system_paper/manuscript_v1.md: §4.1

> Translational substeps integrate constant jerk exactly. Rotation uses a Lie increment with a discretization approximation for noncommuting angular derivatives.

Mathematical/integration implementation assertion, not established by raw run outputs or test-result flags alone.

**Action:** Keep explicitly unverified in this evidence audit; inspect the reference shaper source/derivation.

### Claim 197: unsupported_claim

**Location:** system_paper/manuscript_v1.md: §4.1

> Capture tests use the raw timestamped estimate and its covariance, never the smoother reference state.

Separate raw/reference fields and passing test are consistent with the description; universal data-flow exclusivity cannot be derived from output records.

**Action:** Audit guard inputs in archived source; do not interpret this report as source-level confirmation.

### Claim 198: unsupported_claim

**Location:** system_paper/manuscript_v1.md: §4.2

> $$T_{WF,d}=T_{WG,r}Q_{GE}(s)T_{FE}^{-1},\qquad s\in[0,1].$$

Explicitly unverified mathematical/frame assertion in this experiment audit.

**Action:** Use a separate proof/frame audit with the declared theory sources.

### Claim 199: unsupported_claim

**Location:** system_paper/manuscript_v1.md: §4.2

> $$v_F=v+\omega\times r+Rq'(s)\dot s.$$

No numerical truth test of the complete reference differentiation is supplied in this manifest.

**Action:** Verify geometry and conventions in a separate theory/source audit.

### Claim 200: unsupported_claim

**Location:** system_paper/manuscript_v1.md: §4.2

> The nominal relative path has zero first and second endpoint jets.

Endpoint jets and composed smoothness are mathematical/implementation properties, not identifiable from sampled trajectory records.

**Action:** Audit the path definition and endpoint derivation.

### Claim 201: unsupported_claim

**Location:** system_paper/manuscript_v1.md: §4.2

> Progress demand is filtered through three positive first-order stages with a 0.30 s time constant.

The time constant is declared; current three-stage structure and positivity require filter source. Historical snapshots expose a different predecessor state layout and cannot prove the new recurrence.

**Action:** Verify stage recurrence from the archived governor source.

### Claim 203: unsupported_claim

**Location:** system_paper/manuscript_v1.md: §4.2

> It integrates an independently built nominal-prior model for 0.20 s with the actual servo ramp, the original HQP and sampled contact model. It never reads future physical states.

0.20 s/100 steps are logged. Independence, exact servo/HQP reuse, same controller history and no future-truth access are implementation assertions lacking archived source verification.

**Action:** Provide source/data-flow audit; preserve finite prior-model scope.

### Claim 204: unsupported_claim

**Location:** system_paper/manuscript_v1.md: §4.2

> Local clearance margins use a directional Jacobian covariance propagation and separate bias allowances.

Bias parameters and a numerical gradient test exist. Runtime covariance propagation, constraint admission completeness, held process mode and one-interval transition lag require source.

**Action:** Verify margin/predictor code separately; test-result logs alone do not certify all cases.

### Claim 205: unsupported_claim

**Location:** system_paper/manuscript_v1.md: §4.2

> The sole second-version revision allows the initial 0.20 s of soft task-residual transient, because the first version rejected the multiaxis case at its initial task tick despite a feasible hard set.

Only new mission parameter is task_residual_startup_s=0.2 (plus version/schema). Raw records prove residual rejection at 0.04 s, but sole algorithm change and exact initial hard-set feasibility cannot be independently established without archived source and the new-run QP matrices.

**Action:** Separate verified configuration change from the unverified sole-code-change/hard-set assertions, or supply those inputs.

### Claim 209: unsupported_claim

**Location:** system_paper/manuscript_v1.md: §4.3

> Locking requires the original four estimated inequalities including uncertainty margins, sustained confirmation, a contact/load check and the unchanged prospective latch prediction.

All three recorded latch events satisfy the four value+margin inequalities, contact/load flags and true_capture=true. The sustained-confirmation/unchanged-prediction requirement and enforcement are not proved by event flags alone.

**Action:** Retain observed latch evidence; inspect the guard and preserved baseline source for stronger requirement claims.

### Claim 211: unsupported_claim

**Location:** system_paper/manuscript_v1.md: §5 conditional analysis

> An attained optimum z=0 is equivalent to nonempty original feasibility.

This statement is not rejected as mathematically false. The referenced derivations are outside the exact declared input set; no proof verdict is inferred from numerical consistency.

**Action:** Carry forward to a separate bounded proof audit; do not count this claim as experimentally verified.

### Claim 212: unsupported_claim

**Location:** system_paper/manuscript_v1.md: §5 conditional analysis

> Accepted predicted constraints imply actual sampled constraints only if appropriate state/model/linearization/input-hold error bounds cover their slack.

This statement is not rejected as mathematically false. The referenced derivations are outside the exact declared input set; no proof verdict is inferred from numerical consistency.

**Action:** Carry forward to a separate bounded proof audit; do not count this claim as experimentally verified.

### Claim 213: unsupported_claim

**Location:** system_paper/manuscript_v1.md: §5 conditional analysis

> With no external force or moment, ideal internal interaction pairs conserve total linear and angular momentum.

This statement is not rejected as mathematically false. The referenced derivations are outside the exact declared input set; no proof verdict is inferred from numerical consistency.

**Action:** Carry forward to a separate bounded proof audit; do not count this claim as experimentally verified.

### Claim 214: unsupported_claim

**Location:** system_paper/manuscript_v1.md: §5 conditional analysis

> Joint damping has power minus alpha times the weighted sum of squared joint velocities, hence nonpositive power when alpha and damping gains are nonnegative. Symmetric saturation preserves that sign.

This statement is not rejected as mathematically false. The referenced derivations are outside the exact declared input set; no proof verdict is inferred from numerical consistency.

**Action:** Carry forward to a separate bounded proof audit; do not count this claim as experimentally verified.

### Claim 215: unsupported_claim

**Location:** system_paper/manuscript_v1.md: §5 conditional analysis

> If all relative motion stops, common angular velocity equals the inverse locked inertia times conserved COM angular momentum.

This statement is not rejected as mathematically false. The referenced derivations are outside the exact declared input set; no proof verdict is inferred from numerical consistency.

**Action:** Carry forward to a separate bounded proof audit; do not count this claim as experimentally verified.

### Claim 216: unsupported_claim

**Location:** system_paper/manuscript_v1.md: §5 conditional analysis

> Pose-only free motion cannot identify that common scale.

This statement is not rejected as mathematically false. The referenced derivations are outside the exact declared input set; no proof verdict is inferred from numerical consistency.

**Action:** Carry forward to a separate bounded proof audit; do not count this claim as experimentally verified.

### Claim 217: unsupported_claim

**Location:** system_paper/manuscript_v1.md: §5 conditional analysis

> A finite regressor's selected singular vectors define an update subspace, and a positive physical pseudoinertia imposes inertia triangle conditions; neither proves parameter accuracy or control utility.

This statement is not rejected as mathematically false. The referenced derivations are outside the exact declared input set; no proof verdict is inferred from numerical consistency.

**Action:** Carry forward to a separate bounded proof audit; do not count this claim as experimentally verified.

### Claim 219: missing_evidence

**Location:** system_paper/manuscript_v1.md: §6

> Source and configuration hashes, full packets, references, covariance, QP data, events, actuator inputs, plant trajectories and resource logs are saved for every new attempt, including early failures.

Packets, references/covariance, events, trajectories and scalar resource metrics exist for all 8. Full new-run QP matrices and full per-call timing resources are not in this manifest (tasks.json contains summaries). Source hashes are not source bytes.

**Action:** Provide the full QP/resource artifacts or narrow the saved-data completeness claim to what is actually supplied.

### Claim 221: unsupported_claim

**Location:** system_paper/manuscript_v1.md: §6

> These limits are enforced by the N209 run ledger; the inherited legacy mission metadata fields max_development=6 and max_attempts=16 are not used by this runner.

Actual record count obeys limits and legacy metadata is present. Enforcement and non-use of the conflicting fields are source-level control-flow claims.

**Action:** Verify runner limit checks in archived source; report metadata consistency separately from enforcement.

### Claim 223: missing_evidence

**Location:** system_paper/manuscript_v1.md: §6

> The prespecified distribution and seed contract are retained for a future admitted campaign, without claiming they were validated.

Cannot verify preservation of a distinct future six-cluster/three-seed contract from the declared inputs.

**Action:** Include that protocol artifact or keep preservation explicitly unverified.

### Claim 226: unsupported_claim

**Location:** system_paper/manuscript_v1.md: §6

> Two independent replays are required for each attempt: one initializes the plant once at t=0 and reapplies recorded torque and latch events; another feeds the original packets through the corresponding archived controller source.

All eight validator records report both passes, zero final/action/intermediate errors, and matching packet counts. The t=0-only initialization, no trajectory injection, archived source selection and independence mechanisms cannot be verified from result JSON and source hash maps; replay code/archives are undeclared and no simulations were rerun.

**Action:** State that dual replay is reported by saved validators, or supply the corresponding archived validator/controller source for an implementation audit.

### Claim 228: unsupported_claim

**Location:** system_paper/manuscript_v1.md: §6

> subsequent snapshots were changed to deep copies, and all eight trajectories were revalidated using one common rule.

Uniform correction records and all eight revalidation outputs exist. Mutable-alias cause, subsequent deep-copy fix, and exclusivity of expected-field substitution require logger/validator source; the outputs cannot prove these implementation details.

**Action:** Preserve the disclosed timing correction and separately inspect the fix and common validation rule.

### Claim 232: missing_evidence

**Location:** system_paper/manuscript_v1.md: §7.1

> The archived H2 stop occurs before physical contact.

All five logged terminal pair distances are positive, including pad-to-target geometry at 8.90637 mm, consistent with precontact. However the archived H2 physical contact/event trace and all potential contact pairs are not declared.

**Action:** Supply the historical H2 contact/event record or qualify this to the logged pair distances.

### Claim 235: missing_evidence

**Location:** system_paper/figure_plan.md: F1 standalone caption

> Historical packet decisions and actuator
> torques were reproduced exactly; no new trajectory or repaired success is shown.

Historical torque/decision identity is not independently traceable from this input set.

**Action:** Add historical packet replay identity records and paired torque streams, or remove this unverified sentence from the evidence caption.

### Claim 238: missing_evidence

**Location:** system_paper/manuscript_v1.md: §7.2 Figure 2; Abstract

> Old and revised estimator velocity error norms on the same archived packets, without a new plant rollout.

Error norms and matching timestamps are auditable, and plotted trends match them. The original historical packets and redecision input identity/source are not declared. Equal timestamps do not independently establish same-packet use or absence of a rollout.

**Action:** Provide the archived packet hash and redecision source/provenance record; keep the numerical RMS result separate from unverified same-packet provenance.

### Claim 243: unsupported_claim

**Location:** system_paper/manuscript_v1.md: §7.2

> Covariance update monotonicity, an initial covariance above that fixed point, and contact process noise no smaller than free-motion noise imply the same necessary floor throughout valid operation under this protocol.

Numerical fixed-point and initial dominance are verified, and the observed window obeys the floor. The conditional theorem and universal applicability of its exact covariance recursion to all valid implemented modes need the theory/estimator source excluded from this manifest.

**Action:** Keep the all-valid-operation statement explicitly conditional and unverified in this audit; separately audit the recursion assumptions and proof.

### Claim 251: unsupported_claim

**Location:** system_paper/manuscript_v1.md: §7.4

> Early development controller timing omitted the terminal exception call; predictor timing retains that omission's diagnostic context, and final regressions measure all calls externally.

Summary values support the disclosed timing inconsistency and corrected regression maxima. Raw controller-call duration records/instrumentation source are missing, so complete coverage and external timing cannot be independently confirmed.

**Action:** Retain timing caveat; supply per-call timing/instrumentation to verify quantile denominators and terminal-call coverage.

### Claim 257: missing_evidence

**Location:** system_paper/manuscript_v1.md: §8

> The original full-boundary interface qualification failure remains unchanged, and no real hardware load claim is made.

The absence of a hardware performance claim is clear, but the original full-boundary qualification failure and unchanged status are not evidenced by this manifest.

**Action:** Supply the original qualification result and preservation evidence, or mark the inherited failure statement as unaudited.

### Claim 259: unsupported_claim

**Location:** system_paper/manuscript_v1.md: Reproducibility

> the entry point is `python -m v6_mujoco.feasible_capture --all --resume`. Failed attempts are never implicitly retried.

The entry point and retry semantics cannot be verified from results/configurations alone. No invocation or simulation rerun was attempted.

**Action:** Validate the documented command and retry control flow in a separate source/CLI check.

### Claim 260: ambiguous_mapping

**Location:** system_paper/figure_plan.md: F5 caption

> with the original 5 mm accuracy target (dotted).

The figure visibly draws 5 mm, and H1 remains above it; no declared config establishes that this was the original accuracy target.

**Action:** Identify the original 5 mm requirement source or label it as a displayed reference target.

## Recalculation details

Inclusive windows used an explicit 1e-9 s boundary tolerance to avoid excluding nominal endpoint samples from floating-point accumulation. Completed runs each have 1001 samples in their fixed final two seconds. The noisy [1,7] s window has 3001 valid samples in each run. No failed-prefix performance result was filled in.

The independent CV calculation yields 1.609696703887464 mm/s, agreeing with the reported 1.609697 mm/s after standard rounding. Its DARE/iteration covariance difference is 3.82e-19. This checks the specified numerical recursion; universal applicability to the implemented modes remains unverified here.

The H1 final COM error is 15.275807698397005 mm; raw-block scaled-whitened rank is ten. H1 and H3 prior/identified trajectories have exactly zero maximum torque, qpos and qvel differences, while their logged predictor parameters differ.

All covariance snapshot/trace comparisons agree except invalid t=0 in D03 and R03, each differing by 9.999990000000001e-5. Original discrepancy records and the failed D03 validation remain intact.

## All claims

Each table cell is a separate record. Duplicate manuscript/generated-table occurrences are mapped to the same record rather than counted as new evidence. The JSON contains exact quotes, raw file paths, full recomputed values and actions.

| ID | Location | Paper value | Evidence value | Status |
|---:|---|---|---|---|
| 1 | system_paper/generated_tables/task_outcomes.md: D01_V1_nominal, Use | development | development | exact_match |
| 2 | system_paper/generated_tables/task_outcomes.md: D01_V1_nominal, Status | COMPLETED | COMPLETED | exact_match |
| 3 | system_paper/generated_tables/task_outcomes.md: D01_V1_nominal, End (s) | 27.992 | 27.991999999995365 | rounding_ok |
| 4 | system_paper/generated_tables/task_outcomes.md: D01_V1_nominal, Latch (s) | 7.992 | 7.991999999999342 | rounding_ok |
| 5 | system_paper/generated_tables/task_outcomes.md: D01_V1_nominal, World last2s (deg/s) | 0.0235542 | 0.023554153218418417 | rounding_ok |
| 6 | system_paper/generated_tables/task_outcomes.md: D01_V1_nominal, Peak rho | 0.363223 | 0.36322261973783676 | rounding_ok |
| 7 | system_paper/generated_tables/task_outcomes.md: D01_V1_nominal, Safety violations | 0 | 0 | exact_match |
| 8 | system_paper/generated_tables/task_outcomes.md: D01_V1_nominal, Dual replay | PASS | PASS | exact_match |
| 9 | system_paper/generated_tables/task_outcomes.md: D02_V1_H2, Use | development | development | exact_match |
| 10 | system_paper/generated_tables/task_outcomes.md: D02_V1_H2, Status | NO_VERIFIED_CONTROL | NO_VERIFIED_CONTROL | exact_match |
| 11 | system_paper/generated_tables/task_outcomes.md: D02_V1_H2, End (s) | 0.04 | 0.04000000000000002 | rounding_ok |
| 12 | system_paper/generated_tables/task_outcomes.md: D02_V1_H2, Latch (s) | NOT_EVALUATED | null | exact_match |
| 13 | system_paper/generated_tables/task_outcomes.md: D02_V1_H2, World last2s (deg/s) | NOT_EVALUATED | null | exact_match |
| 14 | system_paper/generated_tables/task_outcomes.md: D02_V1_H2, Peak rho | 0 | 0.0 | exact_match |
| 15 | system_paper/generated_tables/task_outcomes.md: D02_V1_H2, Safety violations | 0 | 0 | exact_match |
| 16 | system_paper/generated_tables/task_outcomes.md: D02_V1_H2, Dual replay | PASS | PASS | exact_match |
| 17 | system_paper/generated_tables/task_outcomes.md: D03_V1_S01, Use | development | development | exact_match |
| 18 | system_paper/generated_tables/task_outcomes.md: D03_V1_S01, Status | NO_VERIFIED_CONTROL | NO_VERIFIED_CONTROL | exact_match |
| 19 | system_paper/generated_tables/task_outcomes.md: D03_V1_S01, End (s) | 7.84 | 7.839999999999359 | rounding_ok |
| 20 | system_paper/generated_tables/task_outcomes.md: D03_V1_S01, Latch (s) | NOT_EVALUATED | null | exact_match |
| 21 | system_paper/generated_tables/task_outcomes.md: D03_V1_S01, World last2s (deg/s) | NOT_EVALUATED | null | exact_match |
| 22 | system_paper/generated_tables/task_outcomes.md: D03_V1_S01, Peak rho | 0.0103905 | 0.01039049937806134 | rounding_ok |
| 23 | system_paper/generated_tables/task_outcomes.md: D03_V1_S01, Safety violations | 0 | 0 | exact_match |
| 24 | system_paper/generated_tables/task_outcomes.md: D03_V1_S01, Dual replay | PASS_TS_AUDIT | PASS_TS_AUDIT | exact_match |
| 25 | system_paper/generated_tables/task_outcomes.md: D04_V2_H2, Use | development | development | exact_match |
| 26 | system_paper/generated_tables/task_outcomes.md: D04_V2_H2, Status | NO_VERIFIED_CONTROL | NO_VERIFIED_CONTROL | exact_match |
| 27 | system_paper/generated_tables/task_outcomes.md: D04_V2_H2, End (s) | 5.9 | 5.899999999999572 | rounding_ok |
| 28 | system_paper/generated_tables/task_outcomes.md: D04_V2_H2, Latch (s) | NOT_EVALUATED | null | exact_match |
| 29 | system_paper/generated_tables/task_outcomes.md: D04_V2_H2, World last2s (deg/s) | NOT_EVALUATED | null | exact_match |
| 30 | system_paper/generated_tables/task_outcomes.md: D04_V2_H2, Peak rho | 0 | 0.0 | exact_match |
| 31 | system_paper/generated_tables/task_outcomes.md: D04_V2_H2, Safety violations | 0 | 0 | exact_match |
| 32 | system_paper/generated_tables/task_outcomes.md: D04_V2_H2, Dual replay | PASS | PASS | exact_match |
| 33 | system_paper/generated_tables/task_outcomes.md: R01_V2_nominal, Use | regression | regression | exact_match |
| 34 | system_paper/generated_tables/task_outcomes.md: R01_V2_nominal, Status | COMPLETED | COMPLETED | exact_match |
| 35 | system_paper/generated_tables/task_outcomes.md: R01_V2_nominal, End (s) | 27.992 | 27.991999999995365 | rounding_ok |
| 36 | system_paper/generated_tables/task_outcomes.md: R01_V2_nominal, Latch (s) | 7.992 | 7.991999999999342 | rounding_ok |
| 37 | system_paper/generated_tables/task_outcomes.md: R01_V2_nominal, World last2s (deg/s) | 0.0235542 | 0.023554153218418417 | rounding_ok |
| 38 | system_paper/generated_tables/task_outcomes.md: R01_V2_nominal, Peak rho | 0.363223 | 0.36322261973783676 | rounding_ok |
| 39 | system_paper/generated_tables/task_outcomes.md: R01_V2_nominal, Safety violations | 0 | 0 | exact_match |
| 40 | system_paper/generated_tables/task_outcomes.md: R01_V2_nominal, Dual replay | PASS | PASS | exact_match |
| 41 | system_paper/generated_tables/task_outcomes.md: R02_V2_H2, Use | regression | regression | exact_match |
| 42 | system_paper/generated_tables/task_outcomes.md: R02_V2_H2, Status | NO_VERIFIED_CONTROL | NO_VERIFIED_CONTROL | exact_match |
| 43 | system_paper/generated_tables/task_outcomes.md: R02_V2_H2, End (s) | 5.9 | 5.899999999999572 | rounding_ok |
| 44 | system_paper/generated_tables/task_outcomes.md: R02_V2_H2, Latch (s) | NOT_EVALUATED | null | exact_match |
| 45 | system_paper/generated_tables/task_outcomes.md: R02_V2_H2, World last2s (deg/s) | NOT_EVALUATED | null | exact_match |
| 46 | system_paper/generated_tables/task_outcomes.md: R02_V2_H2, Peak rho | 0 | 0.0 | exact_match |
| 47 | system_paper/generated_tables/task_outcomes.md: R02_V2_H2, Safety violations | 0 | 0 | exact_match |
| 48 | system_paper/generated_tables/task_outcomes.md: R02_V2_H2, Dual replay | PASS | PASS | exact_match |
| 49 | system_paper/generated_tables/task_outcomes.md: R03_V2_S01, Use | regression | regression | exact_match |
| 50 | system_paper/generated_tables/task_outcomes.md: R03_V2_S01, Status | NO_VERIFIED_CONTROL | NO_VERIFIED_CONTROL | exact_match |
| 51 | system_paper/generated_tables/task_outcomes.md: R03_V2_S01, End (s) | 7.84 | 7.839999999999359 | rounding_ok |
| 52 | system_paper/generated_tables/task_outcomes.md: R03_V2_S01, Latch (s) | NOT_EVALUATED | null | exact_match |
| 53 | system_paper/generated_tables/task_outcomes.md: R03_V2_S01, World last2s (deg/s) | NOT_EVALUATED | null | exact_match |
| 54 | system_paper/generated_tables/task_outcomes.md: R03_V2_S01, Peak rho | 0.0103905 | 0.01039049937806134 | rounding_ok |
| 55 | system_paper/generated_tables/task_outcomes.md: R03_V2_S01, Safety violations | 0 | 0 | exact_match |
| 56 | system_paper/generated_tables/task_outcomes.md: R03_V2_S01, Dual replay | PASS_TS_AUDIT | PASS_TS_AUDIT | exact_match |
| 57 | system_paper/generated_tables/task_outcomes.md: R04_V2_H1, Use | regression | regression | exact_match |
| 58 | system_paper/generated_tables/task_outcomes.md: R04_V2_H1, Status | COMPLETED | COMPLETED | exact_match |
| 59 | system_paper/generated_tables/task_outcomes.md: R04_V2_H1, End (s) | 27.932 | 27.931999999995398 | rounding_ok |
| 60 | system_paper/generated_tables/task_outcomes.md: R04_V2_H1, Latch (s) | 7.932 | 7.9319999999993485 | rounding_ok |
| 61 | system_paper/generated_tables/task_outcomes.md: R04_V2_H1, World last2s (deg/s) | 0.0128079 | 0.012807859564078114 | rounding_ok |
| 62 | system_paper/generated_tables/task_outcomes.md: R04_V2_H1, Peak rho | 0.597206 | 0.597206336221039 | rounding_ok |
| 63 | system_paper/generated_tables/task_outcomes.md: R04_V2_H1, Safety violations | 0 | 0 | exact_match |
| 64 | system_paper/generated_tables/task_outcomes.md: R04_V2_H1, Dual replay | PASS | PASS | exact_match |
| 65 | system_paper/generated_tables/phase_one.md: H2_prior, Classification | LINEAR_HARD_SET_INFEASIBLE | LINEAR_HARD_SET_INFEASIBLE | exact_match |
| 66 | system_paper/generated_tables/phase_one.md: H2_prior, Dimensionless z | 0.0195548 | 0.019554833668535825 | rounding_ok |
| 67 | system_paper/generated_tables/phase_one.md: H2_prior, Conflict row | gripper_contact_pad__tumbling_target_geom | gripper_contact_pad__tumbling_target_geom | exact_match |
| 68 | system_paper/generated_tables/phase_one.md: H2_prior, Required (m/s) | -0.127798 | -0.12779809474308393 | rounding_ok |
| 69 | system_paper/generated_tables/phase_one.md: H2_prior, Box maximum (m/s) | -0.161947 | -0.1619474977954096 | rounding_ok |
| 70 | system_paper/generated_tables/phase_one.md: H2_prior, Shortfall (m/s) | 0.0341494 | 0.034149403052325666 | rounding_ok |
| 71 | system_paper/generated_tables/phase_one.md: S01_noise_delay, Classification | LINEAR_HARD_SET_INFEASIBLE | LINEAR_HARD_SET_INFEASIBLE | exact_match |
| 72 | system_paper/generated_tables/phase_one.md: S01_noise_delay, Dimensionless z | 0.00550255 | 0.005502548932376166 | rounding_ok |
| 73 | system_paper/generated_tables/phase_one.md: S01_noise_delay, Conflict row | gripper_contact_pad__tumbling_target_geom | gripper_contact_pad__tumbling_target_geom | exact_match |
| 74 | system_paper/generated_tables/phase_one.md: S01_noise_delay, Required (m/s) | -0.0372463 | -0.03724633332276628 | rounding_ok |
| 75 | system_paper/generated_tables/phase_one.md: S01_noise_delay, Box maximum (m/s) | -0.0467254 | -0.046725359534771936 | rounding_ok |
| 76 | system_paper/generated_tables/phase_one.md: S01_noise_delay, Shortfall (m/s) | 0.00947903 | 0.009479026212005658 | rounding_ok |
| 77 | system_paper/generated_tables/phase_one.md: D05_final_nominal, Classification | LINEAR_HARD_SET_INFEASIBLE | LINEAR_HARD_SET_INFEASIBLE | exact_match |
| 78 | system_paper/generated_tables/phase_one.md: D05_final_nominal, Dimensionless z | 0.000983 | 0.0009830003257214753 | rounding_ok |
| 79 | system_paper/generated_tables/phase_one.md: D05_final_nominal, Conflict row | gripper_contact_pad__tumbling_target_geom | gripper_contact_pad__tumbling_target_geom | exact_match |
| 80 | system_paper/generated_tables/phase_one.md: D05_final_nominal, Required (m/s) | -0.027238 | -0.02723799318389345 | rounding_ok |
| 81 | system_paper/generated_tables/phase_one.md: D05_final_nominal, Box maximum (m/s) | -0.0288792 | -0.028879216274506264 | rounding_ok |
| 82 | system_paper/generated_tables/phase_one.md: D05_final_nominal, Shortfall (m/s) | 0.00164122 | 0.001641223090612813 | rounding_ok |
| 83 | system_paper/generated_tables/noisy_capture_gate.md: D03_V1_S01, Actual gate ticks | 0 | 0 | exact_match |
| 84 | system_paper/generated_tables/noisy_capture_gate.md: D03_V1_S01, Audited1–7s samples | 3001 | 3001 | exact_match |
| 85 | system_paper/generated_tables/noisy_capture_gate.md: D03_V1_S01, Uncertainty precludes gate | 3001 | 3001 | exact_match |
| 86 | system_paper/generated_tables/noisy_capture_gate.md: D03_V1_S01, Minimum linear3sigma lower bound (m/s) | 0.0016097 | 0.0016096967038863287 | rounding_ok |
| 87 | system_paper/generated_tables/noisy_capture_gate.md: D03_V1_S01, Minimum angular3sigma (rad/s) | 0.00964336 | 0.009643356348791552 | rounding_ok |
| 88 | system_paper/generated_tables/noisy_capture_gate.md: R03_V2_S01, Actual gate ticks | 0 | 0 | exact_match |
| 89 | system_paper/generated_tables/noisy_capture_gate.md: R03_V2_S01, Audited1–7s samples | 3001 | 3001 | exact_match |
| 90 | system_paper/generated_tables/noisy_capture_gate.md: R03_V2_S01, Uncertainty precludes gate | 3001 | 3001 | exact_match |
| 91 | system_paper/generated_tables/noisy_capture_gate.md: R03_V2_S01, Minimum linear3sigma lower bound (m/s) | 0.0016097 | 0.0016096967038863287 | rounding_ok |
| 92 | system_paper/generated_tables/noisy_capture_gate.md: R03_V2_S01, Minimum angular3sigma (rad/s) | 0.00964336 | 0.009643356348791552 | rounding_ok |
| 93 | system_paper/generated_tables/compute_and_momentum.md: D01_V1_nominal, CPU (s) | 279.078 | 279.078125 | rounding_ok |
| 94 | system_paper/generated_tables/compute_and_momentum.md: D01_V1_nominal, p50 (s) | 0.0010205 | 0.0010204999707639217 | rounding_ok |
| 95 | system_paper/generated_tables/compute_and_momentum.md: D01_V1_nominal, p99 (s) | 0.885892 | 0.8858920441567898 | rounding_ok |
| 96 | system_paper/generated_tables/compute_and_momentum.md: D01_V1_nominal, Call max (s) | 0.954045 | 0.954044799786061 | rounding_ok |
| 97 | system_paper/generated_tables/compute_and_momentum.md: D01_V1_nominal, Predictor max (s) | 0.877906 | 0.8779064998961985 | rounding_ok |
| 98 | system_paper/generated_tables/compute_and_momentum.md: D01_V1_nominal, P drift (kg m/s) | 2.63924e-08 | 2.639237500491339e-08 | rounding_ok |
| 99 | system_paper/generated_tables/compute_and_momentum.md: D01_V1_nominal, H drift (kg m2/s) | 4.32438e-07 | 4.324381723050829e-07 | rounding_ok |
| 100 | system_paper/generated_tables/compute_and_momentum.md: D02_V1_H2, CPU (s) | 0.234375 | 0.234375 | exact_match |
| 101 | system_paper/generated_tables/compute_and_momentum.md: D02_V1_H2, p50 (s) | 0.0009455 | 0.0009454998653382063 | rounding_ok |
| 102 | system_paper/generated_tables/compute_and_momentum.md: D02_V1_H2, p99 (s) | 0.00124829 | 0.0012482858030125497 | rounding_ok |
| 103 | system_paper/generated_tables/compute_and_momentum.md: D02_V1_H2, Call max (s) | 0.001275 | 0.0012749996967613697 | rounding_ok |
| 104 | system_paper/generated_tables/compute_and_momentum.md: D02_V1_H2, Predictor max (s) | 0.0634562 | 0.06345620006322861 | rounding_ok |
| 105 | system_paper/generated_tables/compute_and_momentum.md: D02_V1_H2, P drift (kg m/s) | 3.36055e-37 | 3.360548955815672e-37 | rounding_ok |
| 106 | system_paper/generated_tables/compute_and_momentum.md: D02_V1_H2, H drift (kg m2/s) | 5.05352e-15 | 5.05351762583339e-15 | rounding_ok |
| 107 | system_paper/generated_tables/compute_and_momentum.md: D03_V1_S01, CPU (s) | 208.688 | 208.6875 | rounding_ok |
| 108 | system_paper/generated_tables/compute_and_momentum.md: D03_V1_S01, p50 (s) | 0.0012854 | 0.0012853997759521008 | rounding_ok |
| 109 | system_paper/generated_tables/compute_and_momentum.md: D03_V1_S01, p99 (s) | 0.898961 | 0.8989611540455371 | rounding_ok |
| 110 | system_paper/generated_tables/compute_and_momentum.md: D03_V1_S01, Call max (s) | 1.03027 | 1.0302683003246784 | rounding_ok |
| 111 | system_paper/generated_tables/compute_and_momentum.md: D03_V1_S01, Predictor max (s) | 4.42006 | 4.420055899769068 | rounding_ok |
| 112 | system_paper/generated_tables/compute_and_momentum.md: D03_V1_S01, P drift (kg m/s) | 8.11414e-10 | 8.114141418096731e-10 | rounding_ok |
| 113 | system_paper/generated_tables/compute_and_momentum.md: D03_V1_S01, H drift (kg m2/s) | 5.85661e-11 | 5.856609183686693e-11 | rounding_ok |
| 114 | system_paper/generated_tables/compute_and_momentum.md: D04_V2_H2, CPU (s) | 122.828 | 122.828125 | rounding_ok |
| 115 | system_paper/generated_tables/compute_and_momentum.md: D04_V2_H2, p50 (s) | 0.0010532 | 0.0010532001033425331 | rounding_ok |
| 116 | system_paper/generated_tables/compute_and_momentum.md: D04_V2_H2, p99 (s) | 0.954086 | 0.9540860369149595 | rounding_ok |
| 117 | system_paper/generated_tables/compute_and_momentum.md: D04_V2_H2, Call max (s) | 1.06788 | 1.0678828000091016 | rounding_ok |
| 118 | system_paper/generated_tables/compute_and_momentum.md: D04_V2_H2, Predictor max (s) | 4.50248 | 4.502479400020093 | rounding_ok |
| 119 | system_paper/generated_tables/compute_and_momentum.md: D04_V2_H2, P drift (kg m/s) | 4.12197e-10 | 4.1219691349701675e-10 | rounding_ok |
| 120 | system_paper/generated_tables/compute_and_momentum.md: D04_V2_H2, H drift (kg m2/s) | 1.89482e-11 | 1.894819764297476e-11 | rounding_ok |
| 121 | system_paper/generated_tables/compute_and_momentum.md: R01_V2_nominal, CPU (s) | 286.938 | 286.9375 | rounding_ok |
| 122 | system_paper/generated_tables/compute_and_momentum.md: R01_V2_nominal, p50 (s) | 0.0010324 | 0.001032399944961071 | rounding_ok |
| 123 | system_paper/generated_tables/compute_and_momentum.md: R01_V2_nominal, p99 (s) | 0.885949 | 0.8859486638940867 | rounding_ok |
| 124 | system_paper/generated_tables/compute_and_momentum.md: R01_V2_nominal, Call max (s) | 0.937136 | 0.9371364000253379 | rounding_ok |
| 125 | system_paper/generated_tables/compute_and_momentum.md: R01_V2_nominal, Predictor max (s) | 0.862583 | 0.8625827999785542 | rounding_ok |
| 126 | system_paper/generated_tables/compute_and_momentum.md: R01_V2_nominal, P drift (kg m/s) | 2.63924e-08 | 2.639237500491339e-08 | rounding_ok |
| 127 | system_paper/generated_tables/compute_and_momentum.md: R01_V2_nominal, H drift (kg m2/s) | 4.32438e-07 | 4.324381723050829e-07 | rounding_ok |
| 128 | system_paper/generated_tables/compute_and_momentum.md: R02_V2_H2, CPU (s) | 119.25 | 119.25 | exact_match |
| 129 | system_paper/generated_tables/compute_and_momentum.md: R02_V2_H2, p50 (s) | 0.0010655 | 0.0010655000805854797 | rounding_ok |
| 130 | system_paper/generated_tables/compute_and_momentum.md: R02_V2_H2, p99 (s) | 0.897432 | 0.8974320001434535 | rounding_ok |
| 131 | system_paper/generated_tables/compute_and_momentum.md: R02_V2_H2, Call max (s) | 4.18577 | 4.185773300006986 | rounding_ok |
| 132 | system_paper/generated_tables/compute_and_momentum.md: R02_V2_H2, Predictor max (s) | 4.1848 | 4.184798799920827 | rounding_ok |
| 133 | system_paper/generated_tables/compute_and_momentum.md: R02_V2_H2, P drift (kg m/s) | 4.12197e-10 | 4.1219691349701675e-10 | rounding_ok |
| 134 | system_paper/generated_tables/compute_and_momentum.md: R02_V2_H2, H drift (kg m2/s) | 1.89482e-11 | 1.894819764297476e-11 | rounding_ok |
| 135 | system_paper/generated_tables/compute_and_momentum.md: R03_V2_S01, CPU (s) | 210.5 | 210.5 | exact_match |
| 136 | system_paper/generated_tables/compute_and_momentum.md: R03_V2_S01, p50 (s) | 0.0012984 | 0.0012984001077711582 | rounding_ok |
| 137 | system_paper/generated_tables/compute_and_momentum.md: R03_V2_S01, p99 (s) | 0.898508 | 0.8985075800679625 | rounding_ok |
| 138 | system_paper/generated_tables/compute_and_momentum.md: R03_V2_S01, Call max (s) | 4.1804 | 4.180404200218618 | rounding_ok |
| 139 | system_paper/generated_tables/compute_and_momentum.md: R03_V2_S01, Predictor max (s) | 4.17897 | 4.178973699919879 | rounding_ok |
| 140 | system_paper/generated_tables/compute_and_momentum.md: R03_V2_S01, P drift (kg m/s) | 8.11414e-10 | 8.114141418096731e-10 | rounding_ok |
| 141 | system_paper/generated_tables/compute_and_momentum.md: R03_V2_S01, H drift (kg m2/s) | 5.85661e-11 | 5.856609183686693e-11 | rounding_ok |
| 142 | system_paper/generated_tables/compute_and_momentum.md: R04_V2_H1, CPU (s) | 281.812 | 281.8125 | rounding_ok |
| 143 | system_paper/generated_tables/compute_and_momentum.md: R04_V2_H1, p50 (s) | 0.0010078 | 0.0010077999904751778 | rounding_ok |
| 144 | system_paper/generated_tables/compute_and_momentum.md: R04_V2_H1, p99 (s) | 0.874536 | 0.8745355417206884 | rounding_ok |
| 145 | system_paper/generated_tables/compute_and_momentum.md: R04_V2_H1, Call max (s) | 0.915231 | 0.915230699814856 | rounding_ok |
| 146 | system_paper/generated_tables/compute_and_momentum.md: R04_V2_H1, Predictor max (s) | 0.835853 | 0.8358529997058213 | rounding_ok |
| 147 | system_paper/generated_tables/compute_and_momentum.md: R04_V2_H1, P drift (kg m/s) | 1.26511e-08 | 1.2651127855171335e-08 | rounding_ok |
| 148 | system_paper/generated_tables/compute_and_momentum.md: R04_V2_H1, H drift (kg m2/s) | 6.86938e-07 | 6.869382560991592e-07 | rounding_ok |
| 149 | system_paper/generated_tables/sensor_same_packets.md: px, Historical RMS | 3.69399e-05 | 3.6939887148691524e-05 | rounding_ok |
| 150 | system_paper/generated_tables/sensor_same_packets.md: px, New RMS | 8.67523e-06 | 8.67522767890486e-06 | rounding_ok |
| 151 | system_paper/generated_tables/sensor_same_packets.md: py, Historical RMS | 3.70751e-05 | 3.70750892908875e-05 | rounding_ok |
| 152 | system_paper/generated_tables/sensor_same_packets.md: py, New RMS | 8.83375e-06 | 8.833753307349045e-06 | rounding_ok |
| 153 | system_paper/generated_tables/sensor_same_packets.md: pz, Historical RMS | 3.5539e-05 | 3.5538952072789344e-05 | rounding_ok |
| 154 | system_paper/generated_tables/sensor_same_packets.md: pz, New RMS | 8.1245e-06 | 8.124498815443983e-06 | rounding_ok |
| 155 | system_paper/generated_tables/sensor_same_packets.md: Rx, Historical RMS | 0.000146349 | 0.0001463488740523662 | rounding_ok |
| 156 | system_paper/generated_tables/sensor_same_packets.md: Rx, New RMS | 3.70434e-05 | 3.7043431326090713e-05 | rounding_ok |
| 157 | system_paper/generated_tables/sensor_same_packets.md: Ry, Historical RMS | 0.000154649 | 0.00015464857399808332 | rounding_ok |
| 158 | system_paper/generated_tables/sensor_same_packets.md: Ry, New RMS | 3.95669e-05 | 3.956688531720242e-05 | rounding_ok |
| 159 | system_paper/generated_tables/sensor_same_packets.md: Rz, Historical RMS | 0.000153995 | 0.00015399548178699233 | rounding_ok |
| 160 | system_paper/generated_tables/sensor_same_packets.md: Rz, New RMS | 3.8293e-05 | 3.829302022903009e-05 | rounding_ok |
| 161 | system_paper/generated_tables/sensor_same_packets.md: vx, Historical RMS | 0.00217026 | 0.002170262359072463 | rounding_ok |
| 162 | system_paper/generated_tables/sensor_same_packets.md: vx, New RMS | 0.00022671 | 0.00022671025016534385 | rounding_ok |
| 163 | system_paper/generated_tables/sensor_same_packets.md: vy, Historical RMS | 0.00216615 | 0.0021661456594611276 | rounding_ok |
| 164 | system_paper/generated_tables/sensor_same_packets.md: vy, New RMS | 0.000227544 | 0.00022754394951209246 | rounding_ok |
| 165 | system_paper/generated_tables/sensor_same_packets.md: vz, Historical RMS | 0.00207524 | 0.002075244415352518 | rounding_ok |
| 166 | system_paper/generated_tables/sensor_same_packets.md: vz, New RMS | 0.000217612 | 0.0002176123466245102 | rounding_ok |
| 167 | system_paper/generated_tables/sensor_same_packets.md: wx, Historical RMS | 0.00877765 | 0.008777647393479554 | rounding_ok |
| 168 | system_paper/generated_tables/sensor_same_packets.md: wx, New RMS | 0.00122166 | 0.0012216585952662703 | rounding_ok |
| 169 | system_paper/generated_tables/sensor_same_packets.md: wy, Historical RMS | 0.00925421 | 0.009254209672920979 | rounding_ok |
| 170 | system_paper/generated_tables/sensor_same_packets.md: wy, New RMS | 0.00130853 | 0.0013085332094291222 | rounding_ok |
| 171 | system_paper/generated_tables/sensor_same_packets.md: wz, Historical RMS | 0.00923922 | 0.009239217770357546 | rounding_ok |
| 172 | system_paper/generated_tables/sensor_same_packets.md: wz, New RMS | 0.00125285 | 0.0012528480153747295 | rounding_ok |
| 173 | system_paper/manuscript_v1.md: Title qualifier / Abstract / §1 / §8 / §9 | The present candidate has not demonstrated robust capture improvement. | {"attempts": 8, "development": 4, "regression": 4, "completed": 3, "failed": 5, "independent_pairs": 0} | exact_match |
| 174 | system_paper/manuscript_v1.md: Abstract; §1; §7.1; Conclusion | A read-only reconstruction of three archived failures separates an empty linear hard-constraint set from solver or secondary-task failure. | {"H2_prior": {"time": 7.819999999999361, "window": 0.49999999999994493, "count": 26, "z": 0.019554833668535825, "record_z": 0.019554833668535825, "all_z": [0.0, 0.0, 0.0, 0.0, 0.0… | exact_match |
| 175 | system_paper/manuscript_v1.md: Abstract | linear-velocity component RMS errors decrease from approximately 2.08–2.17 to 0.218–0.228 mm/s. | {"old_mm_s": [2.170262359072463, 2.1661456594611277, 2.075244415352518], "new_mm_s": [0.22671025016534385, 0.22754394951209245, 0.2176123466245102]} | rounding_ok |
| 176 | system_paper/manuscript_v1.md: Abstract; §7.3 | A nominal continuous run captures at 7.992 s and completes the prescribed 20 s post-capture interval; its final two-second maximum world angular speed is 0.023554 deg/s. | {"latch": 7.991999999999342, "post_duration": 19.99999999999602, "world_window": 0.023554153218418417, "window_start": 25.991999999996473, "window_count": 1001} | rounding_ok |
| 177 | system_paper/manuscript_v1.md: §3 | seven actuated joints | {"actuators_per_trace": {"D01_V1_nominal": 7, "D02_V1_H2": 7, "D03_V1_S01": 7, "D04_V2_H2": 7, "R01_V2_nominal": 7, "R02_V2_H2": 7, "R03_V2_S01": 7, "R04_V2_H1": 7}} | exact_match |
| 178 | system_paper/manuscript_v1.md: §3 | the fixed 20 mm tool/interface geometry | {"physical_face_offset_m": {"D01_V1_nominal": [0.019799999999999852, 0.019800000000000175], "D02_V1_H2": [0.019799999999999988, 0.019799999999999988], "D03_V1_S01": [0.01979999999… | ambiguous_mapping |
| 179 | system_paper/manuscript_v1.md: §3 | neither receives a commanded base wrench or target spin. | {"max_qfrc_applied": 0.0, "max_xfrc_applied": 0.0, "packet_torque_identity": 0.0} | exact_match |
| 180 | system_paper/manuscript_v1.md: §1; §3; §4.3 | The controller uses its declared nominal prior, never the true target mass matrix. | {"parameter_feedback_flags": [false, false, false, false, false, false, false, false], "isolation_tests": {"passed": true, "result": {"identical_packets_identical_output": true, "… | unsupported_claim |
| 181 | system_paper/manuscript_v1.md: §3 | The frozen capture thresholds are 0.1 mm position, 0.05 degree attitude, 1 mm/s relative linear velocity and 0.2 degree/s relative angular velocity. | {"translation_m": 0.0001, "rotation_deg": 0.05, "linear_m_s": 0.001, "angular_deg_s": 0.2} | exact_match |
| 182 | system_paper/manuscript_v1.md: §3 | After locking, position and attitude limits are 0.5 mm and 0.1 degree. | {"run_capture_config": "Contains approach thresholds; post-lock limit-defining source is not declared."} | missing_evidence |
| 183 | system_paper/manuscript_v1.md: §3 | $$\rho=\\|F\\|/(50\,\mathrm N)+\\|M\\|/(2\,\mathrm{Nm})\le1.$$ | {"load_formula_max_errors": {"D01_V1_nominal": 3.469446951953614e-18, "D02_V1_H2": 0.0, "D03_V1_S01": 0.0, "D04_V2_H2": 0.0, "R01_V2_nominal": 3.469446951953614e-18, "R02_V2_H2": … | exact_match |
| 184 | system_paper/manuscript_v1.md: §3 | All original noncontact clearance, permitted interface penetration, joint, torque and separate linear/angular momentum checks remain active. | {"recorded_violations": 0, "unintended_contacts": 0} | unsupported_claim |
| 185 | system_paper/manuscript_v1.md: §3 | The task allows at most 20 s approach and requires 20 s after locking. | {"approach_deadline_s": 20.0, "post_duration_s": 20.0} | exact_match |
| 186 | system_paper/manuscript_v1.md: §3 | Performance uses the fixed final two seconds, world angular speed at most 0.1 degree/s and target–base relative angular speed at most 0.02 degree/s. | {"evaluation_width_s": 2.0, "thresholds": {"world_deg_s": 0.1, "relative_deg_s": 0.02}, "complete_window_counts": {"D01_V1_nominal": 1001, "D02_V1_H2": 0, "D03_V1_S01": 0, "D04_V2… | exact_match |
| 187 | system_paper/manuscript_v1.md: §3 | The physics and joint servo run every 2 ms; the hierarchical velocity task runs every 20 ms. | {"dt_s": 0.002, "servo_s": 0.002, "task_s": 0.02, "max_trace_dt_error": 1.1084883011491797e-15} | exact_match |
| 188 | system_paper/manuscript_v1.md: §3 | The approach servo applies its actual bounded ramp between task commands. The primary QP minimizes tracking error subject to distance and joint feasibility inequalities. Tracking … | {"available": "Historical QP H,g,A,l,u; new task success/slack summaries and torque arrays."} | unsupported_claim |
| 189 | system_paper/manuscript_v1.md: §4.1 | A 0.25 s history stores the contact/process mode used over each past interval. | {"passed": true, "result": {"mode_history": true, "duplicate_rejected_no_new_NIS": true, "stale_invalid": true, "future_rejected": true, "raw_reference_separation": true}} | unsupported_claim |
| 190 | system_paper/manuscript_v1.md: §4.1 | Batches are sorted by measurement time; duplicate and older out-of-order samples are rejected rather than retrospectively assimilated. | {"events_strictly_increasing": {"D01_V1_nominal": true, "D02_V1_H2": true, "D03_V1_S01": true, "D04_V2_H2": true, "R01_V2_nominal": true, "R02_V2_H2": true, "R03_V2_S01": true, "R… | unsupported_claim |
| 191 | system_paper/manuscript_v1.md: §4.1 | NIS and innovation correction are logged only for new observations. | {"pose_counts": {"D01_V1_nominal": 13997, "D02_V1_H2": 21, "D03_V1_S01": 1305, "D04_V2_H2": 2951, "R01_V2_nominal": 13997, "R02_V2_H2": 2951, "R03_V2_S01": 1305, "R04_V2_H1": 1396… | exact_match |
| 192 | system_paper/manuscript_v1.md: §4.1 | Invalid or excessively old estimates produce an explicit `ESTIMATE_UNRELIABLE` stop. | {"passed": true, "result": {"secondary_degraded_explicit": true, "primary_lock_residual": 0.0, "full_gradient_directional_max_error": 8.59625617910531e-09, "short_trajectory_resul… | unsupported_claim |
| 193 | system_paper/manuscript_v1.md: §4.1 | For noisy operation, development reduced the free-motion acceleration process densities from 0.03/0.2 to 0.003 m/s^(3/2) and 0.02 rad/s^(3/2), while leaving the actual sensor nois… | {"old": {"process_linear_accel": 0.03, "process_angular_accel": 0.2, "contact_process_multiplier": 10.0}, "new": [0.003, 0.02], "sensor_config_differences": []} | exact_match |
| 194 | system_paper/manuscript_v1.md: §4.1 | A contact mode multiplies process density by ten. | {"contact_process_multiplier": 10.0} | exact_match |
| 195 | system_paper/manuscript_v1.md: §4.1 | The primary noisy protocol samples pose every 6 ms and delivers it after 12 ms, with 10 micrometre and 35 microradian component noise standard deviations and the original biases. | {"period": 0.006, "delay": 0.012, "position_sigma": 1e-05, "rotation_sigma": 3.5e-05, "delay_max": 0.01200000000000001} | exact_match |
| 196 | system_paper/manuscript_v1.md: §4.1 | Translational substeps integrate constant jerk exactly. Rotation uses a Lie increment with a discretization approximation for noncommuting angular derivatives. | {"passed": true, "result": {"derivative_errors": [1.2059056843909808e-11, 2.5170476814012665e-11, 1.7055145112560361e-12], "endpoint_relative_jets_zero": true, "zero_progress_keep… | unsupported_claim |
| 197 | system_paper/manuscript_v1.md: §4.1 | Capture tests use the raw timestamped estimate and its covariance, never the smoother reference state. | {"timing_test_raw_reference_separation": true, "latch_guards": [[{"time": 7.991999999999342, "gate_values": [5.7089398062812627e-05, 0.00010267097457374215, 0.0002529573028831783,… | unsupported_claim |
| 198 | system_paper/manuscript_v1.md: §4.2 | $$T_{WF,d}=T_{WG,r}Q_{GE}(s)T_{FE}^{-1},\qquad s\in[0,1].$$ | {"scope": "geometric formula; frame/theory files are not declared"} | unsupported_claim |
| 199 | system_paper/manuscript_v1.md: §4.2 | $$v_F=v+\omega\times r+Rq'(s)\dot s.$$ | {"scope": "mathematical derivative identity under stated frame conventions"} | unsupported_claim |
| 200 | system_paper/manuscript_v1.md: §4.2 | The nominal relative path has zero first and second endpoint jets. | {"scope": "path-definition source not declared"} | unsupported_claim |
| 201 | system_paper/manuscript_v1.md: §4.2 | Progress demand is filtered through three positive first-order stages with a 0.30 s time constant. | {"tau_s": 0.3, "stage_count": "not independently established"} | unsupported_claim |
| 202 | system_paper/manuscript_v1.md: §4.2 | At each task tick the governor tests demands of 1, 0.75, 0.5, 0.25 and 0 times the original maximum virtual-time rate, in descending order. | {"configured": [1, 0.75, 0.5, 0.25, 0], "terminal_candidates": [{"fraction": 1.0, "verified": false, "reason": "PREDICTED_TASK_RESIDUAL", "steps": 100, "horizon_reached_s": 0.2, "… | exact_match |
| 203 | system_paper/manuscript_v1.md: §4.2 | It integrates an independently built nominal-prior model for 0.20 s with the actual servo ramp, the original HQP and sampled contact model. It never reads future physical states. | {"configured_horizon": 0.2, "terminal_horizon": 0.2, "terminal_steps": 100, "isolation_test": true} | unsupported_claim |
| 204 | system_paper/manuscript_v1.md: §4.2 | Local clearance margins use a directional Jacobian covariance propagation and separate bias allowances. | {"bias_allowances": {"position_m": 5e-06, "rotation_rad": 6e-06}, "directional_test": 8.59625617910531e-09} | unsupported_claim |
| 205 | system_paper/manuscript_v1.md: §4.2 | The sole second-version revision allows the initial 0.20 s of soft task-residual transient, because the first version rejected the multiaxis case at its initial task tick despite … | {"mission_diff": [{"path": ".n209.version", "first": "V1", "second": "V2"}, {"path": ".n209.task_residual_startup_s", "first": null, "second": 0.2}, {"path": ".schema", "first": "… | unsupported_claim |
| 206 | system_paper/manuscript_v1.md: §4.2 | The position-error threshold remains 8 mm and the settled linear/angular residual thresholds remain 0.05 m/s and 0.10 rad/s. | {"position_m": 0.008, "linear_m_s": 0.05, "angular_rad_s": 0.1} | exact_match |
| 207 | system_paper/manuscript_v1.md: §4.2; §7.3 | If all candidates fail, `NO_VERIFIED_CONTROL` terminates the simulation. | {"stops": {"D02_V1_H2": [0.04000000000000002, [false, false, false, false, false]], "D03_V1_S01": [7.839999999999359, [false, false, false, false, false]], "D04_V2_H2": [5.8999999… | exact_match |
| 208 | system_paper/manuscript_v1.md: §4.3 | The state machine proceeds through observation, rendezvous, relative approach, capture window, grasp verification, damping transfer and hold. | [{"time": 0.0, "state": "OBSERVE"}, {"time": 0.04000000000000002, "state": "RENDEZVOUS"}, {"time": 6.041999999999557, "state": "RELATIVE_APPROACH"}, {"time": 7.841999999999358, "s… | exact_match |
| 209 | system_paper/manuscript_v1.md: §4.3 | Locking requires the original four estimated inequalities including uncertainty margins, sustained confirmation, a contact/load check and the unchanged prospective latch predictio… | {"latch_events": {"D01_V1_nominal": [{"time": 7.991999999999342, "gate_values": [5.7089398062812627e-05, 0.00010267097457374215, 0.0002529573028831783, 0.0006779714152721345], "ma… | unsupported_claim |
| 210 | system_paper/manuscript_v1.md: §4.3; §7.4 | The inertial estimator remains in shadow mode; its posterior cannot change the approach or damping actions in this campaign. | {"parameter_feedback_used": {"D01_V1_nominal": false, "D02_V1_H2": false, "D03_V1_S01": false, "D04_V2_H2": false, "R01_V2_nominal": false, "R02_V2_H2": false, "R03_V2_S01": false… | exact_match |
| 211 | system_paper/manuscript_v1.md: §5 conditional analysis | An attained optimum z=0 is equivalent to nonempty original feasibility. | {"scope": "mathematical assertion, explicitly not established by this raw-evidence audit"} | unsupported_claim |
| 212 | system_paper/manuscript_v1.md: §5 conditional analysis | Accepted predicted constraints imply actual sampled constraints only if appropriate state/model/linearization/input-hold error bounds cover their slack. | {"scope": "mathematical assertion, explicitly not established by this raw-evidence audit"} | unsupported_claim |
| 213 | system_paper/manuscript_v1.md: §5 conditional analysis | With no external force or moment, ideal internal interaction pairs conserve total linear and angular momentum. | {"scope": "mathematical assertion, explicitly not established by this raw-evidence audit"} | unsupported_claim |
| 214 | system_paper/manuscript_v1.md: §5 conditional analysis | Joint damping has power minus alpha times the weighted sum of squared joint velocities, hence nonpositive power when alpha and damping gains are nonnegative. Symmetric saturation … | {"scope": "mathematical assertion, explicitly not established by this raw-evidence audit"} | unsupported_claim |
| 215 | system_paper/manuscript_v1.md: §5 conditional analysis | If all relative motion stops, common angular velocity equals the inverse locked inertia times conserved COM angular momentum. | {"scope": "mathematical assertion, explicitly not established by this raw-evidence audit"} | unsupported_claim |
| 216 | system_paper/manuscript_v1.md: §5 conditional analysis | Pose-only free motion cannot identify that common scale. | {"scope": "mathematical assertion, explicitly not established by this raw-evidence audit"} | unsupported_claim |
| 217 | system_paper/manuscript_v1.md: §5 conditional analysis | A finite regressor's selected singular vectors define an update subspace, and a positive physical pseudoinertia imposes inertia triangle conditions; neither proves parameter accur… | {"scope": "mathematical assertion, explicitly not established by this raw-evidence audit"} | unsupported_claim |
| 218 | system_paper/manuscript_v1.md: §6 | The archived baseline commit is b1af09f89226fa6f2ae1b362f3d6225b3363cc17. | {"declared_baseline_commit": "b1af09f89226fa6f2ae1b362f3d6225b3363cc17"} | exact_match |
| 219 | system_paper/manuscript_v1.md: §6 | Source and configuration hashes, full packets, references, covariance, QP data, events, actuator inputs, plant trajectories and resource logs are saved for every new attempt, incl… | {"declared_runs": 8, "packet_counts_match_trace": true, "packet_torque_identity": 0.0} | missing_evidence |
| 220 | system_paper/manuscript_v1.md: §6 | The bounded plan allows at most two candidate versions, eight development attempts and four final seen regressions. | {"executed_versions": ["V1", "V2"], "executed_development": 4, "executed_regression": 4, "ledger_limits": {"development": 8, "regression": 4, "holdout": 36, "ablation": 6, "step_s… | exact_match |
| 221 | system_paper/manuscript_v1.md: §6 | These limits are enforced by the N209 run ledger; the inherited legacy mission metadata fields max_development=6 and max_attempts=16 are not used by this runner. | {"ledger_development": 8, "ledger_regression": 4, "legacy_fields": {"max_development": 6, "max_attempts": 16}} | unsupported_claim |
| 222 | system_paper/manuscript_v1.md: §6 | The planned independent design has six physical clusters, three fixed sensor seeds and paired B0/B1 methods, followed by six prespecified B2 and two fine-step trials. | {"planned_arithmetic": {"paired": 36, "B2": 6, "fine_step": 2}, "ledger_limits": {"development": 8, "regression": 4, "holdout": 36, "ablation": 6, "step_sensitivity": 2, "implemen… | exact_match |
| 223 | system_paper/manuscript_v1.md: §6 | The prespecified distribution and seed contract are retained for a future admitted campaign, without claiming they were validated. | {"available_holdout_config": "Three historical H1/H2/H3 cases only; new distribution/seed contract not declared."} | missing_evidence |
| 224 | system_paper/manuscript_v1.md: §6 | No independent B0/B1 pairs or B2 runs were executed. | {"methods": ["B1"], "categories": {"development": 4, "regression": 4}, "frozen": false} | exact_match |
| 225 | system_paper/manuscript_v1.md: §6 | It does not use 500 Hz samples as independent trials, report a population success probability, or compute a significance test from seen repetitions. | {"trace_period_s": 0.002, "manuscript_statistics": "No population estimate, significance test, or independent-trial aggregation found."} | exact_match |
| 226 | system_paper/manuscript_v1.md: §6 | Two independent replays are required for each attempt: one initializes the plant once at t=0 and reapplies recorded torque and latch events; another feeds the original packets thr… | {"validation_outcomes": {"D01_V1_nominal": {"actuator_replay_passed": true, "decision_replay_passed": true, "qpos_error": 0.0, "qvel_error": 0.0, "torque_error": 0.0, "max_interme… | unsupported_claim |
| 227 | system_paper/manuscript_v1.md: §6 | A uniform audit of all eight runs found this discrepancy only at t=0 in the two noisy runs. | {"snapshot_covariance_discrepancies": {"D01_V1_nominal": [], "D02_V1_H2": [], "D03_V1_S01": [{"time": 0.0, "max_diff": 9.999990000000001e-05, "valid": false}], "D04_V2_H2": [], "R… | exact_match |
| 228 | system_paper/manuscript_v1.md: §6 | subsequent snapshots were changed to deep copies, and all eight trajectories were revalidated using one common rule. | {"timestamp_rule_records": {"D01_V1_nominal": [], "D02_V1_H2": [], "D03_V1_S01": [{"time": 0.0, "field": "estimate.covariance", "original_snapshot_difference": 9.999990000000001e-… | unsupported_claim |
| 229 | system_paper/manuscript_v1.md: §6 | Original snapshots and the failed validation are retained. | {"D03_original_decision_replay_passed": false, "D03_original_error": 9.999990000000001e-05, "retained_covariance_discrepancies": 2} | exact_match |
| 230 | system_paper/manuscript_v1.md: §7.1 | In all three failures the distance rows alone and joint-box rows alone are feasible, but their intersection is empty. | {"H2_prior": {"distance_alone_feasible": true, "joint_box_feasible": true, "z": 0.019554833668535825, "shortfall": 0.034149403052325666}, "S01_noise_delay": {"distance_alone_feasi… | exact_match |
| 231 | system_paper/manuscript_v1.md: §7.1 | The 0.034149 m/s deficit is a derivative-command conflict, not a penetration depth. | 0.034149403052325666 | rounding_ok |
| 232 | system_paper/manuscript_v1.md: §7.1 | The archived H2 stop occurs before physical contact. | [{"name": "link7_collision__tumbling_target_geom", "distance_m": 0.028675809419607234, "d_min_m": 0.002}, {"name": "link7_collision__target_contact_plate", "distance_m": 0.0286758… | missing_evidence |
| 233 | system_paper/manuscript_v1.md: §7.1 Figure 1 | Archived failure snapshots over the final 0.5 s. | {"H2_prior": {"window": 0.49999999999994493, "count": 26, "time": 7.819999999999361, "z": 0.019554833668535825}, "S01_noise_delay": {"window": 0.49999999999994493, "count": 26, "t… | rounding_ok |
| 234 | system_paper/manuscript_v1.md: §7.1 Figure 1 | Dimensionless Phase-I relaxation is normalized by 1 m/s for distance rows and 1 rad/s for joint rows. | {"row_units": ["m/s", "m/s", "m/s", "m/s", "m/s", "rad/s", "rad/s", "rad/s", "rad/s", "rad/s", "rad/s", "rad/s"], "scales": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,… | exact_match |
| 235 | system_paper/figure_plan.md: F1 standalone caption | exact historical packet/torque replay | No historical packet or recorded/replayed torque pair is declared for H2/D05; snapshots expose QP/kinematics, not a replay identity series. | missing_evidence |
| 236 | system_paper/manuscript_v1.md: §7.2 | The offline historical S01 window is fixed at absolute time 1–7 s. | {"old_count": 3001, "new_count": 3001, "time_max_error": 0.0} | exact_match |
| 237 | system_paper/manuscript_v1.md: §7.2 | New linear-velocity component RMS values are about one tenth of the archived values; angular components also decrease. | {"linear_new_old_ratios": [0.1044621398964116, 0.10504554415269497, 0.1048610684190396], "angular_new_old_ratios": [0.1391783630057611, 0.14139869915180986, 0.13560109161992898]} | rounding_ok |
| 238 | system_paper/manuscript_v1.md: §7.2 Figure 2; Abstract | Old and revised estimator velocity error norms on the same archived packets, without a new plant rollout. | {"old_new_timestamp_identity": 0.0, "new_rows_fields": ["time", "error", "sigma", "velocity_margin_linear_m_s", "velocity_margin_angular_rad_s"]} | missing_evidence |
| 239 | system_paper/manuscript_v1.md: §7.2 | The noisy runs stop under predictive residual rejection before entering the capture-window state; their actual capture-guard call count is zero. | {"D03": {"events": [{"time": 0.0, "state": "OBSERVE"}, {"time": 0.04000000000000002, "state": "RENDEZVOUS"}, {"time": 6.041999999999557, "state": "RELATIVE_APPROACH"}, {"time": 7.… | exact_match |
| 240 | system_paper/manuscript_v1.md: §7.2 | A zero estimated error would still be insufficient to approve capture with these covariances. | {"D03_V1_S01": {"samples": 3001, "lin_min": 0.0016096967038863287, "ang_min": 0.009643356348791552, "precluded": 3001}, "R03_V2_S01": {"samples": 3001, "lin_min": 0.00160969670388… | exact_match |
| 241 | system_paper/manuscript_v1.md: §7.2 | the posterior fixed point gives a minimum control-time three-sigma velocity allowance of 1.609697 mm/s. | {"post": [[4.1025401035964896e-11, 1.7845527014417233e-09], [1.7845527014417233e-09, 1.799026087229073e-07]], "iteration_count": 59, "iteration_dare_error": 3.816412822972657e-19,… | rounding_ok |
| 242 | system_paper/manuscript_v1.md: §7.2 | Independent DARE and covariance-iteration calculations agree. | {"max_covariance_difference": 3.816412822972657e-19, "fixed_point_residual": 1.411448057701283e-18, "initial_minus_fixed_eigenvalues": [5.897456711769434e-11, 0.0999998200973913]} | exact_match |
| 243 | system_paper/manuscript_v1.md: §7.2 | Covariance update monotonicity, an initial covariance above that fixed point, and contact process noise no smaller than free-motion noise imply the same necessary floor throughout… | {"initial_dominance_eigenvalues": [5.897456711769434e-11, 0.0999998200973913], "observed_window_floor": 0.0016096967038863287} | unsupported_claim |
| 244 | system_paper/manuscript_v1.md: §7.3 | Peak actual rho is approximately 0.363223, with no recorded actual safety violation. | {"peak_rho": 0.36322261973783676, "recorded_violations": 0} | rounding_ok |
| 245 | system_paper/manuscript_v1.md: §7.3 | The first H2 candidate stops at 0.04 s under a soft startup residual check. The second version removes only that initial transient rejection and stops later, at 5.90 s. | {"V1_end": 0.04000000000000002, "V2_end": 5.899999999999572, "rejection_reasons": ["PREDICTED_TASK_RESIDUAL", "PREDICTED_TASK_RESIDUAL"]} | rounding_ok |
| 246 | system_paper/manuscript_v1.md: §7.3 | All five demands are then rejected by the predictive task-residual criterion; | {"fractions": [1.0, 0.75, 0.5, 0.25, 0.0], "reasons": ["PREDICTED_TASK_RESIDUAL", "PREDICTED_TASK_RESIDUAL", "PREDICTED_TASK_RESIDUAL", "PREDICTED_TASK_RESIDUAL", "PREDICTED_TASK_… | exact_match |
| 247 | system_paper/manuscript_v1.md: §7.3 | The noisy S01 first version reaches actual contact near 7.822 s but stops at 7.84 s. | {"contact": 7.821999999999361, "end": 7.839999999999359} | rounding_ok |
| 248 | system_paper/manuscript_v1.md: §7.3 Figure 3 | World/relative angular speeds, actual load utilization, and separate linear/angular momentum drift use absolute time. | {"world_window": 0.023554153218418417, "relative_window": 7.779958935344555e-05, "peak_rho": 0.36322261973783676, "P_drift": 2.639237500491339e-08, "H_drift": 4.324381723050829e-0… | exact_match |
| 249 | system_paper/manuscript_v1.md: §7.3 Figure 4 | Every ledger attempt, with category, stop time and outcome. | {"rows": ["D01_V1_nominal", "D02_V1_H2", "D03_V1_S01", "D04_V2_H2", "R01_V2_nominal", "R02_V2_H2", "R03_V2_S01", "R04_V2_H1"], "task_complete": [true, false, false, false, true, f… | exact_match |
| 250 | system_paper/manuscript_v1.md: §7.4 | Per-call timing substantially exceeds the 20 ms task deadline, with multi-second candidate evaluation at rejection. | {"predictor_min_s": 0.03666110010817647, "rejection_max_s": 4.502479400020093} | exact_match |
| 251 | system_paper/manuscript_v1.md: §7.4 | Early development controller timing omitted the terminal exception call; predictor timing retains that omission's diagnostic context, and final regressions measure all calls exter… | {"D03_controller_max": 1.0302683003246784, "D03_predictor_max": 4.420055899769068, "R03_controller_max": 4.180404200218618, "R03_predictor_max": 4.178973699919879} | unsupported_claim |
| 252 | system_paper/manuscript_v1.md: §7.4 Figure 5 | H1 reaches numerical rank ten but retains about 15.28 mm COM error. | {"torque_max_difference": 0.0, "qpos_max_difference": 0.0, "qvel_max_difference": 0.0, "final_com_mm": 15.275807698397005, "rank_final": 10, "rank_max": 10, "raw_singular": [0.080… | rounding_ok |
| 253 | system_paper/manuscript_v1.md: §7.4 Figure 5 | Historical prior/identified action and state differences are zero despite posterior parameters entering prediction. | {"H1": {"torque_max_difference": 0.0, "qpos_max_difference": 0.0, "qvel_max_difference": 0.0, "prediction_parameter_difference": 7.195352095075755}, "H3": {"torque_max_difference"… | exact_match |
| 254 | system_paper/manuscript_v1.md: §7.4 | raw and whitened singular values computed without regularization. | {"H1": {"raw_singular": [0.08091600200132101, 0.043925842736032054, 0.04058041078217441, 0.03890732228807872, 0.023472914993017548, 0.012720458072240077, 0.00932171239214263, 0.00… | exact_match |
| 255 | system_paper/manuscript_v1.md: §7.4 | `CONTROL_BENEFIT_NOT_DEMONSTRATED` is retained; no adaptive-detumbling advantage is asserted. | {"historical_zero_torque_difference": true, "new_parameter_feedback": false, "new_ablation_trials": 0} | exact_match |
| 256 | system_paper/manuscript_v1.md: §8 | At the selected H2 rejection, the largest candidate spread in terminal virtual progress is only about0.001670 s over the0.20 s horizon, or0.000209 in normalized progress. | {"virtual_time_spread_s": 0.0016700626778529681, "horizon_s": 0.2, "normalized_spread": 0.00020875783473162102} | rounding_ok |
| 257 | system_paper/manuscript_v1.md: §8 | The original full-boundary interface qualification failure remains unchanged, and no real hardware load claim is made. | {"available": "No qualification test record/model-history comparison declared."} | missing_evidence |
| 258 | system_paper/manuscript_v1.md: §8 | New statistical validation, module ablation, independent dynamics/contact replication and external compatible numerical baselines are absent. | {"all_categories": ["development", "regression"], "method": "B1", "new_independent_trials": 0} | exact_match |
| 259 | system_paper/manuscript_v1.md: Reproducibility | the entry point is `python -m v6_mujoco.feasible_capture --all --resume`. Failed attempts are never implicitly retried. | {"source_hashes": "Recorded but corresponding source files not declared; ledger preserves 8 attempts."} | unsupported_claim |
| 260 | system_paper/figure_plan.md: F5 caption | 5.0 | {"plot_target_mm": 5.0, "original_requirement_source": "not declared"} | ambiguous_mapping |

## Audit limits

- No manuscript or declared input was changed; no robot simulation rerun or campaign-ledger write was performed.
- Only declared research files and permitted skill instructions were read; no prior audit, narrative report or other chat was consumed.
- The canonical manuscript is Markdown. Linked undeclared theory, claim matrices, protocol, bibliography and model/source files were not opened.
- Exact/rounding matches to validator, timing and safety result fields do not independently prove the code that generated those fields.
- Mathematical/implementation assertions are explicitly unverified rather than presumed correct; bibliographic/related-work claims require a separate citation audit.
- No population success probability, significance test or independent paired effect can be estimated from these seen attempts.

Authoritative machine artifact: `paper/PAPER_CLAIM_AUDIT.json`. Supporting computations and the raw review response are in `paper/review-traces/experiment-claim-audit/2026-10-09_run02/`.

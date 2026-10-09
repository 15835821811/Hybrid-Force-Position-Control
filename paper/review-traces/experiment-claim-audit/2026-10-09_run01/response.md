# Experiment Claim Audit Report

**Date (UTC):** 2026-10-09T08:37:20.584439+00:00
**Auditor:** paper architect/reviewer (fresh zero-context context)
**Review trace:** `/root/claim_audit_v1`
**Paper:** Estimation- and Feasibility-Aware Continuous Capture and Momentum-Consistent Detumbling with a Free-Floating Single Arm

## Overall Verdict: WARN

WARN: 148 claim records; displayed empirical numbers reconcile, while replay/protocol/source provenance and several implementation or threshold mappings are outside the declared evidence.

**Claim records:** 148. **Material numeric/config/aggregation mismatches:** 0. **Declared hashes verified:** 62/62.

- ambiguous_mapping: 8
- exact_match: 76
- missing_evidence: 19
- rounding_ok: 45

## Main findings

- All three historical Phase-I optima and box certificates reproduce independently; distance-only and joint-only sets are feasible, their terminal intersections infeasible.
- All eight outcome rows match raw statuses, times, peak load, final-window world angular speed and recorded safety counts. Failed windows remain NOT_EVALUATED.
- Historical linear component RMS ranges are 2.075244-2.170262 mm/s (old) and 0.217612-0.227544 mm/s (new), matching the displayed rounding. Each noisy covariance window contains 3001 samples and all fail its necessary uncertainty condition.
- H1 final COM error is 15.275807698397005 mm with rank ten. H1/H3 prior-versus-identified states and torques are exactly identical.
- Replay claims remain unverified: validation.json, packets/events and archived source content were not supplied. The table already says PENDING_OR_FAIL for all eight attempts; the abstract should not imply certified completion.
- Budget mapping remains ambiguous: manuscript caps are two versions/eight development/four regressions, whereas inherited effective mission fields say max_development=6 and max_attempts=16. Actual supplied counts (four development/four regression) violate neither; an authoritative frozen contract is missing.

## Issues and missing evidence

### [WARN] Claim 2: ambiguous_mapping
**Location:** Sec. 3; manuscript_input.md:29

fixed 20 mm tool/interface geometry

**Evidence:** {"flange_to_tool_mm":19.756891996217554,"flange_to_physical_face_mm":19.799999999999986}

No geometry model or T_FE is declared. Recorded offsets are 19.7569 and 19.8 mm; these can be compatible with a nominal 20 mm dimension, but the quantity has no unique frame/dimension mapping here. This is not a demonstrated number error.

**Correction/evidence:** Specify the exact 20 mm dimension and include its model/transform definition.

### [WARN] Claim 7: missing_evidence
**Location:** Sec. 3; manuscript_input.md:31

After locking, position and attitude limits are 0.5 mm and 0.1 degree.

**Evidence:** null

The declared mission/run configurations expose capture/performance thresholds, but not these post-lock safety limits. Observed small errors do not establish the configured guard limits.

**Correction/evidence:** Include the original postgrasp/interface threshold configuration.

### [WARN] Claim 15: missing_evidence
**Location:** Sec. 4.1; manuscript_input.md:43

A 0.25 s history stores the contact/process mode used over each past interval.

**Evidence:** null

History length and process-mode replay behavior are not exposed in declared configs/traces.

**Correction/evidence:** Include estimator source or a bounded-history contract and raw check.

### [WARN] Claim 16: missing_evidence
**Location:** Sec. 4.1; manuscript_input.md:43

Batches are sorted by measurement time; duplicate and older out-of-order samples are rejected rather than retrospectively assimilated.

**Evidence:** null

No original packet/rejection stream or estimator source is declared. Causality/rejection cannot be certified from error outputs alone.

**Correction/evidence:** Audit archived estimator source and packet-level decisions.

### [WARN] Claim 17: ambiguous_mapping
**Location:** Sec. 4.1; manuscript_input.md:45

development reduced the free-motion acceleration process densities from 0.03/0.2 to 0.003 m/s^(3/2) and 0.02 rad/s^(3/2)

**Evidence:** {"new":[0.003,0.02],"generic_mission_defaults":[0.0002,0.001]}

New noisy process densities are exact. The old S01 effective config is not supplied, and generic mission defaults need not equal its noisy overrides. The old 0.03/0.2 cannot be mapped independently.

**Correction/evidence:** Supply archived S01 effective configuration.

### [WARN] Claim 20: missing_evidence
**Location:** Sec. 4.1; manuscript_input.md:47

Capture tests use the raw timestamped estimate and its covariance, never the smoother reference state.

**Evidence:** null

Separate estimate/reference arrays exist, but guard input identity requires source or call-level records.

**Correction/evidence:** Include archived guard source and call-level records.

### [WARN] Claim 21: ambiguous_mapping
**Location:** Sec. 4.2; manuscript_input.md:61

Progress demand is filtered through three positive first-order stages with a 0.30 s time constant.

**Evidence:** {"tau_s":0.3,"stage_count":null}

Time constant matches; number/positivity of stages is not exposed by the allowed new-run inputs.

**Correction/evidence:** Include filter source or internal stage traces.

### [WARN] Claim 28: ambiguous_mapping
**Location:** Sec. 6; manuscript_input.md:100

The bounded plan allows at most two candidate versions, eight development attempts and four final seen regressions.

**Evidence:** {"observed_versions":2,"observed_development":4,"observed_regression":4,"effective_max_development":6,"effective_max_attempts":16}

The eight supplied attempts do not exceed the manuscript budget. However inherited effective cap fields are 6/16; no frozen n209 budget contract/ledger maps them to the stated 8+4 policy. This is a provenance ambiguity, not a demonstrated count violation.

**Correction/evidence:** Supply authoritative frozen cap contract/ledger and explain inherited fields.

### [WARN] Claim 29: missing_evidence
**Location:** Sec. 6; manuscript_input.md:100

The planned independent design has six physical clusters, three fixed sensor seeds and paired B0/B1 methods, followed by six prespecified B2 and two fine-step trials.

**Evidence:** null

Clearly presented as an unexecuted plan; its frozen seed/distribution contract and prespecification are not among allowed inputs.

**Correction/evidence:** Include the planned protocol contract; retain NOT_EVALUATED qualification.

### [WARN] Claim 30: missing_evidence
**Location:** Abstract / Sec. 6; manuscript_input.md:7

We provide conditional geometric and momentum arguments, archived-source dual replay, and an explicit account of unsupported claims.

**Evidence:** null

No validation.json, replay output, original packets/events or archived source contents are declared. The paper table labels all replays PENDING_OR_FAIL. This audit cannot substantiate completion/identity; abstract phrasing may imply more than the displayed status.

**Correction/evidence:** Provide replay results in a fresh audit, or qualify the abstract as replay procedure/status pending verification.

### [WARN] Claim 31: missing_evidence
**Location:** Sec. 6; manuscript_input.md:98

Source and configuration hashes, full packets, references, covariance, QP data, events, actuator inputs, plant trajectories and resource logs are saved for every new attempt, including early failures.

**Evidence:** {"config_identity_and_traces_supplied":true,"packets_events_resource_logs_in_declared_set":false}

Eight config/trace sets support trajectories/reference/covariance/actuation presence, including early failure. Additional asserted packet/event/QP/resource files are outside the input set.

**Correction/evidence:** Provide complete raw artifact manifest and claimed streams.

### [WARN] Claim 60: ambiguous_mapping
**Location:** Figure 2; manuscript_input.md:129

Old and revised estimator velocity error norms on the same archived packets, without a new plant rollout.

**Evidence:** {"time_grid_identical":true,"packet_ids_in_inputs":false}

PNG norms/units and 1-7 s alignment agree, but raw packet identities and reanalysis source are not declared; identical packets/no new rollout cannot be certified from output errors alone.

**Correction/evidence:** Include immutable packet hashes and reanalysis provenance.

### [WARN] Claim 61: ambiguous_mapping
**Location:** Sec. 7.2 table D03_V1_S01 / actual gate ticks; manuscript_input.md:133

| D03_V1_S01 | 0 | 3001 | 3001 | 0.0016097 | 0.00964336 |

**Evidence:** {"phases":[0,1,2,8],"capture_values_all_zero":true,"capture_margins_all_zero":true}

Phase codes and zero-filled guard arrays support inactive capture, but do not directly encode a call count. Phase enum/source/event stream is absent. Counterfactual covariance tests are not actual rejected latch requests.

**Correction/evidence:** Include guard-call records or declared phase-enum source to certify zero calls.

### [WARN] Claim 66: ambiguous_mapping
**Location:** Sec. 7.2 table R03_V2_S01 / actual gate ticks; manuscript_input.md:134

| R03_V2_S01 | 0 | 3001 | 3001 | 0.0016097 | 0.00964336 |

**Evidence:** {"phases":[0,1,2,8],"capture_values_all_zero":true,"capture_margins_all_zero":true}

Phase codes and zero-filled guard arrays support inactive capture, but do not directly encode a call count. Phase enum/source/event stream is absent. Counterfactual covariance tests are not actual rejected latch requests.

**Correction/evidence:** Include guard-call records or declared phase-enum source to certify zero calls.

### [WARN] Claim 78: missing_evidence
**Location:** Sec. 7.3 table D01_V1_nominal / dual replay; manuscript_input.md:143

| D01_V1_nominal | development | COMPLETED | 27.992 | 7.992 | 0.0235542 | 0.363223 | 0 | PENDING_OR_FAIL |

**Evidence:** null

No validation.json or replay result is declared for this attempt. PENDING_OR_FAIL is not a PASS claim, but pending/fail mapping cannot be audited.

**Correction/evidence:** Supply replay validation and display its exact finalized status.

### [WARN] Claim 85: missing_evidence
**Location:** Sec. 7.3 table D02_V1_H2 / dual replay; manuscript_input.md:144

| D02_V1_H2 | development | NO_VERIFIED_CONTROL | 0.04 | NOT_EVALUATED | NOT_EVALUATED | 0 | 0 | PENDING_OR_FAIL |

**Evidence:** null

No validation.json or replay result is declared for this attempt. PENDING_OR_FAIL is not a PASS claim, but pending/fail mapping cannot be audited.

**Correction/evidence:** Supply replay validation and display its exact finalized status.

### [WARN] Claim 92: missing_evidence
**Location:** Sec. 7.3 table D03_V1_S01 / dual replay; manuscript_input.md:145

| D03_V1_S01 | development | NO_VERIFIED_CONTROL | 7.84 | NOT_EVALUATED | NOT_EVALUATED | 0.0103905 | 0 | PENDING_OR_FAIL |

**Evidence:** null

No validation.json or replay result is declared for this attempt. PENDING_OR_FAIL is not a PASS claim, but pending/fail mapping cannot be audited.

**Correction/evidence:** Supply replay validation and display its exact finalized status.

### [WARN] Claim 99: missing_evidence
**Location:** Sec. 7.3 table D04_V2_H2 / dual replay; manuscript_input.md:146

| D04_V2_H2 | development | NO_VERIFIED_CONTROL | 5.9 | NOT_EVALUATED | NOT_EVALUATED | 0 | 0 | PENDING_OR_FAIL |

**Evidence:** null

No validation.json or replay result is declared for this attempt. PENDING_OR_FAIL is not a PASS claim, but pending/fail mapping cannot be audited.

**Correction/evidence:** Supply replay validation and display its exact finalized status.

### [WARN] Claim 106: missing_evidence
**Location:** Sec. 7.3 table R01_V2_nominal / dual replay; manuscript_input.md:147

| R01_V2_nominal | regression | COMPLETED | 27.992 | 7.992 | 0.0235542 | 0.363223 | 0 | PENDING_OR_FAIL |

**Evidence:** null

No validation.json or replay result is declared for this attempt. PENDING_OR_FAIL is not a PASS claim, but pending/fail mapping cannot be audited.

**Correction/evidence:** Supply replay validation and display its exact finalized status.

### [WARN] Claim 113: missing_evidence
**Location:** Sec. 7.3 table R02_V2_H2 / dual replay; manuscript_input.md:148

| R02_V2_H2 | regression | NO_VERIFIED_CONTROL | 5.9 | NOT_EVALUATED | NOT_EVALUATED | 0 | 0 | PENDING_OR_FAIL |

**Evidence:** null

No validation.json or replay result is declared for this attempt. PENDING_OR_FAIL is not a PASS claim, but pending/fail mapping cannot be audited.

**Correction/evidence:** Supply replay validation and display its exact finalized status.

### [WARN] Claim 120: missing_evidence
**Location:** Sec. 7.3 table R03_V2_S01 / dual replay; manuscript_input.md:149

| R03_V2_S01 | regression | NO_VERIFIED_CONTROL | 7.84 | NOT_EVALUATED | NOT_EVALUATED | 0.0103905 | 0 | PENDING_OR_FAIL |

**Evidence:** null

No validation.json or replay result is declared for this attempt. PENDING_OR_FAIL is not a PASS claim, but pending/fail mapping cannot be audited.

**Correction/evidence:** Supply replay validation and display its exact finalized status.

### [WARN] Claim 127: missing_evidence
**Location:** Sec. 7.3 table R04_V2_H1 / dual replay; manuscript_input.md:150

| R04_V2_H1 | regression | COMPLETED | 27.932 | 7.932 | 0.0128079 | 0.597206 | 0 | PENDING_OR_FAIL |

**Evidence:** null

No validation.json or replay result is declared for this attempt. PENDING_OR_FAIL is not a PASS claim, but pending/fail mapping cannot be audited.

**Correction/evidence:** Supply replay validation and display its exact finalized status.

### [WARN] Claim 129: missing_evidence
**Location:** Figure 3 prespecification; manuscript_input.md:157

First nominal development run, chosen before the remaining outcomes.

**Evidence:** null

D01 is identified by the raw run path and config; timing of the illustration choice relative to other outcomes cannot be reconstructed from allowed results.

**Correction/evidence:** Include dated frozen selection contract, or state only that Figure 3 depicts D01.

### [WARN] Claim 136: ambiguous_mapping
**Location:** Figure 4; manuscript_input.md:163

Every ledger attempt, with category, stop time and outcome.

**Evidence:** {"figure_runs":["D01_V1_nominal","D02_V1_H2","D03_V1_S01","D04_V2_H2","R01_V2_nominal","R02_V2_H2","R03_V2_S01","R04_V2_H1"],"declared_new_count":8}

PNG shows every supplied new run with matching outcomes/times. Failed full windows are NE; observed safety-violation cells are No. The complete ledger is not declared, so exhaustive actual-ledger coverage cannot be certified.

**Correction/evidence:** Include immutable attempt ledger.

### [WARN] Claim 143: missing_evidence
**Location:** Figure 5 embedded annotation; manuscript_input.md:171

Archived H1/H3 parameter-error histories and finite-data rank.

**Evidence:** {"figure_COM_target_mm":5,"criterion_in_declared_configs":null}

COM and rank histories agree, but PNG additionally contains a 5 mm target line whose criterion/prespecification is not exposed in declared configurations.

**Correction/evidence:** Include the historical COM target definition or identify it explicitly as an illustrative line.

### [WARN] Claim 144: missing_evidence
**Location:** Sec. 7.4 spectra; manuscript_input.md:173

raw and whitened singular values computed without regularization.

**Evidence:** {"available_field":"singular_values","raw_regressor_and_whitener_supplied":false,"separate_raw_whitened_spectra":false}

Posterior files contain one singular_values array and scaled right singular vectors. Raw regressor, whitening definition and separate unregularized spectra are absent.

**Correction/evidence:** Supply raw regressor/whitener and spectrum outputs or a bounded source audit.

### [WARN] Claim 148: missing_evidence
**Location:** Sec. 8 qualification; manuscript_input.md:181

The original full-boundary interface qualification failure remains unchanged

**Evidence:** null

No historical full-boundary qualification failure/configuration is a declared input. This conservative limitation may be correct but is not independently confirmed here.

**Correction/evidence:** Include the historical qualification raw result or point to a separately bounded audit.

## Scope and limits

- Only declared manuscript snapshot, raw result/config files and five PNGs were scientific inputs. Skill procedure files and inputs.json supplied audit instructions/manifest. No executor summary, plan, prior audit, experiment log or conversation history was used.
- Independent checks included three terminal Phase-I LP re-solves and analytic box maxima; final-window angular speed/load/latch calculations; aligned historical RMS; covariance lower bounds; H1 COM/rank; historical paired action/state arrays; candidate spread and timing.
- No experiment was rerun. No repository-wide ledger completeness, packet replay, archived controller source behavior, original safety guard correctness, calibration, statistical validation or external citation verification is certified.
- Theoretical claims and citation/year claims are outside empirical audit scope. Linked theory notes, notation, generated compute table, figure_plan, claim matrix, references and validation.json were not declared and were not read.
- PNGs were visually checked for quantities, units, plotted scales, caption scope and outcome categories; pixelwise regeneration was not performed.
- Missing evidence/ambiguous mapping means this bounded input package is insufficient, not that the corresponding manuscript statement is false. No material numerical, aggregation or paired-config mismatch was found in the empirical values checked.

## All claim values

| ID | Location | Paper value | Evidence value | Status |
|---|---|---|---|---|
| 1 | Abstract / Sec. 3; manuscript_input.md:7 | {"joint_actuators":7,"commanded_external_wrench":0} | {"ctrl_columns":7,"max_abs_applied_wrench":0.0,"max_abs_applied_generalized_force":0.0} | exact_match |
| 2 | Sec. 3; manuscript_input.md:29 | 20 | {"flange_to_tool_mm":19.756891996217554,"flange_to_physical_face_mm":19.799999999999986} | ambiguous_mapping |
| 3 | Sec. 3 capture thresholds; manuscript_input.md:31 | 0.0001 | 0.0001 | exact_match |
| 4 | Sec. 3 capture thresholds; manuscript_input.md:31 | 0.05 | 0.05 | exact_match |
| 5 | Sec. 3 capture thresholds; manuscript_input.md:31 | 0.001 | 0.001 | exact_match |
| 6 | Sec. 3 capture thresholds; manuscript_input.md:31 | 0.2 | 0.2 | exact_match |
| 7 | Sec. 3; manuscript_input.md:31 | [0.5,0.1] | null | missing_evidence |
| 8 | Sec. 3; manuscript_input.md:33 | [50,2,1] | {"max_difference_from_recorded_utilization":3.469446951953614e-18,"max_rho":0.36322261973783676} | exact_match |
| 9 | Sec. 3; manuscript_input.md:35 | 20 | 20.0 | exact_match |
| 10 | Sec. 3; manuscript_input.md:35 | 20 | 20.0 | exact_match |
| 11 | Sec. 3; manuscript_input.md:35 | 2 | 2.0 | exact_match |
| 12 | Sec. 3; manuscript_input.md:35 | 0.1 | 0.1 | exact_match |
| 13 | Sec. 3; manuscript_input.md:35 | 0.02 | 0.02 | exact_match |
| 14 | Sec. 3; manuscript_input.md:37 | [0.002,0.002,0.02] | [0.002,0.002,0.02] | exact_match |
| 15 | Sec. 4.1; manuscript_input.md:43 | 0.25 | null | missing_evidence |
| 16 | Sec. 4.1; manuscript_input.md:43 | "causal sorting/rejection" | null | missing_evidence |
| 17 | Sec. 4.1; manuscript_input.md:45 | {"old":[0.03,0.2],"new":[0.003,0.02]} | {"new":[0.003,0.02],"generic_mission_defaults":[0.0002,0.001]} | ambiguous_mapping |
| 18 | Sec. 4.1; manuscript_input.md:45 | 10 | 10.0 | exact_match |
| 19 | Sec. 4.1; manuscript_input.md:45 | {"period_s":0.006,"delay_s":0.012,"position_sigma_m":1e-05,"rotation_sigma_rad":3.5e-05} | {"pose_period_s":0.006,"delay_s":0.012,"position_sigma_m":1e-05,"rotation_sigma_rad":3.5e-05,"position_bias_m":[2e-06,-2e-06,1e-06],"rotation_bias_rad":[3e-06,2e-06,-3e-06],"force_sigma_n":0.01,"moment_sigma_nm":0.0005,"seed":20802} | exact_match |
| 20 | Sec. 4.1; manuscript_input.md:47 | "raw-estimate capture guard" | null | missing_evidence |
| 21 | Sec. 4.2; manuscript_input.md:61 | {"stages":3,"tau_s":0.3} | {"tau_s":0.3,"stage_count":null} | ambiguous_mapping |
| 22 | Sec. 4.2; manuscript_input.md:61 | [1,0.75,0.5,0.25,0] | [1.0,0.75,0.5,0.25,0.0] | exact_match |
| 23 | Sec. 4.2; manuscript_input.md:61 | 0.2 | {"configured_s":0.2,"terminal_candidate_horizons_s":[0.2,0.2,0.2,0.2,0.2]} | exact_match |
| 24 | Sec. 4.2; manuscript_input.md:65 | 0.2 | 0.2 | exact_match |
| 25 | Sec. 4.2; manuscript_input.md:65 | 0.008 | 0.008 | exact_match |
| 26 | Sec. 4.2; manuscript_input.md:65 | [0.05,0.1] | [0.05,0.1] | exact_match |
| 27 | Sec. 6; manuscript_input.md:98 | "b1af09f89226fa6f2ae1b362f3d6225b3363cc17" | "b1af09f89226fa6f2ae1b362f3d6225b3363cc17" | exact_match |
| 28 | Sec. 6; manuscript_input.md:100 | [2,8,4] | {"observed_versions":2,"observed_development":4,"observed_regression":4,"effective_max_development":6,"effective_max_attempts":16} | ambiguous_mapping |
| 29 | Sec. 6; manuscript_input.md:100 | {"clusters":6,"seeds":3,"B2":6,"fine_step":2} | null | missing_evidence |
| 30 | Abstract / Sec. 6; manuscript_input.md:7 | "archived-source dual replay" | null | missing_evidence |
| 31 | Sec. 6; manuscript_input.md:98 | "complete archive" | {"config_identity_and_traces_supplied":true,"packets_events_resource_logs_in_declared_set":false} | missing_evidence |
| 32 | Sec. 7.1 table H2_prior / classification; manuscript_input.md:112 | "LINEAR_HARD_SET_INFEASIBLE" | "LINEAR_HARD_SET_INFEASIBLE" | exact_match |
| 33 | Sec. 7.1 table H2_prior / z; manuscript_input.md:112 | "0.0195548" | 0.019554833668535825 | rounding_ok |
| 34 | Sec. 7.1 table H2_prior / row; manuscript_input.md:112 | "gripper_contact_pad__tumbling_target_geom" | "gripper_contact_pad__tumbling_target_geom" | exact_match |
| 35 | Sec. 7.1 table H2_prior / required; manuscript_input.md:112 | "-0.127798" | -0.12779809474308393 | rounding_ok |
| 36 | Sec. 7.1 table H2_prior / box maximum; manuscript_input.md:112 | "-0.161947" | -0.1619474977954096 | rounding_ok |
| 37 | Sec. 7.1 table H2_prior / shortfall; manuscript_input.md:112 | "0.0341494" | 0.034149403052325666 | rounding_ok |
| 38 | Sec. 7.1 H2_prior; manuscript_input.md:117 | [true,true,false] | [true,true,false] | exact_match |
| 39 | Figure 1 H2_prior; manuscript_input.md:121 | 0.5 | {"duration":0.49999999999994493,"samples":26,"all_scales_one":true} | rounding_ok |
| 40 | Sec. 7.1; manuscript_input.md:117 | "before contact" | {"pad_clearance_m":0.008906368798562093,"all_logged_nearby_distances_positive":true} | exact_match |
| 41 | Sec. 7.1 table S01_noise_delay / classification; manuscript_input.md:113 | "LINEAR_HARD_SET_INFEASIBLE" | "LINEAR_HARD_SET_INFEASIBLE" | exact_match |
| 42 | Sec. 7.1 table S01_noise_delay / z; manuscript_input.md:113 | "0.00550255" | 0.005502548932376166 | rounding_ok |
| 43 | Sec. 7.1 table S01_noise_delay / row; manuscript_input.md:113 | "gripper_contact_pad__tumbling_target_geom" | "gripper_contact_pad__tumbling_target_geom" | exact_match |
| 44 | Sec. 7.1 table S01_noise_delay / required; manuscript_input.md:113 | "-0.0372463" | -0.03724633332276628 | rounding_ok |
| 45 | Sec. 7.1 table S01_noise_delay / box maximum; manuscript_input.md:113 | "-0.0467254" | -0.046725359534771936 | rounding_ok |
| 46 | Sec. 7.1 table S01_noise_delay / shortfall; manuscript_input.md:113 | "0.00947903" | 0.009479026212005658 | rounding_ok |
| 47 | Sec. 7.1 S01_noise_delay; manuscript_input.md:117 | [true,true,false] | [true,true,false] | exact_match |
| 48 | Figure 1 S01_noise_delay; manuscript_input.md:121 | 0.5 | {"duration":0.49999999999994493,"samples":26,"all_scales_one":true} | rounding_ok |
| 49 | Sec. 7.1 table D05_final_nominal / classification; manuscript_input.md:114 | "LINEAR_HARD_SET_INFEASIBLE" | "LINEAR_HARD_SET_INFEASIBLE" | exact_match |
| 50 | Sec. 7.1 table D05_final_nominal / z; manuscript_input.md:114 | "0.000983" | 0.0009830003257214753 | rounding_ok |
| 51 | Sec. 7.1 table D05_final_nominal / row; manuscript_input.md:114 | "gripper_contact_pad__tumbling_target_geom" | "gripper_contact_pad__tumbling_target_geom" | exact_match |
| 52 | Sec. 7.1 table D05_final_nominal / required; manuscript_input.md:114 | "-0.027238" | -0.02723799318389345 | rounding_ok |
| 53 | Sec. 7.1 table D05_final_nominal / box maximum; manuscript_input.md:114 | "-0.0288792" | -0.028879216274506264 | rounding_ok |
| 54 | Sec. 7.1 table D05_final_nominal / shortfall; manuscript_input.md:114 | "0.00164122" | 0.001641223090612813 | rounding_ok |
| 55 | Sec. 7.1 D05_final_nominal; manuscript_input.md:117 | [true,true,false] | [true,true,false] | exact_match |
| 56 | Figure 1 D05_final_nominal; manuscript_input.md:121 | 0.5 | {"duration":0.500000000000167,"samples":26,"all_scales_one":true} | rounding_ok |
| 57 | Abstract; manuscript_input.md:7 | {"old_mm_s":[2.08,2.17],"new_mm_s":[0.218,0.228]} | {"old_components_mm_s":[2.170262359072463,2.1661456594611277,2.075244415352518],"new_components_mm_s":[0.22671025016534385,0.22754394951209245,0.2176123466245102]} | rounding_ok |
| 58 | Sec. 7.2; manuscript_input.md:125 | [1,7] | {"counts":[3001,3001],"time_difference_max_s":0.0,"bounds":[1.0000000000000007,6.999999999999451]} | exact_match |
| 59 | Sec. 7.2; manuscript_input.md:125 | "about one tenth / all angular decrease" | {"linear_ratios":[0.1044621398964116,0.10504554415269497,0.1048610684190396],"angular_old_rad_s":[0.008777647393479554,0.009254209672920979,0.009239217770357546],"angular_new_rad_s":[0.0012216585952662703,0.0013085332094291222,0.0012528480153747295]} | exact_match |
| 60 | Figure 2; manuscript_input.md:129 | "same packets/offline" | {"time_grid_identical":true,"packet_ids_in_inputs":false} | ambiguous_mapping |
| 61 | Sec. 7.2 table D03_V1_S01 / actual gate ticks; manuscript_input.md:133 | 0 | {"phases":[0,1,2,8],"capture_values_all_zero":true,"capture_margins_all_zero":true} | ambiguous_mapping |
| 62 | Sec. 7.2 table D03_V1_S01 / sample count; manuscript_input.md:133 | 3001 | 3001 | exact_match |
| 63 | Sec. 7.2 table D03_V1_S01 / uncertainty precludes count; manuscript_input.md:133 | 3001 | 3001 | exact_match |
| 64 | Sec. 7.2 table D03_V1_S01 / linear minimum m/s; manuscript_input.md:133 | 0.0016097 | 0.0016096967038863287 | rounding_ok |
| 65 | Sec. 7.2 table D03_V1_S01 / angular minimum rad/s; manuscript_input.md:133 | 0.00964336 | 0.009643356348791552 | rounding_ok |
| 66 | Sec. 7.2 table R03_V2_S01 / actual gate ticks; manuscript_input.md:134 | 0 | {"phases":[0,1,2,8],"capture_values_all_zero":true,"capture_margins_all_zero":true} | ambiguous_mapping |
| 67 | Sec. 7.2 table R03_V2_S01 / sample count; manuscript_input.md:134 | 3001 | 3001 | exact_match |
| 68 | Sec. 7.2 table R03_V2_S01 / uncertainty precludes count; manuscript_input.md:134 | 3001 | 3001 | exact_match |
| 69 | Sec. 7.2 table R03_V2_S01 / linear minimum m/s; manuscript_input.md:134 | 0.0016097 | 0.0016096967038863287 | rounding_ok |
| 70 | Sec. 7.2 table R03_V2_S01 / angular minimum rad/s; manuscript_input.md:134 | 0.00964336 | 0.009643356348791552 | rounding_ok |
| 71 | Sec. 7.2 scope; manuscript_input.md:137 | "limited necessary-condition test" | {"linear_margin_min":0.0016096967038863287,"linear_threshold":0.001,"angular_margin_min":0.009643356348791552,"angular_threshold":0.003490658503988659} | exact_match |
| 72 | Sec. 7.3 table D01_V1_nominal / status; manuscript_input.md:143 | "COMPLETED" | "COMPLETED" | exact_match |
| 73 | Sec. 7.3 table D01_V1_nominal / end_s; manuscript_input.md:143 | "27.992" | 27.991999999995365 | rounding_ok |
| 74 | Sec. 7.3 table D01_V1_nominal / latch_s; manuscript_input.md:143 | "7.992" | 7.991999999999342 | rounding_ok |
| 75 | Sec. 7.3 table D01_V1_nominal / world last2s deg/s; manuscript_input.md:143 | "0.0235542" | 0.023554153218418417 | rounding_ok |
| 76 | Sec. 7.3 table D01_V1_nominal / peak rho; manuscript_input.md:143 | "0.363223" | 0.36322261973783676 | rounding_ok |
| 77 | Sec. 7.3 table D01_V1_nominal / recorded safety violations; manuscript_input.md:143 | 0 | 0 | exact_match |
| 78 | Sec. 7.3 table D01_V1_nominal / dual replay; manuscript_input.md:143 | "PENDING_OR_FAIL" | null | missing_evidence |
| 79 | Sec. 7.3 table D02_V1_H2 / status; manuscript_input.md:144 | "NO_VERIFIED_CONTROL" | "NO_VERIFIED_CONTROL" | exact_match |
| 80 | Sec. 7.3 table D02_V1_H2 / end_s; manuscript_input.md:144 | "0.04" | 0.04000000000000002 | rounding_ok |
| 81 | Sec. 7.3 table D02_V1_H2 / latch_s; manuscript_input.md:144 | "NOT_EVALUATED" | null | exact_match |
| 82 | Sec. 7.3 table D02_V1_H2 / world last2s deg/s; manuscript_input.md:144 | "NOT_EVALUATED" | null | exact_match |
| 83 | Sec. 7.3 table D02_V1_H2 / peak rho; manuscript_input.md:144 | "0" | 0.0 | exact_match |
| 84 | Sec. 7.3 table D02_V1_H2 / recorded safety violations; manuscript_input.md:144 | 0 | 0 | exact_match |
| 85 | Sec. 7.3 table D02_V1_H2 / dual replay; manuscript_input.md:144 | "PENDING_OR_FAIL" | null | missing_evidence |
| 86 | Sec. 7.3 table D03_V1_S01 / status; manuscript_input.md:145 | "NO_VERIFIED_CONTROL" | "NO_VERIFIED_CONTROL" | exact_match |
| 87 | Sec. 7.3 table D03_V1_S01 / end_s; manuscript_input.md:145 | "7.84" | 7.839999999999359 | rounding_ok |
| 88 | Sec. 7.3 table D03_V1_S01 / latch_s; manuscript_input.md:145 | "NOT_EVALUATED" | null | exact_match |
| 89 | Sec. 7.3 table D03_V1_S01 / world last2s deg/s; manuscript_input.md:145 | "NOT_EVALUATED" | null | exact_match |
| 90 | Sec. 7.3 table D03_V1_S01 / peak rho; manuscript_input.md:145 | "0.0103905" | 0.01039049937806134 | rounding_ok |
| 91 | Sec. 7.3 table D03_V1_S01 / recorded safety violations; manuscript_input.md:145 | 0 | 0 | exact_match |
| 92 | Sec. 7.3 table D03_V1_S01 / dual replay; manuscript_input.md:145 | "PENDING_OR_FAIL" | null | missing_evidence |
| 93 | Sec. 7.3 table D04_V2_H2 / status; manuscript_input.md:146 | "NO_VERIFIED_CONTROL" | "NO_VERIFIED_CONTROL" | exact_match |
| 94 | Sec. 7.3 table D04_V2_H2 / end_s; manuscript_input.md:146 | "5.9" | 5.899999999999572 | rounding_ok |
| 95 | Sec. 7.3 table D04_V2_H2 / latch_s; manuscript_input.md:146 | "NOT_EVALUATED" | null | exact_match |
| 96 | Sec. 7.3 table D04_V2_H2 / world last2s deg/s; manuscript_input.md:146 | "NOT_EVALUATED" | null | exact_match |
| 97 | Sec. 7.3 table D04_V2_H2 / peak rho; manuscript_input.md:146 | "0" | 0.0 | exact_match |
| 98 | Sec. 7.3 table D04_V2_H2 / recorded safety violations; manuscript_input.md:146 | 0 | 0 | exact_match |
| 99 | Sec. 7.3 table D04_V2_H2 / dual replay; manuscript_input.md:146 | "PENDING_OR_FAIL" | null | missing_evidence |
| 100 | Sec. 7.3 table R01_V2_nominal / status; manuscript_input.md:147 | "COMPLETED" | "COMPLETED" | exact_match |
| 101 | Sec. 7.3 table R01_V2_nominal / end_s; manuscript_input.md:147 | "27.992" | 27.991999999995365 | rounding_ok |
| 102 | Sec. 7.3 table R01_V2_nominal / latch_s; manuscript_input.md:147 | "7.992" | 7.991999999999342 | rounding_ok |
| 103 | Sec. 7.3 table R01_V2_nominal / world last2s deg/s; manuscript_input.md:147 | "0.0235542" | 0.023554153218418417 | rounding_ok |
| 104 | Sec. 7.3 table R01_V2_nominal / peak rho; manuscript_input.md:147 | "0.363223" | 0.36322261973783676 | rounding_ok |
| 105 | Sec. 7.3 table R01_V2_nominal / recorded safety violations; manuscript_input.md:147 | 0 | 0 | exact_match |
| 106 | Sec. 7.3 table R01_V2_nominal / dual replay; manuscript_input.md:147 | "PENDING_OR_FAIL" | null | missing_evidence |
| 107 | Sec. 7.3 table R02_V2_H2 / status; manuscript_input.md:148 | "NO_VERIFIED_CONTROL" | "NO_VERIFIED_CONTROL" | exact_match |
| 108 | Sec. 7.3 table R02_V2_H2 / end_s; manuscript_input.md:148 | "5.9" | 5.899999999999572 | rounding_ok |
| 109 | Sec. 7.3 table R02_V2_H2 / latch_s; manuscript_input.md:148 | "NOT_EVALUATED" | null | exact_match |
| 110 | Sec. 7.3 table R02_V2_H2 / world last2s deg/s; manuscript_input.md:148 | "NOT_EVALUATED" | null | exact_match |
| 111 | Sec. 7.3 table R02_V2_H2 / peak rho; manuscript_input.md:148 | "0" | 0.0 | exact_match |
| 112 | Sec. 7.3 table R02_V2_H2 / recorded safety violations; manuscript_input.md:148 | 0 | 0 | exact_match |
| 113 | Sec. 7.3 table R02_V2_H2 / dual replay; manuscript_input.md:148 | "PENDING_OR_FAIL" | null | missing_evidence |
| 114 | Sec. 7.3 table R03_V2_S01 / status; manuscript_input.md:149 | "NO_VERIFIED_CONTROL" | "NO_VERIFIED_CONTROL" | exact_match |
| 115 | Sec. 7.3 table R03_V2_S01 / end_s; manuscript_input.md:149 | "7.84" | 7.839999999999359 | rounding_ok |
| 116 | Sec. 7.3 table R03_V2_S01 / latch_s; manuscript_input.md:149 | "NOT_EVALUATED" | null | exact_match |
| 117 | Sec. 7.3 table R03_V2_S01 / world last2s deg/s; manuscript_input.md:149 | "NOT_EVALUATED" | null | exact_match |
| 118 | Sec. 7.3 table R03_V2_S01 / peak rho; manuscript_input.md:149 | "0.0103905" | 0.01039049937806134 | rounding_ok |
| 119 | Sec. 7.3 table R03_V2_S01 / recorded safety violations; manuscript_input.md:149 | 0 | 0 | exact_match |
| 120 | Sec. 7.3 table R03_V2_S01 / dual replay; manuscript_input.md:149 | "PENDING_OR_FAIL" | null | missing_evidence |
| 121 | Sec. 7.3 table R04_V2_H1 / status; manuscript_input.md:150 | "COMPLETED" | "COMPLETED" | exact_match |
| 122 | Sec. 7.3 table R04_V2_H1 / end_s; manuscript_input.md:150 | "27.932" | 27.931999999995398 | rounding_ok |
| 123 | Sec. 7.3 table R04_V2_H1 / latch_s; manuscript_input.md:150 | "7.932" | 7.9319999999993485 | rounding_ok |
| 124 | Sec. 7.3 table R04_V2_H1 / world last2s deg/s; manuscript_input.md:150 | "0.0128079" | 0.012807859564078115 | rounding_ok |
| 125 | Sec. 7.3 table R04_V2_H1 / peak rho; manuscript_input.md:150 | "0.597206" | 0.597206336221039 | rounding_ok |
| 126 | Sec. 7.3 table R04_V2_H1 / recorded safety violations; manuscript_input.md:150 | 0 | 0 | exact_match |
| 127 | Sec. 7.3 table R04_V2_H1 / dual replay; manuscript_input.md:150 | "PENDING_OR_FAIL" | null | missing_evidence |
| 128 | Abstract / Sec. 7.3; manuscript_input.md:7 | {"latch_s":7.992,"post_s":20,"window_s":2,"world_deg_s":0.023554} | {"latch_s":7.991999999999342,"end_s":27.991999999995365,"post_s":19.99999999999602,"world_deg_s":0.023554153218418417} | rounding_ok |
| 129 | Figure 3 prespecification; manuscript_input.md:157 | "prespecified D01" | null | missing_evidence |
| 130 | Figure 3 quantities; manuscript_input.md:157 | "absolute-time quantities" | {"max_P_drift":2.639237500491339e-08,"max_H_drift":4.324381723050829e-07,"latch_marker_s":7.991999999999342,"shaded_window_s":[25.991999999999344,27.991999999999344]} | exact_match |
| 131 | Sec. 7.3 momentum; manuscript_input.md:153 | "consistency with nonzero momentum" | {"initial_H_norm":0.02617993877991494,"final_H_norm":0.02618006068888797} | exact_match |
| 132 | Sec. 7.3 D02_V1_H2; manuscript_input.md:159 | 0.04 | {"stop_s":0.04000000000000002,"reasons":["PREDICTED_TASK_RESIDUAL","PREDICTED_TASK_RESIDUAL","PREDICTED_TASK_RESIDUAL","PREDICTED_TASK_RESIDUAL","PREDICTED_TASK_RESIDUAL"],"steps":[0,0,0,0,0]} | rounding_ok |
| 133 | Sec. 7.3 D04_V2_H2; manuscript_input.md:159 | 5.9 | {"stop_s":5.899999999999572,"reasons":["PREDICTED_TASK_RESIDUAL","PREDICTED_TASK_RESIDUAL","PREDICTED_TASK_RESIDUAL","PREDICTED_TASK_RESIDUAL","PREDICTED_TASK_RESIDUAL"],"steps":[100,100,100,100,100]} | rounding_ok |
| 134 | Sec. 7.3; manuscript_input.md:159 | 5 | {"count":5,"reasons":["PREDICTED_TASK_RESIDUAL","PREDICTED_TASK_RESIDUAL","PREDICTED_TASK_RESIDUAL","PREDICTED_TASK_RESIDUAL","PREDICTED_TASK_RESIDUAL"],"verified":[false,false,false,false,false]} | exact_match |
| 135 | Sec. 7.3; manuscript_input.md:159 | [7.822,7.84] | {"contact_s":7.821999999999361,"stop_s":7.839999999999359} | rounding_ok |
| 136 | Figure 4; manuscript_input.md:163 | "all attempts" | {"figure_runs":["D01_V1_nominal","D02_V1_H2","D03_V1_S01","D04_V2_H2","R01_V2_nominal","R02_V2_H2","R03_V2_S01","R04_V2_H1"],"declared_new_count":8} | ambiguous_mapping |
| 137 | Sec. 7.4 timing; manuscript_input.md:167 | {"deadline_s":0.02,"multi_second_rejections":true} | {"median_predictor_latency_by_run_s":{"D01_V1_nominal":0.49974825023673475,"D02_V1_H2":0.06345620006322861,"D03_V1_S01":0.5008653998374939,"D04_V2_H2":0.040549349738284945,"R01_V2_nominal":0.4991549502592534,"R02_V2_H2":0.04064954980276525,"R03_V2_S01":0.49924639984965324,"R04_V2_H1":0.49711389979347587},"rejection_latencies_s":{"D02_V1_H2":0.06345620006322861,"D03_V1_S01":4.420055899769068,"D04_V2_H2":4.502479400020093,"R02_V2_H2":4.184798799920827,"R03_V2_S01":4.178973699919879}} | exact_match |
| 138 | Sec. 7.4 timing scope; manuscript_input.md:167 | "early omission/final external scope" | {"D02_control_max_s":0.0012749996967613697,"D02_terminal_predictor_s":0.06345620006322861,"R02_latency_scope":"runner wall time for every controller call, including terminal exceptions"} | exact_match |
| 139 | Sec. 4.3 / Sec. 7.4; manuscript_input.md:69 | false | {"config_feedback":[false,false,false,false,false,false,false,false],"any_trace_feedback":[false,false,false,false,false,false,false,false]} | exact_match |
| 140 | Figure 5 H1; manuscript_input.md:171 | [10,15.28] | {"max_rank":10,"final_rank":10,"final_COM_error_mm":15.275807698397005} | rounding_ok |
| 141 | Figure 5 H1 paired comparison; manuscript_input.md:171 | 0 | {"max_ctrl_diff":0.0,"max_qpos_diff":0.0,"max_qvel_diff":0.0,"identified_model_pi_matches_posterior":true,"same_chosen_alpha":true,"feedback_flags":[false,true]} | exact_match |
| 142 | Figure 5 H3 paired comparison; manuscript_input.md:171 | 0 | {"max_ctrl_diff":0.0,"max_qpos_diff":0.0,"max_qvel_diff":0.0,"identified_model_pi_matches_posterior":true,"same_chosen_alpha":true,"feedback_flags":[false,true]} | exact_match |
| 143 | Figure 5 embedded annotation; manuscript_input.md:171 | "5 mm target line" | {"figure_COM_target_mm":5,"criterion_in_declared_configs":null} | missing_evidence |
| 144 | Sec. 7.4 spectra; manuscript_input.md:173 | "raw and whitened unregularized spectra" | {"available_field":"singular_values","raw_regressor_and_whitener_supplied":false,"separate_raw_whitened_spectra":false} | missing_evidence |
| 145 | Sec. 8; manuscript_input.md:177 | [0.00167,0.2,0.000209] | {"span_s":0.0016700626778529681,"horizon_s":0.2,"normalized_span":0.00020875783473162102} | rounding_ok |
| 146 | Sec. 6; manuscript_input.md:102 | {"independent_pairs":0,"B2":0} | {"declared_new_runs":8,"methods":["B1"]} | exact_match |
| 147 | Conclusion; manuscript_input.md:185 | "no H2/S01 completion" | {"D02_V1_H2":false,"D03_V1_S01":false,"D04_V2_H2":false,"R02_V2_H2":false,"R03_V2_S01":false} | exact_match |
| 148 | Sec. 8 qualification; manuscript_input.md:181 | "unchanged qualification failure" | null | missing_evidence |

## Exact claims and evidence paths

### Claim 1: exact_match
**Location:** Abstract / Sec. 3; manuscript_input.md:7

free-floating seven-joint arm with no active base or target actuators

Seven joint torque columns and zero external applied wrench/generalized force support the recorded actuator scope; model topology/source is not declared.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\trace.npz`

### Claim 2: ambiguous_mapping
**Location:** Sec. 3; manuscript_input.md:29

fixed 20 mm tool/interface geometry

No geometry model or T_FE is declared. Recorded offsets are 19.7569 and 19.8 mm; these can be compatible with a nominal 20 mm dimension, but the quantity has no unique frame/dimension mapping here. This is not a demonstrated number error.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\trace.npz`

### Claim 3: exact_match
**Location:** Sec. 3 capture thresholds; manuscript_input.md:31

0.1 mm position

Matches the declared raw evidence.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\config.json`

### Claim 4: exact_match
**Location:** Sec. 3 capture thresholds; manuscript_input.md:31

0.05 degree attitude

Matches the declared raw evidence.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\config.json`

### Claim 5: exact_match
**Location:** Sec. 3 capture thresholds; manuscript_input.md:31

1 mm/s relative linear velocity

Matches the declared raw evidence.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\config.json`

### Claim 6: exact_match
**Location:** Sec. 3 capture thresholds; manuscript_input.md:31

0.2 degree/s relative angular velocity

Matches the declared raw evidence.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\config.json`

### Claim 7: missing_evidence
**Location:** Sec. 3; manuscript_input.md:31

After locking, position and attitude limits are 0.5 mm and 0.1 degree.

The declared mission/run configurations expose capture/performance thresholds, but not these post-lock safety limits. Observed small errors do not establish the configured guard limits.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\config.json`

### Claim 8: exact_match
**Location:** Sec. 3; manuscript_input.md:33

\rho=\|F\|/(50\,\mathrm N)+\|M\|/(2\,\mathrm{Nm})\le1.

Direct force/moment norm calculation agrees to floating-point precision; Figure 3 also shows threshold 1. Code-level guard enforcement is not certified.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\trace.npz`

### Claim 9: exact_match
**Location:** Sec. 3; manuscript_input.md:35

at most 20 s approach

Matches the declared raw evidence.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\config.json`

### Claim 10: exact_match
**Location:** Sec. 3; manuscript_input.md:35

requires 20 s after locking

Matches the declared raw evidence.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\config.json`

### Claim 11: exact_match
**Location:** Sec. 3; manuscript_input.md:35

fixed final two seconds

Matches the declared raw evidence.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\config.json`

### Claim 12: exact_match
**Location:** Sec. 3; manuscript_input.md:35

world angular speed at most 0.1 degree/s

Matches the declared raw evidence.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\config.json`

### Claim 13: exact_match
**Location:** Sec. 3; manuscript_input.md:35

target–base relative angular speed at most 0.02 degree/s

Matches the declared raw evidence.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\config.json`

### Claim 14: exact_match
**Location:** Sec. 3; manuscript_input.md:37

The physics and joint servo run every 2 ms; the hierarchical velocity task runs every 20 ms.

Matches the declared raw evidence.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\config.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\trace.npz`

### Claim 15: missing_evidence
**Location:** Sec. 4.1; manuscript_input.md:43

A 0.25 s history stores the contact/process mode used over each past interval.

History length and process-mode replay behavior are not exposed in declared configs/traces.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\config.json`

### Claim 16: missing_evidence
**Location:** Sec. 4.1; manuscript_input.md:43

Batches are sorted by measurement time; duplicate and older out-of-order samples are rejected rather than retrospectively assimilated.

No original packet/rejection stream or estimator source is declared. Causality/rejection cannot be certified from error outputs alone.

**Evidence files:**
- None in the declared input set.

### Claim 17: ambiguous_mapping
**Location:** Sec. 4.1; manuscript_input.md:45

development reduced the free-motion acceleration process densities from 0.03/0.2 to 0.003 m/s^(3/2) and 0.02 rad/s^(3/2)

New noisy process densities are exact. The old S01 effective config is not supplied, and generic mission defaults need not equal its noisy overrides. The old 0.03/0.2 cannot be mapped independently.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D03_V1_S01\config.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\configs\adaptive_capture\mission.yaml`

### Claim 18: exact_match
**Location:** Sec. 4.1; manuscript_input.md:45

A contact mode multiplies process density by ten.

Matches the declared raw evidence.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D03_V1_S01\config.json`

### Claim 19: exact_match
**Location:** Sec. 4.1; manuscript_input.md:45

samples pose every 6 ms and delivers it after 12 ms, with 10 micrometre and 35 microradian component noise standard deviations and the original biases.

Sensor YAML and new noisy effective configs agree, including bias and seed. Original S01 packet/config equivalence remains unverified because that config/packet stream is not declared.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\configs\adaptive_capture\sensors.yaml`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D03_V1_S01\config.json`

### Claim 20: missing_evidence
**Location:** Sec. 4.1; manuscript_input.md:47

Capture tests use the raw timestamped estimate and its covariance, never the smoother reference state.

Separate estimate/reference arrays exist, but guard input identity requires source or call-level records.

**Evidence files:**
- None in the declared input set.

### Claim 21: ambiguous_mapping
**Location:** Sec. 4.2; manuscript_input.md:61

Progress demand is filtered through three positive first-order stages with a 0.30 s time constant.

Time constant matches; number/positivity of stages is not exposed by the allowed new-run inputs.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\config.json`

### Claim 22: exact_match
**Location:** Sec. 4.2; manuscript_input.md:61

demands of 1, 0.75, 0.5, 0.25 and 0 times the original maximum virtual-time rate, in descending order.

Matches the declared raw evidence.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\config.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D04_V2_H2\progress_governor.json`

### Claim 23: exact_match
**Location:** Sec. 4.2; manuscript_input.md:61

It integrates an independently built nominal-prior model for 0.20 s with the actual servo ramp, the original HQP and sampled contact model.

Horizon matches config and terminal decisions. Independence, actual ramp/HQP implementation and no-future-state reads require source-level review not supported by this input package.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\config.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D04_V2_H2\progress_governor.json`

### Claim 24: exact_match
**Location:** Sec. 4.2; manuscript_input.md:65

The sole second-version revision allows the initial 0.20 s of soft task-residual transient

Effective H2 configs differ only by startup allowance, schema/version and source identity; initial qpos/qvel match exactly. This checks effective parameters, not exclusive source-level behavior.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D02_V1_H2\config.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D04_V2_H2\config.json`

### Claim 25: exact_match
**Location:** Sec. 4.2; manuscript_input.md:65

position-error threshold remains 8 mm

Matches the declared raw evidence.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\config.json`

### Claim 26: exact_match
**Location:** Sec. 4.2; manuscript_input.md:65

linear/angular residual thresholds remain 0.05 m/s and 0.10 rad/s

Matches the declared raw evidence.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\config.json`

### Claim 27: exact_match
**Location:** Sec. 6; manuscript_input.md:98

The archived baseline commit is b1af09f89226fa6f2ae1b362f3d6225b3363cc17.

Effective new-run config matches n209 YAML. The generic inherited mission YAML has an older base id, overridden in effective configs.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\config.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\configs\n209_paper_system.yaml`

### Claim 28: ambiguous_mapping
**Location:** Sec. 6; manuscript_input.md:100

The bounded plan allows at most two candidate versions, eight development attempts and four final seen regressions.

The eight supplied attempts do not exceed the manuscript budget. However inherited effective cap fields are 6/16; no frozen n209 budget contract/ledger maps them to the stated 8+4 policy. This is a provenance ambiguity, not a demonstrated count violation.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\config.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\configs\n209_paper_system.yaml`

### Claim 29: missing_evidence
**Location:** Sec. 6; manuscript_input.md:100

The planned independent design has six physical clusters, three fixed sensor seeds and paired B0/B1 methods, followed by six prespecified B2 and two fine-step trials.

Clearly presented as an unexecuted plan; its frozen seed/distribution contract and prespecification are not among allowed inputs.

**Evidence files:**
- None in the declared input set.

### Claim 30: missing_evidence
**Location:** Abstract / Sec. 6; manuscript_input.md:7

We provide conditional geometric and momentum arguments, archived-source dual replay, and an explicit account of unsupported claims.

No validation.json, replay output, original packets/events or archived source contents are declared. The paper table labels all replays PENDING_OR_FAIL. This audit cannot substantiate completion/identity; abstract phrasing may imply more than the displayed status.

**Evidence files:**
- None in the declared input set.

### Claim 31: missing_evidence
**Location:** Sec. 6; manuscript_input.md:98

Source and configuration hashes, full packets, references, covariance, QP data, events, actuator inputs, plant trajectories and resource logs are saved for every new attempt, including early failures.

Eight config/trace sets support trajectories/reference/covariance/actuation presence, including early failure. Additional asserted packet/event/QP/resource files are outside the input set.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\config.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\trace.npz`

### Claim 32: exact_match
**Location:** Sec. 7.1 table H2_prior / classification; manuscript_input.md:112

| H2_prior | LINEAR_HARD_SET_INFEASIBLE | 0.0195548 | gripper_contact_pad__tumbling_target_geom | -0.127798 | -0.161947 | 0.0341494 |

Independent SciPy/HiGHS Phase-I solve and analytic joint-box maximization reproduce the displayed standard rounding.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\diagnostics\H2_prior\snapshots.json`

### Claim 33: rounding_ok
**Location:** Sec. 7.1 table H2_prior / z; manuscript_input.md:112

| H2_prior | LINEAR_HARD_SET_INFEASIBLE | 0.0195548 | gripper_contact_pad__tumbling_target_geom | -0.127798 | -0.161947 | 0.0341494 |

Independent SciPy/HiGHS Phase-I solve and analytic joint-box maximization reproduce the displayed standard rounding.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\diagnostics\H2_prior\snapshots.json`

### Claim 34: exact_match
**Location:** Sec. 7.1 table H2_prior / row; manuscript_input.md:112

| H2_prior | LINEAR_HARD_SET_INFEASIBLE | 0.0195548 | gripper_contact_pad__tumbling_target_geom | -0.127798 | -0.161947 | 0.0341494 |

Independent SciPy/HiGHS Phase-I solve and analytic joint-box maximization reproduce the displayed standard rounding.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\diagnostics\H2_prior\snapshots.json`

### Claim 35: rounding_ok
**Location:** Sec. 7.1 table H2_prior / required; manuscript_input.md:112

| H2_prior | LINEAR_HARD_SET_INFEASIBLE | 0.0195548 | gripper_contact_pad__tumbling_target_geom | -0.127798 | -0.161947 | 0.0341494 |

Independent SciPy/HiGHS Phase-I solve and analytic joint-box maximization reproduce the displayed standard rounding.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\diagnostics\H2_prior\snapshots.json`

### Claim 36: rounding_ok
**Location:** Sec. 7.1 table H2_prior / box maximum; manuscript_input.md:112

| H2_prior | LINEAR_HARD_SET_INFEASIBLE | 0.0195548 | gripper_contact_pad__tumbling_target_geom | -0.127798 | -0.161947 | 0.0341494 |

Independent SciPy/HiGHS Phase-I solve and analytic joint-box maximization reproduce the displayed standard rounding.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\diagnostics\H2_prior\snapshots.json`

### Claim 37: rounding_ok
**Location:** Sec. 7.1 table H2_prior / shortfall; manuscript_input.md:112

| H2_prior | LINEAR_HARD_SET_INFEASIBLE | 0.0195548 | gripper_contact_pad__tumbling_target_geom | -0.127798 | -0.161947 | 0.0341494 |

Independent SciPy/HiGHS Phase-I solve and analytic joint-box maximization reproduce the displayed standard rounding.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\diagnostics\H2_prior\snapshots.json`

### Claim 38: exact_match
**Location:** Sec. 7.1 H2_prior; manuscript_input.md:117

In all three failures the distance rows alone and joint-box rows alone are feasible, but their intersection is empty.

Matches the declared raw evidence.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\diagnostics\H2_prior\snapshots.json`

### Claim 39: rounding_ok
**Location:** Figure 1 H2_prior; manuscript_input.md:121

Archived failure snapshots over the final 0.5 s.

Twenty-six 20 ms snapshots span 0.5 s; PNG time/z scales and 1 m/s or 1 rad/s normalization agree.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\diagnostics\H2_prior\snapshots.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\figures\f1_historical_constraint_failure.png`

### Claim 40: exact_match
**Location:** Sec. 7.1; manuscript_input.md:117

The archived H2 stop occurs before physical contact.

Terminal pad separation is 8.906 mm and all logged nearby distances are positive. Complete historical contact-count trace is not supplied; this supports the audited terminal interface statement.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\diagnostics\H2_prior\snapshots.json`

### Claim 41: exact_match
**Location:** Sec. 7.1 table S01_noise_delay / classification; manuscript_input.md:113

| S01_noise_delay | LINEAR_HARD_SET_INFEASIBLE | 0.00550255 | gripper_contact_pad__tumbling_target_geom | -0.0372463 | -0.0467254 | 0.00947903 |

Independent SciPy/HiGHS Phase-I solve and analytic joint-box maximization reproduce the displayed standard rounding.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\diagnostics\S01_noise_delay\snapshots.json`

### Claim 42: rounding_ok
**Location:** Sec. 7.1 table S01_noise_delay / z; manuscript_input.md:113

| S01_noise_delay | LINEAR_HARD_SET_INFEASIBLE | 0.00550255 | gripper_contact_pad__tumbling_target_geom | -0.0372463 | -0.0467254 | 0.00947903 |

Independent SciPy/HiGHS Phase-I solve and analytic joint-box maximization reproduce the displayed standard rounding.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\diagnostics\S01_noise_delay\snapshots.json`

### Claim 43: exact_match
**Location:** Sec. 7.1 table S01_noise_delay / row; manuscript_input.md:113

| S01_noise_delay | LINEAR_HARD_SET_INFEASIBLE | 0.00550255 | gripper_contact_pad__tumbling_target_geom | -0.0372463 | -0.0467254 | 0.00947903 |

Independent SciPy/HiGHS Phase-I solve and analytic joint-box maximization reproduce the displayed standard rounding.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\diagnostics\S01_noise_delay\snapshots.json`

### Claim 44: rounding_ok
**Location:** Sec. 7.1 table S01_noise_delay / required; manuscript_input.md:113

| S01_noise_delay | LINEAR_HARD_SET_INFEASIBLE | 0.00550255 | gripper_contact_pad__tumbling_target_geom | -0.0372463 | -0.0467254 | 0.00947903 |

Independent SciPy/HiGHS Phase-I solve and analytic joint-box maximization reproduce the displayed standard rounding.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\diagnostics\S01_noise_delay\snapshots.json`

### Claim 45: rounding_ok
**Location:** Sec. 7.1 table S01_noise_delay / box maximum; manuscript_input.md:113

| S01_noise_delay | LINEAR_HARD_SET_INFEASIBLE | 0.00550255 | gripper_contact_pad__tumbling_target_geom | -0.0372463 | -0.0467254 | 0.00947903 |

Independent SciPy/HiGHS Phase-I solve and analytic joint-box maximization reproduce the displayed standard rounding.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\diagnostics\S01_noise_delay\snapshots.json`

### Claim 46: rounding_ok
**Location:** Sec. 7.1 table S01_noise_delay / shortfall; manuscript_input.md:113

| S01_noise_delay | LINEAR_HARD_SET_INFEASIBLE | 0.00550255 | gripper_contact_pad__tumbling_target_geom | -0.0372463 | -0.0467254 | 0.00947903 |

Independent SciPy/HiGHS Phase-I solve and analytic joint-box maximization reproduce the displayed standard rounding.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\diagnostics\S01_noise_delay\snapshots.json`

### Claim 47: exact_match
**Location:** Sec. 7.1 S01_noise_delay; manuscript_input.md:117

In all three failures the distance rows alone and joint-box rows alone are feasible, but their intersection is empty.

Matches the declared raw evidence.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\diagnostics\S01_noise_delay\snapshots.json`

### Claim 48: rounding_ok
**Location:** Figure 1 S01_noise_delay; manuscript_input.md:121

Archived failure snapshots over the final 0.5 s.

Twenty-six 20 ms snapshots span 0.5 s; PNG time/z scales and 1 m/s or 1 rad/s normalization agree.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\diagnostics\S01_noise_delay\snapshots.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\figures\f1_historical_constraint_failure.png`

### Claim 49: exact_match
**Location:** Sec. 7.1 table D05_final_nominal / classification; manuscript_input.md:114

| D05_final_nominal | LINEAR_HARD_SET_INFEASIBLE | 0.000983 | gripper_contact_pad__tumbling_target_geom | -0.027238 | -0.0288792 | 0.00164122 |

Independent SciPy/HiGHS Phase-I solve and analytic joint-box maximization reproduce the displayed standard rounding.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\diagnostics\D05_final_nominal\snapshots.json`

### Claim 50: rounding_ok
**Location:** Sec. 7.1 table D05_final_nominal / z; manuscript_input.md:114

| D05_final_nominal | LINEAR_HARD_SET_INFEASIBLE | 0.000983 | gripper_contact_pad__tumbling_target_geom | -0.027238 | -0.0288792 | 0.00164122 |

Independent SciPy/HiGHS Phase-I solve and analytic joint-box maximization reproduce the displayed standard rounding.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\diagnostics\D05_final_nominal\snapshots.json`

### Claim 51: exact_match
**Location:** Sec. 7.1 table D05_final_nominal / row; manuscript_input.md:114

| D05_final_nominal | LINEAR_HARD_SET_INFEASIBLE | 0.000983 | gripper_contact_pad__tumbling_target_geom | -0.027238 | -0.0288792 | 0.00164122 |

Independent SciPy/HiGHS Phase-I solve and analytic joint-box maximization reproduce the displayed standard rounding.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\diagnostics\D05_final_nominal\snapshots.json`

### Claim 52: rounding_ok
**Location:** Sec. 7.1 table D05_final_nominal / required; manuscript_input.md:114

| D05_final_nominal | LINEAR_HARD_SET_INFEASIBLE | 0.000983 | gripper_contact_pad__tumbling_target_geom | -0.027238 | -0.0288792 | 0.00164122 |

Independent SciPy/HiGHS Phase-I solve and analytic joint-box maximization reproduce the displayed standard rounding.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\diagnostics\D05_final_nominal\snapshots.json`

### Claim 53: rounding_ok
**Location:** Sec. 7.1 table D05_final_nominal / box maximum; manuscript_input.md:114

| D05_final_nominal | LINEAR_HARD_SET_INFEASIBLE | 0.000983 | gripper_contact_pad__tumbling_target_geom | -0.027238 | -0.0288792 | 0.00164122 |

Independent SciPy/HiGHS Phase-I solve and analytic joint-box maximization reproduce the displayed standard rounding.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\diagnostics\D05_final_nominal\snapshots.json`

### Claim 54: rounding_ok
**Location:** Sec. 7.1 table D05_final_nominal / shortfall; manuscript_input.md:114

| D05_final_nominal | LINEAR_HARD_SET_INFEASIBLE | 0.000983 | gripper_contact_pad__tumbling_target_geom | -0.027238 | -0.0288792 | 0.00164122 |

Independent SciPy/HiGHS Phase-I solve and analytic joint-box maximization reproduce the displayed standard rounding.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\diagnostics\D05_final_nominal\snapshots.json`

### Claim 55: exact_match
**Location:** Sec. 7.1 D05_final_nominal; manuscript_input.md:117

In all three failures the distance rows alone and joint-box rows alone are feasible, but their intersection is empty.

Matches the declared raw evidence.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\diagnostics\D05_final_nominal\snapshots.json`

### Claim 56: rounding_ok
**Location:** Figure 1 D05_final_nominal; manuscript_input.md:121

Archived failure snapshots over the final 0.5 s.

Twenty-six 20 ms snapshots span 0.5 s; PNG time/z scales and 1 m/s or 1 rad/s normalization agree.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\diagnostics\D05_final_nominal\snapshots.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\figures\f1_historical_constraint_failure.png`

### Claim 57: rounding_ok
**Location:** Abstract; manuscript_input.md:7

linear-velocity component RMS errors decrease from approximately 2.08–2.17 to 0.218–0.228 mm/s.

RMS recomputed on exactly 3001 aligned 1-7 s samples. Component extrema use standard rounding; one historical window is not presented as a seed average.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n208_adaptive_capture\runs\S01_noise_delay\trace.npz`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\diagnostics\S01_noise_delay\new_estimator_redecision.json`

### Claim 58: exact_match
**Location:** Sec. 7.2; manuscript_input.md:125

The offline historical S01 window is fixed at absolute time 1–7 s.

Numerical window agrees. Prespecification cannot be verified from outputs alone.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n208_adaptive_capture\runs\S01_noise_delay\trace.npz`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\diagnostics\S01_noise_delay\new_estimator_redecision.json`

### Claim 59: exact_match
**Location:** Sec. 7.2; manuscript_input.md:125

New linear-velocity component RMS values are about one tenth of the archived values; angular components also decrease.

Ratios are 0.10446-0.10505. Every angular component decreases. Wording correctly combines interface/process changes without claiming ablation causality.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n208_adaptive_capture\runs\S01_noise_delay\trace.npz`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\diagnostics\S01_noise_delay\new_estimator_redecision.json`

### Claim 60: ambiguous_mapping
**Location:** Figure 2; manuscript_input.md:129

Old and revised estimator velocity error norms on the same archived packets, without a new plant rollout.

PNG norms/units and 1-7 s alignment agree, but raw packet identities and reanalysis source are not declared; identical packets/no new rollout cannot be certified from output errors alone.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n208_adaptive_capture\runs\S01_noise_delay\trace.npz`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\diagnostics\S01_noise_delay\new_estimator_redecision.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\figures\f2_same_packet_estimation.png`

### Claim 61: ambiguous_mapping
**Location:** Sec. 7.2 table D03_V1_S01 / actual gate ticks; manuscript_input.md:133

| D03_V1_S01 | 0 | 3001 | 3001 | 0.0016097 | 0.00964336 |

Phase codes and zero-filled guard arrays support inactive capture, but do not directly encode a call count. Phase enum/source/event stream is absent. Counterfactual covariance tests are not actual rejected latch requests.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D03_V1_S01\trace.npz`

### Claim 62: exact_match
**Location:** Sec. 7.2 table D03_V1_S01 / sample count; manuscript_input.md:133

| D03_V1_S01 | 0 | 3001 | 3001 | 0.0016097 | 0.00964336 |

Computed from 3 sqrt(max eigenvalue) of each velocity covariance block on inclusive 1-7 s data; the linear lower bound omits the nonnegative angular-offset contribution.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D03_V1_S01\trace.npz`

### Claim 63: exact_match
**Location:** Sec. 7.2 table D03_V1_S01 / uncertainty precludes count; manuscript_input.md:133

| D03_V1_S01 | 0 | 3001 | 3001 | 0.0016097 | 0.00964336 |

Computed from 3 sqrt(max eigenvalue) of each velocity covariance block on inclusive 1-7 s data; the linear lower bound omits the nonnegative angular-offset contribution.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D03_V1_S01\trace.npz`

### Claim 64: rounding_ok
**Location:** Sec. 7.2 table D03_V1_S01 / linear minimum m/s; manuscript_input.md:133

| D03_V1_S01 | 0 | 3001 | 3001 | 0.0016097 | 0.00964336 |

Computed from 3 sqrt(max eigenvalue) of each velocity covariance block on inclusive 1-7 s data; the linear lower bound omits the nonnegative angular-offset contribution.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D03_V1_S01\trace.npz`

### Claim 65: rounding_ok
**Location:** Sec. 7.2 table D03_V1_S01 / angular minimum rad/s; manuscript_input.md:133

| D03_V1_S01 | 0 | 3001 | 3001 | 0.0016097 | 0.00964336 |

Computed from 3 sqrt(max eigenvalue) of each velocity covariance block on inclusive 1-7 s data; the linear lower bound omits the nonnegative angular-offset contribution.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D03_V1_S01\trace.npz`

### Claim 66: ambiguous_mapping
**Location:** Sec. 7.2 table R03_V2_S01 / actual gate ticks; manuscript_input.md:134

| R03_V2_S01 | 0 | 3001 | 3001 | 0.0016097 | 0.00964336 |

Phase codes and zero-filled guard arrays support inactive capture, but do not directly encode a call count. Phase enum/source/event stream is absent. Counterfactual covariance tests are not actual rejected latch requests.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R03_V2_S01\trace.npz`

### Claim 67: exact_match
**Location:** Sec. 7.2 table R03_V2_S01 / sample count; manuscript_input.md:134

| R03_V2_S01 | 0 | 3001 | 3001 | 0.0016097 | 0.00964336 |

Computed from 3 sqrt(max eigenvalue) of each velocity covariance block on inclusive 1-7 s data; the linear lower bound omits the nonnegative angular-offset contribution.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R03_V2_S01\trace.npz`

### Claim 68: exact_match
**Location:** Sec. 7.2 table R03_V2_S01 / uncertainty precludes count; manuscript_input.md:134

| R03_V2_S01 | 0 | 3001 | 3001 | 0.0016097 | 0.00964336 |

Computed from 3 sqrt(max eigenvalue) of each velocity covariance block on inclusive 1-7 s data; the linear lower bound omits the nonnegative angular-offset contribution.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R03_V2_S01\trace.npz`

### Claim 69: rounding_ok
**Location:** Sec. 7.2 table R03_V2_S01 / linear minimum m/s; manuscript_input.md:134

| R03_V2_S01 | 0 | 3001 | 3001 | 0.0016097 | 0.00964336 |

Computed from 3 sqrt(max eigenvalue) of each velocity covariance block on inclusive 1-7 s data; the linear lower bound omits the nonnegative angular-offset contribution.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R03_V2_S01\trace.npz`

### Claim 70: rounding_ok
**Location:** Sec. 7.2 table R03_V2_S01 / angular minimum rad/s; manuscript_input.md:134

| R03_V2_S01 | 0 | 3001 | 3001 | 0.0016097 | 0.00964336 |

Computed from 3 sqrt(max eigenvalue) of each velocity covariance block on inclusive 1-7 s data; the linear lower bound omits the nonnegative angular-offset contribution.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R03_V2_S01\trace.npz`

### Claim 71: exact_match
**Location:** Sec. 7.2 scope; manuscript_input.md:137

This is evidence of an implemented filter/gate incompatibility over the observed approach window, not an actual rejected latch request or a general impossibility theorem for the sensor or task.

Margins alone exceed gate thresholds at every audited sample, even if error were zero. Scope language avoids a general sensor impossibility or hardware-safe-stop claim.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D03_V1_S01\trace.npz`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D03_V1_S01\config.json`

### Claim 72: exact_match
**Location:** Sec. 7.3 table D01_V1_nominal / status; manuscript_input.md:143

| D01_V1_nominal | development | COMPLETED | 27.992 | 7.992 | 0.0235542 | 0.363223 | 0 | PENDING_OR_FAIL |

Matches raw metrics and directly checked trace at standard displayed precision.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\trace.npz`

### Claim 73: rounding_ok
**Location:** Sec. 7.3 table D01_V1_nominal / end_s; manuscript_input.md:143

| D01_V1_nominal | development | COMPLETED | 27.992 | 7.992 | 0.0235542 | 0.363223 | 0 | PENDING_OR_FAIL |

Matches raw metrics and directly checked trace at standard displayed precision.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\trace.npz`

### Claim 74: rounding_ok
**Location:** Sec. 7.3 table D01_V1_nominal / latch_s; manuscript_input.md:143

| D01_V1_nominal | development | COMPLETED | 27.992 | 7.992 | 0.0235542 | 0.363223 | 0 | PENDING_OR_FAIL |

Matches raw metrics and directly checked trace at standard displayed precision.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\trace.npz`

### Claim 75: rounding_ok
**Location:** Sec. 7.3 table D01_V1_nominal / world last2s deg/s; manuscript_input.md:143

| D01_V1_nominal | development | COMPLETED | 27.992 | 7.992 | 0.0235542 | 0.363223 | 0 | PENDING_OR_FAIL |

Direct maximum of the world angular-speed norm over the prescribed final two-second window matches metric and standard rounding.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\trace.npz`

### Claim 76: rounding_ok
**Location:** Sec. 7.3 table D01_V1_nominal / peak rho; manuscript_input.md:143

| D01_V1_nominal | development | COMPLETED | 27.992 | 7.992 | 0.0235542 | 0.363223 | 0 | PENDING_OR_FAIL |

Matches raw metrics and directly checked trace at standard displayed precision.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\trace.npz`

### Claim 77: exact_match
**Location:** Sec. 7.3 table D01_V1_nominal / recorded safety violations; manuscript_input.md:143

| D01_V1_nominal | development | COMPLETED | 27.992 | 7.992 | 0.0235542 | 0.363223 | 0 | PENDING_OR_FAIL |

The raw recorded actual_safety_violations array is empty. This verifies reporting of recorded violations; completeness/correctness of the original guard requires safety config/source not supplied here.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\trace.npz`

### Claim 78: missing_evidence
**Location:** Sec. 7.3 table D01_V1_nominal / dual replay; manuscript_input.md:143

| D01_V1_nominal | development | COMPLETED | 27.992 | 7.992 | 0.0235542 | 0.363223 | 0 | PENDING_OR_FAIL |

No validation.json or replay result is declared for this attempt. PENDING_OR_FAIL is not a PASS claim, but pending/fail mapping cannot be audited.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\config.json`

### Claim 79: exact_match
**Location:** Sec. 7.3 table D02_V1_H2 / status; manuscript_input.md:144

| D02_V1_H2 | development | NO_VERIFIED_CONTROL | 0.04 | NOT_EVALUATED | NOT_EVALUATED | 0 | 0 | PENDING_OR_FAIL |

Matches raw metrics and directly checked trace at standard displayed precision.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D02_V1_H2\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D02_V1_H2\trace.npz`

### Claim 80: rounding_ok
**Location:** Sec. 7.3 table D02_V1_H2 / end_s; manuscript_input.md:144

| D02_V1_H2 | development | NO_VERIFIED_CONTROL | 0.04 | NOT_EVALUATED | NOT_EVALUATED | 0 | 0 | PENDING_OR_FAIL |

Matches raw metrics and directly checked trace at standard displayed precision.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D02_V1_H2\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D02_V1_H2\trace.npz`

### Claim 81: exact_match
**Location:** Sec. 7.3 table D02_V1_H2 / latch_s; manuscript_input.md:144

| D02_V1_H2 | development | NO_VERIFIED_CONTROL | 0.04 | NOT_EVALUATED | NOT_EVALUATED | 0 | 0 | PENDING_OR_FAIL |

NOT_EVALUATED correctly denotes no latch or no complete prescribed performance window, rather than zero error.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D02_V1_H2\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D02_V1_H2\trace.npz`

### Claim 82: exact_match
**Location:** Sec. 7.3 table D02_V1_H2 / world last2s deg/s; manuscript_input.md:144

| D02_V1_H2 | development | NO_VERIFIED_CONTROL | 0.04 | NOT_EVALUATED | NOT_EVALUATED | 0 | 0 | PENDING_OR_FAIL |

NOT_EVALUATED correctly denotes no latch or no complete prescribed performance window, rather than zero error.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D02_V1_H2\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D02_V1_H2\trace.npz`

### Claim 83: exact_match
**Location:** Sec. 7.3 table D02_V1_H2 / peak rho; manuscript_input.md:144

| D02_V1_H2 | development | NO_VERIFIED_CONTROL | 0.04 | NOT_EVALUATED | NOT_EVALUATED | 0 | 0 | PENDING_OR_FAIL |

Matches raw metrics and directly checked trace at standard displayed precision.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D02_V1_H2\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D02_V1_H2\trace.npz`

### Claim 84: exact_match
**Location:** Sec. 7.3 table D02_V1_H2 / recorded safety violations; manuscript_input.md:144

| D02_V1_H2 | development | NO_VERIFIED_CONTROL | 0.04 | NOT_EVALUATED | NOT_EVALUATED | 0 | 0 | PENDING_OR_FAIL |

The raw recorded actual_safety_violations array is empty. This verifies reporting of recorded violations; completeness/correctness of the original guard requires safety config/source not supplied here.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D02_V1_H2\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D02_V1_H2\trace.npz`

### Claim 85: missing_evidence
**Location:** Sec. 7.3 table D02_V1_H2 / dual replay; manuscript_input.md:144

| D02_V1_H2 | development | NO_VERIFIED_CONTROL | 0.04 | NOT_EVALUATED | NOT_EVALUATED | 0 | 0 | PENDING_OR_FAIL |

No validation.json or replay result is declared for this attempt. PENDING_OR_FAIL is not a PASS claim, but pending/fail mapping cannot be audited.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D02_V1_H2\config.json`

### Claim 86: exact_match
**Location:** Sec. 7.3 table D03_V1_S01 / status; manuscript_input.md:145

| D03_V1_S01 | development | NO_VERIFIED_CONTROL | 7.84 | NOT_EVALUATED | NOT_EVALUATED | 0.0103905 | 0 | PENDING_OR_FAIL |

Matches raw metrics and directly checked trace at standard displayed precision.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D03_V1_S01\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D03_V1_S01\trace.npz`

### Claim 87: rounding_ok
**Location:** Sec. 7.3 table D03_V1_S01 / end_s; manuscript_input.md:145

| D03_V1_S01 | development | NO_VERIFIED_CONTROL | 7.84 | NOT_EVALUATED | NOT_EVALUATED | 0.0103905 | 0 | PENDING_OR_FAIL |

Matches raw metrics and directly checked trace at standard displayed precision.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D03_V1_S01\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D03_V1_S01\trace.npz`

### Claim 88: exact_match
**Location:** Sec. 7.3 table D03_V1_S01 / latch_s; manuscript_input.md:145

| D03_V1_S01 | development | NO_VERIFIED_CONTROL | 7.84 | NOT_EVALUATED | NOT_EVALUATED | 0.0103905 | 0 | PENDING_OR_FAIL |

NOT_EVALUATED correctly denotes no latch or no complete prescribed performance window, rather than zero error.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D03_V1_S01\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D03_V1_S01\trace.npz`

### Claim 89: exact_match
**Location:** Sec. 7.3 table D03_V1_S01 / world last2s deg/s; manuscript_input.md:145

| D03_V1_S01 | development | NO_VERIFIED_CONTROL | 7.84 | NOT_EVALUATED | NOT_EVALUATED | 0.0103905 | 0 | PENDING_OR_FAIL |

NOT_EVALUATED correctly denotes no latch or no complete prescribed performance window, rather than zero error.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D03_V1_S01\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D03_V1_S01\trace.npz`

### Claim 90: rounding_ok
**Location:** Sec. 7.3 table D03_V1_S01 / peak rho; manuscript_input.md:145

| D03_V1_S01 | development | NO_VERIFIED_CONTROL | 7.84 | NOT_EVALUATED | NOT_EVALUATED | 0.0103905 | 0 | PENDING_OR_FAIL |

Matches raw metrics and directly checked trace at standard displayed precision.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D03_V1_S01\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D03_V1_S01\trace.npz`

### Claim 91: exact_match
**Location:** Sec. 7.3 table D03_V1_S01 / recorded safety violations; manuscript_input.md:145

| D03_V1_S01 | development | NO_VERIFIED_CONTROL | 7.84 | NOT_EVALUATED | NOT_EVALUATED | 0.0103905 | 0 | PENDING_OR_FAIL |

The raw recorded actual_safety_violations array is empty. This verifies reporting of recorded violations; completeness/correctness of the original guard requires safety config/source not supplied here.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D03_V1_S01\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D03_V1_S01\trace.npz`

### Claim 92: missing_evidence
**Location:** Sec. 7.3 table D03_V1_S01 / dual replay; manuscript_input.md:145

| D03_V1_S01 | development | NO_VERIFIED_CONTROL | 7.84 | NOT_EVALUATED | NOT_EVALUATED | 0.0103905 | 0 | PENDING_OR_FAIL |

No validation.json or replay result is declared for this attempt. PENDING_OR_FAIL is not a PASS claim, but pending/fail mapping cannot be audited.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D03_V1_S01\config.json`

### Claim 93: exact_match
**Location:** Sec. 7.3 table D04_V2_H2 / status; manuscript_input.md:146

| D04_V2_H2 | development | NO_VERIFIED_CONTROL | 5.9 | NOT_EVALUATED | NOT_EVALUATED | 0 | 0 | PENDING_OR_FAIL |

Matches raw metrics and directly checked trace at standard displayed precision.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D04_V2_H2\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D04_V2_H2\trace.npz`

### Claim 94: rounding_ok
**Location:** Sec. 7.3 table D04_V2_H2 / end_s; manuscript_input.md:146

| D04_V2_H2 | development | NO_VERIFIED_CONTROL | 5.9 | NOT_EVALUATED | NOT_EVALUATED | 0 | 0 | PENDING_OR_FAIL |

Matches raw metrics and directly checked trace at standard displayed precision.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D04_V2_H2\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D04_V2_H2\trace.npz`

### Claim 95: exact_match
**Location:** Sec. 7.3 table D04_V2_H2 / latch_s; manuscript_input.md:146

| D04_V2_H2 | development | NO_VERIFIED_CONTROL | 5.9 | NOT_EVALUATED | NOT_EVALUATED | 0 | 0 | PENDING_OR_FAIL |

NOT_EVALUATED correctly denotes no latch or no complete prescribed performance window, rather than zero error.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D04_V2_H2\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D04_V2_H2\trace.npz`

### Claim 96: exact_match
**Location:** Sec. 7.3 table D04_V2_H2 / world last2s deg/s; manuscript_input.md:146

| D04_V2_H2 | development | NO_VERIFIED_CONTROL | 5.9 | NOT_EVALUATED | NOT_EVALUATED | 0 | 0 | PENDING_OR_FAIL |

NOT_EVALUATED correctly denotes no latch or no complete prescribed performance window, rather than zero error.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D04_V2_H2\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D04_V2_H2\trace.npz`

### Claim 97: exact_match
**Location:** Sec. 7.3 table D04_V2_H2 / peak rho; manuscript_input.md:146

| D04_V2_H2 | development | NO_VERIFIED_CONTROL | 5.9 | NOT_EVALUATED | NOT_EVALUATED | 0 | 0 | PENDING_OR_FAIL |

Matches raw metrics and directly checked trace at standard displayed precision.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D04_V2_H2\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D04_V2_H2\trace.npz`

### Claim 98: exact_match
**Location:** Sec. 7.3 table D04_V2_H2 / recorded safety violations; manuscript_input.md:146

| D04_V2_H2 | development | NO_VERIFIED_CONTROL | 5.9 | NOT_EVALUATED | NOT_EVALUATED | 0 | 0 | PENDING_OR_FAIL |

The raw recorded actual_safety_violations array is empty. This verifies reporting of recorded violations; completeness/correctness of the original guard requires safety config/source not supplied here.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D04_V2_H2\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D04_V2_H2\trace.npz`

### Claim 99: missing_evidence
**Location:** Sec. 7.3 table D04_V2_H2 / dual replay; manuscript_input.md:146

| D04_V2_H2 | development | NO_VERIFIED_CONTROL | 5.9 | NOT_EVALUATED | NOT_EVALUATED | 0 | 0 | PENDING_OR_FAIL |

No validation.json or replay result is declared for this attempt. PENDING_OR_FAIL is not a PASS claim, but pending/fail mapping cannot be audited.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D04_V2_H2\config.json`

### Claim 100: exact_match
**Location:** Sec. 7.3 table R01_V2_nominal / status; manuscript_input.md:147

| R01_V2_nominal | regression | COMPLETED | 27.992 | 7.992 | 0.0235542 | 0.363223 | 0 | PENDING_OR_FAIL |

Matches raw metrics and directly checked trace at standard displayed precision.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R01_V2_nominal\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R01_V2_nominal\trace.npz`

### Claim 101: rounding_ok
**Location:** Sec. 7.3 table R01_V2_nominal / end_s; manuscript_input.md:147

| R01_V2_nominal | regression | COMPLETED | 27.992 | 7.992 | 0.0235542 | 0.363223 | 0 | PENDING_OR_FAIL |

Matches raw metrics and directly checked trace at standard displayed precision.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R01_V2_nominal\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R01_V2_nominal\trace.npz`

### Claim 102: rounding_ok
**Location:** Sec. 7.3 table R01_V2_nominal / latch_s; manuscript_input.md:147

| R01_V2_nominal | regression | COMPLETED | 27.992 | 7.992 | 0.0235542 | 0.363223 | 0 | PENDING_OR_FAIL |

Matches raw metrics and directly checked trace at standard displayed precision.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R01_V2_nominal\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R01_V2_nominal\trace.npz`

### Claim 103: rounding_ok
**Location:** Sec. 7.3 table R01_V2_nominal / world last2s deg/s; manuscript_input.md:147

| R01_V2_nominal | regression | COMPLETED | 27.992 | 7.992 | 0.0235542 | 0.363223 | 0 | PENDING_OR_FAIL |

Direct maximum of the world angular-speed norm over the prescribed final two-second window matches metric and standard rounding.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R01_V2_nominal\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R01_V2_nominal\trace.npz`

### Claim 104: rounding_ok
**Location:** Sec. 7.3 table R01_V2_nominal / peak rho; manuscript_input.md:147

| R01_V2_nominal | regression | COMPLETED | 27.992 | 7.992 | 0.0235542 | 0.363223 | 0 | PENDING_OR_FAIL |

Matches raw metrics and directly checked trace at standard displayed precision.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R01_V2_nominal\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R01_V2_nominal\trace.npz`

### Claim 105: exact_match
**Location:** Sec. 7.3 table R01_V2_nominal / recorded safety violations; manuscript_input.md:147

| R01_V2_nominal | regression | COMPLETED | 27.992 | 7.992 | 0.0235542 | 0.363223 | 0 | PENDING_OR_FAIL |

The raw recorded actual_safety_violations array is empty. This verifies reporting of recorded violations; completeness/correctness of the original guard requires safety config/source not supplied here.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R01_V2_nominal\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R01_V2_nominal\trace.npz`

### Claim 106: missing_evidence
**Location:** Sec. 7.3 table R01_V2_nominal / dual replay; manuscript_input.md:147

| R01_V2_nominal | regression | COMPLETED | 27.992 | 7.992 | 0.0235542 | 0.363223 | 0 | PENDING_OR_FAIL |

No validation.json or replay result is declared for this attempt. PENDING_OR_FAIL is not a PASS claim, but pending/fail mapping cannot be audited.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R01_V2_nominal\config.json`

### Claim 107: exact_match
**Location:** Sec. 7.3 table R02_V2_H2 / status; manuscript_input.md:148

| R02_V2_H2 | regression | NO_VERIFIED_CONTROL | 5.9 | NOT_EVALUATED | NOT_EVALUATED | 0 | 0 | PENDING_OR_FAIL |

Matches raw metrics and directly checked trace at standard displayed precision.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R02_V2_H2\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R02_V2_H2\trace.npz`

### Claim 108: rounding_ok
**Location:** Sec. 7.3 table R02_V2_H2 / end_s; manuscript_input.md:148

| R02_V2_H2 | regression | NO_VERIFIED_CONTROL | 5.9 | NOT_EVALUATED | NOT_EVALUATED | 0 | 0 | PENDING_OR_FAIL |

Matches raw metrics and directly checked trace at standard displayed precision.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R02_V2_H2\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R02_V2_H2\trace.npz`

### Claim 109: exact_match
**Location:** Sec. 7.3 table R02_V2_H2 / latch_s; manuscript_input.md:148

| R02_V2_H2 | regression | NO_VERIFIED_CONTROL | 5.9 | NOT_EVALUATED | NOT_EVALUATED | 0 | 0 | PENDING_OR_FAIL |

NOT_EVALUATED correctly denotes no latch or no complete prescribed performance window, rather than zero error.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R02_V2_H2\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R02_V2_H2\trace.npz`

### Claim 110: exact_match
**Location:** Sec. 7.3 table R02_V2_H2 / world last2s deg/s; manuscript_input.md:148

| R02_V2_H2 | regression | NO_VERIFIED_CONTROL | 5.9 | NOT_EVALUATED | NOT_EVALUATED | 0 | 0 | PENDING_OR_FAIL |

NOT_EVALUATED correctly denotes no latch or no complete prescribed performance window, rather than zero error.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R02_V2_H2\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R02_V2_H2\trace.npz`

### Claim 111: exact_match
**Location:** Sec. 7.3 table R02_V2_H2 / peak rho; manuscript_input.md:148

| R02_V2_H2 | regression | NO_VERIFIED_CONTROL | 5.9 | NOT_EVALUATED | NOT_EVALUATED | 0 | 0 | PENDING_OR_FAIL |

Matches raw metrics and directly checked trace at standard displayed precision.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R02_V2_H2\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R02_V2_H2\trace.npz`

### Claim 112: exact_match
**Location:** Sec. 7.3 table R02_V2_H2 / recorded safety violations; manuscript_input.md:148

| R02_V2_H2 | regression | NO_VERIFIED_CONTROL | 5.9 | NOT_EVALUATED | NOT_EVALUATED | 0 | 0 | PENDING_OR_FAIL |

The raw recorded actual_safety_violations array is empty. This verifies reporting of recorded violations; completeness/correctness of the original guard requires safety config/source not supplied here.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R02_V2_H2\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R02_V2_H2\trace.npz`

### Claim 113: missing_evidence
**Location:** Sec. 7.3 table R02_V2_H2 / dual replay; manuscript_input.md:148

| R02_V2_H2 | regression | NO_VERIFIED_CONTROL | 5.9 | NOT_EVALUATED | NOT_EVALUATED | 0 | 0 | PENDING_OR_FAIL |

No validation.json or replay result is declared for this attempt. PENDING_OR_FAIL is not a PASS claim, but pending/fail mapping cannot be audited.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R02_V2_H2\config.json`

### Claim 114: exact_match
**Location:** Sec. 7.3 table R03_V2_S01 / status; manuscript_input.md:149

| R03_V2_S01 | regression | NO_VERIFIED_CONTROL | 7.84 | NOT_EVALUATED | NOT_EVALUATED | 0.0103905 | 0 | PENDING_OR_FAIL |

Matches raw metrics and directly checked trace at standard displayed precision.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R03_V2_S01\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R03_V2_S01\trace.npz`

### Claim 115: rounding_ok
**Location:** Sec. 7.3 table R03_V2_S01 / end_s; manuscript_input.md:149

| R03_V2_S01 | regression | NO_VERIFIED_CONTROL | 7.84 | NOT_EVALUATED | NOT_EVALUATED | 0.0103905 | 0 | PENDING_OR_FAIL |

Matches raw metrics and directly checked trace at standard displayed precision.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R03_V2_S01\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R03_V2_S01\trace.npz`

### Claim 116: exact_match
**Location:** Sec. 7.3 table R03_V2_S01 / latch_s; manuscript_input.md:149

| R03_V2_S01 | regression | NO_VERIFIED_CONTROL | 7.84 | NOT_EVALUATED | NOT_EVALUATED | 0.0103905 | 0 | PENDING_OR_FAIL |

NOT_EVALUATED correctly denotes no latch or no complete prescribed performance window, rather than zero error.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R03_V2_S01\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R03_V2_S01\trace.npz`

### Claim 117: exact_match
**Location:** Sec. 7.3 table R03_V2_S01 / world last2s deg/s; manuscript_input.md:149

| R03_V2_S01 | regression | NO_VERIFIED_CONTROL | 7.84 | NOT_EVALUATED | NOT_EVALUATED | 0.0103905 | 0 | PENDING_OR_FAIL |

NOT_EVALUATED correctly denotes no latch or no complete prescribed performance window, rather than zero error.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R03_V2_S01\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R03_V2_S01\trace.npz`

### Claim 118: rounding_ok
**Location:** Sec. 7.3 table R03_V2_S01 / peak rho; manuscript_input.md:149

| R03_V2_S01 | regression | NO_VERIFIED_CONTROL | 7.84 | NOT_EVALUATED | NOT_EVALUATED | 0.0103905 | 0 | PENDING_OR_FAIL |

Matches raw metrics and directly checked trace at standard displayed precision.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R03_V2_S01\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R03_V2_S01\trace.npz`

### Claim 119: exact_match
**Location:** Sec. 7.3 table R03_V2_S01 / recorded safety violations; manuscript_input.md:149

| R03_V2_S01 | regression | NO_VERIFIED_CONTROL | 7.84 | NOT_EVALUATED | NOT_EVALUATED | 0.0103905 | 0 | PENDING_OR_FAIL |

The raw recorded actual_safety_violations array is empty. This verifies reporting of recorded violations; completeness/correctness of the original guard requires safety config/source not supplied here.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R03_V2_S01\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R03_V2_S01\trace.npz`

### Claim 120: missing_evidence
**Location:** Sec. 7.3 table R03_V2_S01 / dual replay; manuscript_input.md:149

| R03_V2_S01 | regression | NO_VERIFIED_CONTROL | 7.84 | NOT_EVALUATED | NOT_EVALUATED | 0.0103905 | 0 | PENDING_OR_FAIL |

No validation.json or replay result is declared for this attempt. PENDING_OR_FAIL is not a PASS claim, but pending/fail mapping cannot be audited.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R03_V2_S01\config.json`

### Claim 121: exact_match
**Location:** Sec. 7.3 table R04_V2_H1 / status; manuscript_input.md:150

| R04_V2_H1 | regression | COMPLETED | 27.932 | 7.932 | 0.0128079 | 0.597206 | 0 | PENDING_OR_FAIL |

Matches raw metrics and directly checked trace at standard displayed precision.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R04_V2_H1\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R04_V2_H1\trace.npz`

### Claim 122: rounding_ok
**Location:** Sec. 7.3 table R04_V2_H1 / end_s; manuscript_input.md:150

| R04_V2_H1 | regression | COMPLETED | 27.932 | 7.932 | 0.0128079 | 0.597206 | 0 | PENDING_OR_FAIL |

Matches raw metrics and directly checked trace at standard displayed precision.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R04_V2_H1\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R04_V2_H1\trace.npz`

### Claim 123: rounding_ok
**Location:** Sec. 7.3 table R04_V2_H1 / latch_s; manuscript_input.md:150

| R04_V2_H1 | regression | COMPLETED | 27.932 | 7.932 | 0.0128079 | 0.597206 | 0 | PENDING_OR_FAIL |

Matches raw metrics and directly checked trace at standard displayed precision.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R04_V2_H1\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R04_V2_H1\trace.npz`

### Claim 124: rounding_ok
**Location:** Sec. 7.3 table R04_V2_H1 / world last2s deg/s; manuscript_input.md:150

| R04_V2_H1 | regression | COMPLETED | 27.932 | 7.932 | 0.0128079 | 0.597206 | 0 | PENDING_OR_FAIL |

Direct maximum of the world angular-speed norm over the prescribed final two-second window matches metric and standard rounding.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R04_V2_H1\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R04_V2_H1\trace.npz`

### Claim 125: rounding_ok
**Location:** Sec. 7.3 table R04_V2_H1 / peak rho; manuscript_input.md:150

| R04_V2_H1 | regression | COMPLETED | 27.932 | 7.932 | 0.0128079 | 0.597206 | 0 | PENDING_OR_FAIL |

Matches raw metrics and directly checked trace at standard displayed precision.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R04_V2_H1\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R04_V2_H1\trace.npz`

### Claim 126: exact_match
**Location:** Sec. 7.3 table R04_V2_H1 / recorded safety violations; manuscript_input.md:150

| R04_V2_H1 | regression | COMPLETED | 27.932 | 7.932 | 0.0128079 | 0.597206 | 0 | PENDING_OR_FAIL |

The raw recorded actual_safety_violations array is empty. This verifies reporting of recorded violations; completeness/correctness of the original guard requires safety config/source not supplied here.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R04_V2_H1\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R04_V2_H1\trace.npz`

### Claim 127: missing_evidence
**Location:** Sec. 7.3 table R04_V2_H1 / dual replay; manuscript_input.md:150

| R04_V2_H1 | regression | COMPLETED | 27.932 | 7.932 | 0.0128079 | 0.597206 | 0 | PENDING_OR_FAIL |

No validation.json or replay result is declared for this attempt. PENDING_OR_FAIL is not a PASS claim, but pending/fail mapping cannot be audited.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R04_V2_H1\config.json`

### Claim 128: rounding_ok
**Location:** Abstract / Sec. 7.3; manuscript_input.md:7

A nominal continuous run captures at 7.992 s and completes the prescribed 20 s post-capture interval; its final two-second maximum world angular speed is 0.023554 deg/s.

eq_active transition and trace end reproduce the stated values. A 4e-12 s floating-point deficit is representation of 10000 post-lock 2 ms steps, not an incomplete window.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\trace.npz`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\config.json`

### Claim 129: missing_evidence
**Location:** Figure 3 prespecification; manuscript_input.md:157

First nominal development run, chosen before the remaining outcomes.

D01 is identified by the raw run path and config; timing of the illustration choice relative to other outcomes cannot be reconstructed from allowed results.

**Evidence files:**
- None in the declared input set.

### Claim 130: exact_match
**Location:** Figure 3 quantities; manuscript_input.md:157

World/relative angular speeds, actual load utilization, and separate linear/angular momentum drift use absolute time.

PNG axes, units, scales, latch marker and shaded final interval were visually checked against raw quantities. Pixelwise figure regeneration was not performed.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\trace.npz`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\figures\f3_nominal_continuous_task.png`

### Claim 131: exact_match
**Location:** Sec. 7.3 momentum; manuscript_input.md:153

that residual is consistent with an internally actuated assembly carrying nonzero angular momentum.

Nonzero H about 0.02618 kg m2/s supports a consistency observation, not a claim of optimality or sole causal explanation.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\trace.npz`

### Claim 132: rounding_ok
**Location:** Sec. 7.3 D02_V1_H2; manuscript_input.md:159

The first H2 candidate stops at 0.04 s under a soft startup residual check.

Times/rejection reasons match. Physical initial states and configs match between H2 versions apart from the declared startup allowance; code-exclusive change claims require source.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D02_V1_H2\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D02_V1_H2\progress_governor.json`

### Claim 133: rounding_ok
**Location:** Sec. 7.3 D04_V2_H2; manuscript_input.md:159

The second version removes only that initial transient rejection and stops later, at 5.90 s.

Times/rejection reasons match. Physical initial states and configs match between H2 versions apart from the declared startup allowance; code-exclusive change claims require source.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D04_V2_H2\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D04_V2_H2\progress_governor.json`

### Claim 134: exact_match
**Location:** Sec. 7.3; manuscript_input.md:159

All five demands are then rejected by the predictive task-residual criterion

Matches the declared raw evidence.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D04_V2_H2\progress_governor.json`

### Claim 135: rounding_ok
**Location:** Sec. 7.3; manuscript_input.md:159

The noisy S01 first version reaches actual contact near 7.822 s but stops at 7.84 s.

Matches the declared raw evidence.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D03_V1_S01\trace.npz`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D03_V1_S01\metrics.json`

### Claim 136: ambiguous_mapping
**Location:** Figure 4; manuscript_input.md:163

Every ledger attempt, with category, stop time and outcome.

PNG shows every supplied new run with matching outcomes/times. Failed full windows are NE; observed safety-violation cells are No. The complete ledger is not declared, so exhaustive actual-ledger coverage cannot be certified.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\figures\f4_all_attempt_outcomes.png`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D02_V1_H2\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D03_V1_S01\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D04_V2_H2\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R01_V2_nominal\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R02_V2_H2\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R03_V2_S01\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R04_V2_H1\metrics.json`

### Claim 137: exact_match
**Location:** Sec. 7.4 timing; manuscript_input.md:167

Per-call timing substantially exceeds the 20 ms task deadline, with multi-second candidate evaluation at rejection.

Antecedent is predictor calls: all median predictor latencies exceed 20 ms. Later H2/S01 rejection calls are 4.18-4.50 s; D02 initial rejection is 0.063 s, so the multi-second observation is not universal. Ordinary controller callbacks have lower median latency.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\progress_governor.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D02_V1_H2\progress_governor.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D03_V1_S01\progress_governor.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D04_V2_H2\progress_governor.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R01_V2_nominal\progress_governor.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R02_V2_H2\progress_governor.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R03_V2_S01\progress_governor.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R04_V2_H1\progress_governor.json`

### Claim 138: exact_match
**Location:** Sec. 7.4 timing scope; manuscript_input.md:167

Early development controller timing omitted the terminal exception call; predictor timing retains that omission's diagnostic context, and final regressions measure all calls externally.

D02 control max excludes the longer exception call; final raw metrics explicitly declare every-call runner wall timing. Full instrumentation correctness is not source-audited.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D02_V1_H2\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R02_V2_H2\metrics.json`

### Claim 139: exact_match
**Location:** Sec. 4.3 / Sec. 7.4; manuscript_input.md:69

The inertial estimator remains in shadow mode; its posterior cannot change the approach or damping actions in this campaign.

All eight configs and sample flags are false. This audits recorded operation, not absence of any hypothetical hidden source path.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\config.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D02_V1_H2\config.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D03_V1_S01\config.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D04_V2_H2\config.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R01_V2_nominal\config.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R02_V2_H2\config.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R03_V2_S01\config.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R04_V2_H1\config.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\trace.npz`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D02_V1_H2\trace.npz`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D03_V1_S01\trace.npz`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D04_V2_H2\trace.npz`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R01_V2_nominal\trace.npz`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R02_V2_H2\trace.npz`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R03_V2_S01\trace.npz`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R04_V2_H1\trace.npz`

### Claim 140: rounding_ok
**Location:** Figure 5 H1; manuscript_input.md:171

H1 reaches numerical rank ten but retains about 15.28 mm COM error.

COM=first moment/mass. 15.2758076984 mm rounds to 15.28; rank ten coexists with substantial COM error.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n208_adaptive_capture\runs\H1_prior\trace.npz`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n208_adaptive_capture\runs\H1_prior\posteriors.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n208_adaptive_capture\runs\H1_prior\config.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\figures\f5_historical_identification.png`

### Claim 141: exact_match
**Location:** Figure 5 H1 paired comparison; manuscript_input.md:171

Historical prior/identified action and state differences are zero despite posterior parameters entering prediction.

Elementwise action/state differences are exactly zero; historical effective configs differ only in feedback and mode. Identified model_pi equals posterior while action choices remain identical.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n208_adaptive_capture\runs\H1_prior\trace.npz`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n208_adaptive_capture\runs\H1_identified\trace.npz`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n208_adaptive_capture\runs\H1_prior\config.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n208_adaptive_capture\runs\H1_identified\config.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n208_adaptive_capture\runs\H1_prior\governor.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n208_adaptive_capture\runs\H1_identified\governor.json`

### Claim 142: exact_match
**Location:** Figure 5 H3 paired comparison; manuscript_input.md:171

Historical prior/identified action and state differences are zero despite posterior parameters entering prediction.

Elementwise action/state differences are exactly zero; historical effective configs differ only in feedback and mode. Identified model_pi equals posterior while action choices remain identical.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n208_adaptive_capture\runs\H3_prior\trace.npz`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n208_adaptive_capture\runs\H3_identified\trace.npz`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n208_adaptive_capture\runs\H3_prior\config.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n208_adaptive_capture\runs\H3_identified\config.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n208_adaptive_capture\runs\H3_prior\governor.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n208_adaptive_capture\runs\H3_identified\governor.json`

### Claim 143: missing_evidence
**Location:** Figure 5 embedded annotation; manuscript_input.md:171

Archived H1/H3 parameter-error histories and finite-data rank.

COM and rank histories agree, but PNG additionally contains a 5 mm target line whose criterion/prespecification is not exposed in declared configurations.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\figures\f5_historical_identification.png`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n208_adaptive_capture\runs\H1_prior\config.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n208_adaptive_capture\runs\H3_prior\config.json`

### Claim 144: missing_evidence
**Location:** Sec. 7.4 spectra; manuscript_input.md:173

raw and whitened singular values computed without regularization.

Posterior files contain one singular_values array and scaled right singular vectors. Raw regressor, whitening definition and separate unregularized spectra are absent.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n208_adaptive_capture\runs\H1_prior\posteriors.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n208_adaptive_capture\runs\H3_prior\posteriors.json`

### Claim 145: rounding_ok
**Location:** Sec. 8; manuscript_input.md:177

the largest candidate spread in terminal virtual progress is only about0.001670 s over the0.20 s horizon, or0.000209 in normalized progress.

D04 and R02 terminal candidate spreads match. Dividing 0.001670062677853 by the 8 s template gives 0.000208757834732, both standard-rounded.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D04_V2_H2\progress_governor.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R02_V2_H2\progress_governor.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D04_V2_H2\config.json`

### Claim 146: exact_match
**Location:** Sec. 6; manuscript_input.md:102

No independent B0/B1 pairs or B2 runs were executed.

Only B1 seen development/regression cases are supplied; no pairs/B2/fine-step outcomes are asserted. Global ledger completeness is outside the declared evidence.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D01_V1_nominal\config.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D02_V1_H2\config.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D03_V1_S01\config.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D04_V2_H2\config.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R01_V2_nominal\config.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R02_V2_H2\config.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R03_V2_S01\config.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R04_V2_H1\config.json`

### Claim 147: exact_match
**Location:** Conclusion; manuscript_input.md:185

the selected governor does not establish completion in the required multiaxis and noisy cases.

Every supplied multiaxis/noisy case terminates NO_VERIFIED_CONTROL without a full postcapture window. Manuscript does not infer success probabilities/significance from seen deterministic repetitions.

**Evidence files:**
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D02_V1_H2\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D03_V1_S01\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\D04_V2_H2\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R02_V2_H2\metrics.json`
- `C:\Users\admin\.codex\worktrees\n209-feasible-capture\力位形混合控制\output\fpmfc\n209_paper_system\runs\R03_V2_S01\metrics.json`

### Claim 148: missing_evidence
**Location:** Sec. 8 qualification; manuscript_input.md:181

The original full-boundary interface qualification failure remains unchanged

No historical full-boundary qualification failure/configuration is a declared input. This conservative limitation may be correct but is not independently confirmed here.

**Evidence files:**
- None in the declared input set.


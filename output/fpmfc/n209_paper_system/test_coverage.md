# Tests mapped to physical and implementation risks

The latest timestamped tests_*.json is the authoritative result; earlier test
failures are retained. tests_v2.log contains one fixture failure because the
initial configuration had no clearance pair within the test's0.12 m selector.
The test now directly selects target-distance/intended-surface pairs and checks
an independent combined tangent perturbation; no production tolerance changed.

|Objective risk|Evidence|
|---|---|
|P/H, passive target, no state injection|momentum_tests, isolated physics_tests, contact_tests; actual safety rows and actuator replay|
|Ideal reference reduces to nominal|reference_tests|
|SE(3) pose/twist/tool-offset derivatives|legacy derivative_tests and new reference_tests|
|Progress/endpoint derivative consistency|reference_tests, endpoint relative jets, zero-progress transport counterexample|
|Delay, duplicate/older out-of-order, stale/future, no fresh packet|timing_tests; declared rejection policy, not a general smoother|
|Estimate correction does not jump independent reference|timing_tests with a deliberately discontinuous target observation|
|Phase-I feasible/infeasible classification|phase_tests synthetic LP and three exact historical failure snapshots plus box certificate|
|Explicit secondary failure fallback|safety_interface_tests injects only a per-instance secondary failure; checks primary feasibility and zero task-lock change|
|Distance gradient/base-target drift signs and frame|safety_interface_tests independent full-tangent directional finite difference; archived actual gradient/drift snapshot|
|Predictor isolation/no future state|predictor_tests unchanged online data/histories plus own model; source API accepts only packet/estimate|
|Zero progress does not stop target|reference_tests with rotating target|
|No verified control/invalid estimates/timeout cannot count as success|explicit per-instance controller abort tests; short-trajectory summarize tests|
|Capture guard independent of smooth reference|predictor_tests perturbs only shaped reference and verifies raw gate identity|
|Future blocks, parameters and action identity|legacy prediction_tests gates before assimilation; per-run intermediate-state packet replay and historical parameter-entry audit|
|No full post-window acceptance for early stop|safety_interface_tests tests all three stop labels with short trace|
|Same packets with different evaluation truth give same new algorithm decisions|safety_interface_tests uses two different-mass plants, two independent B1 controllers, identical packets and zero robot steps|

Additional release checks verify original capture thresholds, task time budget,
read-only historical byte identities, per-run source archive hashes, trace/replay
identity and evidence links. These are reproducibility and interface checks,
not OS-level isolation, all-disturbance safety or completed unseen evaluation.

# Prespecified N209 experiment protocol

The independent statistical unit is a physical target scenario. Six scenario
clusters, three sensor seeds each, paired B0/B1 give eighteen pairs, not thirty-six
independent systems. Engineering promotion target: at least15/18 B1 completions
and zero actual safety violations. Report all failures in denominators. No test
of significance is required; six clusters cannot establish population robustness.

Development uses only known nominal,H2,S01 and declared noise-only/delay-only
variants. H1/H3 are already seen; no historical sample is an independent holdout.
After the selected version passes final nominal,H2,S01,H1 regression, freeze code,
configurations, source models, analyses, protocol, generator and random algorithm.
Only then materialize actual new scenario numbers. This file specifies rules,
not generated or executed holdouts.

Generator design: NumPy PCG64, fixed seed2091009; no rejected sample is replaced
because of a control outcome. Generate positive uniform boxes inside a cube with
half extent .15 m; C=R diag(a²/3) R^T, I=m(tr(C)I3-C). Draw all geometry before
running either method. Resample only a physically unsupported box, at most100
draws; log rejected construction values. COM components uniform[-.015,.015] m;
half lengths uniform[.07,.105] m; rotation vector components uniform[-.25,.25] rad;
require abs(COM)+abs(R)@a <= .15, positive mass and inertia triangle conditions.
Robot home and target geometry remain fixed. Initial linear velocity is defined
at the target geometric origin, zero world m/s, not at COM. Initial angular
velocity is expressed in target body frame. No initial pose perturbation in V1.

1. Light: mass uniform[12,18] kg; omega body [0.2,0.3,4.7] deg/s.
2. Heavy, rotated axes: mass uniform[28,36] kg; omega [-.3,.4,5.0] deg/s.
3. Multiaxis: mass uniform[18,26] kg; omega [-.6,.8,5.2] deg/s.
4. Faster multiaxis: mass uniform[18,30] kg; omega [.9,-.8,5.8] deg/s.
5. Different COM direction: mass uniform[18,26] kg; positive COM-x and negative
   COM-y from magnitudes uniform[.005,.015] m; omega [.3,-.3,4.8] deg/s.
6. Contact mismatch: mass uniform[18,26] kg; omega [.2,.2,5.0] deg/s; actual
   contact solref time constant scaled1.2, controller model unchanged.

All three sensor seeds209201,209202,209203 use original noisy sensor conditions:
6 ms pose samples,12 ms fixed delivery delay, position sigma10 um and rotation
sigma35 urad, original biases [2,-2,1] um and [3,2,-3] urad, force sigma.01 N,
moment sigma.0005 Nm. Noise draws remain keyed by channel and absolute index.
Same exogenous seed does not imply same physical observations on differing paths.

B2 ablations: scenarios3 and6, all three seeds, selected now. Fine-step selection:
scenarios1 and4, seed209201; physics1 ms, servo2 ms, HQP20 ms, sensor absolute time
unchanged. If a designated coarse case fails, report sensitivity not evaluable;
never replace it with a more favorable success. No new videos required; maximum2
if needed, selected scenarios1 and4 and same seed before execution.

Primary outcomes: entire task completed, actual safety violations, false latch,
failure phase/reason. Secondary outcomes: capture time/four true quantities,
rho/contact peaks, complete fixed2 s world/relative angular-speed maxima, rebound,
P/H separately, energy/work, geometry and joint margins, task residuals, governor
interventions, estimator error by phase and command latency p50/p95/p99/max plus
2/20 ms misses. Failed runs' missing post window is NOT_EVALUATED.
Report paired changes and six-cluster descriptive effects; any bootstrap uses
whole physical clusters, fixed bootstrap seed209301 and10000 draws. Completion
curves use common absolute deadline; do not compare only survivor completion times.

Per run save immutable source/config identity, packets, all timestamps, raw and
shaped state, covariance/innovations, progress/candidates, matrices and constraints,
controls/actual torques, state/contact/latch events, wrench, P/H, energy, shadow
parameters, latency and resource data. Replay actuator/events from one t0 reset;
separately replay packet decisions. No recorded state is injected into replay.
Physical task failure cannot be converted to success by a replay pass.

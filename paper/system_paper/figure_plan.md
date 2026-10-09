# Scientific figure plan

Registered while V1 development is running. No unseen validation data exist.
All plot data are read from JSON/NPZ and preserve unsuccessful outcomes.
No video is required to establish any claim in this V1.

|ID|Type/source|Caption scope|Selection|
|---|---|---|---|
|F1|Three-panel line plot; diagnostics/H2,S01,D05 snapshots|Original frozen hard-set feasibility over final0.5 s; Phase-I dimensionless z and box-row contradiction are separate quantities. Historical redecision exactly matches torque. No new trajectory or repaired success is implied.|All three required historical failures|
|F2|Two-panel estimator error lines; S01 original trace + new estimator redecision|Geometric-origin linear/angular velocity error under identical archived measurements. Dashed new-estimator curve is an offline redecision on the original trajectory, not new control performance. Norm errors and marginal covariance are different quantities.|Original S01 full prefix, display1-7 s|
|F3|Four-panel continuous task trace; nominal run|Absolute-time target world/relative spin, actual rho, and separate P/H drift, with latch and final fixed window. No numerical residual is subtracted from safety quantities.|First V1 nominal, retained even if later versions differ|
|F4|Outcome matrix; run_ledger and metrics|Every N209 attempted physical run, actual stop time and task outcome. Seen development/regression labels; no independent success probability inferred. Not-evaluated groups appear in accompanying table rather than zero-success bars.|All attempts, no deletion|
|F5|Historical identification lines/table; H1/H3 supporting audit|COM error and data rank are separate panels; torque identity0 is reported in caption/table. Full rank does not establish COM precision or action benefit.|H1_prior and H3_prior, both retained|

System flow is an editable Mermaid diagram in manuscript_v1.md, outside the
data-driven plotting workflow. Holdout/ablation scientific plots are PLANNED
until admission and execution; no placeholder simulated numbers are drawn.

F1 caption: Frozen constraint analysis over the final 0.5 s of historical H2,
S01 and D05 failures (left to right). Each Phase-I linear program uses a
clearance-row scale of 1 m/s and a joint-row scale of 1 rad/s. The dimensionless
relaxation z diagnoses the original linearized hard set; it is not penetration
or a new feasible control. Panels have separate vertical scales. Dashed vertical
lines mark the failed endpoints. Historical packet decisions and actuator
torques were reproduced exactly; no new trajectory or repaired success is shown.

F2 caption: State-estimation reanalysis on the archived noisy, delayed S01
measurement stream. The upper and lower panels show Euclidean norms of
geometric-origin linear-velocity and angular-velocity errors against
evaluation-only truth, respectively, over absolute time 1-7 s. The solid N208
trace and dashed N209 offline estimator output use the same original packets
and trajectory. This preterminal interval does not show a new closed-loop
capture. Error norms are not covariance or confidence bands; lower errors here
do not establish calibrated uncertainty or performance on unseen trajectories.

F3 caption: First N209 nominal development simulation, D01_V1_nominal, from home
through event latch and 20 s after latch. All panels use absolute time. The
dashed vertical line marks latch at 7.992 s, and the shaded interval is the fixed
final 2 s window, [25.992, 27.992] s. Upper left: target angular-speed norms in
the world frame (solid) and relative to the base (dashed), on a logarithmic
vertical axis; dotted thresholds are 0.1 and 0.02 deg/s, respectively. Upper
right: unadjusted dimensionless actual load utilization rho, with its limit at
1. Lower panels: Euclidean norms of total robot-plus-target linear and angular
momentum changes from the initial sample; angular momentum is about the system
center of mass. Numerical drift is not subtracted from safety quantities. This
single seen development scenario does not establish robustness.

F4 caption: All N209 simulation attempts recorded in the run ledger at figure
generation, retaining development/regression category, algorithm version,
recorded status and actual stop time. Columns separately report task completion,
an observed safety violation, and detumbling over the complete fixed evaluation
window. Yes and No are literal binary outcomes; black means Yes, white means No,
and gray NE means NOT_EVALUATED. A No in the safety column refers only to the
observed trajectory prefix, including an early stop; it is not full-horizon
safety certification. Missing post-latch windows, unavailable metrics and
unavailable stop times remain NE. Running attempts are retained. No independent
validation trials are implied, and these seen attempts cannot estimate success
probability on unseen scenario clusters.

F5 caption: Historical shadow identification on H1_prior (solid) and H3_prior
(dashed). Upper panel: Euclidean center-of-mass (COM) estimation error against
evaluation-only truth, with the original 5 mm accuracy target (dotted). Lower
panel: the recorded data-selected numerical rank of the scaled, whitened
regressor, with 10 possible parameter directions. Full numerical rank does not
establish COM accuracy: H1 remains above the target. In the separate complete
prior/identified paired records, the maximum actual actuator-torque difference
is exactly 0 N m for both H1 and H3 despite prediction-parameter changes. These
historical records do not demonstrate a control benefit or independent
validation of identification accuracy.

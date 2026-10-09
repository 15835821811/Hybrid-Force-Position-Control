# N208 requirements and parameter policy

Material Passport: N208 implementation and simulation evidence; new work based on
877446208453c441c226b5b5a613856f00c4b3fd; no inherited end-to-end success.
Policy registered before the first N208 robot integration, 2026-10-09 Asia/Shanghai.

| Class | Parameters and source | Permitted changes | Deadline |
|---|---|---|---|
| Hardware model | N206 XML robot geometry, seven motors, limits, armature, damping; 20 mm installation / 0.205738190495 kg tool | None in this campaign | Already frozen |
| Interface assumptions | N201 candidate2 weld; 50 N + 2 Nm additive utilization; pre-latch 20 N / 2 mm; post-latch 0.5 mm / 0.1 deg | No load or deformation gate relaxation; separate declared contact mismatch only | Already frozen |
| Capture requirement | translation 0.1 mm, rotation 0.05 deg, relative speed 1 mm/s and 0.2 deg/s | No relaxation; event confirmation initially 40 ms | Before validation |
| Performance requirement | target world speed <=0.1 deg/s; target-base relative <=0.02 deg/s over last 2 s of 20 s after latch | No retrospective window selection; report MOMENTUM_LIMITED separately | Before first run |
| Mission resources | approach <=20 s, post-latch 20 s, no recovery retry; state observation >=40 ms | Duration template 8 s initially, can be redesigned during six development attempts | Before validation |
| Unknown truth | target mass, COM, full tensor, spin; geometrically supported distributions inside 0.30 m cube | dev nominal 20 kg sphere-equivalent tensor 0.3; holdouts generated as positive mass distributions | Freeze before each first integration; never update online |
| Controller prior | mass [10,40] kg, COM each axis +/-20 mm, positive pseudoinertia and second moments within cube; central 20 kg / 0.3 identity | Bounded development only; target truth is excluded | Before validation |
| Damping prior | seven N200 coefficients, designed offline for nominal known 20 kg case; carried as fixed prior, not recomputed per truth | Fixed baseline; optional bounded scalar governor only after evidence | Before validation |
| Sensor assumptions | geometric-frame pose, base navigation, encoders, actuator torque; ideal interface F/T at target grasp point | Ideal regression and predeclared noisy delayed condition; optimistic simulation sensor, no visual-network claim | Before associated run |
| Numerical | MuJoCo 3.3.2 RK4; dt 2 ms / selected fine 1 ms; HQP 20 ms, servo 2 ms initial choices | Model evidence may justify development change, never silently change hardware bandwidth | Before validation |
| Estimation | SO(3) error-state pose/twist EKF; contact process uncertainty; scaled momentum differences and physical constraint fit | Natural information first; shadow until rank, physical and future-block prediction gates | Before validation |

Physical scope: local short-horizon microgravity; no orbital gradients, atmosphere,
residual thrusters or slosh. Passive joints keep declared damping. Software weld is
an equivalent interface, not a tested gripper. No target actuator, mocap control,
state injection, mass rewrite, fixed 3 N objective or target reaction-force forcing.
The original 40 mm robot clearances and N206 1 mm new target clearances remain.
Finite sampled priors and covariance margins are not robust safety proofs.

Development allocation: at most six full robot attempts including constructor/
implementation failures and early stops; all candidates and changed parameters
entered in run_ledger.json. Then freeze once. Up to three independent holdouts,
each paired prior/identified (six); one fine run, two pressure cases, one explicit
compatibility reserve. Total <=16. Synthetic tests, isolated body tests and input
replays counted separately. Initial compute budget 28,800 accumulated process CPU
seconds, plus conservative wall-time accounting. No dependent validation if the
development safety/admission evidence fails. Unused attempts do not require running.

Identification targets (only sufficiently excited data): mass 5%, COM 5 mm,
COM-inertia Frobenius error 10%. Rank excludes regularization and constraints;
unobservable directions preserve prior uncertainty. No full identification claim
without independent prediction and independent truth-error evaluation. Task success
and full identification are separate fields.

At startup log all source/config/model hashes. Sensor noise is keyed by channel and
absolute sample index. Evaluation reads truth separately and may terminate unsafe
experiments; evaluation cannot cause latch or improve controller estimates.

## Development amendments (before independent validation)

The six development attempts are now exhausted. No hardware, tool, actuator,
contact/weld coefficient, capture, holding, load or performance gate was relaxed.

- D01/D02 demonstrated event capture and nominal fixed-window detumbling.
  D01's early mixed-unit identification weighting is retained as historical
  diagnostic only; D02 uses state-Jacobian covariance whitening.
- D03 varied approach template duration to8.4 s. It reached the kinematic latch
  gate but failed actual latch rho1.04634; the partial-metrics serialization
  failure was recovered from original data with an explicit separate identity.
- Existing finite-prior brake prediction was moved before latch admission.
  D03 packet-prefix redecision rejected its unsafe latch without plant steps.
- D04 noisy/delayed sensing exposed a numerical EOM residual. Same-state
  diagnostics justify reducing the numerical solver tolerance from1e-10 to1e-12:
  one extra Newton iteration removed the residual. Neither physical thresholds
  nor contact coefficients changed; historical replays retain their tolerance.
- D05 showed original tracking precision could not satisfy the finite-prior
  prospective latch forecast (rho1.685 for the upper scale). The capture
  hardware gate was not changed and the forecast gate was not bypassed.
- D06 is the single final candidate: position/orientation kinematic gains scaled
  by3 throughout the run, keeping shape, servo, joint, speed and torque limits.
  It completed at27.992 s, latch7.992 s; fixed-window detumbling passed.
  Prospective latch and postlatch finite-prior predicted rho limit remains0.8.

No safe excitation was added. Final parameter feedback remains conditional on
causal prediction improvement and observable physical estimates. Loss of trust
returns the predictive parameters toward the prior at the same bounded rate.
Natural-task rank10 is a data diagnostic, not automatic full identification.

Formal freeze and independent cases are recorded in experiment_manifest.json
and holdout_identity.json. Subsequent failures cannot be used for retuning.

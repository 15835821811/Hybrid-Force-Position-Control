# Conditional proof package for N209

Theorems below describe stated models. They do not prove the entire switched,
noisy, compliant-contact algorithm globally stable. Implementation coverage is
reported separately. The working title is not a novelty or performance claim.

## A. Reference geometry and transport

**Claim.** Let the desired flange pose be
$T_{WF}(t)=T_{WG}(t)Q_{GE}(s(t))T_{FE}^{-1}$, with fixed tool mounting
$T_{FE}$ and a twice continuously differentiable relative path $Q_{GE}$.
Then the fixed mounting is respected, nominal target/path timing recovers the
nominal flange trajectory, and the transport derivatives below are exact.

**Status:** PROVABLE AS STATED; `PROVED_UNDER_ASSUMPTIONS` for differentiable
target/path states; `NUMERICALLY_CHECKED` for implementation derivatives.

**Assumptions/notation.** $R(t)\in SO(3)$ and $p(t)$ denote the target geometric
frame, whose rigid transform to G is fixed. Absorb that transform and the inverse
tool mounting into relative flange offset $q(s)$ and rotation $U(s)$, so
$p_F=p+Rq$ and $R_F=RU$. Define $r=Rq$, spatial target angular velocity
$\omega$ by $\dot R=[\omega]_\times R$, target acceleration $a=\ddot p$ and
$\alpha=\dot\omega$. Define $\eta(s)$ by
$U'(s)U(s)^T=[\eta(s)]_\times$. Primes denote differentiation with respect to
dimensionless $s$. Assume $s(t)$ is C2 inside each progress segment.

**Strategy/dependencies.** Product rule, chain rule, and
$R[x]_\times R^T=[Rx]_\times$; no stability theorem is used.

1. Right multiplication by $T_{FE}$ gives
   $T_{WF}T_{FE}=T_{WG}Q_{GE}$, exactly the requested tool-interface pose.
   For a nominal path define
   $Q_{GE}(s)=T_{WG,n}(8s)^{-1}T_{WF,n}(8s)T_{FE}$. Substitution with
   $T_{WG}(t)=T_{WG,n}(8s(t))$ cancels inverse transforms and returns
   $T_{WF,n}(8s(t))$. The time-derivative equality also requires matching target
   derivatives at that time, not merely matching a single pose sample.
2. Differentiate $p_F=p+Rq$ to obtain
   $$v_F=v+\omega\times r+Rq'\dot s.$$
   Differentiate again, applying $\dot r=\omega\times r+Rq'\dot s$:
   $$a_F=a+\alpha\times r+\omega\times(\omega\times r)
          +2\omega\times Rq'\dot s+Rq''\dot s^2+Rq'\ddot s.$$
   Both copies of the Coriolis term arise from differentiating $Rq'\dot s$
   and $\omega\times r$.
3. Multiplying $\dot R_F$ by $R_F^T$ gives
   $[\omega_F]_\times=[\omega]_\times+R[\eta]_\times R^T\dot s$, hence
   $$\omega_F=\omega+R\eta\dot s,$$
   $$\alpha_F=\alpha+\omega\times(R\eta)\dot s
                    +R\eta'\dot s^2+R\eta\ddot s.$$
4. At the nominal endpoint the flange jet is the fixed mounted target jet.
   The first equation then forces $q'(1)=0$; the acceleration equation forces
   $q''(1)=0$ because nominal progress rate is nonzero. The angular equations
   give $\eta(1)=\eta'(1)=0$. Thus clamping the relative path at its endpoint
   preserves the physical pose/twist/acceleration even if virtual filter rate
   has not decayed to zero. It does not make the scalar progress C2 at the
   clamp; the conclusion concerns the composed physical reference.
5. If $\dot s=0$, $\omega_F=\omega$ and
   $v_F=v+\omega\times r$. A rotating target with nonzero $\omega$ is a
   counterexample to any claim that pausing relative progress freezes the
   target. Therefore pausing cannot solve arbitrary transport infeasibility. ∎

**Implementation limits.** The legacy nominal target has constant center
velocity and spatial angular velocity. The shaped target adds explicit
$a,\alpha$ terms. Translation uses a constant-jerk substep; rotation uses a
Lie increment with a stated discretization approximation, not an exact
noncommuting angular-acceleration solution. No pose-to-twist consistency is
claimed across unsmoothed measurement jumps in B0. Tests evaluate finite
differences and the fixed tool offset on isolated references, not new missions.

## B. Frozen feasibility and conditional safety

**Claim B1.** For finite positive row scales $s_i$, the Phase-I LP
$$\min_{u,z}z,\quad l_i-zs_i\le a_i^Tu\le b_i+zs_i,\quad z\ge0$$
has optimum zero if and only if its original finite-dimensional linear
constraint set is nonempty (assuming the LP attains its optimum).

**Status:** PROVABLE AS STATED; `PROVED_UNDER_ASSUMPTIONS`. Historical floating
point classifications are `NUMERICALLY_CHECKED`, not symbolic exact LP proofs.

**Proof strategy/dependencies:** two feasible constructions, nonnegativity.
1. Given original feasible $u$, $(u,0)$ is feasible for Phase-I. Its objective
   is zero and all admissible objectives are nonnegative, so optimum is zero.
2. An attained optimum $(u,0)$ satisfies the original inequalities directly.
   For these finite row/bound problems the minimizer returned by HiGHS is
   checked again in original units. A positive value below tolerance cannot
   certify emptiness; unresolved numerical cases must remain unresolved. ∎

**Independent box certificate.** For a row $a^Tu\ge l$ and joint box
$L\le u\le U$, every admissible $u$ satisfies
$$a^Tu\le\sum_{a_j\ge0}a_jU_j+\sum_{a_j<0}a_jL_j=:M.$$
Each scalar inequality follows from multiplying its bound by $a_j$ with the
correct sign. Summing proves the inequality. If $M<l$, the combined set is
empty. The three audited failures satisfy this by margins much larger than
the numerical constraint tolerance. Original rows have units of m/s; the
reported deficit is not a collision penetration measured in metres.

**Claim B2 (weakened).** At a sampled instant, suppose a computed command
has nominal constraint slack $m_i$ and actual-minus-predicted left side is
bounded in magnitude by $\epsilon_i$, including state error, linearization,
input-ramp effects and numerical residual. If $m_i\ge\epsilon_i$, then
the actual sampled constraint holds.

**Status:** PROVABLE AFTER EXTRA ASSUMPTION; `PROVED_UNDER_ASSUMPTIONS` for
the inequality; applying it to every N209 tick is `OPEN_GAP`.

**Proof.** For a lower constraint, $a_i^Tu-l_i\ge m_i$ nominally and the
error is at least $-\epsilon_i$. Its actual slack is therefore at least
$m_i-\epsilon_i\ge0$. For an upper constraint apply the same argument to
its negation. If the numerical solver permits a tolerance violation, that
tolerance must be included in $\epsilon_i$, not discarded. ∎

**What is not proved.** Directional covariance propagation is not a hard
error bound. Finite-prior predictions have no certified model discrepancy
bound. Sampled satisfaction is not intersample satisfaction without a valid
motion bound, and finite-horizon acceptance does not imply recursive
feasibility. A changed soft task leaves the same frozen hard set unchanged.
NoVerifiedControl is a simulation termination, not a hardware-safe recovery
law. The fully actuated CERG theorem is not transplanted to the passive base.

## C. Momentum and internal damping

**Claim.** For a free rigid-body assembly with no external resultant force
or torque, internal ideal forces/couples cancel in the global balances. Its
linear momentum and angular momentum about a fixed inertial origin are
constant. Nonnegative internal joint damping does nonpositive work. If
relative motion stops, nonzero conserved $H_C$ generally implies nonzero
common angular velocity.

**Status:** PROVABLE AS STATED under the ideal hypotheses;
`PROVED_UNDER_ASSUMPTIONS`. Numerical/contact residuals are independent
measured quantities, not removed from the physical ledger.

**Assumptions.** Joint torques act as opposite internal couples; interface
forces act at a common point, with opposite applied couples; masses are fixed.
Let each body have COM $c_i$, momentum $p_i=m_iv_i$ and spatial COM inertia
$I_i$. Let $O$ be fixed. All derivatives exist between switching events.

**Dependencies/strategy:** Newton-Euler balance, paired internal cancellation,
the mechanical power identity, and positive locked inertia.

1. Summing $\dot p_i=f_i^{ext}+\sum_j f_{ij}$ cancels
   $f_{ij}+f_{ji}=0$. With zero external resultant,
   $\dot P=0$ for $P=\sum_i p_i$.
2. Define $H_O=\sum_i[(c_i-O)\times p_i+I_i\omega_i]$.
   The term $v_i\times m_iv_i$ vanishes. Combining each spin derivative
   with its force lever arm yields the applied moment about O. At a shared
   contact point x, $(x-O)\times(f_{ij}+f_{ji})=0$; internal couples also
   cancel. Zero external moment therefore implies $\dot H_O=0$.
3. The total COM satisfies $\dot C=P/M$, so the derivative of
   $(C-O)\times P$ is $(P/M)\times P=0$. Consequently
   $H_C=H_O-(C-O)\times P$ is constant as well.
4. The generalized internal actuator power is $\tau^T\dot q_j$.
   With $\tau_i=-\alpha D_i\dot q_i$, $\alpha\ge0,D_i\ge0$,
   $$P_{act}=-\alpha\sum_iD_i\dot q_i^2\le0.$$
   Symmetric torque saturation preserves the opposite sign of each
   $\tau_i$ and $\dot q_i$, so the inequality also holds after saturation.
   This concerns the post-latch damping command, not approach servo power.
5. If every relative angular/linear velocity vanishes, the assembly moves
   rigidly. Its COM angular momentum is $H_C=I_L(q)\omega_c$. Positive
   physical mass distribution makes $I_L$ positive definite here; thus
   $\omega_c=I_L^{-1}H_C$. If $H_C\ne0$, invertibility forbids
   $\omega_c=0$. Nonpositive power alone does not prove convergence to
   zero relative motion or a particular reachable configuration. ∎

**Open risks.** Soft weld/contact potentials, damping, switch energy and
time-discretization errors are outside ideal cancellation assumptions.
N209 records actual wrench power, P/H separately, and total energy residual;
it does not subtract numerical geometric moments to make conservation pass.

## D. Finite information and inertial identification

**Claim D1.** Pose-only observation of an isolated torque-free rigid target
cannot identify a common positive scaling of its mass and inertia when the
initial twist and COM geometry are otherwise identical.

**Status:** PROVABLE AS STATED; `PROVED_UNDER_ASSUMPTIONS`.

**Assumptions/strategy.** No external input/gravity, fixed COM offset, rigid
body with $m>0,I_C\succ0$, and a common scale $c>0$. Newton-Euler equations
about COM are $m\dot v_C=0$ and
$I_C\dot\omega+\omega\times I_C\omega=0$ in body coordinates.

1. Replacing $(m,I_C)$ by $(cm,cI_C)$ multiplies both equations by c;
   cancelling c gives identical twist ODEs and initial conditions.
2. Kinematic equations are independent of that scale, so their identical
   solutions yield identical pose observations of any fixed geometric frame.
   No estimator using only these observations can distinguish the two
   parameter settings. Contact with a known robot or calibrated wrench can
   remove this ambiguity; the isolated claim says nothing about that case. ∎

**Claim D2.** Given a whitened finite data regressor $A$ and diagonal parameter
scale matrix $S\succ0$, a correction $\Delta\pi=SV_rz$ only modifies the
span of selected right singular vectors of $AS$. It leaves the orthogonal
scaled null component of the prior unchanged. If the physical pseudoinertia
is positive semidefinite with positive mass, it corresponds to nonnegative
second central moments and an inertia satisfying triangle inequalities.

**Status:** PROVABLE AS STATED for this algebraic update and exact physical
constraint; `NUMERICALLY_CHECKED` for constrained-solver outputs.

1. Complete $V_r$ to an orthonormal basis $[V_r,V_0]$. Then
   $V_0^TS^{-1}\Delta\pi=V_0^TV_rz=0$. This proves the update restriction;
   it does not prove that every small discarded singular value is structurally
   unobservable over every possible trajectory.
2. Write $h=mc$ and
   $$J(\pi)=\begin{bmatrix}\Sigma&h\\h^T&m\end{bmatrix},
     \qquad\Sigma=\tfrac12\operatorname{tr}(I_O)I_3-I_O.$$
   For $m>0$, completing the square in $[x^T,y]J[x^T,y]^T$ shows
   $J\succeq0$ iff $\Sigma-hh^T/m\succeq0$. The latter is the central
   second-moment matrix. If its eigenvalues are $s_1,s_2,s_3\ge0$, the
   principal moments are $s_2+s_3,s_1+s_3,s_1+s_2$, whose triangle
   inequalities follow by subtracting one from the sum of the others.
3. A finite regressor rank, posterior covariance, parameter error on an
   independent truth evaluation, and improved control action are distinct
   statements. None logically implies the others without additional
   assumptions/evidence. No finite-time full-parameter theorem is claimed. ∎

**Geometric boundary.** Axis-wise COM and second-moment inequalities are
necessary enclosure checks, not a proof that every fitted distribution fits
an arbitrary nonconvex body. Generated validation inertias use explicit
positive distributions inside the declared cube. The estimator's physical
regularization does not create missing data information.

## Verification and remaining gaps

All formulas use a fixed origin/frame and preserve tool mounting. The A/B/C/D
claims above survived only with their explicit local/ideal assumptions.
The full safety/stability theorem, calibrated joint uncertainty bound,
recursive feasibility, hardware fallback, global identification and control
benefit remain `OPEN_GAP`. Numerical tests and historical certificates are
linked through claim_evidence_matrix.csv. No author/unit/funding or hardware
experiment is invented.

## E. A covariance floor of the implemented noisy linear-velocity guard

**Claim.** Consider one of the estimator's uncoupled translation blocks
with state(position,velocity), position measurement variance R>0, sample interval
h and minimum white acceleration density q>0. Let

$$F(h)=\begin{bmatrix}1&h\\0&1\end{bmatrix},\quad
Q(h)=q^2\begin{bmatrix}h^3/3&h^2/2\\h^2/2&h\end{bmatrix},\quad H=[1\;0].$$

Let $P_*\succ0$ be a posterior fixed point of
$P_*={\cal M}(FP_*F^T+Q)$, where
$\mathcal M(X)=(X^{-1}+H^TR^{-1}H)^{-1}$.
Assume the initialized posterior satisfies $P_0\succeq P_*$, process density
never falls below q, measurements occur on the fixed h grid (with possible omissions),
and the newest measurement is at least delta old at control time. Then the
implemented linear-speed uncertainty allowance has the necessary lower bound

$$m_v\ge 3\sqrt{(P_*)_{vv}+q^2\delta}.$$

**Status:** `PROVED_UNDER_ASSUMPTIONS` for the order statement;
`NUMERICALLY_CHECKED` for the particular fixed point and initial order below.
This is a bound on the filter's computed covariance allowance, not a bound on
true estimation error and not a calibrated confidence statement.

**Proof.** For positive definite X>=Y in Loewner order, inversion gives
X^-1<=Y^-1. Adding the same H^T R^-1 H and inverting again proves
M(X)>=M(Y). Propagation X->FXF^T+Q is also order preserving. Each actual
process covariance exceeds the free-mode Q by a positive semidefinite amount;
split propagation through contact-history intervals preserves this ordering.
Starting from P0>=P*, induction over measurement times gives Pk>=P*.
Omitting a measurement cannot decrease covariance relative to applying it.
The velocity row of F is[0,1], so propagation by delta increases velocity
variance by at least q² delta. The largest eigenvalue of the three-dimensional
velocity covariance is at least this component variance. The implemented guard
also adds a nonnegative angular-offset allowance, so omitting that term gives
the stated lower bound. Capture requires a nonnegative relative-speed norm
plus this margin to be <=1 mm/s; a larger margin precludes approval. ∎

For the recorded noisy configuration, h=0.006 s, delta=0.012 s,
R=(10 micrometres)² and q=0.003 m/s^(3/2). The exact translation structure
has no orientation coupling in its covariance blocks; the Joseph rotation
reset operates only on the rotational block. After the first pose the scalar
initial covariance is diag(R,0.1), independently of measured values. The
contact multiplier is at least one. `uncertainty_floor.py` solves the DARE and
independently iterates the covariance recursion, checks the fixed-point residual
and positive eigenvalues of P0-P*, and obtains a minimum three-sigma allowance
of approximately1.609697 mm/s. Raw numerical matrices, residuals, sensor values
and source hashes are in `output/fpmfc/n209_paper_system/uncertainty_floor.json`.
This numerical value agrees with the minimum recorded over1–7 s.

Consequently, under the stated implemented covariance recursion/protocol, the
unchanged1 mm/s gate cannot be approved after a valid noisy estimate. The early
invalid estimates also cannot approve capture. This conditional incompatibility
is specific to the current covariance model and guard; it does not show that
the physical sensor, another estimator or the passive-base task is impossible.
No process covariance was reduced to force a pass, and no further physical
trial is needed to identify this necessary-condition obstruction.

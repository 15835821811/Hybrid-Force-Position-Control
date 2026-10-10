# S01 source ambiguities and declared interpretation

The source PDF SHA256 is recorded in source_parameters.yaml. PDF pages 4-11
(printed 961-968) were read both as extracted text and rendered page images.
Tables 2/3, initial conditions, and PSO constants were transcribed row by row.
Reported values 15.6 s, 0.2686 rad, 5.22%, and 25.27% are evidence of what the
authors report, never calibration targets for this implementation.

|Locator|Verified fact|Gap / treatment|
|---|---|---|
|Eq10-17, 961-962|Zero initial momentum; generalized end-effector and base-motion maps; type I is base attitude.|Use world linear and body angular free-joint velocity explicitly. SO(3) attitude is used instead of singular ZXZ Euler rates; this is a declared representation change.|
|Eq18, Fig2/3, 962-963|Type II is arm shape; V is parallel to joint 1 and psi is the signed reference-to-elbow-plane angle.|Use the positive joint axis for V. Angle derivative is independently obtained by differentiating atan2, then checked against engine-coordinate finite differences.|
|Eq19/23, 962|Printed analytic shape derivative contains an auxiliary vector l.|The text does not explicitly define l. The normal reading l=w cross V disagrees at the preregistered configurations (maximum about 0.2283); retained in equation_tests_diagnostic_l_normal.json. The projected reference vector l=(w cross V) cross unit(w) gives the derivative of Eq18. Both residuals are reported, with this interpretation labelled, rather than silently repairing the source.|
|Eq26, 964|Optimize grasp-point velocity/radial alignment assuming zero endpoint velocity.|Use signed acos of the dot product, relative to the initial conserved robot COM. No contact impulse is simulated or predicted from this angle.|
|Eq27/28, 964|N=I-Jg^+Jg annihilates endpoint velocity.|The wording concerning reactionless motion is not a proof that A N=0. Both residuals are measured; the latter is generally nonzero.|
|Eq29-31, 964|Primary endpoint velocity plus shape-nullspace motion.|A literal total shape rate in Eq30 adds Jpsi*qdot_primary to the desired total rate. The executable compatibility interpretation subtracts this primary contribution. Literal and residual readings are separately tested; no claim that the omitted decomposition was explicitly printed.|
|Eq32/33, 964-965|Impedance maps wrench error to a pose increment.|Desired wrench is zero in the source experiment. S01 is precontact, so contact/force dynamics are not reproduced.|
|Eq34, 965|Printed qdot_d=Jg^+(xdot_ed+Delta x).|The addition mixes velocity and pose increment unless a derivative/discretization is supplied. Preserve the printed form; no silent unit repair and no claim of reproducing compliant contact.|
|Table2, 965; initial angles, 966|DH angles [-90,180,0,...] and actual initial angles [0,45,0,90,0,45,0] are distinct.|Assume standard DH with the table angles as fixed offsets, applied exactly once. DH-to-base mounting is not supplied. A declared Rx(pi) underslung installation points toward the stated negative-base-z target half-space. Identity placement was a rejected static construction check before any trajectory.|
|Table3, 966|Base plus links total 524 kg; diagonal inertias and zero products given.|COM positions, tensor reference points and frame axes are absent. Midpoint COM and pre-Rx principal axes are declared assumptions. All tensors satisfy positive definiteness and triangle inequalities. The large B1 inertia is preserved, not scaled to match the paper curves. Equivalent mass-support ellipsoids are reported; thin displayed centerlines are not physical collision envelopes.|
|Target conditions, 966|Target origin [0,0,-0.7657]m, grasp point [0,-0.25,-0.7657]m, reported orientations and 5deg/s spin.|Spin axis, full Euler convention, target inertia and exact mounting/geometry are missing. Freeze a z-axis principal spin and extrinsic xyz angle reading, with zero center translation. Target is prescribed planning geometry only, not a simulated driven satellite or grasped body.|
|Eq35, 966|G=w1*delta_o^2+w2*alpha^2.|w1/w2, scale, and delta_o aggregation are not specified. Declare dimensionless peak world angular-speed and alignment components with weights 1/1. Drift and RMS remain separately reported. PSO cognitive/social factors 1.5/2 are not objective weights.|
|PSO, 966|20 particles, inertia .9, velocity ±10% span, cognitive1.5/social2, 1e-5 adjacent-best threshold, maximum1000 iterations.|Bounds and seeds absent. This batch freezes 6 generations (120 calls) per cell, 3 strategies x 2 seeds. REDUCED_BUDGET; adjacent-best criterion is reported, not used to spoil equal budgets or certify convergence.|
|Fig9/10, 967|Base curves use mm/s and deg/s; fixed psi0 and psi90 comparisons.|Exact common-time protocol and percentage aggregation are not fully established. Protocol A and fair time optimization protocol B are declared comparisons on this model. No cross-platform percentage equivalence.|
|Conclusion, 968|Experimental validation is precontact; collision and postcapture stabilization are future work.|No force regulation, latching or detumbling is executed in S01.|

Physical collision qualification is SOURCE_LIMITED because solid geometry is
absent. Positive rigid-body inertial dynamics, joint/torque limits and tracking
gates are evaluated separately. An advisory sampled centerline distance is not
called a safety margin. The low-level computed-torque servo also differs from
the source ADAMS joint-motion drive, and all compared dynamics share it.

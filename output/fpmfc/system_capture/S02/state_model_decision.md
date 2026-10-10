# S02 state model decision

Selected CA2 from two frozen density multipliers of the same 18-dimensional model. Densities come from independent development acceleration increments, not old CV units or guard-only tuning. Selection used state error, blind prediction error, NIS and coverage. The validation seeds and [2,6] s suffix were frozen first.

State order: dp_W,dtheta_W,dv_W,domega_W,da_W,dalpha_W. Means p,R,v,omega,a,alpha. Velocity and acceleration start at zero with declared public variances. No true twist, COM or inertia is read. White jerk spectral intensities are the squares of the saved densities (m/s^(5/2), rad/s^(5/2)). Translation Q is the integrated white-jerk matrix. The repaired C2 rotation mean and angular variational transition use RK4 substeps <=2 ms; process covariance is a positive-weight Gauss-Legendre integral of noise-factor outer products. The original E0 direct-Q RK4 defect and source are retained separately. Left-error reset uses the left Jacobian, checked independently by group perturbation; this new formulation does not rewrite archived CV.

Past propagation uses causal mode history. Forecast holds the latest observed mode with no future measurements/contact truth. The governor combines this covariance with an independent prior-plant mean; this is an approximation, not a joint consistent probability model. Guard thresholds, multiplier, shaping, candidates and horizon remain identical.

Known limits: ideal contact spikes have increased NIS; free noisy information improves but prolonged contact margins remain incompatible. Admission is to at most four frozen diagnostic trials, conditional on E0 and subsequent safety checks.

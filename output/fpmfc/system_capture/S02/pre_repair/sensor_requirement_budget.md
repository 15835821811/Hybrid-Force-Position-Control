# Capture information budget

The original CV conditional linear 3sigma floor is 1.6096967038874643 mm/s under 6 ms pose sampling, age >=12 ms, 10 micrometre noise and 0.003 m/s^(3/2) acceleration density. This is a computed covariance allowance bound, not actual-error or all-estimator impossibility.

The new CA numerical periodic free-mode diagnostic at ages 12/14/16 ms gives about 0.362/0.369/0.376 mm/s linear guard margin, leaving about 0.638/0.631/0.624 mm/s for relative tracking. The contact-mode diagnostic is about 1.266–1.367 mm/s and leaves no linear budget. These are fixed-protocol, zero-angular-rate diagnostics; the time-series files retain the actual nonlinear ranges.

All target position/orientation and velocity/angular-velocity cross terms and lever-arm orientation derivatives are propagated offline. Tool velocity includes omega cross fixed flange-to-interface offset. Robot navigation/encoders, timestamps and fixed calibration are exact SIMULATION_ASSUMPTION channels, not demonstrated hardware capabilities. With hardware their covariance, correlations, time bias and calibration must consume additional budget. A 3sigma direction approximation is not an overall or all-time 99.7% guarantee.

E3 removes only target pose delay as an authorized diagnostic. No hardware protocol replacement is deployed. Further sensing requirements must address contact-transition bandwidth and both angular/linear uncertainty, with positive tracking reserve, not only free-motion RMS.

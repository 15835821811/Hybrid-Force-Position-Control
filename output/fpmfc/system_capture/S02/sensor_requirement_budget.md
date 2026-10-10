# Capture information budget

The original CV conditional linear 3sigma floor is 1.6096967038874643 mm/s under 6 ms pose sampling, age >=12 ms, 10 micrometre noise and 0.003 m/s^(3/2) acceleration density. This is a computed covariance allowance bound, not actual-error or all-estimator impossibility.

The new CA numerical periodic free-mode diagnostic at ages 12/14/16 ms gives about 0.362/0.369/0.376 mm/s linear guard margin, leaving about 0.638/0.631/0.624 mm/s for relative tracking. The contact-mode diagnostic is about 1.266–1.367 mm/s and leaves no linear budget. These are fixed-protocol, zero-angular-rate diagnostics; the time-series files retain the actual nonlinear ranges.

All target position/orientation and velocity/angular-velocity cross terms and lever-arm orientation derivatives are propagated offline. Tool velocity includes omega cross fixed flange-to-interface offset. Robot navigation/encoders, timestamps and fixed calibration are exact SIMULATION_ASSUMPTION channels, not demonstrated hardware capabilities. With hardware their covariance, correlations, time bias and calibration must consume additional budget. A 3sigma direction approximation is not an overall or all-time 99.7% guarantee.

The authorized E3 design removes only target pose delay, but E3 was removed from the remaining run plan when the E0 implementation repair consumed an attempt. No zero-delay robot-task result is claimed. No hardware protocol replacement is deployed. Further sensing requirements must address contact-transition bandwidth and both angular/linear uncertainty, with positive tracking reserve, not only free-motion RMS.

## Offline sensing-design diagnostic

This eight-row design table holds the selected CA densities and contact multiplier fixed. It is not another estimator candidate or an E3 robot result. Original bias bounds remain unchanged. Robot/clock/calibration errors remain zero only under the stated simulation assumptions.

|Pose period (ms)|Delay (ms)|Noise scale|Max linear margin (mm/s)|Max angular margin (deg/s)|All four necessary budgets positive|
|---:|---:|---:|---:|---:|---|
|6|12|1|1.3672|0.2704|False|
|6|0|1|1.0747|0.2033|False|
|6|12|0.5|1.0308|0.2061|False|
|6|0|0.5|0.7656|0.1451|True|
|2|2|1|0.8178|0.1547|True|
|2|0|1|0.7765|0.1453|True|
|2|2|0.5|0.5823|0.1103|True|
|2|0|0.5|0.5455|0.1019|True|

Noise scale 1 means 10 micrometres / 35 microradians per component; scale 0.5 means 5 micrometres / 17.5 microradians. Ages cover all phases of the unchanged 2 ms controller clock.

These are stationary zero-rate covariance recursions, not actual-error guarantees, contact-transition validation, hardware specifications or authorization to replace the sensor. A positive remaining budget must still cover tracking error and all extra hardware errors. The observed startup/latch innovation spikes are not solved by this table.

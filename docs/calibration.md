# Calibration protocol

This is the canonical entry point for estimator calibration. The executable checklist is
[`calibration_checklist.md`](calibration_checklist.md); its checks must be completed for each
vehicle/sensor installation before a profile is treated as calibrated.

IMU white-noise amplitude densities and bias random-walk densities are continuous-time quantities.
Pass unit-tagged Allan-variance output through `convert_allan_variance.py`; it validates units and
squares amplitude densities into the PSD parameters consumed by the ESKF. The checked input,
generated YAML, and report are `allan_variance_example.json`, `allan_variance_example.yaml`, and
`allan_variance_example_report.md`.

The remaining required measurements are IMU-to-body axis/sign convention, GNSS antenna lever arm,
wheel sign and scale, and sensor time offsets. Record the calibration dataset, method, uncertainty,
date, temperature range, and profile values. Temperature compensation is admitted only when a
temperature-swept stationary dataset demonstrates repeatable residual structure and holdout benefit;
the registered datasets currently do not provide that evidence.


# Diagnostics and health API

The ROS 2 component publishes a typed status stream in addition to the existing
legacy `std_msgs/Float64MultiArray` debug topics. Typed messages are stable at
the field level and should be used by monitoring, bag analysis, and safety
supervision tools.

## Topics

- `/<node>/status` — `kalman_filter_localization_msgs/EstimatorStatus`
- `/<node>/diagnostics` — `diagnostic_msgs/DiagnosticArray`
- `/<node>/debug/measurement_quality_typed` — `kalman_filter_localization_msgs/MeasurementQuality`
- `/<node>/debug/observability_typed` — `kalman_filter_localization_msgs/ObservabilityStatus`
- `/<node>/debug/replay_timing_typed` — `kalman_filter_localization_msgs/ReplayTiming`

The default node name is `ekf_localization`, so the default prefix is
`/ekf_localization`.

## Health and mode

`EstimatorStatus.health` uses the following levels:

- `UNKNOWN`: no initial pose or no sensor input has arrived.
- `OK`: the estimator state and covariance satisfy numerical invariants.
- `DEGRADED`: the estimator is operating with reduced aiding, for example during
  GNSS outage, initialization, or stale wheel input.
- `FAULT`: a numerical invariant failed, or GNSS is unavailable without a
  configured vehicle aid.

`EstimatorStatus.mode` identifies the current operating condition:

- `UNINITIALIZED`, `INITIALIZING`, `STATIONARY`
- `OPEN_SKY`, `URBAN`, `OUTAGE`, `REACQUISITION`, `FAULT`

The status message also includes sensor availability, filter time, covariance
standard-deviation summaries, wheel scale, input counters, measurement accept /
reject counters, replay rewinds, and reorder-buffer counters. A consumer should
use `health` for the coarse decision and `mode`/`summary` for the reason.

When `enable_sensor_fault_isolation` is enabled, the status additionally reports
per-sensor isolation flags and fault-event counters for IMU, GNSS, wheel, and
odometry. A sensor is isolated after `sensor_fault_trip_count` consecutive bad
samples or rejected updates, held for `sensor_fault_hold_sec`, and then allowed
one probe sample for recovery. The default is disabled to preserve historical
regression behavior; production profiles should enable it after selecting
appropriate thresholds.

## Measurement quality

Every position, velocity, and course update that reaches the common quality
path emits a `MeasurementQuality` record. `accepted` is the decision, while
`reject_reason` is machine-readable and `reject_reason_text` is intended for
logs and dashboards. `raw_nis` is computed before adaptive/robust variance
scaling; `used_nis` is the value associated with the update covariance.

## Replay timing

When measurement replay is enabled, each replayed measurement emits its sensor
time, arrival time, filter time before correction, apply time, source, and replay
status. `queued_future` means the event was accepted into the bounded future
queue; it has not yet changed the estimator state.

## Compatibility policy

The legacy array topics are retained for one migration period. New consumers
must use the typed messages. The typed package is intentionally separate from
the estimator implementation so other ROS 2 nodes can depend on the message
contract without depending on Eigen or the EKF core.

## Online calibration and vehicle aids

Stationary initialization estimates the initial IMU gyro bias and roll/pitch.
`estimate_wheel_speed_scale_factor` provides bounded online wheel-scale
calibration from GNSS-derived velocity or configured absolute-Odometry body
speed when the vehicle is moving straight and the timestamps are close. Select
the reference with `wheel_scale_reference: "gnss"`, `"odom"`, or `"either"`.
The current scale is exposed as
`EstimatorStatus.wheel_scale_factor`; the configured min/max bounds and sample
window are safety limits, not tuning suggestions.

Wheel speed plus non-holonomic constraints are provided by the default
`kalman_filter_localization/GroundVehicleModel` plugin and are reported through
`ObservabilityStatus`, including the selected `vehicle_model`, turning/slip
state, vertical-constraint policy, and adaptive constraint variance. The
`kalman_filter_localization/PlanarVehicleModel` plugin keeps lateral NHC but
does not force vertical body velocity to zero. External vehicle plugins can
implement the ROS-free `core::VehicleModel` interface and export it through
pluginlib without changing the core EKF message contract.

Set `vehicle_model_plugin` to the exported plugin class name. A plugin receives
timestamped body velocity, yaw rate, lateral acceleration, and optional wheel
innovation, and returns a bounded NHC policy. It must reject invalid
configuration or non-finite input rather than silently applying a constraint.

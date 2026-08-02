# `kf_doctor`

`kf_doctor` is an offline preflight tool for a localization rosbag. It is meant
to find wiring and data-quality errors before EKF tuning begins.

## Usage

```bash
ros2 run kalman_filter_localization kf_doctor \
  --bag input_bag \
  --output-report .data/doctor/report.json \
  --output-profile .data/doctor/profile.yaml
```

Topic names are discovered by message type and name when the optional topic
arguments are omitted:

```bash
ros2 run kalman_filter_localization kf_doctor \
  --bag input_bag \
  --imu-topic /sensing/imu/imu_data \
  --gnss-topic /gnss/fix \
  --wheel-topic /wheel_speed
```

The command returns zero for `pass` and `warn`, one when a required check fails,
and two for a tool/input error. A missing GNSS or wheel topic is a warning; a
missing IMU or non-monotonic sensor time is an error.

## Report contents

The JSON report contains:

- all bag topic types and selected roles;
- per-topic counts, sensor/header time ranges, frame IDs, rate/gap statistics,
  and covariance validity;
- IMU rate, stationary sample count, acceleration norm, component standard
  deviations, and heuristic continuous-time PSD values;
- stable check records with `severity`, `code`, `message`, and optional details;
- a conservative `profile_skeleton` suitable as a starting point, not as an
  automatically accepted tuned configuration.

The IMU noise estimate uses only samples whose angular rate is small and whose
acceleration norm is close to configured gravity. It converts sample standard
deviation to an amplitude-density estimate using the detected sample rate and
then squares it for the continuous-time EKF PSD. This is intentionally labeled
heuristic; use Allan variance and `docs/calibration_checklist.md` for final
calibration.

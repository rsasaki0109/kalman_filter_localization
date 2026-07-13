# Kalman Filter Localization

Kalman Filter Localization is a ROS 2 package for 3D localization with GNSS, IMU, and odometry.

## Node
`ekf_localization_node`

Input:
- `/initial_pose` (`geometry_msgs/PoseStamped`)
- `/gnss_pose` (`geometry_msgs/PoseStamped`) by default
- `/gnss/fix` (`sensor_msgs/NavSatFix`) when `gnss_input_type: "navsatfix"`
- `/imu` (`sensor_msgs/Imu`)
- `/odom` (`nav_msgs/Odometry`)
- `/gnss/velocity` (`geometry_msgs/TwistWithCovarianceStamped`, optional Doppler velocity)
- `/tf` (`/base_link` -> `/imu_link`)

Output:
- `/current_pose` (`geometry_msgs/PoseStamped`)
- `/current_odometry` (`nav_msgs/Odometry`, including velocity and covariance)
- `/tf` (`reference_frame_id` -> `robot_frame_id`) when `publish_tf: true`

## Params

| Name | Type | Default value | Description |
|---|---|---:|---|
| `pub_period` | int | 10 | Publish period `[ms]` |
| `output_odometry_topic` | string | `/ekf_localization/current_odometry` | Odometry output topic |
| `publish_tf` | bool | false | Publish the estimated reference-to-robot transform |
| `var_gnss_xy` | double | 0.1 | GNSS position variance in XY `[m^2]` |
| `var_gnss_z` | double | 0.15 | GNSS position variance in Z `[m^2]` |
| `gnss_position_robust_loss` | string | `none` | Soft robust loss: `none`, `huber`, or `cauchy` |
| `gnss_position_robust_tuning` | double | 2.5 | Robust loss transition in normalized residual units |
| `max_gnss_position_robust_variance_scale` | double | 100.0 | Maximum robust covariance multiplier |

The optional robust loss uses `sqrt(NIS)` as a normalized residual. Huber increases
measurement covariance linearly beyond the tuning constant, while Cauchy increases it
quadratically. Hard innovation/NIS gates still take precedence; robust scaling handles
the measurements that remain inside those gates without an accept/reject discontinuity.
| `gnss_input_type` | string | `pose` | GNSS input type: `pose` or `navsatfix` |
| `gnss_pose_topic` | string | `gnss_pose` | `PoseStamped` GNSS input topic |
| `gnss_navsatfix_topic` | string | `/gnss/fix` | `NavSatFix` GNSS input topic when `gnss_input_type: "navsatfix"` |
| `gnss_navsatfix_use_first_fix_as_origin` | bool | true | Use the first valid `NavSatFix` as the local ENU origin |
| `gnss_navsatfix_use_position_covariance` | bool | false | Use per-message NavSatFix ENU covariance |
| `gnss_navsatfix_min_variance_xy` | double | 0.0001 | XY variance floor `[m^2]` |
| `gnss_navsatfix_min_variance_z` | double | 0.0001 | Z variance floor `[m^2]` |
| `gnss_navsatfix_max_variance_xy` | double | 100.0 | XY variance ceiling `[m^2]` |
| `gnss_navsatfix_max_variance_z` | double | 100.0 | Z variance ceiling `[m^2]` |
| `gnss_lever_arm_x` | double | 0.0 | GNSS antenna X offset from body/IMU origin `[m]` |
| `gnss_lever_arm_y` | double | 0.0 | GNSS antenna Y offset from body/IMU origin `[m]` |
| `gnss_lever_arm_z` | double | 0.0 | GNSS antenna Z offset from body/IMU origin `[m]` |
| `compensate_gnss_delay` | bool | false | Extrapolate delayed GNSS position to the latest IMU time |
| `gnss_time_offset_sec` | double | 0.0 | Offset added to the GNSS message timestamp `[s]` |
| `max_gnss_delay_compensation_sec` | double | 0.5 | Reject observations older than this limit `[s]` |
| `var_odom_xyz` | double | 0.1 | Odometry variance `[m^2]` |
| `var_imu_w` | double | 0.01 | Angular velocity variance `[(deg/sec)^2]` |
| `var_imu_acc` | double | 0.01 | Accelerometer variance `[(m/sec^2)^2]` |
| `use_continuous_process_noise_density` | bool | false | Interpret IMU noise parameters as continuous PSDs |
| `use_second_order_state_transition` | bool | false | Use a second-order continuous-to-discrete state transition |
| `use_second_order_process_noise` | bool | false | Propagate continuous noise into coupled states |
| `use_gnss` | bool | true | Whether GNSS is used |
| `use_odom` | bool | false | Whether odometry is used |

When `use_continuous_process_noise_density` is enabled, `var_imu_acc` and
`var_imu_w` are treated as continuous-time power spectral densities and discretized
with `Qd = Qc * dt`. This makes covariance growth substantially independent of the IMU
sampling rate. The default remains disabled to preserve existing tuned profiles that
use the legacy per-sample `dt^2` convention.

`use_second_order_state_transition` constructs a continuous-time error-state Jacobian
and applies `Phi = I + Fc*dt + 0.5*(Fc*dt)^2`. This includes position, velocity,
attitude, gyro-bias, and accelerometer-bias cross-couplings from one consistent model.
Enable it together with continuous process noise for new profiles; existing profiles
retain the hand-discretized legacy transition by default.

`use_second_order_process_noise` integrates continuous error-state noise through the
transition dynamics up to third order in `dt`. This generates position–velocity and
other cross-covariance terms within each IMU step. Enable it together with continuous
process noise and the second-order state transition.
| `use_nonholonomic_constraint` | bool | false | Constrain body lateral and vertical velocity |
| `var_nhc_lateral_velocity` | double | 0.05 | Lateral pseudo-measurement variance `[(m/s)^2]` |
| `var_nhc_vertical_velocity` | double | 0.02 | Vertical pseudo-measurement variance `[(m/s)^2]` |
| `min_nhc_forward_speed_mps` | double | 0.5 | Minimum absolute forward speed for NHC `[m/s]` |
| `nhc_adaptive_yaw_rate_radps` | double | 0.5 | Yaw-rate scale where NHC covariance starts increasing |
| `nhc_adaptive_lateral_accel_mps2` | double | 1.5 | Lateral-acceleration scale for adaptive covariance |
| `max_nhc_variance_scale` | double | 100.0 | Maximum adaptive NHC covariance multiplier |
| `use_zupt` | bool | false | Apply zero-velocity updates after stationary detection |
| `zupt_max_angular_velocity_radps` | double | 0.02 | Maximum gyro norm for stationary detection `[rad/s]` |
| `zupt_max_acceleration_error_mps2` | double | 0.2 | Maximum acceleration-norm error from gravity `[m/s^2]` |
| `zupt_max_speed_mps` | double | 0.3 | Maximum estimated speed for stationary detection `[m/s]` |
| `zupt_min_stationary_duration_sec` | double | 0.5 | Required continuous stationary duration `[s]` |
| `var_zupt_velocity` | double | 0.01 | Zero-velocity observation variance `[(m/s)^2]` |
| `use_zihr` | bool | false | Estimate gyro bias from stationary angular-rate observations |
| `var_zihr_gyro` | double | 1.0e-5 | Stationary gyro observation variance `[(rad/s)^2]` |

The non-holonomic constraint assumes a wheeled vehicle whose body-frame lateral and
vertical velocities are normally close to zero. Its covariance is weakened during
high yaw-rate or lateral-acceleration motion. Keep it disabled for holonomic, airborne,
marine, or intentionally sliding platforms.

ZUPT additionally constrains all three velocity axes to zero after the IMU and estimated
speed remain stationary for the configured duration. Because constant-speed motion can
look stationary to an IMU, `zupt_max_speed_mps` is an essential second condition. Keep
ZUPT disabled when no reliable low-speed estimate is available.

ZIHR uses the same stationary detector but treats measured angular velocity as gyro
bias while the platform is stopped. It can be enabled independently of ZUPT and is
most useful when `initial_imu_gyro_bias_covariance` is nonzero so the bias state is
allowed to converge.
| `use_gnss_doppler_velocity` | bool | false | Fuse receiver-provided ENU Doppler velocity |
| `gnss_doppler_velocity_topic` | string | `/ekf_localization_node/gnss/velocity` | Doppler velocity input topic |
| `use_gnss_doppler_velocity_covariance` | bool | true | Use the message linear-velocity covariance when valid |
| `use_gnss_doppler_course_yaw` | bool | false | Derive and fuse course yaw from Doppler velocity |
| `min_gnss_doppler_course_speed_mps` | double | 1.0 | Minimum horizontal speed for Doppler course yaw `[m/s]` |

`use_gnss_doppler_course_yaw` assumes that vehicle heading follows its direction of
travel. Keep it disabled for platforms that move sideways or frequently reverse unless
the velocity direction is corrected upstream.

### NavSatFix input

If your RTK receiver publishes `sensor_msgs/NavSatFix`, set:

```yaml
gnss_input_type: "navsatfix"
gnss_navsatfix_topic: "/gnss/fix"
```

The node converts WGS84 latitude/longitude/altitude to local ENU `PoseStamped` internally and fuses it with the same GNSS variance parameters. By default, the first valid fix becomes the ENU origin, so `/initial_pose` should use the same local frame. To use a fixed origin, set `gnss_navsatfix_use_first_fix_as_origin: false` and provide `gnss_navsatfix_origin_latitude`, `gnss_navsatfix_origin_longitude`, and `gnss_navsatfix_origin_altitude`.

When enabled, NavSatFix covariance entries 0, 4, and 8 are used as ENU X/Y/Z
measurement variances. Unknown, non-finite, or non-positive covariance falls back to
the configured fixed GNSS variance. Floors prevent overconfident receiver output and
ceilings prevent a single message from effectively disabling correction.

The GNSS lever arm is expressed in `robot_frame_id`: X forward, Y left, and Z up.
The current estimated orientation rotates this offset into the reference frame before
the antenna observation is evaluated. The EKF measurement Jacobian includes the
lever-arm sensitivity to attitude error, allowing antenna position during turns to
correct both body position and attitude. Leaving all three values at zero preserves
the original position-only observation model.

Optional GNSS delay compensation extrapolates the lever-arm-corrected position with the
current estimated velocity. A positive `gnss_time_offset_sec` means the effective GNSS
measurement time is later than its message timestamp. This constant-velocity model is
intended for known, short receiver delays; it is not equivalent to state rewind and IMU
replay during aggressive acceleration or turning.

## Evaluating localization accuracy

`evaluate_localization` compares `/current_pose` with a timestamped reference trajectory.
It reports 3D, horizontal, vertical, and yaw RMSE, plus the maximum 3D error.

```bash
ros2 run kalman_filter_localization evaluate_localization \
  --bag path/to/bag \
  --estimate-topic /current_pose \
  --reference-topic /ground_truth/pose \
  --output-json metrics.json \
  --output-csv errors.csv
```

Both topics may contain `geometry_msgs/PoseStamped` or `nav_msgs/Odometry`. A reference
CSV can be used instead with `--reference-csv`. For CSV-only evaluation, provide
`--estimate-csv` and `--reference-csv`. CSV files require `stamp,x,y,z` and either a
`yaw` column in radians or `qx,qy,qz,qw` quaternion columns.

Reference poses are linearly interpolated at estimate timestamps. Samples spanning a
reference gap larger than `--max-reference-gap` (default: 0.2 seconds) are excluded.
Use `--time-offset` when a known sensor timestamp offset must be compensated.

For automated regression testing, configure acceptance thresholds. The command exits
with status 1 when a threshold is exceeded (or when the match ratio is too low), and
status 2 for invalid input:

```bash
ros2 run kalman_filter_localization evaluate_localization \
  --estimate-csv result.csv \
  --reference-csv ground_truth.csv \
  --max-rmse-3d 0.10 \
  --max-yaw-rmse-deg 3.0 \
  --min-match-ratio 0.95 \
  --output-json metrics.json
```

Available limits are `--max-rmse-3d`, `--max-rmse-horizontal`,
`--max-rmse-vertical`, `--max-yaw-rmse-deg`, `--max-error-3d`, and
`--min-match-ratio`. The JSON output includes `passed` and
`threshold_failures` fields for CI artifact processing.

The repository includes a small deterministic CSV regression dataset under
`kalman_filter_localization_ros2/test/data`. Its expected metrics and acceptance
limits are tested on both supported ROS distributions by GitHub Actions. This dataset
tests the evaluator; vehicle-specific rosbag baselines can use the same CLI and limits
without committing large bag files to the repository.

## Comparing ablation profiles

Four starting profiles are provided: `research_baseline`, `research_nhc`,
`research_robust`, and `research_full`. After producing one estimate CSV per profile,
generate a comparison table with the first estimate treated as the baseline:

```bash
ros2 run kalman_filter_localization compare_localization_results \
  --reference-csv ground_truth.csv \
  --estimate baseline=baseline.csv \
  --estimate nhc=nhc.csv \
  --estimate robust=robust.csv \
  --estimate full=full.csv \
  --output-csv comparison.csv \
  --output-markdown comparison.md
```

The research profiles are ablation starting points, not sensor-independent tuned
defaults. Lever arm and delay parameters remain dataset/hardware-specific and must be
configured separately.

For UrbanNav Tokyo, the complete four-profile run can be reproduced with one command.
The input bag must publish `/gnss_pose` and `/sensing/imu/imu_data`; the reference CSV
uses the same columns described above.

The official `UrbanNav-TK-20181219` archive provides `imu.csv`, `reference.csv`, rover
and base RINEX files. One reproducible way to produce the GNSS solution and ROS 2 inputs
is:

```bash
rnx2rtkp -p 2 -f 2 -t -s , -o odaiba_ublox_rtk.pos \
  Odaiba/rover_ublox.obs Odaiba/base_trimble.obs Odaiba/base.nav

ros2 run kalman_filter_localization prepare_urbannav_tokyo \
  --imu-csv Odaiba/imu.csv \
  --reference-csv Odaiba/reference.csv \
  --rtklib-pos odaiba_ublox_rtk.pos \
  --output-bag odaiba_input_bag \
  --output-reference-csv odaiba_reference.csv
```

The converter expresses both trajectories in an ENU frame whose origin is the first
Applanix reference position. It converts the dataset IMU from forward-right-down to
ROS forward-left-up axes and repeats the initial reference pose before sensor playback
so DDS discovery cannot lose initialization.

```bash
share="$(ros2 pkg prefix kalman_filter_localization)/share/kalman_filter_localization"
ros2 run kalman_filter_localization run_urbannav_ablation \
  --input-bag odaiba_input_bag \
  --reference-csv odaiba_reference.csv \
  --output-dir results/urbannav_tokyo \
  --base-profile "$share/param/profiles/urbannav_tokyo_tuned.yaml" \
  --profiles-dir "$share/param/profiles"
```

Each run stores its merged parameter YAML, process logs, recorded estimate bag, and
`estimate.csv`. The top-level directory contains `manifest.json`, `comparison.csv`,
and `comparison.md`. Use `--dry-run` to validate inputs and generate the exact configs
and command manifest without starting ROS processes. Output directories must initially
be absent or empty so previous results cannot be mixed into a new experiment.

## References

- K Feng, "A New Quaternion-Based Kalman Filter", 2017
- Joan Sola, "Quaternion kinematics for the error-state Kalman filter", 2017
- Daniel Choukroun et al, "A Novel Quaternion Kalman Filter", 2006
- "An Improved EKF - The Error State Extended Kalman Filter"
- Weikun Zhen, Sam Zeng, and Sebastian Scherer, "Robust Localization and Localizability Estimation with a Rotating Laser Scanner", 2017

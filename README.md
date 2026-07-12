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
| `gnss_input_type` | string | `pose` | GNSS input type: `pose` or `navsatfix` |
| `gnss_pose_topic` | string | `gnss_pose` | `PoseStamped` GNSS input topic |
| `gnss_navsatfix_topic` | string | `/gnss/fix` | `NavSatFix` GNSS input topic when `gnss_input_type: "navsatfix"` |
| `gnss_navsatfix_use_first_fix_as_origin` | bool | true | Use the first valid `NavSatFix` as the local ENU origin |
| `var_odom_xyz` | double | 0.1 | Odometry variance `[m^2]` |
| `var_imu_w` | double | 0.01 | Angular velocity variance `[(deg/sec)^2]` |
| `var_imu_acc` | double | 0.01 | Accelerometer variance `[(m/sec^2)^2]` |
| `use_gnss` | bool | true | Whether GNSS is used |
| `use_odom` | bool | false | Whether odometry is used |
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

## References

- K Feng, "A New Quaternion-Based Kalman Filter", 2017
- Joan Sola, "Quaternion kinematics for the error-state Kalman filter", 2017
- Daniel Choukroun et al, "A Novel Quaternion Kalman Filter", 2006
- "An Improved EKF - The Error State Extended Kalman Filter"
- Weikun Zhen, Sam Zeng, and Sebastian Scherer, "Robust Localization and Localizability Estimation with a Rotating Laser Scanner", 2017

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
- `/tf` (`/base_link` -> `/imu_link`)

Output:
- `/current_pose` (`geometry_msgs/PoseStamped`)

## Params

| Name | Type | Default value | Description |
|---|---|---:|---|
| `pub_period` | int | 10 | Publish period `[ms]` |
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

### NavSatFix input

If your RTK receiver publishes `sensor_msgs/NavSatFix`, set:

```yaml
gnss_input_type: "navsatfix"
gnss_navsatfix_topic: "/gnss/fix"
```

The node converts WGS84 latitude/longitude/altitude to local ENU `PoseStamped` internally and fuses it with the same GNSS variance parameters. By default, the first valid fix becomes the ENU origin, so `/initial_pose` should use the same local frame. To use a fixed origin, set `gnss_navsatfix_use_first_fix_as_origin: false` and provide `gnss_navsatfix_origin_latitude`, `gnss_navsatfix_origin_longitude`, and `gnss_navsatfix_origin_altitude`.

## References

- K Feng, "A New Quaternion-Based Kalman Filter", 2017
- Joan Sola, "Quaternion kinematics for the error-state Kalman filter", 2017
- Daniel Choukroun et al, "A Novel Quaternion Kalman Filter", 2006
- "An Improved EKF - The Error State Extended Kalman Filter"
- Weikun Zhen, Sam Zeng, and Sebastian Scherer, "Robust Localization and Localizability Estimation with a Rotating Laser Scanner", 2017

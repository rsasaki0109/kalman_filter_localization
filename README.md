# Kalman Filter Localization

A GNSS/IMU/odometry error-state EKF for ROS 2. It estimates position, velocity,
attitude, and IMU biases.

![Localization pipeline](docs/images/localization_pipeline.svg)

## Features

- GNSS pose / `NavSatFix` / Doppler velocity
- Gyroscope and accelerometer bias estimation
- GNSS NIS gating and Huber/Cauchy robust losses
- GNSS antenna lever arm and short-delay compensation
- NHC, ZUPT, and ZIHR
- Continuous-time process noise and second-order discretization
- CSV evaluation and UrbanNav Tokyo ablation tools

## Build and run

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch kalman_filter_localization ekf.launch.py
```

Main topics:

| Direction | Topic | Type |
|---|---|---|
| input | `/ekf_localization/initial_pose` | `geometry_msgs/PoseStamped` |
| input | `/gnss_pose` | `geometry_msgs/PoseStamped` |
| input | `/sensing/imu/imu_data` | `sensor_msgs/Imu` |
| output | `/ekf_localization/current_pose` | `geometry_msgs/PoseStamped` |
| output | `/ekf_localization/current_odometry` | `nav_msgs/Odometry` |

See [`ekf.yaml`](kalman_filter_localization_ros2/param/ekf.yaml) for the default
parameters and [`param/profiles`](kalman_filter_localization_ros2/param/profiles) for
dataset-specific configurations.

## GNSS input

`PoseStamped` is the default input. To use `NavSatFix` directly:

```yaml
gnss_input_type: "navsatfix"
gnss_navsatfix_topic: "/gnss/fix"
gnss_navsatfix_use_first_fix_as_origin: true
gnss_navsatfix_use_position_covariance: true
```

Configure the antenna offset with `gnss_lever_arm_{x,y,z}`. Known short delays use
`compensate_gnss_delay` and `gnss_time_offset_sec`.

## Evaluation

```bash
ros2 run kalman_filter_localization evaluate_localization \
  --estimate-csv result.csv \
  --reference-csv ground_truth.csv \
  --output-json metrics.json \
  --output-csv errors.csv
```

CSV files use `stamp,x,y,z,yaw`. The evaluator reports RMSE, maximum 3D error, and
match ratio, with optional acceptance thresholds for CI.

## UrbanNav Tokyo ablation

![UrbanNav Odaiba ablation](docs/images/urbannav_odaiba_ablation.svg)

The Odaiba u-blox RTK solution contains GNSS outages of up to 81.8 seconds. With the
initial parameters, the robust profile reduced 3D RMSE by 12.5% versus baseline. The
NHC and full profiles require further tuning.

Convert the official CSV files and RTKLIB solution into a ROS 2 bag:

```bash
ros2 run kalman_filter_localization prepare_urbannav_tokyo \
  --imu-csv Odaiba/imu.csv \
  --reference-csv Odaiba/reference.csv \
  --rtklib-pos odaiba_ublox_rtk.pos \
  --output-bag odaiba_input_bag \
  --output-reference-csv odaiba_reference.csv
```

Run all four profiles and generate CSV/Markdown comparisons:

```bash
share="$(ros2 pkg prefix kalman_filter_localization)/share/kalman_filter_localization"
ros2 run kalman_filter_localization run_urbannav_ablation \
  --input-bag odaiba_input_bag \
  --reference-csv odaiba_reference.csv \
  --output-dir results/urbannav_tokyo \
  --base-profile "$share/param/profiles/urbannav_tokyo_tuned.yaml" \
  --profiles-dir "$share/param/profiles"
```

The output contains merged YAML, trajectory CSV, bag, and logs for each profile, plus
`comparison.csv`, `comparison.md`, and `manifest.json`.

## Test

```bash
colcon test
colcon test-result --verbose
```

## References

- Joan Sola, *Quaternion kinematics for the error-state Kalman filter*, 2017
- K. Feng, *A New Quaternion-Based Kalman Filter*, 2017
- [UrbanNav Dataset](https://github.com/IPNL-POLYU/UrbanNavDataset)

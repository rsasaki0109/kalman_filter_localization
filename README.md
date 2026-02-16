# Kalman Filter Localization
This repository contains two packages:

- `kalman_filter_localization` (ROS2): EKF localization node/component using GNSS/IMU/Odometry.
- `kalman_filter_localization_core` (ROS2-free): header-only EKF core library + unit tests.

## Build
```bash
cd /path/to/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to kalman_filter_localization
```

## Test
```bash
cd /path/to/ros2_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select kalman_filter_localization_core kalman_filter_localization --ctest-args --output-on-failure
```

## Using Core Library
```cpp
#include <kalman_filter_localization/core/ekf.hpp>
```

```cmake
find_package(kalman_filter_localization_core REQUIRED)
target_link_libraries(your_target kalman_filter_localization_core::core)
```

## node
ekf_localization_node
- input  
- ekf_localization/initial_pose (geometry_msgs/PoseStamped)  
- ekf_localization/gnss_pose (geometry_msgs/PoseStamped)  
- ekf_localization/imu (sensor_msgs/Imu)  
- ekf_localization/odom (nav_msgs/Odometry)  
- tf (e.g. base_link (robot frame) -> imu_link (imu frame))  
- output  
- ekf_localization/current_pose (geometry_msgs/PoseStamped)

## params

|Name|Type|Default value|Description|
|---|---|---|---|
|pub_period|int|10|publish period[ms]|
|max_imu_dt_sec|double|0.5|maximum IMU dt accepted by prediction update [s]|
|gravity_mps2|double|9.80665|gravity magnitude subtracted from IMU acceleration in +Z [m/s^2] (set 0.0 if IMU acceleration is gravity-compensated)|
|var_gnss_xy|double|0.1|variance of a gnss receiver about position xy[m^2]|
|var_gnss_z|double|0.15|variance of a gnss receiver about position z[m^2]|
|var_odom_xyz|double|0.2|variance of an odometry[m^2]|
|var_imu_w|double|0.01|variance of an angular velocity sensor[(rad/sec)^2]|
|var_imu_acc|double|0.01|variance of an accelerometer[(m/sec^2)^2]|
|use_gnss|bool|true|whether gnss is used or not |
|use_odom|bool|false|whether odom(lo/vo) is used or not |

## Open-Data Benchmark

`tools/` contains scripts to evaluate and tune on open datasets.

1. `tools/record_pose_csv.py`  
   Record `PoseStamped`, `Odometry`, or `Imu` (orientation only) topic to CSV.
2. `tools/evaluate_trajectory.py`  
   Compute ATE-like metrics (3D/XY RMSE, P95, bias) from estimated vs ground-truth CSV.
3. `tools/run_open_data_sweep.py`  
   Run bag playback + EKF parameter grid search and output `summary.csv` (optionally record attitude reference and plot best run).
4. `tools/navsatfix_to_pose.py`  
   Convert `/fix (NavSatFix)` to `/gnss_pose (PoseStamped)` for EKF input/evaluation.
5. `tools/plot_pose_csv.py`  
   Plot XY trajectory (START/GOAL markers) and z+RPY time series. Yaw reference uses attitude reference CSV (if provided), otherwise GT quaternion (if available), otherwise course from GT positions.

Example:

```bash
cd /path/to/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/kalman_filter_localization/tools/run_open_data_sweep.py \
  --bag-path /path/to/open_data_bag \
  --ground-truth-topic /gnss_pose \
  --param-grid-json src/kalman_filter_localization/tools/param_grid_example.json \
  --output-dir /tmp/kfl_benchmark
```

Outputs:
- `/tmp/kfl_benchmark/summary.csv`: all runs + metrics
- `/tmp/kfl_benchmark/ranking_by_rmse_3d.csv`: successful runs sorted by `rmse_3d_m`
- `/tmp/kfl_benchmark/run_xxx/`: per-run logs, CSV, and metrics JSON

For open data that has `/fix` (NavSatFix) instead of `/gnss_pose`, either run the converter first or let the sweep start it automatically with `--enable-navsatfix-to-pose`.

```bash
python3 src/kalman_filter_localization/tools/navsatfix_to_pose.py \
  --input-topic /fix \
  --output-topic /gnss_pose \
  --output-frame-id map
```

For Autoware Istanbul all-sensors bags, `tools/param_grid_istanbul_quick.json` is a good starting point (it fixes `gravity_mps2: 0.0`).

Then run a quick sweep on selected topics:

```bash
python3 src/kalman_filter_localization/tools/run_open_data_sweep.py \
  --bag-path /path/to/eagleye_sample_ros2_no_velodyne \
  --param-grid-json src/kalman_filter_localization/tools/param_grid_quick.json \
  --output-dir /tmp/kfl_benchmark_quick \
  --imu-topic /imu/data_raw \
  --gnss-topic /gnss_pose \
  --ground-truth-topic /gnss_pose \
  --play-topics /imu/data_raw /fix \
  --enable-navsatfix-to-pose \
  --navsatfix-input-topic /fix \
  --navsatfix-output-topic /gnss_pose \
  --play-rate 20.0 \
  --max-runs 4
```

If you also want plots for the best run and an attitude reference, set `--attitude-reference-topic` and `--plot-best`:

```bash
python3 src/kalman_filter_localization/tools/run_open_data_sweep.py \
  --bag-path /path/to/open_data_bag \
  --param-grid-json src/kalman_filter_localization/tools/param_grid_quick.json \
  --output-dir /tmp/kfl_benchmark_quick \
  --imu-topic /sensing/imu/imu_data \
  --gnss-topic /gnss_pose \
  --ground-truth-topic /gnss_pose \
  --attitude-reference-topic /sensing/imu/imu_data \
  --plot-best
```

Plot the resulting CSV for a run:

```bash
python3 src/kalman_filter_localization/tools/plot_pose_csv.py \
  --estimated-csv /tmp/kfl_benchmark_quick/run_003/estimated.csv \
  --ground-truth-csv /tmp/kfl_benchmark_quick/run_003/ground_truth.csv \
  --output-dir /tmp/kfl_benchmark_quick/run_003 \
  --prefix run_003
```

To compare attitude against an IMU-or-INS reference, record the IMU orientation as CSV and pass it as `--attitude-reference-csv`:

```bash
python3 src/kalman_filter_localization/tools/record_pose_csv.py \
  --topic /sensing/imu/imu_data \
  --msg-type imu \
  --output /tmp/imu_orientation.csv
```

```bash
python3 src/kalman_filter_localization/tools/plot_pose_csv.py \
  --estimated-csv /tmp/kfl_benchmark_quick/run_003/estimated.csv \
  --ground-truth-csv /tmp/kfl_benchmark_quick/run_003/ground_truth.csv \
  --attitude-reference-csv /tmp/imu_orientation.csv \
  --output-dir /tmp/kfl_benchmark_quick/run_003 \
  --prefix run_003_with_imu_ref
```

## Dataset Profiles

Tuned profile files are available under `kalman_filter_localization_ros2/param/profiles/`.

- `eagleye_sample_ros2_no_velodyne.yaml`
  - Derived from open-data sweep on `eagleye_sample_ros2_no_velodyne`
  - Key tuned values:
    - `var_imu_w: 0.005`
    - `var_imu_acc: 0.01`
    - `max_imu_dt_sec: 0.5`
    - `var_gnss_xy: 0.2`
    - `var_gnss_z: 0.1`

- `istanbul_all_sensors_bag.yaml`
  - Profile for Autoware Istanbul all-sensors bags
  - Key values:
    - `gravity_mps2: 0.0` (IMU acceleration appears gravity-compensated)

Example launch with this profile and IMU frame override:

```bash
ros2 launch kalman_filter_localization ekf.launch.py \
  ekf_param_dir:=$(ros2 pkg prefix --share kalman_filter_localization)/param/profiles/eagleye_sample_ros2_no_velodyne.yaml \
  robot_frame_id:=base_link \
  imu_frame_id:=imu
```

## demo

[rosbag demo data(ROS1)](https://drive.google.com/file/d/1CYuip5dApvcF-xrB2f5s8pdBu7MGCDxP/view)

```
rviz2 -d $(ros2 pkg prefix --share kalman_filter_localization)/rviz/ekfl_demo.rviz
```

```
ros2 launch kalman_filter_localization ekf.launch.py
```

```
ros2 topic pub ekf_localization/initial_pose geometry_msgs/PoseStamped '{header: {stamp: {sec: 1532228824, nanosec: 55000000}, frame_id: "map"}, pose: {position: {x: 0, y: 0, z: 10}, orientation: {z: 1, w: 0}}}' --once
```

```
ros2 bag play -s rosbag_v2 test.bag
```


![demo](./images/demo_ekfl.gif)    
blue:initial pose, red:gnss pose, green: fusion pose

## references

- K Feng,"A New Quaternion-Based Kalman Filter",2017
- Joan Solà,"Quaternion kinematics for the error-state Kalman filter",2017
- Daniel Choukroun et al,"A Novel Quaternion Kalman Filter",2006
- An Improved EKF - The Error State Extended Kalman Filter
- Weikun Zhen, Sam Zeng, and Sebastian Scherer. "Robust Localization and Localizability Estimation with a Rotating Laser Scanner" , 2017.

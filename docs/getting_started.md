# Getting Started

## Build

```bash
cd /path/to/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to kalman_filter_localization
source install/setup.bash
```

## Run node

```bash
# start EKF
ros2 launch kalman_filter_localization ekf.launch.py

# publish initial pose first (example)
ros2 topic pub ekf_localization/initial_pose geometry_msgs/PoseStamped \
  '{header: {stamp: {sec: 1532228824, nanosec: 55000000}, frame_id: "map"},
    pose: {position: {x: 0, y: 0, z: 10}, orientation: {z: 1, w: 0}}}' --once
```

## Required inputs / outputs

- Input: `ekf_localization/initial_pose`, `ekf_localization/imu`, `ekf_localization/gnss_pose`, `ekf_localization/odom`
- Output: `ekf_localization/current_pose`
- TF: `robot_frame_id` -> `imu_frame_id` transform must be available (default `base_link` -> `imu_link`)

## Tips

- Set `gravity_mps2` to `0.0` if IMU linear acceleration already includes gravity compensation.
- Use `output_stamp_source` (`latest_input`, `imu`, `ros_time`) depending on your timestamp alignment.

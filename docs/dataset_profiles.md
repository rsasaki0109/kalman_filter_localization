# Dataset Profiles

Tuned profiles are under `kalman_filter_localization_ros2/param/profiles/`.

- `eagleye_sample_ros2_no_velodyne.yaml`
  - tuned values (example): `var_imu_w`, `var_imu_acc`, `max_imu_dt_sec`, `var_gnss_xy`, `var_gnss_z`
- `istanbul_all_sensors_bag.yaml`
  - use for Istanbul all-sensors with IMU orientation fusion
  - key: `gravity_mps2: 0.0`, `use_imu_orientation: true`
  - current tuned seed: `var_imu_orientation_rpy: 0.001`, `var_imu_w: 0.02`, `var_imu_acc: 0.05`, `var_gnss_xy: 0.02`, `max_imu_dt_sec: 0.5`
- `istanbul_all_sensors_bag4_6.yaml`
  - use for Istanbul bags 4-6 when you want the local orientation optimum plus GNSS-derived velocity correction
  - key deltas from shared seed: `var_imu_orientation_rpy: 0.0015`, `use_gnss_velocity: true`, `propagate_gnss_velocity_cross_state: true`, `var_gnss_velocity_xy: 0.02`, `min_gnss_velocity_distance_m: 0.05`
- `istanbul_all_sensors_bag5_6.yaml`
  - legacy alias kept for compatibility with earlier notes
- `istanbul_all_sensors_bag_flat_ground.yaml`
  - fallback profile: flat-ground + GNSS course yaw mode
  - key: `use_flat_ground: true`, `use_gnss_course_yaw: true`

Choose the Istanbul profile automatically from a bag path:

```bash
python3 src/kalman_filter_localization/tools/select_istanbul_profile.py \
  --bag-path data/istanbul/all-sensors-bag5_compressed
```

Launch with profile:

```bash
ros2 launch kalman_filter_localization ekf.launch.py \
  ekf_param_dir:=$(ros2 pkg prefix --share kalman_filter_localization)/param/profiles/eagleye_sample_ros2_no_velodyne.yaml \
  robot_frame_id:=base_link \
  imu_frame_id:=imu
```

For Istanbul bags, you can wire the selector directly into launch:

```bash
ros2 launch kalman_filter_localization ekf.launch.py \
  ekf_param_dir:=$(python3 src/kalman_filter_localization/tools/select_istanbul_profile.py \
    --bag-path data/istanbul/all-sensors-bag5_compressed) \
  robot_frame_id:=base_link \
  imu_frame_id:=imu
```

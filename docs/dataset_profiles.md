# Dataset Profiles

Tuned profiles are under `kalman_filter_localization_ros2/param/profiles/`.

- `eagleye_sample_ros2_no_velodyne.yaml`
  - tuned values (example): `var_imu_w`, `var_imu_acc`, `max_imu_dt_sec`, `var_gnss_xy`, `var_gnss_z`
- `istanbul_all_sensors_bag.yaml`
  - use for Istanbul all-sensors with IMU orientation fusion
  - key: `gravity_mps2: 0.0`, `use_imu_orientation: true`
- `istanbul_all_sensors_bag_flat_ground.yaml`
  - fallback profile: flat-ground + GNSS course yaw mode
  - key: `use_flat_ground: true`, `use_gnss_course_yaw: true`

Launch with profile:

```bash
ros2 launch kalman_filter_localization ekf.launch.py \
  ekf_param_dir:=$(ros2 pkg prefix --share kalman_filter_localization)/param/profiles/eagleye_sample_ros2_no_velodyne.yaml \
  robot_frame_id:=base_link \
  imu_frame_id:=imu
```

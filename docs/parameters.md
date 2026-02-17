# Parameters

|Name|Type|Default value|Description|
|---|---|---|---|
|pub_period|int|10|publish period [ms]|
|max_imu_dt_sec|double|0.5|maximum IMU dt accepted by prediction update [s]|
|gravity_mps2|double|9.80665|gravity magnitude subtracted from IMU acceleration in +Z [m/s^2] (set 0.0 if IMU acceleration is gravity-compensated)|
|var_gnss_xy|double|0.1|variance of a GNSS receiver about position XY [m^2]|
|var_gnss_z|double|0.15|variance of a GNSS receiver about position Z [m^2]|
|var_odom_xyz|double|0.2|variance of odometry [m^2]|
|var_imu_w|double|0.01|variance of angular velocity sensor [(rad/s)^2]|
|var_imu_acc|double|0.01|variance of accelerometer [(m/s^2)^2]|
|use_imu_orientation|bool|false|fuse `Imu.orientation` as an attitude observation |
|use_imu_orientation_covariance|bool|true|use `Imu.orientation_covariance` when valid|
|var_imu_orientation_rpy|double|0.01|fallback variance for IMU orientation observation [rad^2]|
|use_flat_ground|bool|false|enable flat-ground pseudo-observation for roll/pitch|
|var_flat_ground_rp|double|0.03|pseudo-observation variance for roll/pitch [rad^2]|
|use_gnss_course_yaw|bool|false|enable yaw updates from GNSS course (consecutive GNSS positions)|
|var_gnss_course_yaw|double|0.05|GNSS-course yaw variance [rad^2]|
|min_gnss_course_distance_m|double|1.0|minimum GNSS displacement for course yaw [m]|
|min_gnss_course_speed_mps|double|0.5|minimum speed for course yaw [m/s]|
|max_gnss_course_dt_sec|double|1.0|maximum GNSS dt for course yaw [s]|
|max_gnss_course_dyaw_rad|double|3.14159|maximum allowed GNSS-course yaw innovation before skip [rad]|
|use_gnss|bool|true|enable GNSS pose update|
|use_odom|bool|false|enable odom update|
|output_stamp_source|string|latest_input|timestamp source: `latest_input`, `imu`, `ros_time`|

## Parameter files

- Core profile: `kalman_filter_localization_ros2/param/ekf.yaml`
- Open-data tuned profiles: `kalman_filter_localization_ros2/param/profiles/`

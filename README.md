# Kalman Filter Localization  
Kalman Filter Localization  is a ros2 package of Kalman Filter Based Localization in 3D using GNSS/IMU/Odom(Visual Odom/Lidar Odom).

## node
kalman_filter_localization
- input  
/gnss_pose  (geometry_msgs/PoseStamed)   
/imu  (sensor_msgs/Imu)  
/odom (geometry_msgs/PoseStamped)  
- output  
/curent_pose (PoseStamped)

## params

|Name|Type|Default value|Description|
|---|---|---|---|
|var_gnss|double|0.1|variance of a gnss receiver[m]|
|var_odom|double|0.1|variance of an odometry[m]|
|var_imu_w|double|0.01|variance of an angular velocity sensor[deg/sec]|
|var_imu_acc|double|0.01|variance of an accelerometer[m^2/sec]|
|use_gnss|bool|true|whether gnss is used or not |
|use_odom|bool|false|whether odom(lo/vo) is used or not |

## demo

[rosbag demo data](https://drive.google.com/file/d/1CYuip5dApvcF-xrB2f5s8pdBu7MGCDxP/view)

```
ros2 bag play -s rosbag_v2 test.bag
```

```
ros2 topic pub ekf_localization/initial_pose geometry_msgs/PoseStamped '{header: {stamp: {sec: 1532228824, nanosec: 55000000}, frame_id: "map"}, pose: {position: {x: 0, y: 0, z: 10}, orientation: {z: 1, w: 0}}}' --once
```

```
ros2 run kalman_filter_localization ekf_localization_node /ekf_localization/gnss_pose:=/gnss_pose /ekf_localization/imu:=/imu
```

![demo](./images/demo_ekfl.gif)    
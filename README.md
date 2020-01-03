# Kalman Filter Localization  
Kalman Filter Localization  is a ros2 package of Kalman Filter Based Localization in 3D using GNSS/IMU/(Odom/VO/LO).

## node
kalman_filter_localization
- input 
/gnss_pose  (PoseStamed)   
/imu  (Imu)  
/odom (PoseStamped)
- output  
/curent_pose (PoseStamped)

## params


## demo
```
# Download the example bag
wget -P ???
``` 

```
ros2 bag play -s rosbag_v2 test.bag
```

```
ros2 topic pub ekf_localization/initial_pose geometry_msgs/PoseStamped '{header: {stamp: {sec: 1532228824, nanosec: 55000000}, frame_id: "map"}, pose: {position: {x: 0, y: 0}, orientation: {z: 0, w: 1}}}'
```

```
ros2 run kalman_filter_localization ekf_localization_node /ekf_localization/gnss_pose:=/gnss_pose /ekf_localization/imu:=/imu
```

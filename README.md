# Kalman Filter Localization  
Kalman Filter Localization  is a ros2 package of Kalman Filter Based Localization in 3D using GNSS/IMU/(Odom/VO/LO).

## node
kalman_filter_localization
- input 
GNSS  (PoseStamed) 
/imu  (Imu)
/odom (PoseStamped)
- output 
/curent_pose (PoseStampeWithCovarience)

## params


## demo
```
# Download the example bag
wget -P ???
``` 

```
ros2 bag play ???
```

```
ros2 run kalman_filter_localization ekf_localization
```
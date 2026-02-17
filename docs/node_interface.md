# Node Interface

## Topics

- `ekf_localization/initial_pose` (`geometry_msgs/PoseStamped`)
- `ekf_localization/imu` (`sensor_msgs/Imu`)
- `ekf_localization/gnss_pose` (`geometry_msgs/PoseStamped`)
- `ekf_localization/odom` (`nav_msgs/Odometry`)
- `ekf_localization/current_pose` (`geometry_msgs/PoseStamped`) *(published)*

## Core library usage

```cpp
#include <kalman_filter_localization/core/ekf.hpp>
```

```cmake
find_package(kalman_filter_localization_core REQUIRED)
target_link_libraries(your_target kalman_filter_localization_core::core)
```

The core is intentionally ROS2-free, so it can be reused in offline tests and scripts.

# Demo

A sample RViz and launch command:

```bash
rviz2 -d $(ros2 pkg prefix --share kalman_filter_localization)/rviz/ekfl_demo.rviz
ros2 launch kalman_filter_localization ekf.launch.py
```

Demo bag (ROS1): [rosbag data](https://drive.google.com/file/d/1CYuip5dApvcF-xrB2f5s8pdBu7MGCDxP/view)

![demo](../images/demo_ekfl.gif)

- blue: initial pose
- red: GNSS pose
- green: fusion pose

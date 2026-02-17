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

## Open-data result snapshots

- Open-data benchmark snapshot (sample results): [Open Data Results](open_data_results.md)

### Istanbul all-sensors bag (course-yaw tuned)

![Course yaw trajectory](results/open_data/istanbul_bag1_course_yaw/trajectory_xy.png)

![Course yaw z / RPY](results/open_data/istanbul_bag1_course_yaw/timeseries_z_rpy.png)

### Istanbul all-sensors bag (flat-ground tuned)

![Flat-ground trajectory](results/open_data/istanbul_bag1_flat_ground/trajectory_xy.png)

![Flat-ground z / RPY](results/open_data/istanbul_bag1_flat_ground/timeseries_z_rpy.png)

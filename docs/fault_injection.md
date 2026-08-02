# Fault-injection checks

The ROS-free fault monitor is covered by
`kalman_filter_localization_core/test/test_sensor_fault_monitor.cpp`. It checks
independent sensor state, trip threshold, hold time, probe recovery, reset, and
invalid configuration.

For a node-level check, launch the estimator with:

```bash
ros2 run kalman_filter_localization ekf_localization_node --ros-args \
  -p enable_sensor_fault_isolation:=true \
  -p sensor_fault_trip_count:=3 \
  -p sensor_fault_hold_sec:=30.0 \
  -p max_gnss_position_innovation_m:=0.1
```

Publish one valid initial pose, then three GNSS poses with a position more than
0.1 m from the estimate. The expected status is
`gnss_isolated=true`, `mode=OUTAGE`, and `gnss_fault_events=1`. No other sensor
isolation flag should change. After the hold interval, a valid GNSS probe can
clear the isolation.

This is an operational safety check, not a substitute for dataset regression:
open-sky, urban, outage, and reacquisition benchmarks remain required before a
profile is accepted.

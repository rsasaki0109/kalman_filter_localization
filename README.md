# Kalman Filter Localization

This repository contains:

- `kalman_filter_localization` (ROS2): EKF node (`ekf_localization_node`) using GNSS/IMU/Odometry.
- `kalman_filter_localization_core` (ROS2-free): header-only EKF core library + unit tests.

## Quick Start

```bash
cd /path/to/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to kalman_filter_localization
source install/setup.bash
```

## Docs

- [Getting Started](docs/getting_started.md)
- [Node Interface](docs/node_interface.md)
- [Parameters](docs/parameters.md)
- [Open Data Workflow](docs/open_data_workflow.md)
- [Dataset Profiles](docs/dataset_profiles.md)
- [Demo](docs/demo.md)
- [Open Data Results](docs/open_data_results.md)
- [References](docs/references.md)

## Test

```bash
cd /path/to/ros2_ws
colcon test --packages-select kalman_filter_localization_core kalman_filter_localization --ctest-args --output-on-failure
```

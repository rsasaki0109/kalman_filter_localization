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
- [Agent Design Loop](docs/agent_design_loop.md)
- [Cycle Log Template](docs/cycle_log_template.md)
- [KF Performance Report (HTML)](docs/agent_design_loop_report.html)
- [Node Interface](docs/node_interface.md)
- [Parameters](docs/parameters.md)
- [Open Data Workflow](docs/open_data_workflow.md)
- [Dataset Profiles](docs/dataset_profiles.md)
- [Demo](docs/demo.md)
- [Open Data Results](docs/open_data_results.md)
- [References](docs/references.md)

## KF Report Update

```bash
python3 src/kalman_filter_localization/tools/update_agent_design_loop_report.py \
  --summary-csv src/kalman_filter_localization/docs/results/open_data/istanbul_bag1_course_yaw/summary.csv \
  --summary-csv src/kalman_filter_localization/docs/results/open_data/istanbul_bag3_course_yaw/summary.csv
```

## Agent Cycle (Sweep + Report)

```bash
src/kalman_filter_localization/tools/run_agent_cycle.sh \
  data/istanbul/all-sensors-bag1_compressed \
  /tmp/kfl_cycle01 \
  src/kalman_filter_localization/tools/param_grid_istanbul_quick.json \
  cycle01 \
  src/kalman_filter_localization/docs/results/agent_loop_summary_paths.txt
```

## Test

```bash
cd /path/to/ros2_ws
colcon test --packages-select kalman_filter_localization_core kalman_filter_localization --ctest-args --output-on-failure
```

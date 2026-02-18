# Open Data Results

## Istanbul `all-sensors-bag1_uncompressed` (snapshot)

Results here are generated from `run_open_data_sweep.py`/suite runs and stored in this repository so they can be referenced from docs without re-running.

- Snapshot date: 2026-02-18
- Run source: `/tmp/kfl_istanbul_bag1_rerun_20260218/...`

### Course-yaw tuning case (GNSS course yaw enabled)

| Metric | Value |
| --- | --- |
| RMSE 3D [m] | 0.03423 |
| RMSE XY [m] | 0.03312 |
| RMSE 3D (no bias) [m] | 0.03262 |
| Bias Z [m] | 0.00149 |
| Attitude angle RMSE [deg] | 0.00870 |
| Roll RMSE [deg] | 0.00561 |
| Pitch RMSE [deg] | 0.00649 |
| Yaw RMSE [deg] | 0.00131 |
| Best run id | `rerun_006` |
| Reference | `gt_quat` |

**Best parameters (rerun_006)**  
`use_imu_orientation = true`, `use_imu_orientation_covariance = true`,  
`var_imu_orientation_rpy = 0.001`, `var_imu_w = 0.005`, `var_imu_acc = 0.05`,  
`var_gnss_xy = 0.05`, `var_gnss_z = 0.1`, `max_imu_dt_sec = 1.0`

![Course-yaw trajectory](results/open_data/istanbul_bag1_course_yaw/trajectory_xy.png)

![Course-yaw z+RPY](results/open_data/istanbul_bag1_course_yaw/timeseries_z_rpy.png)

- [summary.csv](results/open_data/istanbul_bag1_course_yaw/summary.csv)
- [ranking_by_rmse_3d.csv](results/open_data/istanbul_bag1_course_yaw/ranking_by_rmse_3d.csv)
- [suite_summary.csv](results/open_data/istanbul_bag1_course_yaw/suite_summary.csv)

### Istanbul `all-sensors-bag3_compressed` (bag3 focused)

| Metric | Value |
| --- | --- |
| RMSE 3D [m] | 7.68750 |
| RMSE XY [m] | 7.65635 |
| RMSE 3D (no bias) [m] | 6.37190 |
| Bias Z [m] | 0.36961 |
| Attitude angle RMSE [deg] | 0.29720 |
| Roll RMSE [deg] | 0.12326 |
| Pitch RMSE [deg] | 0.13354 |
| Yaw RMSE [deg] | 0.23121 |
| Best run id | `run_004` |
| Reference | `gt_quat` |

**Best parameters (`run_004`)**  
`use_imu_orientation = true`, `use_imu_orientation_covariance = true`,  
`var_imu_orientation_rpy = 0.0015`, `var_imu_w = 0.01`, `var_imu_acc = 0.01`,  
`var_gnss_xy = 0.05`, `var_gnss_z = 0.1`, `max_imu_dt_sec = 1.0`

![bag3 trajectory](results/open_data/istanbul_bag3_course_yaw/trajectory_xy.png)

![bag3 z+RPY](results/open_data/istanbul_bag3_course_yaw/timeseries_z_rpy.png)

- [summary.csv](results/open_data/istanbul_bag3_course_yaw/summary.csv)
- [ranking_by_rmse_3d.csv](results/open_data/istanbul_bag3_course_yaw/ranking_by_rmse_3d.csv)
- [ranking_by_rmse_3d_nobias.csv](results/open_data/istanbul_bag3_course_yaw/ranking_by_rmse_3d_nobias.csv)
- [suite_summary.csv](results/open_data/istanbul_bag3_course_yaw/suite_summary.csv)


## Flat-ground tuning case (GNSS course yaw off)

| Metric | Value |
| --- | --- |
| RMSE 3D [m] | 0.04817 |
| RMSE XY [m] | 0.01647 |
| RMSE 3D (no bias) [m] | 0.02408 |
| Bias Z [m] | -0.0411 |
| Attitude angle RMSE [deg] | 4.86 |
| Roll RMSE [deg] | 0.40 |
| Pitch RMSE [deg] | 2.69 |
| Yaw RMSE [deg] | 4.03 |
| Best run id | `run_004` |
| Reference | `gt_quat` |

**Best parameters (run_004)**  
`use_flat_ground = true`, `use_imu_orientation = false`, `use_gnss_course_yaw = false`,  
`var_gnss_xy = 0.2`, `var_gnss_z = 0.1`, `var_imu_w = 0.005`, `var_imu_acc = 0.01`

![Flat-ground trajectory](results/open_data/istanbul_bag1_flat_ground/trajectory_xy.png)

![Flat-ground z+RPY](results/open_data/istanbul_bag1_flat_ground/timeseries_z_rpy.png)

- [suite_summary.csv](results/open_data/istanbul_bag1_flat_ground/suite_summary.csv)
- [run_summary.csv](results/open_data/istanbul_bag1_flat_ground/run_summary.csv)
- [ranking_by_rmse_3d.csv](results/open_data/istanbul_bag1_flat_ground/ranking_by_rmse_3d.csv)

## Reproduce from scratch

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/kalman_filter_localization/tools/run_open_data_sweep.py \
  --bag-path data/istanbul/all-sensors-bag1_uncompressed \
  --param-grid-json src/kalman_filter_localization/tools/param_grid_istanbul_quick.json \
  --output-dir /tmp/kfl_open_data_bag1_demo \
  --ground-truth-topic /ins_pose \
  --imu-topic /sensing/imu/imu_data \
  --gnss-topic /gnss_pose \
  --play-topics /sensing/imu/imu_data /gnss/fix /lvx_client/gsof/ins_solution_49
```

The produced files are:
- `summary.csv`
- `ranking_by_rmse_3d.csv`
- `ranking_by_rmse_3d_nobias.csv`
- `open_data_report_*.html` (single-bag sweep)
- `best_plots/*`

Copy them into `docs/results/open_data/<case>/` if you want to publish in the repository.

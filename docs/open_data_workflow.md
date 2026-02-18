# Open Data Workflow

`tools/` contains scripts to evaluate and tune on open datasets.

## Scripts

1. `tools/record_pose_csv.py`  
   Record `PoseStamped`, `Odometry`, or `Imu`(orientation only) topic to CSV.
2. `tools/evaluate_trajectory.py`  
   Compute 3D/XY RMSE, P95, bias from estimated and ground-truth CSV.
3. `tools/run_open_data_sweep.py`  
   Bag playback + parameter sweep.
4. `tools/run_istanbul_suite.py`  
   Convenience wrapper for multiple Istanbul bags and report generation.
5. `tools/navsatfix_to_pose.py`  
   `/fix` -> `/gnss_pose` converter.
6. `tools/applanix_nav_solution_to_pose.py`  
   Applanix INS converter (requires `applanix_msgs`).
7. `tools/gsof49_to_imu.py`  
   Convert Applanix GSOF49 to IMU.
8. `tools/plot_pose_csv.py`  
   Plot XY trajectory and `z+RPY` time series.

Representative published results are shown in:

- [Open Data Results](open_data_results.md)

## Minimal run

```bash
cd /path/to/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/kalman_filter_localization/tools/run_open_data_sweep.py \
  --bag-path /path/to/open_data_bag \
  --ground-truth-topic /gnss_pose \
  --param-grid-json src/kalman_filter_localization/tools/param_grid_example.json \
  --output-dir /tmp/kfl_benchmark \
  --initial-yaw-source-topic /gnss_pose \
  --initial-yaw-source-msg-type pose_stamped \
  --initial-yaw-timeout-sec 8.0
```

Output files:

- `summary.csv`: all runs + metrics
- `ranking_by_rmse_3d.csv`
- `ranking_by_rmse_3d_nobias.csv`
- `open_data_report_*.html` (single-bag sweep)

If you need IMU-based attitude comparison, add `--attitude-reference-topic ... --attitude-reference-msg-type imu` and plot with `--plot-best`.

### 姿勢を POS-LV に寄せる

`/sensing/imu/imu_data` の姿勢がノイズで安定しない場合は、`/lvx_client/gsof/ins_solution_49` から
直接 `Imu` を作って EKF の IMU 入力に使うのが有効です。

```bash
python3 src/kalman_filter_localization/tools/gsof49_to_imu.py \
  --input-topic /lvx_client/gsof/ins_solution_49 \
  --output-topic /ins_imu \
  --output-frame-id base_link \
  --output-mode ros
```

その後、同じ条件で比較するときは `imu-topic` を `/ins_imu` に切り替え、姿勢参照は POS-LV の姿勢 (`/ins_pose`) を使います。

```bash
python3 src/kalman_filter_localization/tools/run_open_data_sweep.py \
  --imu-topic /ins_imu \
  --gnss-topic /gnss_pose \
  --ground-truth-topic /ins_pose \
  --attitude-reference-topic /ins_pose \
  --attitude-reference-msg-type pose_stamped \
  --attitude-min-speed-mps 0.0 \
  --initial-yaw-source-topic /ins_pose \
  --initial-yaw-source-msg-type pose_stamped \
  --initial-yaw-timeout-sec 8.0 \
  --plot-best \
  --enable-applanix-to-imu \
  --applanix-imu-output-topic /ins_imu \
  --applanix-imu-output-mode ros \
  --enable-applanix-to-pose \
  --applanix-input-topic /lvx_client/gsof/ins_solution_49 \
  --applanix-output-topic /ins_pose \
  --applanix-origin-navsatfix-topic /gnss/fix \
  --param-grid-json src/kalman_filter_localization/tools/param_grid_istanbul_quick.json \
  --output-dir /tmp/kfl_istanbul_ins_att
```

必要なら `param_grid_istanbul_quick.json` / `param_grid_istanbul_flat_ground_quick.json` に
`var_imu_orientation_rpy` を追加してチューニングすると、さらに姿勢を寄せやすくなります。

## Istanbul example

- If needed, build applanix msgs once:

```bash
git clone https://github.com/autowarefoundation/applanix.git src/applanix
colcon build --symlink-install --packages-select applanix_msgs
```

- Run quick INS-ground-truth sweep:

```bash
ROS_LOG_DIR=/tmp/ros2_logs python3 src/kalman_filter_localization/tools/run_open_data_sweep.py \
  --bag-path data/istanbul/all-sensors-bag1_compressed \
  --param-grid-json src/kalman_filter_localization/tools/param_grid_istanbul_quick.json \
  --output-dir /tmp/kfl_istanbul_bag1_ins_gt \
  --imu-topic /sensing/imu/imu_data \
  --gnss-topic /gnss_pose \
  --ground-truth-topic /ins_pose \
  --play-topics /sensing/imu/imu_data /gnss/fix /lvx_client/gsof/ins_solution_49 \
  --enable-navsatfix-to-pose \
  --navsatfix-input-topic /gnss/fix \
  --navsatfix-output-topic /gnss_pose \
  --enable-applanix-to-pose \
  --applanix-input-topic /lvx_client/gsof/ins_solution_49 \
  --applanix-output-topic /ins_pose \
  --applanix-origin-navsatfix-topic /gnss/fix \
  --applanix-orientation-mode ros \
  --enable-applanix-to-imu \
  --applanix-imu-output-topic /ins_imu \
  --applanix-imu-output-mode ros
```

## Multi-bag suite

```bash
ROS_LOG_DIR=/tmp/ros2_logs python3 src/kalman_filter_localization/tools/run_istanbul_suite.py \
  --output-dir /tmp/kfl_istanbul_suite \
  --param-grid-json src/kalman_filter_localization/tools/param_grid_istanbul_quick.json \
  --ground-truth ins_pose \
  --initial-yaw-source-topic /ins_pose
```

For `--ground-truth ins_pose`, suite run automatically starts
`tools/gsof49_to_imu.py` and uses `/ins_imu` as EKF IMU input plus the attitude reference.

Output files:

- `istanbul_suite_summary_<timestamp>.csv`
- `open_data_suite_report_*.html`
- `best_plots/*`

## Timestamp notes

Some open datasets mix timestamp domains (e.g. Unix epoch vs GPS TOW).
Scripts support `--time-normalize` / `--time-align` options.

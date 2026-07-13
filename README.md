# Kalman Filter Localization

ROS 2向けのGNSS/IMU/odometry Error-State EKFです。姿勢・速度・位置とIMU biasを推定します。

![Localization pipeline](docs/images/localization_pipeline.svg)

## Features

- GNSS pose / `NavSatFix` / Doppler velocity
- IMU gyro・加速度bias推定
- GNSS外れ値のNIS gate、Huber/Cauchy loss
- GNSS antenna lever arm・短時間delay補償
- NHC、ZUPT、ZIHR
- 連続時間process noiseと2次離散化
- CSV評価、UrbanNav Tokyo ablation実行

## Build and run

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch kalman_filter_localization ekf.launch.py
```

主なtopic:

| Direction | Topic | Type |
|---|---|---|
| input | `/ekf_localization/initial_pose` | `geometry_msgs/PoseStamped` |
| input | `/gnss_pose` | `geometry_msgs/PoseStamped` |
| input | `/sensing/imu/imu_data` | `sensor_msgs/Imu` |
| output | `/ekf_localization/current_pose` | `geometry_msgs/PoseStamped` |
| output | `/ekf_localization/current_odometry` | `nav_msgs/Odometry` |

設定は [`ekf.yaml`](kalman_filter_localization_ros2/param/ekf.yaml)、データセット別設定は
[`param/profiles`](kalman_filter_localization_ros2/param/profiles) にあります。

## GNSS input

`PoseStamped`が標準です。`NavSatFix`を直接使う場合:

```yaml
gnss_input_type: "navsatfix"
gnss_navsatfix_topic: "/gnss/fix"
gnss_navsatfix_use_first_fix_as_origin: true
gnss_navsatfix_use_position_covariance: true
```

antenna offsetは`gnss_lever_arm_{x,y,z}`、既知の短いdelayは
`compensate_gnss_delay`と`gnss_time_offset_sec`で設定します。

## Evaluation

```bash
ros2 run kalman_filter_localization evaluate_localization \
  --estimate-csv result.csv \
  --reference-csv ground_truth.csv \
  --output-json metrics.json \
  --output-csv errors.csv
```

CSV列は`stamp,x,y,z,yaw`です。RMSE、最大3D誤差、match率を出力し、CI用の閾値も指定できます。

## UrbanNav Tokyo ablation

![UrbanNav Odaiba ablation](docs/images/urbannav_odaiba_ablation.svg)

Odaibaのu-blox RTK解には最長81.8秒のGNSS欠測があります。初期パラメータでの結果では、
robust profileがbaselineに対して3D RMSEを12.5%改善しました。NHC/fullは要調整です。

公式CSVとRTKLIB解をROS 2 bagへ変換:

```bash
ros2 run kalman_filter_localization prepare_urbannav_tokyo \
  --imu-csv Odaiba/imu.csv \
  --reference-csv Odaiba/reference.csv \
  --rtklib-pos odaiba_ublox_rtk.pos \
  --output-bag odaiba_input_bag \
  --output-reference-csv odaiba_reference.csv
```

4構成を実行してCSV/Markdown比較表を生成:

```bash
share="$(ros2 pkg prefix kalman_filter_localization)/share/kalman_filter_localization"
ros2 run kalman_filter_localization run_urbannav_ablation \
  --input-bag odaiba_input_bag \
  --reference-csv odaiba_reference.csv \
  --output-dir results/urbannav_tokyo \
  --base-profile "$share/param/profiles/urbannav_tokyo_tuned.yaml" \
  --profiles-dir "$share/param/profiles"
```

出力には各構成の合成YAML、軌跡CSV、bag、ログ、`comparison.csv`、`comparison.md`、
`manifest.json`が含まれます。

## Test

```bash
colcon test
colcon test-result --verbose
```

## References

- Joan Sola, *Quaternion kinematics for the error-state Kalman filter*, 2017
- K. Feng, *A New Quaternion-Based Kalman Filter*, 2017
- [UrbanNav Dataset](https://github.com/IPNL-POLYU/UrbanNavDataset)

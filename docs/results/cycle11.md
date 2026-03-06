# Agent Cycle Log: cycle11

## Meta
- `Cycle ID`: `cycle11`
- `Date`: `2026-03-06`
- `Owner`: `Codex`
- `Focus`: `GT に対して hard bag で残る動的ずれをアルゴリズム側で縮める`

## Problem
- bag5-6 は tuned profile でも GT に対して中盤以降のずれが残っていた。
- 残差分解では bag5 は後半で cross-track が増え、bag1 は along-track 主体だった。
- `GNSS course yaw` を足すだけでは bag5 が `0.070996 -> 0.077067 m` と悪化した。

## Root Cause Hypothesis
- 既存 EKF は `position / velocity / orientation` の 9-state だが、観測側は実質 `GNSS position` と `IMU orientation` に偏っていた。
- hard bag では position update だけでは velocity state の補正が足りず、旋回や加減速で propagation error が残る。
- したがって tuning ではなく、`GNSS 位置差分から world velocity を直接観測する update` が必要だった。

## Change
### Core
- `kalman_filter_localization_core/include/kalman_filter_localization/core/ekf.hpp`
  - `observationUpdateVelocityWithStatus()` を追加
  - velocity state (`vx, vy, vz`) への直接観測更新を追加

### ROS2 component
- `kalman_filter_localization_ros2/src/ekf_localization_component.cpp`
  - consecutive GNSS pose から world velocity を計算する `updateVelocityFromGnss()` を追加
  - 新規パラメータを追加:
    - `use_gnss_velocity`
    - `var_gnss_velocity_xy`
    - `var_gnss_velocity_z`
    - `min_gnss_velocity_distance_m`
    - `max_gnss_velocity_dt_sec`
    - `max_gnss_velocity_innovation_mps`

### Profiles
- shared profile は保守的に `use_gnss_velocity: false` のまま維持
- `istanbul_all_sensors_bag5_6.yaml` は以下を追加
  - `use_gnss_velocity: true`
  - `var_gnss_velocity_xy: 0.02`
  - `var_gnss_velocity_z: 0.5`
  - `min_gnss_velocity_distance_m: 0.05`
  - `max_gnss_velocity_dt_sec: 1.0`
  - `max_gnss_velocity_innovation_mps: 5.0`

## Validation
### Unit / lint
- `colcon test --packages-select kalman_filter_localization_core`
- result: `40 tests, 0 errors, 0 failures, 5 skipped`

### Representative runs
- bag5 velocity grid: `/tmp/kfl_bag5_velocity_grid/summary.csv`
  - best: `run_001`
  - `rmse_3d = 0.045324 m`
- bag1 single-run with velocity update: `/tmp/kfl_bag1_velocity_best/summary.csv`
  - `rmse_3d = 0.033611 m`
- bag6 single-run with velocity update: `/tmp/kfl_bag6_velocity_best/summary.csv`
  - `rmse_3d = 0.056318 m`
- bag6 control (same params, `use_gnss_velocity=false`): `/tmp/kfl_bag6_baseline_current/summary.csv`
  - `rmse_3d = 0.148325 m`

### Integrated validation runner
- output: `/tmp/kfl_profile_validation_velocity/profile_validation_summary.csv`
- report: `/tmp/kfl_profile_validation_velocity/kf_profile_validation_report.html`
- results:
  - bag1 shared: `0.030555 m`
  - bag5 bag5-6 profile: `0.046773 m`
  - bag6 bag5-6 profile: `0.055112 m`

## Outcome
- hard bag の GT ずれは parameter retune ではなく observation model の不足だった。
- `GNSS velocity update` を bag5-6 profile に入れることで:
  - bag5: `0.070996 -> 0.046773 m`
  - bag6: `0.148325 -> 0.055112 m`
- shared bag1 は dedicated change を入れずに `0.030555 m` で維持できた。

## Decision
- `GNSS velocity update` は profile split 前提で採用する。
- shared profile にはまだ広げない。
- bag5-6 dedicated profile にのみ常時有効化する。

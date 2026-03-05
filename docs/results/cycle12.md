# Agent Cycle Log: cycle12

## Meta
- `Cycle ID`: `cycle12`
- `Date`: `2026-03-06`
- `Owner`: `Codex`
- `Focus`: `bag4 を dedicated velocity-update profile 側へ組み込み、selector と運用 validation を閉じる`

## Problem
- cycle11 後の shared validation で bag4 が `0.121449 m`、rerun で `0.128397 m` と threshold `0.12 m` を安定して割ったとは言えなかった。
- 一方で bag5-6 では `GNSS velocity update` が大きく効いていた。

## Probe
- bag4 + shared profile: `/tmp/kfl_profile_validation_bag4_rerun/profile_validation_summary.csv`
  - `rmse_3d = 0.128397 m`
- bag4 + velocity update (`var_imu_orientation_rpy=0.001`): `/tmp/kfl_bag4_velocity_probe/summary.csv`
  - `rmse_3d = 0.057770 m`
- bag4 + bag5-6-equivalent profile (`var_imu_orientation_rpy=0.0015`): `/tmp/kfl_bag4_bag56_profile_probe/summary.csv`
  - `rmse_3d = 0.052777 m`

## Decision
- bag4 も bag5-6 と同じ dedicated velocity-update profile に入れる。
- dedicated profile の適用範囲を `bag5-6` から `bag4-6` に拡張する。

## Changes
- Added: `kalman_filter_localization_ros2/param/profiles/istanbul_all_sensors_bag4_6.yaml`
- Updated: `tools/select_istanbul_profile.py`
  - selector now routes bag4/5/6 to `istanbul_all_sensors_bag4_6.yaml`
- Updated: `tools/run_istanbul_profile_validation.py`
  - validation runner now uses the same bag4-6 split
- Updated docs:
  - `docs/dataset_profiles.md`
  - `docs/open_data_workflow.md`
- Kept: `istanbul_all_sensors_bag5_6.yaml`
  - retained as a legacy alias for compatibility with earlier notes

## Final Validation
- output: `/tmp/kfl_profile_validation_all_bags_cycle12/profile_validation_summary.csv`
- report: `/tmp/kfl_profile_validation_all_bags_cycle12/kf_profile_validation_report.html`

| Bag | Profile | rmse_3d | Threshold | Status |
| --- | --- | --- | --- | --- |
| bag1 | `istanbul_all_sensors_bag.yaml` | `0.032152 m` | `0.05` | pass |
| bag2 | `istanbul_all_sensors_bag.yaml` | `0.046696 m` | `0.06` | pass |
| bag3 | `istanbul_all_sensors_bag.yaml` | `0.054705 m` | `0.08` | pass |
| bag4 | `istanbul_all_sensors_bag4_6.yaml` | `0.068701 m` | `0.12` | pass |
| bag5 | `istanbul_all_sensors_bag4_6.yaml` | `0.032177 m` | `0.09` | pass |
| bag6 | `istanbul_all_sensors_bag4_6.yaml` | `0.054656 m` | `0.15` | pass |

## Outcome
- selector split is now:
  - bag1-3: shared profile
  - bag4-6: dedicated velocity-update profile
- validation is green again with `6/6 pass`
- naming is now aligned with behavior (`bag4_6`)

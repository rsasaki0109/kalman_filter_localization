# Agent Cycle Log: cycle07

## Meta
- `Cycle ID`: `cycle07`
- `Date`: `2026-03-06`
- `Owner`: `Codex`
- `Dataset`: `data/istanbul/all-sensors-bag5_compressed`, `data/istanbul/all-sensors-bag6_compressed`
- `Base Profile`: `kalman_filter_localization_ros2/param/profiles/istanbul_all_sensors_bag.yaml`

## Goal
- `Objective`: bag5-6 の局所最適 `var_imu_orientation_rpy=0.0015` が rerun でも shared profile を上回るか確認し、残るなら専用 profile として固定する
- `Success Criteria`: contemporaneous rerun で bag5 と bag6 の両方が shared profile より改善する
- `Constraints`: `var_imu_w=0.02`, `var_imu_acc=0.05`, `var_gnss_xy=0.02`, `max_imu_dt_sec=0.5` は固定し、`var_imu_orientation_rpy` のみ比較する
- `Stop Condition`: rerun 結果から bag5-6 専用 profile の追加可否を判断できる

## Execution
### Compared Configs
- `shared current`
  - `var_imu_orientation_rpy=0.001`
  - `var_gnss_xy=0.02`
- `bag5-6 local`
  - `var_imu_orientation_rpy=0.0015`
  - `var_gnss_xy=0.02`

### Commands
```bash
python3 src/kalman_filter_localization/tools/run_open_data_sweep.py \
  --bag-path data/istanbul/all-sensors-bag5_compressed \
  --param-grid-json /tmp/param_grid_istanbul_shared_current.json \
  --output-dir /tmp/kfl_cycle07_bag5_current \
  --imu-topic /ins_imu \
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
  --enable-applanix-to-imu \
  --applanix-imu-output-topic /ins_imu \
  --applanix-imu-output-mode ros \
  --initial-yaw-source-topic /ins_pose \
  --initial-yaw-source-msg-type pose_stamped
```

同じ比較を bag5 local, bag6 current, bag6 local にも実行した。

### Changed Files
- `src/kalman_filter_localization/kalman_filter_localization_ros2/param/profiles/istanbul_all_sensors_bag5_6.yaml`: bag5-6 専用 profile を追加
- `src/kalman_filter_localization/docs/dataset_profiles.md`: 新 profile を docs に追加
- `src/kalman_filter_localization/docs/results/cycle07.md`: rerun 比較結果を記録

### Artifacts
- Bag5 current: `/tmp/kfl_cycle07_bag5_current/summary.csv`
- Bag5 local: `/tmp/kfl_cycle07_bag5_local/summary.csv`
- Bag6 current: `/tmp/kfl_cycle07_bag6_current/summary.csv`
- Bag6 local: `/tmp/kfl_cycle07_bag6_local/summary.csv`

## Self-Evaluation
### Rerun Comparison
| Bag | Shared Current | Bag5-6 Local | Delta | Judgment |
| --- | --- | --- | --- | --- |
| `bag5` | `0.076233 m` | `0.071789 m` | `-0.004444 m` | local wins |
| `bag6` | `0.135246 m` | `0.126397 m` | `-0.008849 m` | local wins |

### Aggregate
| Metric | Shared Current | Bag5-6 Local | Delta |
| --- | --- | --- | --- |
| Mean `rmse_3d` over bag5-6 | `0.105740 m` | `0.099093 m` | `-0.006647 m` |

### Observations
- bag5 と bag6 の contemporaneous rerun でも `var_imu_orientation_rpy=0.0015` が両方勝った。
- 改善幅は大きくはないが、run-to-run variance を考慮しても一方向に揃っている。
- bag4 は shared profile と同じ `0.001` を好むため、専用 profile を切る対象は bag5-6 のみに絞るのが妥当。

### Issues
- bag6 は rerun 自体の揺れが大きく、絶対値はまだ安定していない。
- bag5-6 profile は汎用 default ではなく、dataset-group 専用の補助 profile として扱うべき。

### Root Cause Hypotheses
- bag5-6 は shared profile より少し緩い orientation fallback の方が translation と整合しやすい。
- ただしこの差分は二次調整で、主因は cycle05 で確定した `var_gnss_xy=0.02` 側にある。

## Next Cycle
- `Chosen Changes`: shared profile は維持し、bag5-6 専用 profile を追加する
- `Deferred Changes`: bag1-2 専用 profile の切り出し
- `Preparation Needed`: bag5-6 専用 profile を使う運用では、再現性確認 run を定期的に挟む

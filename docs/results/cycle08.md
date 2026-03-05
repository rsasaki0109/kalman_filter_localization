# Agent Cycle Log: cycle08

## Meta
- `Cycle ID`: `cycle08`
- `Date`: `2026-03-06`
- `Owner`: `Codex`
- `Dataset`: `data/istanbul/all-sensors-bag1_compressed`, `data/istanbul/all-sensors-bag2_compressed`
- `Base Profile`: `kalman_filter_localization_ros2/param/profiles/istanbul_all_sensors_bag.yaml`

## Goal
- `Objective`: bag1-2 が shared current (`var_gnss_xy=0.02`) より local candidate (`var_gnss_xy=0.05`) を一貫して好むか確認し、必要なら bag1-2 専用 profile を切る
- `Success Criteria`: bag1 と bag2 の両方で local candidate が shared current を上回る
- `Constraints`: `var_imu_orientation_rpy=0.001`, `var_imu_w=0.02`, `var_imu_acc=0.05`, `max_imu_dt_sec=0.5` は固定し、`var_gnss_xy` のみ比較する
- `Stop Condition`: bag1-2 専用 profile を追加するか、shared current に統一するかを判断できる

## Execution
### Compared Configs
- `shared current`
  - `var_gnss_xy=0.02`
- `bag1-2 local candidate`
  - `var_gnss_xy=0.05`

### Commands
```bash
python3 src/kalman_filter_localization/tools/run_open_data_sweep.py \
  --bag-path data/istanbul/all-sensors-bag1_compressed \
  --param-grid-json /tmp/param_grid_istanbul_shared_current.json \
  --output-dir /tmp/kfl_cycle08_bag1_current \
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

同じ比較を bag1 local, bag2 current, bag2 local にも実行した。

### Changed Files
- `src/kalman_filter_localization/docs/results/cycle08.md`: bag1-2 比較結果を記録

### Artifacts
- Bag1 current: `/tmp/kfl_cycle08_bag1_current/summary.csv`
- Bag1 local: `/tmp/kfl_cycle08_bag1_local/summary.csv`
- Bag2 current: `/tmp/kfl_cycle08_bag2_current/summary.csv`
- Bag2 local: `/tmp/kfl_cycle08_bag2_local/summary.csv`

## Self-Evaluation
### Comparison
| Bag | Shared Current (`0.02`) | Local Candidate (`0.05`) | Delta | Judgment |
| --- | --- | --- | --- | --- |
| `bag1` | `0.029604 m` | `0.034965 m` | `+0.005361 m` | shared current wins |
| `bag2` | `0.041848 m` | `0.051357 m` | `+0.009509 m` | shared current wins |

### Aggregate
| Metric | Shared Current | Local Candidate | Delta |
| --- | --- | --- | --- |
| Mean `rmse_3d` over bag1-2 | `0.035726 m` | `0.043161 m` | `+0.007435 m` |

### Observations
- bag1 でも bag2 でも `var_gnss_xy=0.02` の shared current が勝った。
- cycle05 の時点では bag1-2 に小さな退行が見えていたが、contemporaneous rerun では逆転しなかった。
- 現時点で bag1-2 専用 profile を切る根拠は消えた。

### Issues
- shared current の方が良いとはいえ、bag2 は rerun ごとに数 mm から 1 cm の揺れがある。
- bag1-2 側は「専用 profile が必要」ではなく「shared current を定期的に再確認する」段階。

### Root Cause Hypotheses
- `var_gnss_xy=0.02` は bag1-2 でも十分に過信ではなく、shared current のほうが位置補正に効いている。
- cycle05 で見えた bag1-2 側の regressions は run-to-run variance の寄与が大きかった可能性が高い。

## Next Cycle
- `Chosen Changes`: bag1-2 専用 profile は作らず、shared current を維持する
- `Deferred Changes`: bag1-2 用 profile 分離
- `Preparation Needed`: 追加検証をするなら、同一設定の rerun 分散を先に測る

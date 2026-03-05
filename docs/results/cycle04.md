# Agent Cycle Log: cycle04

## Meta
- `Cycle ID`: `cycle04`
- `Date`: `2026-03-06`
- `Owner`: `Codex`
- `Dataset`: `data/istanbul/all-sensors-bag1_compressed` ... `data/istanbul/all-sensors-bag6_compressed`
- `Comparison Target`: `kalman_filter_localization_ros2/param/profiles/istanbul_all_sensors_bag.yaml`

## Goal
- `Objective`: bag1 で最良だった `run_013` 系設定が Istanbul bags 1-6 に一般化するかを確認し、十分なら profile に反映する
- `Success Criteria`: `run_013` が現 profile より複数 bag で安定して良い、または少なくとも悪化しない
- `Constraints`: isolated `ROS_DOMAIN_ID` を維持し、各 bag は single-run 比較で済ませる
- `Stop Condition`: bag2-6 の head-to-head と bag1 既存比較が揃い、profile 更新可否を決められる

## Execution
### Compared Configs
- `run_013 candidate`
  - `var_imu_orientation_rpy = 0.001`
  - `var_imu_w = 0.02`
  - `var_imu_acc = 0.05`
  - `var_gnss_xy = 0.05`
  - `var_gnss_z = 0.1`
  - `max_imu_dt_sec = 0.5`
- `previous Istanbul profile`
  - `var_imu_orientation_rpy = 0.005`
  - `var_imu_w = 0.005`
  - `var_imu_acc = 0.01`
  - `var_gnss_xy = 0.05`
  - `var_gnss_z = 0.1`
  - `max_imu_dt_sec = 1.0`

### Commands
```bash
src/kalman_filter_localization/tools/run_agent_cycle.sh \
  data/istanbul/all-sensors-bag2_compressed \
  /tmp/kfl_cycle04_bag2_run013 \
  /tmp/param_grid_run013.json \
  cycle04_bag2_run013 \
  src/kalman_filter_localization/docs/results/agent_loop_summary_paths.txt

src/kalman_filter_localization/tools/run_agent_cycle.sh \
  data/istanbul/all-sensors-bag2_compressed \
  /tmp/kfl_cycle04_bag2_profile \
  /tmp/param_grid_istanbul_profile_single.json \
  cycle04_bag2_profile \
  src/kalman_filter_localization/docs/results/agent_loop_summary_paths.txt
```

同じ比較を bag3-6 にも実行した。bag1 は `cycle03` の `run_013` と `run_018` を比較に使った。

### Changed Files
- `src/kalman_filter_localization/kalman_filter_localization_ros2/param/profiles/istanbul_all_sensors_bag.yaml`: cross-bag で優位だった値へ更新
- `src/kalman_filter_localization/docs/dataset_profiles.md`: Istanbul profile の tuned seed を更新
- `src/kalman_filter_localization/docs/results/cycle04.md`: cycle04 の比較結果を記録

### Artifacts
- Candidate bag2: `/tmp/kfl_cycle04_bag2_run013/summary.csv`
- Candidate bag3: `/tmp/kfl_cycle04_bag3_run013/summary.csv`
- Candidate bag4: `/tmp/kfl_cycle04_bag4_run013/summary.csv`
- Candidate bag5: `/tmp/kfl_cycle04_bag5_run013/summary.csv`
- Candidate bag6: `/tmp/kfl_cycle04_bag6_run013/summary.csv`
- Profile bag2: `/tmp/kfl_cycle04_bag2_profile/summary.csv`
- Profile bag3: `/tmp/kfl_cycle04_bag3_profile/summary.csv`
- Profile bag4: `/tmp/kfl_cycle04_bag4_profile/summary.csv`
- Profile bag5: `/tmp/kfl_cycle04_bag5_profile/summary.csv`
- Profile bag6: `/tmp/kfl_cycle04_bag6_profile/summary.csv`

## Self-Evaluation
### Head-to-Head by Bag
| Bag | Candidate `rmse_3d` | Previous Profile `rmse_3d` | Delta | Judgment |
| --- | --- | --- | --- | --- |
| `bag1` | `0.030301 m` | `0.037639 m` | `-0.007339 m` | candidate wins |
| `bag2` | `0.046265 m` | `0.060551 m` | `-0.014286 m` | candidate wins |
| `bag3` | `0.063612 m` | `0.086047 m` | `-0.022435 m` | candidate wins |
| `bag4` | `0.148769 m` | `0.171332 m` | `-0.022563 m` | candidate wins |
| `bag5` | `0.099093 m` | `0.148421 m` | `-0.049328 m` | candidate wins |
| `bag6` | `0.105116 m` | `0.139104 m` | `-0.033989 m` | candidate wins |

### Aggregate
| Metric | Candidate | Previous Profile | Delta |
| --- | --- | --- | --- |
| Mean `rmse_3d` over bag1-6 | `0.082193 m` | `0.107183 m` | `-0.024990 m` |
| Wins | `6 / 6` | `0 / 6` | n/a |

### Bag2-6 Candidate Details
| Bag | Status | `rmse_3d` | `rmse_3d_nobias` | `yaw_rmse_deg` | `initial_yaw_label` |
| --- | --- | --- | --- | --- | --- |
| `bag2` | `ok` | `0.046265 m` | `0.032875 m` | `0.006638` | `yaw_init = yaw_poslv` |
| `bag3` | `ok` | `0.063612 m` | `0.059410 m` | `0.006253` | `yaw_init = yaw_poslv` |
| `bag4` | `ok` | `0.148769 m` | `0.110310 m` | `0.007682` | `yaw_init = yaw_poslv` |
| `bag5` | `ok` | `0.099093 m` | `0.095567 m` | `0.007999` | `yaw_init = yaw_poslv` |
| `bag6` | `ok` | `0.105116 m` | `0.094287 m` | `0.008263` | `yaw_init = yaw_poslv` |

### Observations
- `run_013` 系設定は bag1-6 のすべてで previous profile より良かった。
- 改善幅は特に bag5 と bag6 で大きく、IMU propagation を弱めた効果が目立つ。
- ただし bag4-6 は candidate でも `0.1 m` 近辺またはそれ以上で、profile 更新だけでは十分ではない。
- `initial_yaw` は全 bag で `yaw_poslv` を維持しており、今回の差分は yaw init failure ではない。

### Issues
- cross-bag で優位ではあるが、bag4-6 の絶対性能はまだ高すぎる。
- 今回は 2 設定だけの head-to-head なので、bag4-6 に最適な局所解は未探索。
- `agent_loop_summary_paths.txt` には `/tmp` 配下の一時 summary が増えており、長期的には整理が必要。

### Root Cause Hypotheses
- Istanbul all-sensors 系では、旧 profile は IMU 予測を信じすぎていた可能性が高い。
- `var_imu_w = 0.02`, `var_imu_acc = 0.05`, `max_imu_dt_sec = 0.5` の組み合わせは、GNSS 補正を通しやすくする方向に働いている。
- bag4-6 の残差は profile seed の問題だけでなく、bag 固有の時刻整合、入力品質、または更に強い GNSS/姿勢重み最適化の不足が関わっている。

## Improvement Candidates
| Candidate | Expected Gain | Risk | Adopt Next |
| --- | --- | --- | --- |
| bag4-6 だけを対象に狭い grid を切る | 高誤差 bag の底上げができる | low | yes |
| `var_gnss_xy < 0.05` を cross-bag で再探索する | 更に GNSS を強めて位置誤差を下げられる可能性がある | medium | yes |
| `var_imu_orientation_rpy` を cross-bag 最適化する | yaw 指標を改善できる可能性がある | medium | yes |
| profile を旧値に戻す | なし | high | no |

## Next Cycle
- `Chosen Changes`: 更新後 profile を基準に、bag4-6 向けの狭い再探索を行う
- `Deferred Changes`: multi-bag 用の比較ランナー整備
- `Preparation Needed`: bag4-6 用の small grid を定義し、single-run ではなく limited sweep を回す

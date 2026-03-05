# Agent Cycle Log: cycle05

## Meta
- `Cycle ID`: `cycle05`
- `Date`: `2026-03-06`
- `Owner`: `Codex`
- `Dataset`: `data/istanbul/all-sensors-bag1_compressed` ... `data/istanbul/all-sensors-bag6_compressed`
- `Base Profile`: `kalman_filter_localization_ros2/param/profiles/istanbul_all_sensors_bag.yaml`

## Goal
- `Objective`: bag4-6 の高誤差を狭い grid で詰めつつ、profile に反映できる cross-bag compromise があるかを確認する
- `Success Criteria`: bag4-6 を current profile より下げる設定を見つけ、必要なら bags 1-3 でも退行が許容範囲か確認する
- `Constraints`: `var_imu_w=0.02`, `var_imu_acc=0.05`, `max_imu_dt_sec=0.5` は固定し、`var_gnss_xy` と `var_imu_orientation_rpy` のみ再探索する
- `Stop Condition`: bag4-6 の refine 結果と cross-bag 候補の検証結果が揃い、profile 更新可否を決められる

## Execution
### Refinement Grid
- File: `src/kalman_filter_localization/tools/param_grid_istanbul_bag456_refine.json`
- Values:
  - `var_imu_orientation_rpy`: `0.0005`, `0.001`, `0.002`
  - `var_gnss_xy`: `0.02`, `0.03`, `0.05`
  - other parameters fixed to `var_imu_w=0.02`, `var_imu_acc=0.05`, `max_imu_dt_sec=0.5`

### Commands
```bash
python3 src/kalman_filter_localization/tools/run_open_data_sweep.py \
  --bag-path data/istanbul/all-sensors-bag4_compressed \
  --param-grid-json src/kalman_filter_localization/tools/param_grid_istanbul_bag456_refine.json \
  --output-dir /tmp/kfl_cycle05_bag4_refine \
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

同じ sweep を bag5, bag6 にも実行した。  
その後、cross-bag compromise 候補として `var_imu_orientation_rpy=0.001`, `var_gnss_xy=0.02` を bag1-3 に 1 run ずつ追加確認した。

### Changed Files
- `src/kalman_filter_localization/tools/param_grid_istanbul_bag456_refine.json`: bag4-6 用の狭い再探索 grid を追加
- `src/kalman_filter_localization/kalman_filter_localization_ros2/param/profiles/istanbul_all_sensors_bag.yaml`: `var_gnss_xy` を `0.02` に更新
- `src/kalman_filter_localization/docs/dataset_profiles.md`: Istanbul tuned seed を更新
- `src/kalman_filter_localization/docs/results/cycle05.md`: cycle05 の結果を記録

### Artifacts
- Bag4 refine: `/tmp/kfl_cycle05_bag4_refine/summary.csv`
- Bag5 refine: `/tmp/kfl_cycle05_bag5_refine/summary.csv`
- Bag6 refine: `/tmp/kfl_cycle05_bag6_refine/summary.csv`
- Bag1 candidate002: `/tmp/kfl_cycle05_bag1_candidate002/summary.csv`
- Bag2 candidate002: `/tmp/kfl_cycle05_bag2_candidate002/summary.csv`
- Bag3 candidate002: `/tmp/kfl_cycle05_bag3_candidate002/summary.csv`

## Self-Evaluation
### Bag4-6 Refinement Winners
| Bag | Best `rmse_3d` | Previous Cross-Bag Profile | Delta | Best Params |
| --- | --- | --- | --- | --- |
| `bag4` | `0.089979 m` | `0.148769 m` | `-0.058790 m` | `var_imu_orientation_rpy=0.001`, `var_gnss_xy=0.02` |
| `bag5` | `0.071342 m` | `0.099093 m` | `-0.027751 m` | `var_imu_orientation_rpy=0.002`, `var_gnss_xy=0.02` |
| `bag6` | `0.087507 m` | `0.105116 m` | `-0.017609 m` | `var_imu_orientation_rpy=0.002`, `var_gnss_xy=0.02` |

### Best Shared Pair on Bag4-6
| Params | Mean `rmse_3d` over bag4-6 |
| --- | --- |
| `var_imu_orientation_rpy=0.001`, `var_gnss_xy=0.02` | `0.086518 m` |
| `var_imu_orientation_rpy=0.002`, `var_gnss_xy=0.02` | `0.091937 m` |
| `var_imu_orientation_rpy=0.001`, `var_gnss_xy=0.05` | `0.120660 m` |

### Cross-Bag Candidate Check (`var_imu_orientation_rpy=0.001`, `var_gnss_xy=0.02`)
| Bag | Current Profile (`0.001`, `0.05`) | Candidate (`0.001`, `0.02`) | Delta | Judgment |
| --- | --- | --- | --- | --- |
| `bag1` | `0.030301 m` | `0.034560 m` | `+0.004260 m` | regression |
| `bag2` | `0.046265 m` | `0.047348 m` | `+0.001083 m` | near-flat regression |
| `bag3` | `0.063612 m` | `0.054765 m` | `-0.008847 m` | improvement |
| `bag4` | `0.148769 m` | `0.089979 m` | `-0.058790 m` | large improvement |
| `bag5` | `0.099093 m` | `0.077384 m` | `-0.021709 m` | improvement |
| `bag6` | `0.105116 m` | `0.092192 m` | `-0.012924 m` | improvement |

### Aggregate
| Metric | Current Profile (`0.001`, `0.05`) | Candidate (`0.001`, `0.02`) | Delta |
| --- | --- | --- | --- |
| Mean `rmse_3d` over bag1-6 | `0.082193 m` | `0.066038 m` | `-0.016155 m` |
| Wins | `2 / 6` | `4 / 6` | n/a |

### Observations
- bag4-6 では `var_gnss_xy=0.02` が一貫して効いた。`0.05` に戻すと明確に悪化した。
- `var_imu_orientation_rpy` の最適値は bag ごとに割れた。bag4 は `0.001`、bag5-6 は `0.002` が最良だった。
- profile に入れる共通値としては、bags 1-6 全体で検証した `var_imu_orientation_rpy=0.001`, `var_gnss_xy=0.02` が最も defensible だった。
- bag1 と bag2 では小さな退行があるが、bag3-6 の改善幅がそれを上回った。

### Issues
- 単一 profile では bag4 と bag5/6 の orientation 最適値を同時に満たせていない。
- bag1 を最良のまま維持したいなら、dataset 固有 override か adaptive tuning が必要。
- 今回は `var_imu_w` / `var_imu_acc` / `max_imu_dt_sec` を固定したので、bag4-6 にはまだ追加改善余地がある。

### Root Cause Hypotheses
- 残差の主要因は「GNSS をどこまで強く使うか」の違いで、bag4-6 では current profile がまだ GNSS XY を弱く見積もっていた。
- orientation fallback weight は二次要因で、各 bag の姿勢ノイズ特性差を吸収している。
- cross-bag shared seed と per-bag optimum は分離して考えるべき段階に入っている。

## Improvement Candidates
| Candidate | Expected Gain | Risk | Adopt Next |
| --- | --- | --- | --- |
| bag4-6 で `var_imu_orientation_rpy` をさらに細かく詰める | local optimum をもう少し下げられる | low | yes |
| bag1-2 専用の seed を別 profile として切る | regression を避けつつ cross-bag 平均も保てる | medium | yes |
| `var_imu_w` と `var_imu_acc` を bag4-6 で再度触る | 追加の位置改善が出る可能性がある | medium | no |

## Next Cycle
- `Chosen Changes`: shared Istanbul profile は `var_gnss_xy=0.02` に更新し、bag4-6 の orientation 局所最適化は別途継続する
- `Deferred Changes`: bag1-2 専用 profile の分離
- `Preparation Needed`: bag4-6 向けに `var_imu_orientation_rpy` を `0.001`-`0.002` 間で細かく刻んだ micro-grid を用意する

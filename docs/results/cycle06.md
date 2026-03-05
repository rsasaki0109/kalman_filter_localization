# Agent Cycle Log: cycle06

## Meta
- `Cycle ID`: `cycle06`
- `Date`: `2026-03-06`
- `Owner`: `Codex`
- `Dataset`: `data/istanbul/all-sensors-bag4_compressed` ... `data/istanbul/all-sensors-bag6_compressed`
- `Base Profile`: `kalman_filter_localization_ros2/param/profiles/istanbul_all_sensors_bag.yaml`

## Goal
- `Objective`: `var_gnss_xy=0.02` を固定した上で `var_imu_orientation_rpy` の micro-grid を回し、bag4-6 に共有できる安定な局所最適があるかを確認する
- `Success Criteria`: `var_imu_orientation_rpy` の shared 値を `0.001` から動かすだけの明確な改善が出る
- `Constraints`: `var_imu_w=0.02`, `var_imu_acc=0.05`, `max_imu_dt_sec=0.5`, `var_gnss_xy=0.02` は固定
- `Stop Condition`: bag4-6 の micro-grid 結果から shared 値を更新するか、現状維持かを判断できる

## Execution
### Micro Grid
- File: `src/kalman_filter_localization/tools/param_grid_istanbul_bag456_rpy_micro.json`
- Values:
  - `var_imu_orientation_rpy`: `0.001`, `0.00125`, `0.0015`, `0.00175`, `0.002`
  - `var_gnss_xy`: `0.02`

### Commands
```bash
python3 src/kalman_filter_localization/tools/run_open_data_sweep.py \
  --bag-path data/istanbul/all-sensors-bag4_compressed \
  --param-grid-json src/kalman_filter_localization/tools/param_grid_istanbul_bag456_rpy_micro.json \
  --output-dir /tmp/kfl_cycle06_bag4_rpy_micro \
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

同じ micro-grid を bag5, bag6 にも実行した。

### Changed Files
- `src/kalman_filter_localization/tools/param_grid_istanbul_bag456_rpy_micro.json`: orientation micro-grid を追加
- `src/kalman_filter_localization/docs/results/cycle06.md`: cycle06 の結論を記録

### Artifacts
- Bag4 micro: `/tmp/kfl_cycle06_bag4_rpy_micro/summary.csv`
- Bag5 micro: `/tmp/kfl_cycle06_bag5_rpy_micro/summary.csv`
- Bag6 micro: `/tmp/kfl_cycle06_bag6_rpy_micro/summary.csv`

## Self-Evaluation
### Per-Bag Best
| Bag | Best `rmse_3d` | Best `var_imu_orientation_rpy` |
| --- | --- | --- |
| `bag4` | `0.099007 m` | `0.001` |
| `bag5` | `0.071023 m` | `0.0015` |
| `bag6` | `0.080600 m` | `0.0015` |

### Shared-Value Comparison
| `var_imu_orientation_rpy` | Mean `rmse_3d` over bag4-6 | Worst Bag |
| --- | --- | --- |
| `0.001` | `0.088267 m` | `0.099007 m` |
| `0.00125` | `0.113794 m` | `0.142585 m` |
| `0.0015` | `0.090355 m` | `0.119443 m` |
| `0.00175` | `0.094026 m` | `0.117816 m` |
| `0.002` | `0.102969 m` | `0.142331 m` |

### Observations
- bag4 は `0.001` 側を好み、bag5-6 は `0.0015` 側を好んだ。
- ただし shared 値としては `0.001` の平均が最良だった。
- `0.00125` と `0.002` は worst-case を明確に悪化させる。
- cycle05 で観測した `0.001`, `0.02` の shared 勝ち筋は、この micro-grid でも覆らなかった。
- 同じ設定でも rerun ごとに数 mm から 1 cm 級の揺れがあり、微差だけで profile を分割するのは危険。

### Issues
- bag4 と bag5-6 の局所最適が一致しない。
- micro-grid の差分は、run-to-run variance と同程度の領域もある。
- 今回の micro-grid は `var_gnss_xy=0.02` 固定なので、orientation と GNSS の相互作用はまだ完全には切れていない。

### Root Cause Hypotheses
- bag4 は orientation observation をより強く拘束した方が安定する。
- bag5-6 は少し緩めた `0.0015` 近傍で translation が良くなるが、その改善は shared profile を分割する決定打ではない。
- 現段階では `var_gnss_xy=0.02` の効果が主で、`var_imu_orientation_rpy` は二次調整パラメータとみなすのが妥当。

## Improvement Candidates
| Candidate | Expected Gain | Risk | Adopt Next |
| --- | --- | --- | --- |
| bag4 専用に `var_imu_orientation_rpy < 0.001` を試す | bag4 のみ追加改善できる可能性がある | medium | yes |
| bag5-6 向けに `0.0014`-`0.0016` を再確認する | 局所最適の再現性を測れる | low | yes |
| shared profile を `0.0015` に上げる | 共有 profile が bag4 を悪化させる | high | no |

## Next Cycle
- `Chosen Changes`: shared profile は `var_imu_orientation_rpy=0.001`, `var_gnss_xy=0.02` のまま維持する
- `Deferred Changes`: bag 群ごとの専用 profile 分離
- `Preparation Needed`: bag4 と bag5-6 を分けた追加検証を行う場合は、再現性確認 run を先に挟む

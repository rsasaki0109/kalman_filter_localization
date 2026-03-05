# Agent Cycle Log: cycle09

## Meta
- `Cycle ID`: `cycle09`
- `Date`: `2026-03-06`
- `Owner`: `Codex`
- `Scope`: Istanbul profile selection and validation tooling

## Goal
- `Objective`: bag5-6 専用 profile を実運用で使える形にし、profile 選択と validation を人手判断なしで回せるようにする
- `Success Criteria`: bag path から profile を自動選択でき、validation job が bag ごとの profile で summary を出せる
- `Constraints`: shared profile と bag5-6 profile の定義は変えず、tooling 側で吸収する
- `Stop Condition`: selector / validator の smoke run が通る

## Execution
### Added Tools
- `tools/select_istanbul_profile.py`
  - bag path から推奨 profile を返す
  - bag5-6 は `istanbul_all_sensors_bag5_6.yaml`
  - それ以外は `istanbul_all_sensors_bag.yaml`
- `tools/run_istanbul_profile_validation.py`
  - bag ごとに profile 相当の single-run grid を生成
  - `run_open_data_sweep.py` を 1 本ずつ回して `profile_validation_summary.csv` を出力

### Smoke Commands
```bash
python3 src/kalman_filter_localization/tools/select_istanbul_profile.py \
  --bag-path data/istanbul/all-sensors-bag5_compressed

ROS_LOG_DIR=/tmp/ros2_logs python3 src/kalman_filter_localization/tools/run_istanbul_profile_validation.py \
  --output-dir /tmp/kfl_profile_validation_smoke \
  --bags all-sensors-bag1_compressed all-sensors-bag5_compressed
```

### Changed Files
- `src/kalman_filter_localization/tools/select_istanbul_profile.py`: bag-aware selector を追加
- `src/kalman_filter_localization/tools/run_istanbul_profile_validation.py`: validation job を追加
- `src/kalman_filter_localization/docs/dataset_profiles.md`: selector / launch usage を追記
- `src/kalman_filter_localization/docs/open_data_workflow.md`: validator usage を追記
- `src/kalman_filter_localization/docs/results/cycle09.md`: cycle09 の結果を記録

### Artifacts
- Smoke summary: `/tmp/kfl_profile_validation_smoke/profile_validation_summary.csv`
- Smoke bag1 output: `/tmp/kfl_profile_validation_smoke/all-sensors-bag1_compressed/summary.csv`
- Smoke bag5 output: `/tmp/kfl_profile_validation_smoke/all-sensors-bag5_compressed/summary.csv`

## Self-Evaluation
### Selector Smoke
| Bag | Selected Profile |
| --- | --- |
| `all-sensors-bag2_compressed` | `istanbul_all_sensors_bag.yaml` |
| `all-sensors-bag5_compressed` | `istanbul_all_sensors_bag5_6.yaml` |

### Validator Smoke
| Bag | Profile | `rmse_3d` | Status |
| --- | --- | --- | --- |
| `all-sensors-bag1_compressed` | `istanbul_all_sensors_bag.yaml` | `0.032762 m` | `ok` |
| `all-sensors-bag5_compressed` | `istanbul_all_sensors_bag5_6.yaml` | `0.076393 m` | `ok` |

### Observations
- selector は bag5-6 だけを確実に専用 profile へ振り分けた。
- validator は bag ごとに `selected_param_grid.json` を生成し、summary まで吐けた。
- 実運用では selector を launch 引数に噛ませれば、人手で profile 名を覚える必要がない。

### Issues
- validator は Istanbul all-sensors + `ins_pose` ground truth 前提の specialized tool で、汎用 suite ではない。
- validation run は単発なので、分散監視をするなら CI か cron で定期実行する必要がある。

## Next Cycle
- `Chosen Changes`: selector と validator を標準運用の入口として使う
- `Deferred Changes`: rerun variance を定期監視する CI / cron の追加
- `Preparation Needed`: 実運用環境で bag path を selector に渡す箇所を 1 か所に寄せる

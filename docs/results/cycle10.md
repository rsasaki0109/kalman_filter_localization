# Agent Cycle Log: cycle10

## Meta
- `Cycle ID`: `cycle10`
- `Date`: `2026-03-06`
- `Owner`: `Codex`
- `Scope`: Istanbul profile validation automation

## Goal
- `Objective`: validator を cron でそのまま回せる形にし、閾値超過で失敗させる
- `Success Criteria`: threshold-aware validator と cron wrapper が追加され、smoke run が 0 exit で通る
- `Constraints`: bag-aware profile selection は cycle09 のまま維持し、監視側だけ追加する
- `Stop Condition`: wrapper 実行で `profile_validation_summary.csv` と `latest` symlink が作られ、閾値判定列が出る

## Execution
### Added Files
- `src/kalman_filter_localization/tools/istanbul_profile_validation_thresholds.json`
  - bag ごとの `max_rmse_3d_m` 閾値
- `src/kalman_filter_localization/tools/run_istanbul_profile_validation_cron.sh`
  - timestamped output dir 作成
  - `latest` symlink 更新
  - threshold-aware validator 実行
- `src/kalman_filter_localization/tools/istanbul_profile_validation.crontab.example`
  - cron 設定例

### Updated Files
- `src/kalman_filter_localization/tools/run_istanbul_profile_validation.py`
  - `--thresholds-json` を追加
  - `rmse_3d_threshold_m`, `rmse_3d_margin_m`, `threshold_status` を summary に追加
  - 閾値超過時は非ゼロ終了
- `src/kalman_filter_localization/docs/open_data_workflow.md`
  - cron wrapper と threshold 運用を追記

### Smoke Command
```bash
KFL_VALIDATION_OUTPUT_BASE=/tmp/kfl_validation_cron_smoke \
  src/kalman_filter_localization/tools/run_istanbul_profile_validation_cron.sh \
  all-sensors-bag1_compressed all-sensors-bag5_compressed
```

### Artifacts
- Smoke latest symlink: `/tmp/kfl_validation_cron_smoke/latest`
- Smoke summary: `/tmp/kfl_validation_cron_smoke/20260306_140036/profile_validation_summary.csv`
- Cron example: `src/kalman_filter_localization/tools/istanbul_profile_validation.crontab.example`

## Self-Evaluation
### Smoke Results
| Bag | Profile | `rmse_3d` | Threshold | Margin | Threshold Status |
| --- | --- | --- | --- | --- | --- |
| `all-sensors-bag1_compressed` | `istanbul_all_sensors_bag.yaml` | `0.029397 m` | `0.05 m` | `+0.020603 m` | `pass` |
| `all-sensors-bag5_compressed` | `istanbul_all_sensors_bag5_6.yaml` | `0.075750 m` | `0.09 m` | `+0.014250 m` | `pass` |

### Observations
- wrapper は timestamped output dir を作り、`latest` symlink まで更新した。
- threshold-aware validator は summary に margin を出し、今回の smoke は両 bag とも `pass` だった。
- cron 導入時に必要なのは `run_istanbul_profile_validation_cron.sh` を定期実行するだけになった。

### Issues
- threshold は現時点の経験的 guardrail なので、dataset や bag 品質が変われば再調整が必要。
- cron 自体の登録はリポジトリ外の運用作業で、ここでは雛形までしか置いていない。

## Next Cycle
- `Chosen Changes`: threshold-aware validator + cron wrapper を運用入口として使う
- `Deferred Changes`: CI への移植と通知連携
- `Preparation Needed`: 実マシンで crontab へ `istanbul_profile_validation.crontab.example` を適用する

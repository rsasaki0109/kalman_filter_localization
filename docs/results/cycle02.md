# Agent Cycle Log: cycle02

## Meta
- `Cycle ID`: `cycle02`
- `Date`: `2026-03-06`
- `Owner`: `Codex`
- `Dataset`: `data/istanbul/all-sensors-bag1_compressed`
- `Baseline`: `src/kalman_filter_localization/docs/results/open_data/istanbul_bag1_course_yaw/summary.csv`

## Goal
- `Objective`: bag 再生グラフを ambient ROS graph から隔離し、既知ベスト設定で baseline 級の結果を再現する
- `Success Criteria`: `/gnss/fix` の原点が Istanbul bag の値で固定され、`rmse_3d` が `0.1 m` 未満に戻る
- `Constraints`: 既存 `run_open_data_sweep.py` / `run_agent_cycle.sh` の流れを崩さない
- `Stop Condition`: 1 設定 verify と full sweep の smoke run が両方 `ok` で通る

## Task Breakdown
| Priority | Task | Expected Effect | Status |
| --- | --- | --- | --- |
| P0 | ambient `/gnss/fix` 混入の有無を確認する | 原点ずれの真因を特定する | done |
| P1 | sweep 全体を isolated `ROS_DOMAIN_ID` で実行する | bag 外トピックの混入を防ぐ | done |
| P2 | known-good 1 設定で baseline 再現を確認する | fix の妥当性を確認する | done |
| P3 | full sweep の先頭 run が安定して通ることを確認する | 本流再開可否を判断する | done |

## Execution
### Commands
```bash
ROS_LOG_DIR=/tmp/ros2_logs python3 src/kalman_filter_localization/tools/run_open_data_sweep.py \
  --bag-path data/istanbul/all-sensors-bag1_compressed \
  --param-grid-json /tmp/param_grid_cycle01_isolated.json \
  --output-dir /tmp/kfl_cycle01_isolated \
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
  --initial-yaw-source-msg-type pose_stamped \
  --initial-yaw-timeout-sec 20.0

src/kalman_filter_localization/tools/run_agent_cycle.sh \
  data/istanbul/all-sensors-bag1_compressed \
  /tmp/kfl_cycle02 \
  src/kalman_filter_localization/tools/param_grid_istanbul_quick.json \
  cycle02 \
  src/kalman_filter_localization/docs/results/agent_loop_summary_paths.txt
```

### Changed Files
- `src/kalman_filter_localization/tools/run_open_data_sweep.py`: auto-isolated `ROS_DOMAIN_ID` を追加
- `src/kalman_filter_localization/docs/open_data_workflow.md`: isolated domain の挙動を追記

### Artifacts
- Isolated verify: `/tmp/kfl_cycle01_isolated/summary.csv`
- Isolated verify report: `/tmp/kfl_cycle01_isolated/open_data_report_20260306_094839.html`
- Full sweep smoke: `/tmp/kfl_cycle02/summary.csv`

## Self-Evaluation
| Metric | Baseline | Current | Delta | Judgment |
| --- | --- | --- | --- | --- |
| `rmse_3d` verify | `0.034230 m` | `0.034132 m` | `-0.000098 m` | pass |
| `rmse_3d_nobias` verify | `0.032617 m` | `0.028496 m` | `-0.004121 m` | pass |
| `yaw_rmse` verify | `0.001306 deg` | `0.007589 deg` | `+0.006283 deg` | pass |
| `rmse_3d` smoke run_001 | n/a | `0.039831 m` | n/a | pass |

### Observations
- ambient graph 上に別の `/gnss/fix` が存在すると、`navsatfix_to_pose.py` と `applanix_nav_solution_to_pose.py` が誤った原点を先に掴み、`/ins_pose` が巨大オフセットで壊れることを再現した。
- `run_open_data_sweep.py` に isolated `ROS_DOMAIN_ID` を入れた後は、偽 `/gnss/fix` を流していても `origin set from NavSatFix lat=40.81343596 lon=29.36309577 alt=50.306` に戻った。
- known-good 1 設定では `rmse_3d = 0.034132 m` まで復帰し、baseline と同等になった。
- full sweep は時間が長いため、`run_001` が `rmse_3d = 0.039831 m` で安定通過したところまで確認して停止した。

### Issues
- full 48-run sweep 自体はまだ完走させていない。
- ambient graph の混入を避けるため、将来の運用でも `ROS_DOMAIN_ID` の扱いは明示した方が安全。

### Root Cause Hypotheses
- 主因は `/ins_imu` 単体の timestamp 異常ではなく、default domain 上の ambient `/gnss/fix` を拾って ground truth origin が壊れていたこと。
- prefetch と本番 run の実装改善は有効だったが、決定打は graph isolation だった。

## Improvement Candidates
| Candidate | Expected Gain | Risk | Adopt Next |
| --- | --- | --- | --- |
| full 48-run sweep を isolated domain のまま完走する | 全設定のランキングを回復できる | 低 | yes |
| `run_agent_cycle.sh` に `ROS_DOMAIN_ID` をログ出力する | 運用時の追跡がしやすい | 低 | yes |
| cycle report に `ros_domain_id` を載せる | 再現性が上がる | 低 | no |

## Next Cycle
- `Chosen Changes`: isolated domain fix のまま full sweep を完走させる
- `Deferred Changes`: HTML report への `ros_domain_id` 表示
- `Preparation Needed`: 長時間 run を回す時間枠の確保

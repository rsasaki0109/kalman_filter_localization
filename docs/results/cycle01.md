# Agent Cycle Log: cycle01

## Meta
- `Cycle ID`: `cycle01`
- `Date`: `2026-03-06`
- `Owner`: `Codex`
- `Dataset`: `data/istanbul/all-sensors-bag1_compressed`
- `Baseline`: `src/kalman_filter_localization/docs/results/open_data/istanbul_bag1_course_yaw/summary.csv`

## Goal
- `Objective`: compressed Istanbul bag で `initial_yaw` を正しく取得し、少なくとも 1 run を `ok` で評価完了まで通す
- `Success Criteria`: `initial_yaw_label = yaw_poslv` かつ `summary.csv` に `status=ok` の run が出る
- `Constraints`: 既存 `run_agent_cycle.sh` / `run_open_data_sweep.py` ベースで対処する
- `Stop Condition`: 無効 run の原因が再現でき、修正後の verify run が評価完了する

## Task Breakdown
| Priority | Task | Expected Effect | Status |
| --- | --- | --- | --- |
| P0 | `initial_yaw` fallback の原因を切り分ける | 無効な sweep を止める | done |
| P1 | timeout と初期姿勢 publish の実装を修正する | `yaw_poslv` と EKF 出力を回復する | done |
| P2 | 1 設定で verify run を流す | 評価完了の確認 | done |

## Execution
### Commands
```bash
src/kalman_filter_localization/tools/run_agent_cycle.sh \
  data/istanbul/all-sensors-bag1_compressed \
  /tmp/kfl_cycle01 \
  src/kalman_filter_localization/tools/param_grid_istanbul_quick.json \
  cycle01 \
  src/kalman_filter_localization/docs/results/agent_loop_summary_paths.txt

python3 src/kalman_filter_localization/tools/run_open_data_sweep.py \
  --bag-path data/istanbul/all-sensors-bag1_compressed \
  --param-grid-json /tmp/param_grid_cycle01_verify.json \
  --output-dir /tmp/kfl_cycle01_verify \
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
```

### Changed Files
- `src/kalman_filter_localization/tools/run_agent_cycle.sh`: ROS setup と `initial_yaw_timeout` の扱いを修正
- `src/kalman_filter_localization/tools/run_open_data_sweep.py`: `initial_yaw` prefetch と初期姿勢 publish を安定化
- `src/kalman_filter_localization/docs/open_data_workflow.md`: compressed bag 向け timeout 注意を追加
- `src/kalman_filter_localization/docs/agent_design_loop.md`: 実行例の timeout を更新
- `src/kalman_filter_localization/docs/agent_design_loop_report.html`: サンプルコマンドの timeout を更新

### Artifacts
- Partial sweep: `/tmp/kfl_cycle01/summary.csv`
- Verify run: `/tmp/kfl_cycle01_verify/summary.csv`
- Verify report: `/tmp/kfl_cycle01_verify/open_data_report_20260306_090926.html`
- Prefetch logs: `/tmp/kfl_cycle01_verify/_initial_yaw_prefetch/`

## Self-Evaluation
| Metric | Baseline | Current | Delta | Judgment |
| --- | --- | --- | --- | --- |
| `initial_yaw_label` | `yaw_init = yaw_poslv` | `yaw_init = yaw_poslv` | matched | pass |
| `rmse_3d` | `0.034230 m` | `5.316257 m` | `+5.282027 m` | fail |
| `rmse_3d_nobias` | `0.032617 m` | `5.234733 m` | `+5.202116 m` | fail |
| `yaw_error` | `0.001306 deg` | `0.129556 deg` | `+0.128249 deg` | fail |

### Observations
- 最初の `run_agent_cycle.sh` 実行では `initial_yaw_label = fallback (pose arg)` となり、`rmse_3d` が `1.7e6 m` から `1.0e7 m` 級まで破綻した。
- compressed bag は `ros2 bag play` の展開に約 10 秒かかり、`--initial-yaw-timeout-sec 8.0` では `/ins_pose` の初回サンプル前に timeout していた。
- timeout を `20.0` に上げ、さらに prefetch を追加したことで verify run は `status=ok` まで到達した。

### Issues
- verify run でも `skip EKF prediction update due to too large IMU dt` と `non-positive IMU dt` が継続している。
- `gsof49_to_imu.py` 由来の `/ins_imu` stamp が 36 秒級のジャンプを含み、序盤の予測更新が落ちている。
- ベースラインの `0.034 m` に対して、現状 verify run は `5.316 m` と大きく悪化している。

### Root Cause Hypotheses
- compressed bag の展開待ちで `initial_yaw` 取得が遅れ、旧実装では fallback yaw に落ちていた。
- 初回 run で yaw 取得と本番再生を同じタイムラインで行う設計が、初期姿勢 publish と IMU 時刻処理を不安定にしていた。
- 位置誤差の主因は `initial_yaw` よりも、依然残る `/ins_imu` の時刻不整合である可能性が高い。

## Improvement Candidates
| Candidate | Expected Gain | Risk | Adopt Next |
| --- | --- | --- | --- |
| `gsof49_to_imu.py` の stamp 推移を追跡し、36 秒ジャンプの原因を潰す | `rmse_3d` を大幅に改善できる可能性が高い | 中 | yes |
| verify run と既存 baseline run の `estimated.csv` / `ground_truth.csv` を比較する | 位置バイアスと評価条件差を切り分けやすい | 低 | yes |
| 修正後に `run_agent_cycle.sh` で full sweep を再実行する | 実運用フローが回復する | 中 | no |

## Next Cycle
- `Chosen Changes`: `/ins_imu` の timestamp source と単調化ロジックを調査する
- `Deferred Changes`: full 48-run sweep の再実行
- `Preparation Needed`: `gsof49_to_imu.py` の入力 stamp と出力 stamp を run ログか CSV で可視化する

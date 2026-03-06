# Agent Cycle Log: cycle03

## Meta
- `Cycle ID`: `cycle03`
- `Date`: `2026-03-06`
- `Owner`: `Codex`
- `Dataset`: `data/istanbul/all-sensors-bag1_compressed`
- `Baseline`: `src/kalman_filter_localization/docs/results/open_data/istanbul_bag1_course_yaw/summary.csv`
- `Output Dir`: `/tmp/kfl_cycle03`
- `ROS_DOMAIN_ID`: `196`

## Goal
- `Objective`: isolated `ROS_DOMAIN_ID` 前提で quick grid 48 本を完走し、bag1 の best parameter を確定する
- `Success Criteria`: `summary.csv` の全 run が `status=ok` で埋まり、baseline `rerun_006` (`rmse_3d = 0.034230 m`) を上回る設定を 1 本以上得る
- `Constraints`: `run_agent_cycle.sh` の既存フローを維持し、bag 外 graph の混入を防ぐ
- `Stop Condition`: 48/48 完走、ranking 生成、HTML report 更新まで完了する

## Task Breakdown
| Priority | Task | Expected Effect | Status |
| --- | --- | --- | --- |
| P0 | runner に domain 情報を残す | 再現時に graph isolation を追跡できる | done |
| P1 | isolated domain で full 48-run sweep を完走させる | quick grid 全体を比較可能にする | done |
| P2 | top config を抽出して baseline と比較する | 次サイクルの起点を固める | done |
| P3 | cycle log と report を更新する | 知見を docs に固定する | done |

## Execution
### Commands
```bash
src/kalman_filter_localization/tools/run_agent_cycle.sh \
  data/istanbul/all-sensors-bag1_compressed \
  /tmp/kfl_cycle03 \
  src/kalman_filter_localization/tools/param_grid_istanbul_quick.json \
  cycle03 \
  src/kalman_filter_localization/docs/results/agent_loop_summary_paths.txt
```

### Changed Files
- `src/kalman_filter_localization/tools/run_agent_cycle.sh`: `requested_ros_domain_id` と実際の `ros_domain_id` を cycle log から追えるようにした
- `src/kalman_filter_localization/docs/results/cycle03.md`: cycle03 の結果を記録

### Artifacts
- Summary: `/tmp/kfl_cycle03/summary.csv`
- RMSE ranking: `/tmp/kfl_cycle03/ranking_by_rmse_3d.csv`
- No-bias ranking: `/tmp/kfl_cycle03/ranking_by_rmse_3d_nobias.csv`
- HTML report: `/tmp/kfl_cycle03/open_data_report_20260306_104110.html`
- Updated docs report: `src/kalman_filter_localization/docs/agent_design_loop_report.html`
- Cycle log: `/tmp/kfl_cycle03/run_agent_cycle.log`

### Top Configs
| Rank | Run | rmse_3d | rmse_3d_nobias | yaw_rmse_deg | var_imu_orientation_rpy | var_imu_w | var_imu_acc | var_gnss_xy | max_imu_dt_sec |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | `run_013` | `0.030301 m` | `0.026628 m` | `0.005717` | `0.001` | `0.02` | `0.05` | `0.05` | `0.5` |
| 2 | `run_045` | `0.030652 m` | `0.025484 m` | `0.010169` | `0.01` | `0.02` | `0.05` | `0.05` | `0.5` |
| 3 | `run_029` | `0.031090 m` | `0.028238 m` | `0.008125` | `0.005` | `0.02` | `0.05` | `0.05` | `0.5` |
| 4 | `run_046` | `0.032657 m` | `0.026832 m` | `0.010189` | `0.01` | `0.02` | `0.05` | `0.05` | `1.0` |
| 5 | `run_002` | `0.032887 m` | `0.030482 m` | `0.007701` | `0.001` | `0.005` | `0.01` | `0.05` | `1.0` |

## Self-Evaluation
| Metric | Baseline | Current | Delta | Judgment |
| --- | --- | --- | --- | --- |
| `status=ok` count | n/a | `48 / 48` | n/a | pass |
| Best `rmse_3d` | `0.034230 m` | `0.030301 m` | `-0.003930 m` | pass |
| Best `rmse_3d_nobias` | `0.032617 m` | `0.025484 m` | `-0.007134 m` | pass |
| Best `yaw_rmse_deg` | `0.001306 deg` | `0.005678 deg` | `+0.004372 deg` | pass |
| Median `rmse_3d` | n/a | `0.037983 m` | n/a | pass |
| Worst `rmse_3d` | n/a | `0.051655 m` | n/a | pass |

### Observations
- isolated domain で `ros_domain_id: 196` が割り当てられ、48 本とも `status=ok` で完走した。
- top 4 はすべて `var_imu_w = 0.02`, `var_imu_acc = 0.05`, `var_gnss_xy = 0.05` を共有している。
- `max_imu_dt_sec = 0.5` が上位を占め、`1.0` は同条件比較でやや不利だった。
- `var_gnss_xy = 0.2` は全体として悪化方向で、top 10 にほとんど入らなかった。
- `var_imu_orientation_rpy` は `0.001`, `0.005`, `0.01` のどれでも上位に入ったが、best overall は `0.001` だった。

## Why run_013 Won
### Parameter Semantics
- `var_imu_w` と `var_imu_acc` は prediction の process noise `Q` にそのまま入る。値を上げるほど IMU 積分に対する過信を下げ、後段の観測更新が入りやすくなる。
- `var_gnss_xy` は GNSS 位置観測の covariance `R` にそのまま入る。値を下げるほど XY 位置更新が強くなる。
- `var_imu_orientation_rpy` は orientation update の covariance `R` に入る。今回の `gsof49_to_imu.py` は `orientation_covariance` を `NaN` で publish しているため、この fallback 値が実際に使われている。
- `max_imu_dt_sec` は prediction を通す IMU dt の上限で、これを超えた積分は skip される。

### One-Factor Deltas Around `run_013`
| Change from `run_013` | Compare Run | Delta `rmse_3d` | Delta `rmse_3d_nobias` | Delta `yaw_rmse_deg` | Interpretation |
| --- | --- | --- | --- | --- | --- |
| `var_imu_w: 0.02 -> 0.005` | `run_005` | `+0.003992 m` | `+0.001076 m` | `+0.001856` | gyro 積分を信じすぎると位置も yaw も悪化 |
| `var_imu_acc: 0.05 -> 0.01` | `run_009` | `+0.007759 m` | `+0.005300 m` | `-0.000018` | 加速度積分を信じすぎると translational drift が増える |
| `var_gnss_xy: 0.05 -> 0.2` | `run_015` | `+0.009170 m` | `+0.005091 m` | `-0.000034` | GNSS 更新を弱めると位置誤差が最も悪化する |
| `max_imu_dt_sec: 0.5 -> 1.0` | `run_014` | `+0.005312 m` | `+0.001862 m` | `-0.000039` | guard を緩めると位置が悪化する傾向がある |
| `var_imu_orientation_rpy: 0.001 -> 0.005` | `run_029` | `+0.000789 m` | `+0.001610 m` | `+0.002408` | orientation weight は効くが支配因子ではない |
| `var_imu_orientation_rpy: 0.001 -> 0.01` | `run_045` | `+0.000352 m` | `-0.001145 m` | `+0.004452` | 位置 bias には有利でも yaw は悪化しやすい |

### Working Hypothesis
- この bag では GNSS XY が十分に強いので、`var_gnss_xy = 0.05` で位置観測を強めた方が明確に得をする。
- 一方で IMU の角速度・加速度は prediction には使うが、そこを強く信じると drift を持ち込みやすい。`var_imu_w = 0.02` と `var_imu_acc = 0.05` は「IMU は short-term propagation に使うが、長くは信用しない」という設定になっている。
- `var_imu_acc` の感度が最も大きいので、この dataset では加速度の扱いが主な支配因子と考えるのが妥当。
- `var_imu_orientation_rpy` は translation 最適化より yaw/bias trade-off を動かしている。`0.001` は overall `rmse_3d`、`0.01` は `rmse_3d_nobias` に寄る。
- `max_imu_dt_sec = 0.5` は上位群と相関しているが、`run_013` / `run_014` のログでは dt skip warning を確認できていない。したがって現時点では主因ではなく安全側の guard とみなす。

### Implication
- 次サイクルでは `var_imu_w = 0.02`, `var_imu_acc = 0.05`, `var_gnss_xy = 0.05` を固定し、`var_imu_orientation_rpy` と `var_gnss_xy < 0.05` を狭く再探索するのが筋が良い。
- ただし profile 既定値へ昇格させる前に、bag2-6 で同じ傾向が出るかを確認する必要がある。

### Issues
- `yaw_rmse_deg` は歴史的 baseline `rerun_006` よりまだ大きい。
- 今回の最適化は bag1 のみで、bag2-6 への一般化は未確認。
- 長時間実行後も古い `/tmp/kfl_cycle01` / `/tmp/kfl_cycle02` の補助プロセスが環境に残っており、graph isolation なしだと運用混乱の原因になりうる。

### Root Cause Hypotheses
- catastrophic failure は解消済みで、残る差分は graph contamination ではなく純粋な parameter sensitivity で説明できる。
- translational error の最適点は `var_imu_w = 0.02`, `var_imu_acc = 0.05`, `var_gnss_xy = 0.05` 近傍にある可能性が高い。
- yaw 指標の最適点は translational 最適点と完全には一致していない。

## Improvement Candidates
| Candidate | Expected Gain | Risk | Adopt Next |
| --- | --- | --- | --- |
| `run_013` を暫定 default candidate として再実行する | best config の再現性を確認できる | low | yes |
| `run_013` と `run_045` を bag2-6 に横展開する | bag1 過学習を避けられる | low | yes |
| `var_gnss_xy < 0.05` を含む狭い再探索を行う | さらに数 mm 改善できる可能性がある | medium | yes |
| yaw 指標優先の別 grid を切る | 姿勢精度の改善余地を見られる | medium | no |

## Next Cycle
- `Chosen Changes`: `run_013` を基準設定として複数 bag で再検証し、再現性を確認する
- `Deferred Changes`: yaw 最適化専用 grid の追加
- `Preparation Needed`: bag2-6 に同じ isolated sweep を流すための実行時間を確保する

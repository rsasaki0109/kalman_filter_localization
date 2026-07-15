# GNSS/IMU/Wheel ESKF 完成ロードマップ

最終更新: 2026-07-16

## 1. ゴール

本リポジトリを、単に特定 bag で低 RMSE が出る実装ではなく、次を満たす
ROS 2 向け車両ローカライザにする。

- 数式、座標系、単位、時刻の定義が一意で、コードと一致している。
- IMU レート、センサ遅延、GNSS outage、外れ値に対して統計的に整合する。
- yaw と IMU bias が、観測可能なときだけ安定して収束する。
- open-sky、urban canyon、GNSS outage、reacquisition を再現可能に評価できる。
- 全パラメータが物理量または統計量として説明でき、bag ごとの手調整を減らす。
- 現行 LICENSE の BSD-3-Clause 条項を維持し、GPL 実装は設計比較にだけ用いる。

対象は loosely coupled GNSS/INS/wheel ESKF とする。raw pseudorange の tightly
coupled GNSS、LiDAR/VIO、factor graph への全面移行は、このロードマップの必須範囲に
含めない。

## 2. 現状監査

旧計画に書かれていた 15-state 化、accelerometer/gyro bias、GNSS course yaw、
continuous process-noise density、2次離散化、Joseph update、lever arm、GNSS velocity、
NHC、wheel speed、ZUPT/ZIHR、NIS/robust loss はすでに実装済みである。

現在の再現済み基準値は次の通り。

| シナリオ | 構成 | 現在値 |
|---|---|---:|
| Applanix open-sky | baseline | 3D RMSE 0.2560 m、yaw 1.1363 deg |
| Odaiba GNSS-continuous | wheel auto | aligned 3D RMSE 0.2210 m、yaw 1.5484 deg |
| Odaiba 81.8 s outage | wheel + NHC auto | 3D RMSE 8.1117 m |
| Odaiba 81.8 s outage | wheel + NHC fixed | 3D RMSE 6.3358 m |
| Istanbul bag4/5/6 | accel bias on | 3D RMSE 0.075/0.055/0.066 m、yaw 2.46--4.92 deg |

ただし、現在値だけでは filter consistency を証明できない。コード監査で以下の
優先課題が見つかった。

1. 右乗算姿勢誤差を nominal quaternion に注入した後、ESKF の covariance reset
   Jacobian を適用していない。
2. process noise は legacy、continuous、2次近似の3つの独立スイッチで構成され、
   無効な組み合わせを作れる。現在の2次 `Qd` は厳密離散化ではない。
3. 遅延 GNSS は過去状態で更新せず、現在速度で測定位置を外挿している。姿勢、bias、
   covariance、途中の IMU を反映できない。
4. GNSS 位置差分から作る velocity/course を同じ位置更新と併用しており、相関を無視して
   情報を二重利用する可能性がある。
5. lever arm 使用時の GNSS NIS は実際の観測 Jacobian を使わず `Ppp + R` だけで
   計算している。
6. 更新式が明示的な `inverse()` に依存し、分解失敗、非正定値、condition number を
   診断していない。
7. 有限差分 Jacobian、ESKF reset、Monte Carlo NEES/NIS、out-of-sequence update の
   自動テストがない。
8. 初期 position/velocity/attitude covariance、静止初期化、IMU noise 同定の手順が
   十分に定義されていない。

## 3. 調査結果から採用する方針

### 数理と伝播

- [Solà の ESKF 定式化](https://arxiv.org/abs/1711.02508)を基準に、右乗算誤差、
  quaternion の向き、注入、reset を一つの数式仕様に固定する。
- [OpenVINS の IMU propagation 導出](https://docs.openvins.com/propagation.html)を
  独立な照合先にする。実装はコピーせず、`F`, `G`, `Qc`, `Phi`, `Qd` の符号と単位を
  比較する。
- `Phi = exp(Fc dt)` と Van Loan 法による `Qd` を reference 実装にする。100--400 Hz
  の通常運用で高速近似が必要なら、reference との誤差をテストした上で別 backend とする。
- mean propagation は現在サンプルだけの ZOH から、最低でも2サンプル midpoint へ進める。
  高角速度データで必要性を確認してから coning/sculling 補正を追加する。

### ノイズと calibration

- `var_imu_*` を曖昧な「分散」ではなく continuous-time noise density / bias random-walk
  PSD として定義し、YAML 名と単位を明記する。
- [Kalibr IMU noise model](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)と
  [allan_variance_ros](https://github.com/ori-drs/allan_variance_ros)に合わせた変換ツールと
  手順を用意する。静止 Allan 結果は実走時の温度・振動を含まないため、NIS/NEES で
  validation する。
- offline calibration を先に完成させる。online extrinsic/time-offset estimation は、
  運動による観測可能性判定まで設計できた後の拡張とする。

### 非同期センサ

- [MINS](https://arxiv.org/abs/2309.15390)の非同期融合・時空間 calibration と、
  [PX4 EKF2](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/ekf2)の delayed
  fusion/output predictor を設計比較に使う。
- 本実装では最初に bounded state/IMU history を持ち、測定時刻へ rewind、update、現在へ
  repropagate する。定速外挿は fallback に降格する。

### 車両拘束と GNSS

- [Eagleye](https://github.com/MapIV/eagleye)は wheel scale、heading、stop 判定の比較先、
  [KF-GINS](https://github.com/i2Nav-WHU/KF-GINS)は古典的 GNSS/INS mechanization と
  lever arm の比較先にする。
- yaw は receiver の Doppler velocity/course を第一選択、位置差分 course を fallback とする。
  位置差分を使う場合は、そのサンプルを位置・速度・course と独立に三重利用しない。
- NHC は常時真と仮定せず、yaw rate、lateral acceleration、wheel/GNSS innovation から
  slip/turn を判定して分散を増やすか更新を止める。
- wheel scale は現在の moving median を基準として保持し、状態追加は observability と
  Monte Carlo 検証後に判断する。

### 評価

- [evo の APE/RPE 定義](https://github.com/MichaelGrupp/evo/wiki/Metrics)に合わせ、絶対誤差と
  局所 drift を分ける。絶対系の試験では後付け SE(3)/Sim(3) alignment を禁止する。
- [UrbanNav](https://github.com/IPNL-POLYU/UrbanNavDataset)を urban/outage 試験に使い、
  open-sky と同じ閾値で混ぜない。
- RMSE だけでなく NEES、NIS、coverage、reacquisition jump/settling、outage endpoint drift、
  CPU、メモリ、決定性を合否判定に含める。

### OSS の扱い

| OSS | 主に見る箇所 | 方針 |
|---|---|---|
| OpenVINS | propagation、FEJ、初期化、simulation | 導出とテスト設計を参照。GPL コードはコピーしない |
| MINS | async fusion、wheel、online calibration | architecture と observability を参照。コードはコピーしない |
| PX4 EKF2 | delayed fusion、innovation diagnostics、bias learning | production pattern を参照 |
| KF-GINS | mechanization、GNSS update、lever arm | 数式照合。GPL コードはコピーしない |
| Eagleye | wheel scale、stop/heading 推定 | ROS interface と vehicle logic を比較 |
| GTSAM | IMU preintegration、BSD 実装 | offline oracle/比較器の候補。runtime 依存にはしない |
| robot_localization | ROS frame/topic conventions | interoperability 比較のみ |

## 4. 実装フェーズ

各フェーズは前の gate を通ってから進める。パラメータ sweep で数理不整合を隠さない。

### Phase 0: baseline を凍結する

- [x] 現在の commit、bag checksum、merged YAML、ROS distro、依存 version、実行 command を
      manifest に保存する。
- [x] Applanix、Odaiba continuous、Odaiba outage、Istanbul bag4--6 の current baseline を
      同じ evaluator で再実行する。
- [x] `legacy` と `research_full` の両 propagation mode を保存する。
- [x] random seed、message ordering、playback rate を固定し、同一入力3回の差を測る。
- [x] tuning 用 segment と最終 holdout segment を分離し、Shinjuku を checksum 付きで
      `docs/holdout_registry.md` に blind 登録する。

完了条件: 任意の開発者が command 1本で同じ manifest と許容誤差内の metrics を生成できる。

### Phase 1: 数式仕様と検証 harness

- [x] `docs/eskf_math.md` に world/body/IMU/base/GNSS frame、ENU/FLU、quaternion、重力、
      右乗算誤差、state/error-state、単位を定義する。
- [x] continuous nominal dynamics、`Fc`, `G`, `Qc`、全 observation `h/H/R`、injection/reset を
      コードの index と対応させる。
- [x] propagation と全 observation（position、lever arm、orientation、world/body velocity、
      forward wheel、NHC）の有限差分 Jacobian test を追加する。
- [x] Eigen の明示 inverse を LDLT/LLT solve に置き換え、数値テストを追加する。
- [x] covariance の symmetry、有限性、PSD、quaternion norm を各 update 後に検査可能にする。

完了条件: 代表状態100ケースで解析 Jacobian と数値 Jacobianが一致し、符号規約を変えると
テストが確実に失敗する。

### Phase 2: ESKF core を理論通りにする

- [x] error injection 後に右乗算誤差用 reset Jacobian を covariance に適用する。
- [x] `Fc/G/Qc` 構築を propagation から分離し、単体テスト可能にする。
- [x] exact `Phi/Qd` reference backend を実装する。Gauss-Markov bias decay と driving noise も
      同じ連続時間モデルから離散化する。
- [x] 3 boolean の組み合わせを `propagation_model: legacy|exact|fast` の排他的設定へ移行する。
      旧 parameter は1リリース互換変換し、矛盾時は起動失敗にする。
- [x] midpoint mean integration を追加し、legacy との差を ablation する。
- [x] innovation covariance を分解で解き、非正定値・ill-conditioned update を拒否して理由を
      diagnostic に出す。
- [x] covariance cap と hard state reset は安全弁として残すが、通常 profile では無効にする。

完了条件:

- constant `Fc/G/Qc` で 1 step と分割 propagation の `Phi/Qd` が数値誤差内で一致する。
- `Qd` と更新後 `P` が PSD、rate 50/100/200/400 Hz で1秒後 covariance が一致する。
- Monte Carlo simulation の NEES が設定した信頼区間に入り、legacy より悪化しない。

### Phase 3: timestamp と out-of-sequence measurement

- [x] 全入力の sensor stamp、arrival time、filter time、適用 time を debug 出力する。
- [x] IMU sample、nominal state、covariance を保持する bounded history buffer を追加する。
- [x] GNSS/odom/wheel を測定時刻へ rewind-update-repropagate する。
- [x] history より古い測定、未来測定、duplicate、逆順 stamp の policy と counter を定義する。
- [x] publication は最新予測 state、correction は過去 state という2時刻設計にする。
- [x] synthetic delayed bag で 0--500 ms、jitter、drop、reordering を sweep する。
- [x] 現在の constant-velocity GNSS delay compensation は deprecated fallback とする。

完了条件: 同じ物理データへ既知遅延を加えても、補償後 trajectory と covariance が無遅延 run に
所定誤差内で一致する。repropagation 中も output stamp が逆行しない。

### Phase 4: 初期化と calibration

- [x] 静止 window から gyro bias、roll/pitch、stationary 判定 confidence を初期化する。
- [x] yaw の初期化源を dual-antenna、Doppler/course、external pose の優先順で明示する。
- [x] 初期 position/velocity/attitude/bias covariance を sensor accuracy から設定可能にする。
- [x] Allan variance 出力から YAML へ単位付き変換する script と report を追加する。
- [x] IMU-to-base rotation、GNSS lever arm、wheel frame、符号、scale、時刻 offset の offline
      calibration checklist を追加する。
- [x] 短い Istanbul bag では初期化 window と prior covariance を ablation し、bag 長依存を測る。
- [x] 温度が取れる IMU では bias-vs-temperature を解析する。モデル追加は有意差がある場合だけ。

完了条件: 手入力する初期 bias を使わず、短時間 bag でも初期化失敗理由が診断できる。

### Phase 5: measurement model と observability

- [x] lever arm を含む実際の `H P H^T + R` で全 NIS を計算する共通 update API を作る。
- [x] receiver-native GNSS position/velocity covariance を full 3x3 で扱えるようにする。
- [x] Doppler velocity/course を第一 yaw aid にし、位置差分 fallback の相関を管理する。
- [x] GNSS position、velocity、course の quality gate と reject reason を共通化する。
- [x] forward wheel update の full cross-state gain と現在の decoupled gain を ablation し、yaw/bias
      observability と consistency で選ぶ。
- [x] NHC の adaptive variance を slip/turn detector として独立実装し、状態遷移を可視化する。
- [x] ZUPT/ZIHR は同じ stationary detector を共有し、停止中だけ bias 学習を強化する。
- [x] gyro/accel bias の学習を excitation/observability に応じて許可・抑制する。
- [x] GNSS reacquisition は最初の1点へ hard reset せず、連続した quality、innovation、velocity の
      整合を確認して段階的に再融合する。

完了条件:

- Istanbul yaw RMSE 中央値を current baseline から30%以上改善し、position RMSE の悪化を5%以内にする。
- slip を注入した simulation で NHC が過信せず、非 slip 区間の利得を維持する。
- rejected/accepted update の NIS 分布と理由が machine-readable である。

### Phase 6: 評価基盤と CI

- [x] evaluator に APE、distance/time RPE、horizontal/vertical/yaw、outage endpoint drift、
      reacquisition overshoot/settling time を追加する。
- [x] state/covariance ground truth がある simulation で NEES、各 update の NIS、95% coverage を出す。
- [x] segment 定義、alignment policy、interpolation tolerance、欠測率を manifest に保存する。
- [x] profile sweep は tuning split だけで実行し、holdout は最終1回の合否判定に使う。
- [x] unit test、synthetic Monte Carlo、短い bag smoke test、parameter schema test を CI に入れる。
- [x] full datasets は nightly/manual job とし、結果 artifact と regression summary を保存する。
- [x] real-time factor、callback latency、history memory、rewind 回数を計測する。

回帰 gate:

- open-sky、urban continuous の RMSE/yaw は Phase 0 baseline の +5% 以内。
- Odaiba 81.8 s outage は auto-scale 8.1117 m、fixed-scale 6.3358 m を悪化させない。
- いずれの run も NaN、非 PSD、時刻逆行、silent measurement drop を許さない。
- 同一入力3回の結果は floating-point tolerance 内で一致する。
- accuracy を満たしても NEES/NIS が系統的に過小なら不合格とする。

Current final-code status (2026-07-16): build/test、Istanbul、open-sky、repeatability、NEES/NIS、
input-count gate は合格。Odaiba の adaptive vertical-NHC fixed profile は urban continuous
0.25605 m / 1.3209 deg と 81.8 s outage 3.1455 m、auto-scale companion は outage 6.0311 m で、
それぞれの凍結 gate を通過した。詳細は `docs/research_evaluation.md` を参照。

### Phase 7: optional research track

Phase 0--6 完了後、同じ benchmark で必要性を判定する。

- [ ] invariant EKF/FEJ による unobservable yaw/translation の consistency 改善。
- [ ] online wheel scale、lever arm、time offset calibration。
- [ ] fixed-lag smoother または GTSAM を offline oracle とした EKF 誤差分析。
- [ ] raw GNSS pseudorange/Doppler の tightly coupled fusion と RAIM/NLOS 対応。
- [ ] LiDAR/VIO/radar velocity aid。

採用条件は、holdout で統計的に有意な改善があり、複雑性・計算量・license の増加を説明できること。

## 5. PR 分割

大きな一括変更を避け、次の順で独立に review 可能にする。

1. 数式仕様、parameter 単位、baseline manifest。
2. 有限差分 Jacobian、PSD、Monte Carlo test harness。
3. injection reset、solver、共通 update API。
4. exact discretization と midpoint propagation。
5. state history と delayed measurement replay。
6. stationary initialization と Allan/YAML tooling。
7. GNSS position/velocity/course の相関・NIS 修正。
8. wheel/NHC/slip/observability 改善。
9. evaluator、CI、最終 ablation、documentation。

各 PR は core unit test、該当 synthetic test、最低1つの実 bag regression を添付する。

## 6. 最終成果物

- `docs/eskf_math.md`: 実装と1対1対応する数理仕様。
- `docs/calibration.md`: IMU noise、extrinsic、lever arm、time offset、wheel scale の手順。
- `docs/evaluation_protocol.md`: split、segment、alignment、metrics、合否条件。
- exact/fast propagation と delayed-measurement replay を備えた ROS-free core。
- typed diagnostics と machine-readable update/reject records。
- synthetic simulator、Monte Carlo consistency test、dataset regression runner。
- open-sky、urban continuous、outage、reacquisition、short-bag の最終 ablation report。

この順序では、まず「正しいが遅い reference」を作り、それを oracle に高速化する。yaw や RMSE の
追加 tuning は、数理・時刻・calibration の gate を通過した後に行う。

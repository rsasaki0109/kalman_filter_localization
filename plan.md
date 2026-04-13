# ESKF Accel Bias 実装改善プラン

## 背景

EKF に accelerometer bias 推定を追加したが、実装の結合不備で
position accuracy が劣化していた (0.05m → 0.26m+)。
本プランは修正の経緯と残タスクを記録する。

## 完了済み

### 1. ESKF コア修正 (ekf.hpp)

- **状態ベクトル拡張**: 12→15 error state (accel bias 3軸追加)
- **予測 Jacobian F 修正**:
  - dp/dtheta, dp/dba 結合を追加
  - dv/dba 結合を追加
  - dtheta/dtheta に gyro_skew 項を追加
- **quaternion 積分順序**: `dq * q` → `q * dq` (body-frame)
- **Joseph-form covariance update**: 数値安定性向上
- **P 対称化**: 毎更新後に `P = 0.5*(P+P')`
- **accel bias decay**: first-order Gauss-Markov (tau_acc_bias)

### 2. Process noise L 行列修正

- **問題**: Q に dt² が含まれているのに L でも dt を掛けていた
  → velocity の process noise が dt⁴ スケール (正しくは dt²)
  → filter が予測を過信 → 位置精度 0.26m に劣化
- **修正**: L から dt 係数を除去、Q の dt² スケーリングと整合
- **結果**: bag5 で 0.260m → 0.055m に回復

### 3. ROS2 component (ekf_localization_component.cpp)

- var_imu_acc_bias, initial_imu_acc_bias_covariance, tau_acc_bias_sec パラメータ追加
- /current_accel_bias 診断トピック追加

### 4. Istanbul プロファイル更新

- raw IMU モード: use_imu_orientation=false, use_flat_ground=true
- var_flat_ground_rp=0.05 (sweep で 3 bag 共通の最適値)
- accel bias on: var_imu_acc_bias=1e-8, tau_acc_bias_sec=3600

### 5. 検証結果 (L fix 後、var_flat_ground_rp sweep best)

| bag | rmse_3d [m] | yaw_rmse [deg] | att_rmse [deg] |
|-----|---:|---:|---:|
| bag4 (17s) | 0.075 | 4.77 | 5.59 |
| bag5 (30s) | 0.055 | 4.92 | 5.80 |
| bag6 (22s) | 0.066 | 2.46 | 3.93 |

条件: raw IMU, POSLV yaw init only, accel bias on, gyro bias off

## 残タスク

### 高優先

- [ ] yaw_rmse が 2.5〜5° と大きい。
      GNSS course yaw や gyro bias 推定で改善できるか検討。
- [ ] bag4 の rmse_3d=0.075m がやや大きい (17s で bias 収束が間に合わない可能性)。
      短い bag への対策 (initial bias covariance 調整等) を検討。

### 中優先

- [ ] var_flat_ground_rp の最適値が bag 間で異なる (bag4=0.1, bag5/6=0.05)。
      ロバストな単一値を決めるか、bag 長に応じた adaptive 制御を検討。
- [ ] process noise Q/L の離散化を理論的に正しい形に統一する。
      現状は Q=var*dt², L=I (旧コード互換) で動いているが、
      continuous PSD → discrete の正しい導出に基づく実装に整理すべき。

### 低優先

- [ ] gyro bias 推定 (var_imu_gyro_bias > 0) を有効にして yaw 改善を試す
- [ ] longer bag (bag1〜bag3) での検証
- [ ] PR 作成・マージ

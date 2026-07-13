// Copyright (c) 2020, Ryohei Sasaki
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#ifndef KALMAN_FILTER_LOCALIZATION__CORE__EKF_ESTIMATOR_HPP_
#define KALMAN_FILTER_LOCALIZATION__CORE__EKF_ESTIMATOR_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cstdint>
#include <cmath>

// NOTE:
// This file intentionally contains no ROS2 includes so it can be reused in
// non-ROS2 contexts.
namespace kalman_filter_localization
{
namespace core
{
class EKFEstimator
{
public:
  struct Pose
  {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  };

  struct State
  {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()};
    Eigen::Vector3d accel_bias{Eigen::Vector3d::Zero()};
  };

  enum class PredictionUpdateStatus : std::uint8_t
  {
    kUpdated = 0,
    kSkippedNoTimeBase,
    kNonPositiveDt,
    kDtTooLarge,
  };

  enum class ObservationUpdateStatus : std::uint8_t
  {
    kUpdated = 0,
    kInvalidMeasurement,
    kInvalidVariance,
  };

  enum class RobustLoss : std::uint8_t
  {
    kNone = 0,
    kHuber,
    kCauchy,
  };

  static double computeRobustVarianceScale(
    const double normalized_residual,
    const RobustLoss loss,
    const double tuning_constant,
    const double max_scale)
  {
    if (!std::isfinite(normalized_residual) || normalized_residual <= tuning_constant ||
      !(tuning_constant > 0.0) || !(max_scale >= 1.0))
    {
      return 1.0;
    }
    double scale = 1.0;
    if (loss == RobustLoss::kHuber) {
      scale = normalized_residual / tuning_constant;
    } else if (loss == RobustLoss::kCauchy) {
      const double ratio = normalized_residual / tuning_constant;
      scale = 1.0 + ratio * ratio;
    }
    return std::min(scale, max_scale);
  }

  EKFEstimator()
  : previous_time_imu_(0.0),
    has_previous_time_imu_(false),
    P_(EigenMatrixErrorState::Identity() * 100),
    var_imu_w_{0.33},
    var_imu_acc_{0.33},
    var_imu_gyro_bias_{0.0},
    var_imu_acc_bias_{0.0},
    use_continuous_process_noise_density_{false},
    use_second_order_state_transition_{false},
    use_second_order_process_noise_{false},
    max_prediction_dt_sec_{0.5},
    initial_gyro_bias_covariance_{0.0},
    initial_accel_bias_covariance_{0.0},
    tau_gyro_bias_{3600.0},
    tau_acc_bias_{3600.0}
  {
    /* x  = [p v q bg ba] = [x y z vx vy vz qx qy qz qw bgx bgy bgz bax bay baz] */
    x_ << 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
    applyInitialBiasCovariances();
  }

/* state
* x  = [p v q bg ba] = [x y z vx vy vz qx qy qz qw bgx bgy bgz bax bay baz]
* dx = [dp dv dth dbg dba] =
*      [dx dy dz dvx dvy dvz dthx dthy dthz dbgx dbgy dbgz dbax dbay dbaz]
*
* pos_k = pos_{k-1} + vel_k * dt + (1/2) * (Rot(q_{k-1}) (acc_{k-1}^{imu} - ba_{k-1}) - g) *dt^2
* vel_k = vel_{k-1} + (Rot(quat_{k-1}) (acc_{k-1}^{imu} - ba_{k-1}) - g) *dt
* quat_k = Rot((w_{k-1}^{imu} - bg_{k-1})*dt)*quat_{k-1}
* bg_k = exp(-dt/tau_bg) * bg_{k-1} + noise
* ba_k = exp(-dt/tau_ba) * ba_{k-1} + noise
*
* covariance
* P_{k} = F_k P_{k-1} F_k^T + L Q_k L^T
*/
  // Backward-compatible API: computes dt from timestamp internally and ignores status.
  void predictionUpdate(
    const double current_time_imu,
    const Eigen::Vector3d & gyro,
    const Eigen::Vector3d & linear_acceleration
  )
  {
    (void)predictionUpdateWithStatus(current_time_imu, gyro, linear_acceleration);
  }

  // Returns status instead of printing/logging.
  PredictionUpdateStatus predictionUpdateWithStatus(
    const double current_time_imu,
    const Eigen::Vector3d & gyro,
    const Eigen::Vector3d & linear_acceleration
  )
  {
    // First sample: initialize the time base and skip propagation since dt is unknown.
    if (!has_previous_time_imu_) {
      previous_time_imu_ = current_time_imu;
      has_previous_time_imu_ = true;
      return PredictionUpdateStatus::kSkippedNoTimeBase;
    }

    const double dt_imu = current_time_imu - previous_time_imu_;
    // Always advance the internal time base to allow recovery from large dt.
    previous_time_imu_ = current_time_imu;
    return predictionUpdateDt(dt_imu, gyro, linear_acceleration);
  }

  // Preferred API for non-ROS2 usage: caller supplies dt and handles errors/logging.
  PredictionUpdateStatus predictionUpdateDt(
    const double dt_imu,
    const Eigen::Vector3d & gyro,
    const Eigen::Vector3d & linear_acceleration
  )
  {
    if (dt_imu <= 0.0) {
      return PredictionUpdateStatus::kNonPositiveDt;
    }
    if (dt_imu > max_prediction_dt_sec_) {
      return PredictionUpdateStatus::kDtTooLarge;
    }

    const Eigen::Vector3d gyro_bias = x_.segment(STATE::BGX, 3);
    const Eigen::Vector3d accel_bias = x_.segment(STATE::BAX, 3);
    const Eigen::Vector3d unbiased_gyro = gyro - gyro_bias;

    // Integrate angular velocity using the exponential map.
    const Eigen::Vector3d wdt = unbiased_gyro * dt_imu;
    const double wdt_norm = wdt.norm();
    const Eigen::Quaterniond quat_wdt =
      (wdt_norm > 0.0) ?
      Eigen::Quaterniond(Eigen::AngleAxisd(wdt_norm, wdt / wdt_norm)) :
      Eigen::Quaterniond::Identity();
    const Eigen::Vector3d acc = Eigen::Vector3d(
      linear_acceleration.x(),
      linear_acceleration.y(),
      linear_acceleration.z());
    const Eigen::Vector3d unbiased_acc = acc - accel_bias;

    // state
    Eigen::Quaterniond previous_quat(x_(STATE::QW), x_(STATE::QX), x_(STATE::QY), x_(STATE::QZ));
    previous_quat.normalize();
    const Eigen::Matrix3d rot_mat = previous_quat.toRotationMatrix();

    // pos
    x_.segment(STATE::X, 3) = x_.segment(STATE::X, 3) + dt_imu * x_.segment(STATE::VX, 3) +
      0.5 * dt_imu * dt_imu * (rot_mat * unbiased_acc - gravity_);
    // vel
    x_.segment(STATE::VX, 3) =
      x_.segment(STATE::VX, 3) + dt_imu * (rot_mat * unbiased_acc - gravity_);
    // quat
    const Eigen::Quaterniond predicted_quat = (previous_quat * quat_wdt).normalized();
    x_.segment(STATE::QX, 4) = Eigen::Vector4d(
      predicted_quat.x(), predicted_quat.y(), predicted_quat.z(), predicted_quat.w());
    // imu biases
    const double gyro_bias_decay =
      (tau_gyro_bias_ > 0.0 &&
      std::isfinite(tau_gyro_bias_)) ? std::exp(-dt_imu / tau_gyro_bias_) : 1.0;
    const double accel_bias_decay =
      (tau_acc_bias_ > 0.0 &&
      std::isfinite(tau_acc_bias_)) ? std::exp(-dt_imu / tau_acc_bias_) : 1.0;
    x_.segment(STATE::BGX, 3) = gyro_bias_decay * x_.segment(STATE::BGX, 3);
    x_.segment(STATE::BAX, 3) = accel_bias_decay * x_.segment(STATE::BAX, 3);

    // Error-state transition matrix.
    Eigen::Matrix3d acc_skew;
    acc_skew <<
      0, -unbiased_acc(2), unbiased_acc(1),
      unbiased_acc(2), 0, -unbiased_acc(0),
      -unbiased_acc(1), unbiased_acc(0), 0;
    Eigen::Matrix3d gyro_skew;
    gyro_skew <<
      0, -unbiased_gyro(2), unbiased_gyro(1),
      unbiased_gyro(2), 0, -unbiased_gyro(0),
      -unbiased_gyro(1), unbiased_gyro(0), 0;
    EigenMatrixErrorState F = EigenMatrixErrorState::Identity();
    EigenMatrixErrorState Fc = EigenMatrixErrorState::Zero();
    if (use_second_order_state_transition_) {
      Fc.block<3, 3>(ERROR_STATE::DX, ERROR_STATE::DVX) = Eigen::Matrix3d::Identity();
      Fc.block<3, 3>(ERROR_STATE::DVX, ERROR_STATE::DTHX) = rot_mat * (-acc_skew);
      Fc.block<3, 3>(ERROR_STATE::DVX, ERROR_STATE::DBAX) = -rot_mat;
      Fc.block<3, 3>(ERROR_STATE::DTHX, ERROR_STATE::DTHX) = -gyro_skew;
      Fc.block<3, 3>(ERROR_STATE::DTHX, ERROR_STATE::DBGX) = -Eigen::Matrix3d::Identity();
      if (tau_gyro_bias_ > 0.0 && std::isfinite(tau_gyro_bias_)) {
        Fc.block<3, 3>(ERROR_STATE::DBGX, ERROR_STATE::DBGX) =
          (-1.0 / tau_gyro_bias_) * Eigen::Matrix3d::Identity();
      }
      if (tau_acc_bias_ > 0.0 && std::isfinite(tau_acc_bias_)) {
        Fc.block<3, 3>(ERROR_STATE::DBAX, ERROR_STATE::DBAX) =
          (-1.0 / tau_acc_bias_) * Eigen::Matrix3d::Identity();
      }
      const EigenMatrixErrorState Fc_dt = Fc * dt_imu;
      F += Fc_dt + 0.5 * Fc_dt * Fc_dt;
    } else {
      F.block<3, 3>(ERROR_STATE::DX, ERROR_STATE::DVX) =
        dt_imu * Eigen::Matrix3d::Identity();
      F.block<3, 3>(ERROR_STATE::DX, ERROR_STATE::DTHX) =
        0.5 * rot_mat * (-acc_skew) * dt_imu * dt_imu;
      F.block<3, 3>(ERROR_STATE::DX, ERROR_STATE::DBAX) =
        -0.5 * rot_mat * dt_imu * dt_imu;
      F.block<3, 3>(ERROR_STATE::DVX, ERROR_STATE::DTHX) =
        rot_mat * (-acc_skew) * dt_imu;
      F.block<3, 3>(ERROR_STATE::DVX, ERROR_STATE::DBAX) = -rot_mat * dt_imu;
      F.block<3, 3>(ERROR_STATE::DTHX, ERROR_STATE::DTHX) =
        Eigen::Matrix3d::Identity() - gyro_skew * dt_imu;
      F.block<3, 3>(ERROR_STATE::DTHX, ERROR_STATE::DBGX) =
        -dt_imu * Eigen::Matrix3d::Identity();
      F.block<3, 3>(ERROR_STATE::DBGX, ERROR_STATE::DBGX) =
        gyro_bias_decay * Eigen::Matrix3d::Identity();
      F.block<3, 3>(ERROR_STATE::DBAX, ERROR_STATE::DBAX) =
        accel_bias_decay * Eigen::Matrix3d::Identity();
    }

    // Q
    Eigen::Matrix<double, 12, 12> Q = Eigen::Matrix<double, 12, 12>::Zero();
    const double imu_white_noise_scale =
      use_continuous_process_noise_density_ ? dt_imu : dt_imu * dt_imu;
    Q.block<3, 3>(0, 0) =
      var_imu_acc_ * Eigen::Matrix3d::Identity() * imu_white_noise_scale;
    Q.block<3, 3>(3, 3) =
      var_imu_w_ * Eigen::Matrix3d::Identity() * imu_white_noise_scale;
    Q.block<3, 3>(6, 6) = var_imu_gyro_bias_ * Eigen::Matrix3d::Identity() * dt_imu;
    Q.block<3, 3>(9, 9) = var_imu_acc_bias_ * Eigen::Matrix3d::Identity() * dt_imu;

    // L  –  noise input matrix.
    // Q already carries the discrete dt² / dt scaling, so L must NOT
    // multiply by dt again (otherwise process noise is under-counted).
    Eigen::Matrix<double, num_error_state_, 12> L =
      Eigen::Matrix<double, num_error_state_, 12>::Zero();
    L.block<3, 3>(ERROR_STATE::DVX, 0) = rot_mat;
    L.block<3, 3>(ERROR_STATE::DTHX, 3) = Eigen::Matrix3d::Identity();
    L.block<3, 3>(ERROR_STATE::DBGX, 6) = Eigen::Matrix3d::Identity();
    L.block<3, 3>(ERROR_STATE::DBAX, 9) = Eigen::Matrix3d::Identity();

    EigenMatrixErrorState discrete_process_noise;
    if (use_continuous_process_noise_density_ && use_second_order_process_noise_) {
      Eigen::Matrix<double, 12, 12> Qc = Eigen::Matrix<double, 12, 12>::Zero();
      Qc.block<3, 3>(0, 0) = var_imu_acc_ * Eigen::Matrix3d::Identity();
      Qc.block<3, 3>(3, 3) = var_imu_w_ * Eigen::Matrix3d::Identity();
      Qc.block<3, 3>(6, 6) = var_imu_gyro_bias_ * Eigen::Matrix3d::Identity();
      Qc.block<3, 3>(9, 9) = var_imu_acc_bias_ * Eigen::Matrix3d::Identity();
      const EigenMatrixErrorState Qe = L * Qc * L.transpose();
      const EigenMatrixErrorState Fc2 = Fc * Fc;
      const double dt2 = dt_imu * dt_imu;
      const double dt3 = dt2 * dt_imu;
      discrete_process_noise =
        Qe * dt_imu +
        0.5 * (Fc * Qe + Qe * Fc.transpose()) * dt2 +
        (Fc2 * Qe + 2.0 * Fc * Qe * Fc.transpose() +
        Qe * Fc.transpose() * Fc.transpose()) * (dt3 / 6.0);
    } else {
      discrete_process_noise = L * Q * L.transpose();
    }
    P_ = F * P_ * F.transpose() + discrete_process_noise;
    P_ = 0.5 * (P_ + P_.transpose());
    return PredictionUpdateStatus::kUpdated;
  }

  void resetImuTimeBase()
  {
    previous_time_imu_ = 0.0;
    has_previous_time_imu_ = false;
  }

  // Backward-compatible API: orientation observation update without status.
  void observationUpdateOrientation(
    const Eigen::Quaterniond & y_quat,
    const Eigen::Vector3d & variance_rpy_rad2)
  {
    (void)observationUpdateOrientationWithStatus(y_quat, variance_rpy_rad2);
  }

  // Orientation observation update. This directly constrains the attitude error state.
  //
  // The measurement is a quaternion in the same frame convention as the EKF state
  // (world <- body), with per-axis RPY variances (rad^2) used as a diagonal noise model.
  ObservationUpdateStatus observationUpdateOrientationWithStatus(
    const Eigen::Quaterniond & y_quat,
    const Eigen::Vector3d & variance_rpy_rad2)
  {
    if (!variance_rpy_rad2.allFinite() || (variance_rpy_rad2.array() <= 0.0).any()) {
      return ObservationUpdateStatus::kInvalidVariance;
    }
    if (!y_quat.coeffs().allFinite() || y_quat.norm() <= 0.0) {
      return ObservationUpdateStatus::kInvalidMeasurement;
    }

    const Eigen::Quaterniond y = y_quat.normalized();
    Eigen::Quaterniond q_est(x_(STATE::QW), x_(STATE::QX), x_(STATE::QY), x_(STATE::QZ));
    q_est.normalize();

    // Error quaternion: dq ~= q_est^{-1} * y
    Eigen::Quaterniond dq = q_est.conjugate() * y;
    dq.normalize();
    // Ensure shortest-arc representation (q and -q are equivalent).
    if (dq.w() < 0.0) {
      dq.coeffs() *= -1.0;
    }

    // Axis-angle innovation (robust even if the initial error is not tiny).
    const Eigen::AngleAxisd aa(dq);
    const Eigen::Vector3d innov = aa.axis() * aa.angle();
    if (!innov.allFinite()) {
      return ObservationUpdateStatus::kInvalidMeasurement;
    }

    // Measurement model: innov ≈ dtheta + noise
    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    R(0, 0) = variance_rpy_rad2.x();
    R(1, 1) = variance_rpy_rad2.y();
    R(2, 2) = variance_rpy_rad2.z();

    Eigen::Matrix<double, 3, num_error_state_> H =
      Eigen::Matrix<double, 3, num_error_state_>::Zero();
    H.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();

    const Eigen::Matrix3d S = H * P_ * H.transpose() + R;
    const Eigen::Matrix<double, num_error_state_, 3> K = P_ * H.transpose() * S.inverse();
    const Eigen::Matrix<double, num_error_state_, 1> dx = K * innov;

    applyErrorState(dx);
    const EigenMatrixErrorState I = EigenMatrixErrorState::Identity();
    const EigenMatrixErrorState A = I - K * H;
    P_ = A * P_ * A.transpose() + K * R * K.transpose();
    P_ = 0.5 * (P_ + P_.transpose());
    return ObservationUpdateStatus::kUpdated;
  }

/*
* y = pobs = [xobs yobs zobs]
*
* K = P_k H^T (H P_k H^T + R)^{-1}
*
* dx = K (y_k - p_k )
*
* p_x = p_{k-1} + dp_k
* v_k = v_{k-1} + dv_k
* q_k = Rot(dth) q_{k-1}
*
* P_k = (I - KH)*P_{k-1}
*/
  void observationUpdate(
    const Eigen::Vector3d & y,
    const Eigen::Vector3d & variance
  )
  {
    (void)observationUpdateWithStatus(y, variance);
  }

  ObservationUpdateStatus observationUpdateWithStatus(
    const Eigen::Vector3d & y,
    const Eigen::Vector3d & variance
  )
  {
    if (!y.allFinite()) {
      return ObservationUpdateStatus::kInvalidMeasurement;
    }
    if (!variance.allFinite() || (variance.array() <= 0.0).any()) {
      return ObservationUpdateStatus::kInvalidVariance;
    }

    // error state
    Eigen::Matrix3d R;
    R <<
      variance.x(), 0, 0,
      0, variance.y(), 0,
      0, 0, variance.z();
    Eigen::Matrix<double, 3, num_error_state_> H =
      Eigen::Matrix<double, 3, num_error_state_>::Zero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d S = H * P_ * H.transpose() + R;
    const Eigen::Matrix<double, num_error_state_, 3> K = P_ * H.transpose() * S.inverse();
    const Eigen::Matrix<double, num_error_state_, 1> dx = K * (y - x_.segment(STATE::X, 3));

    applyErrorState(dx);

    const EigenMatrixErrorState I = EigenMatrixErrorState::Identity();
    const EigenMatrixErrorState A = I - K * H;
    P_ = A * P_ * A.transpose() + K * R * K.transpose();
    P_ = 0.5 * (P_ + P_.transpose());
    return ObservationUpdateStatus::kUpdated;
  }

  ObservationUpdateStatus observationUpdatePositionWithLeverArmWithStatus(
    const Eigen::Vector3d & antenna_position_world,
    const Eigen::Vector3d & antenna_lever_arm_body,
    const Eigen::Vector3d & variance)
  {
    if (!antenna_position_world.allFinite() || !antenna_lever_arm_body.allFinite()) {
      return ObservationUpdateStatus::kInvalidMeasurement;
    }
    if (!variance.allFinite() || (variance.array() <= 0.0).any()) {
      return ObservationUpdateStatus::kInvalidVariance;
    }
    Eigen::Quaterniond orientation(
      x_(STATE::QW), x_(STATE::QX), x_(STATE::QY), x_(STATE::QZ));
    orientation.normalize();
    const Eigen::Matrix3d world_from_body = orientation.toRotationMatrix();
    const Eigen::Vector3d predicted_antenna =
      x_.segment(STATE::X, 3) + world_from_body * antenna_lever_arm_body;
    Eigen::Matrix3d lever_arm_skew;
    lever_arm_skew <<
      0.0, -antenna_lever_arm_body.z(), antenna_lever_arm_body.y(),
      antenna_lever_arm_body.z(), 0.0, -antenna_lever_arm_body.x(),
      -antenna_lever_arm_body.y(), antenna_lever_arm_body.x(), 0.0;

    Eigen::Matrix<double, 3, num_error_state_> H =
      Eigen::Matrix<double, 3, num_error_state_>::Zero();
    H.block<3, 3>(0, ERROR_STATE::DX) = Eigen::Matrix3d::Identity();
    H.block<3, 3>(0, ERROR_STATE::DTHX) = -world_from_body * lever_arm_skew;
    const Eigen::Matrix3d R = variance.asDiagonal();
    const Eigen::Matrix3d S = H * P_ * H.transpose() + R;
    const Eigen::Matrix<double, num_error_state_, 3> K =
      P_ * H.transpose() * S.inverse();
    const Eigen::Matrix<double, num_error_state_, 1> dx =
      K * (antenna_position_world - predicted_antenna);
    applyErrorState(dx);
    const EigenMatrixErrorState I = EigenMatrixErrorState::Identity();
    const EigenMatrixErrorState A = I - K * H;
    P_ = A * P_ * A.transpose() + K * R * K.transpose();
    P_ = 0.5 * (P_ + P_.transpose());
    return ObservationUpdateStatus::kUpdated;
  }

  ObservationUpdateStatus observationUpdateGyroBiasWithStatus(
    const Eigen::Vector3d & measured_stationary_gyro,
    const Eigen::Vector3d & variance)
  {
    if (!measured_stationary_gyro.allFinite()) {
      return ObservationUpdateStatus::kInvalidMeasurement;
    }
    if (!variance.allFinite() || (variance.array() <= 0.0).any()) {
      return ObservationUpdateStatus::kInvalidVariance;
    }
    Eigen::Matrix<double, 3, num_error_state_> H =
      Eigen::Matrix<double, 3, num_error_state_>::Zero();
    H.block<3, 3>(0, ERROR_STATE::DBGX) = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d R = variance.asDiagonal();
    const Eigen::Matrix3d S = H * P_ * H.transpose() + R;
    const Eigen::Matrix<double, num_error_state_, 3> K =
      P_ * H.transpose() * S.inverse();
    const Eigen::Matrix<double, num_error_state_, 1> dx =
      K * (measured_stationary_gyro - x_.segment(STATE::BGX, 3));
    applyErrorState(dx);
    const EigenMatrixErrorState I = EigenMatrixErrorState::Identity();
    const EigenMatrixErrorState A = I - K * H;
    P_ = A * P_ * A.transpose() + K * R * K.transpose();
    P_ = 0.5 * (P_ + P_.transpose());
    return ObservationUpdateStatus::kUpdated;
  }

  void observationUpdateVelocity(
    const Eigen::Vector3d & y,
    const Eigen::Vector3d & variance
  )
  {
    (void)observationUpdateVelocityWithStatus(y, variance);
  }

  ObservationUpdateStatus observationUpdateVelocityWithStatus(
    const Eigen::Vector3d & y,
    const Eigen::Vector3d & variance,
    const bool propagate_cross_state = false
  )
  {
    if (!y.allFinite()) {
      return ObservationUpdateStatus::kInvalidMeasurement;
    }
    if (!variance.allFinite() || (variance.array() <= 0.0).any()) {
      return ObservationUpdateStatus::kInvalidVariance;
    }

    Eigen::Matrix3d R;
    R <<
      variance.x(), 0, 0,
      0, variance.y(), 0,
      0, 0, variance.z();

    Eigen::Matrix<double, 3, num_error_state_> H =
      Eigen::Matrix<double, 3, num_error_state_>::Zero();
    H.block<3, 3>(0, ERROR_STATE::DVX) = Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, num_error_state_, 3> K =
      Eigen::Matrix<double, num_error_state_, 3>::Zero();

    if (propagate_cross_state) {
      K = P_ * H.transpose() * (H * P_ * H.transpose() + R).inverse();
    } else {
      const Eigen::Matrix3d P_vv = P_.block<3, 3>(ERROR_STATE::DVX, ERROR_STATE::DVX);
      K.block<3, 3>(ERROR_STATE::DVX, 0) = P_vv * (P_vv + R).inverse();
    }

    const Eigen::Matrix<double, num_error_state_, 1> dx = K * (y - x_.segment(STATE::VX, 3));

    applyErrorState(dx);

    const EigenMatrixErrorState I = EigenMatrixErrorState::Identity();
    const EigenMatrixErrorState A = I - K * H;
    P_ = A * P_ * A.transpose() + K * R * K.transpose();
    P_ = 0.5 * (P_ + P_.transpose());
    return ObservationUpdateStatus::kUpdated;
  }

  // Constrain lateral and vertical velocity expressed in the body frame.
  // For right-multiplicative attitude error, h = R^T v has Jacobians
  // dh / d(dtheta) = skew(h) and dh / d(dv) = R^T.
  ObservationUpdateStatus observationUpdateBodyVelocityConstraintWithStatus(
    const Eigen::Vector2d & lateral_vertical_velocity,
    const Eigen::Vector2d & variance)
  {
    if (!lateral_vertical_velocity.allFinite()) {
      return ObservationUpdateStatus::kInvalidMeasurement;
    }
    if (!variance.allFinite() || (variance.array() <= 0.0).any()) {
      return ObservationUpdateStatus::kInvalidVariance;
    }

    Eigen::Quaterniond orientation(
      x_(STATE::QW), x_(STATE::QX), x_(STATE::QY), x_(STATE::QZ));
    orientation.normalize();
    const Eigen::Matrix3d body_from_world = orientation.toRotationMatrix().transpose();
    const Eigen::Vector3d body_velocity = body_from_world * x_.segment(STATE::VX, 3);

    Eigen::Matrix<double, 2, num_error_state_> H =
      Eigen::Matrix<double, 2, num_error_state_>::Zero();
    Eigen::Matrix3d body_velocity_skew;
    body_velocity_skew <<
      0.0, -body_velocity.z(), body_velocity.y(),
      body_velocity.z(), 0.0, -body_velocity.x(),
      -body_velocity.y(), body_velocity.x(), 0.0;
    H.block<2, 3>(0, ERROR_STATE::DVX) = body_from_world.block<2, 3>(1, 0);
    H.block<2, 3>(0, ERROR_STATE::DTHX) = body_velocity_skew.block<2, 3>(1, 0);

    const Eigen::Matrix2d R = variance.asDiagonal();
    const Eigen::Matrix2d S = H * P_ * H.transpose() + R;
    const Eigen::Matrix<double, num_error_state_, 2> K =
      P_ * H.transpose() * S.inverse();
    const Eigen::Vector2d predicted = body_velocity.tail<2>();
    const Eigen::Matrix<double, num_error_state_, 1> dx =
      K * (lateral_vertical_velocity - predicted);

    applyErrorState(dx);
    const EigenMatrixErrorState I = EigenMatrixErrorState::Identity();
    const EigenMatrixErrorState A = I - K * H;
    P_ = A * P_ * A.transpose() + K * R * K.transpose();
    P_ = 0.5 * (P_ + P_.transpose());
    return ObservationUpdateStatus::kUpdated;
  }

  // Observe all velocity components in the body frame. This supports a forward wheel-speed
  // measurement together with lateral/vertical non-holonomic constraints in one consistent
  // update. For right-multiplicative attitude error, h = R^T v and
  // dh / d(dtheta) = skew(h), dh / d(dv) = R^T.
  ObservationUpdateStatus observationUpdateBodyVelocityWithStatus(
    const Eigen::Vector3d & measured_body_velocity,
    const Eigen::Vector3d & variance)
  {
    if (!measured_body_velocity.allFinite()) {
      return ObservationUpdateStatus::kInvalidMeasurement;
    }
    if (!variance.allFinite() || (variance.array() <= 0.0).any()) {
      return ObservationUpdateStatus::kInvalidVariance;
    }

    Eigen::Quaterniond orientation(
      x_(STATE::QW), x_(STATE::QX), x_(STATE::QY), x_(STATE::QZ));
    orientation.normalize();
    const Eigen::Matrix3d body_from_world = orientation.toRotationMatrix().transpose();
    const Eigen::Vector3d predicted = body_from_world * x_.segment(STATE::VX, 3);
    Eigen::Matrix3d predicted_skew;
    predicted_skew <<
      0.0, -predicted.z(), predicted.y(),
      predicted.z(), 0.0, -predicted.x(),
      -predicted.y(), predicted.x(), 0.0;

    Eigen::Matrix<double, 3, num_error_state_> H =
      Eigen::Matrix<double, 3, num_error_state_>::Zero();
    H.block<3, 3>(0, ERROR_STATE::DVX) = body_from_world;
    H.block<3, 3>(0, ERROR_STATE::DTHX) = predicted_skew;
    const Eigen::Matrix3d R = variance.asDiagonal();
    const Eigen::Matrix3d S = H * P_ * H.transpose() + R;
    const Eigen::Matrix<double, num_error_state_, 3> K =
      P_ * H.transpose() * S.inverse();
    const Eigen::Matrix<double, num_error_state_, 1> dx =
      K * (measured_body_velocity - predicted);

    applyErrorState(dx);
    const EigenMatrixErrorState I = EigenMatrixErrorState::Identity();
    const EigenMatrixErrorState A = I - K * H;
    P_ = A * P_ * A.transpose() + K * R * K.transpose();
    P_ = 0.5 * (P_ + P_.transpose());
    return ObservationUpdateStatus::kUpdated;
  }

  // Observe only forward body-frame velocity. A wheel-speed sensor does not measure
  // lateral or vertical velocity; those are separate non-holonomic constraints.
  ObservationUpdateStatus observationUpdateBodyForwardSpeedWithStatus(
    const double measured_forward_speed,
    const double variance)
  {
    if (!std::isfinite(measured_forward_speed)) {
      return ObservationUpdateStatus::kInvalidMeasurement;
    }
    if (!(variance > 0.0) || !std::isfinite(variance)) {
      return ObservationUpdateStatus::kInvalidVariance;
    }
    Eigen::Quaterniond orientation(
      x_(STATE::QW), x_(STATE::QX), x_(STATE::QY), x_(STATE::QZ));
    orientation.normalize();
    const Eigen::Matrix3d body_from_world = orientation.toRotationMatrix().transpose();
    const Eigen::Vector3d predicted = body_from_world * x_.segment(STATE::VX, 3);
    Eigen::Matrix<double, 1, num_error_state_> H =
      Eigen::Matrix<double, 1, num_error_state_>::Zero();
    H.block<1, 3>(0, ERROR_STATE::DVX) = body_from_world.row(0);
    const Eigen::Matrix2d horizontal_velocity_covariance =
      P_.block<2, 2>(ERROR_STATE::DVX, ERROR_STATE::DVX);
    const Eigen::RowVector2d horizontal_velocity_jacobian =
      body_from_world.block<1, 2>(0, 0);
    const double innovation_variance =
      (horizontal_velocity_jacobian * horizontal_velocity_covariance *
      horizontal_velocity_jacobian.transpose())(0, 0) + variance;
    Eigen::Matrix<double, num_error_state_, 1> K =
      Eigen::Matrix<double, num_error_state_, 1>::Zero();
    K.block<2, 1>(ERROR_STATE::DVX, 0) =
      horizontal_velocity_covariance * horizontal_velocity_jacobian.transpose() /
      innovation_variance;
    const Eigen::Matrix<double, num_error_state_, 1> dx =
      K * (measured_forward_speed - predicted.x());
    applyErrorState(dx);
    const EigenMatrixErrorState I = EigenMatrixErrorState::Identity();
    const EigenMatrixErrorState A = I - K * H;
    P_ = A * P_ * A.transpose() + variance * K * K.transpose();
    P_ = 0.5 * (P_ + P_.transpose());
    return ObservationUpdateStatus::kUpdated;
  }

  void setTauGyroBias(const double tau_gyro_bias)
  {
    tau_gyro_bias_ = tau_gyro_bias;
  }

  void setVarImuGyroBias(const double var_imu_gyro_bias)
  {
    var_imu_gyro_bias_ = var_imu_gyro_bias;
  }

  bool setInitialGyroBiasCovariance(const double covariance)
  {
    if (!(covariance >= 0.0) || !std::isfinite(covariance)) {
      return false;
    }
    initial_gyro_bias_covariance_ = covariance;
    applyInitialBiasCovariances();
    return true;
  }

  void setTauAccBias(const double tau_acc_bias)
  {
    tau_acc_bias_ = tau_acc_bias;
  }

  void setVarImuAccBias(const double var_imu_acc_bias)
  {
    var_imu_acc_bias_ = var_imu_acc_bias;
  }

  bool setInitialAccelBiasCovariance(const double covariance)
  {
    if (!(covariance >= 0.0) || !std::isfinite(covariance)) {
      return false;
    }
    initial_accel_bias_covariance_ = covariance;
    applyInitialBiasCovariances();
    return true;
  }

  void setVarImuGyro(const double var_imu_w)
  {
    var_imu_w_ = var_imu_w;
  }

  void setVarImuAcc(const double var_imu_acc)
  {
    var_imu_acc_ = var_imu_acc;
  }

  void setUseContinuousProcessNoiseDensity(const bool enabled)
  {
    use_continuous_process_noise_density_ = enabled;
  }

  void setUseSecondOrderStateTransition(const bool enabled)
  {
    use_second_order_state_transition_ = enabled;
  }

  void setUseSecondOrderProcessNoise(const bool enabled)
  {
    use_second_order_process_noise_ = enabled;
  }

  bool setMaxPredictionDtSec(const double max_prediction_dt_sec)
  {
    if (!(max_prediction_dt_sec > 0.0) || !std::isfinite(max_prediction_dt_sec)) {
      return false;
    }
    max_prediction_dt_sec_ = max_prediction_dt_sec;
    return true;
  }

  double getMaxPredictionDtSec() const
  {
    return max_prediction_dt_sec_;
  }

  bool setGravityZ(const double gravity_z_mps2)
  {
    if (!std::isfinite(gravity_z_mps2) || gravity_z_mps2 < 0.0) {
      return false;
    }
    gravity_ = Eigen::Vector3d(0.0, 0.0, gravity_z_mps2);
    return true;
  }

  double getGravityZ() const
  {
    return gravity_.z();
  }

  bool setInitialXChecked(const Eigen::Ref<const Eigen::VectorXd> & x)
  {
    if (x.size() != num_state_) {
      return false;
    }
    x_ = x;
    return true;
  }

  void setInitialX(Eigen::VectorXd x)
  {
    (void)setInitialXChecked(x);
  }

  // Typed state accessors. These avoid leaking the internal state vector layout.
  void setPose(const Pose & pose)
  {
    setState({pose.position, getVelocity(), pose.orientation, getGyroBias(), getAccelBias()});
  }

  void setState(const State & state)
  {
    x_.segment(STATE::X, 3) = state.position;
    x_.segment(STATE::VX, 3) = state.velocity;
    const Eigen::Quaterniond q = state.orientation.normalized();
    x_(STATE::QX) = q.x();
    x_(STATE::QY) = q.y();
    x_(STATE::QZ) = q.z();
    x_(STATE::QW) = q.w();
    x_.segment(STATE::BGX, 3) = state.gyro_bias;
    x_.segment(STATE::BAX, 3) = state.accel_bias;
  }

  Pose getPose() const
  {
    Pose pose;
    pose.position = getPosition();
    pose.orientation = getOrientation();
    return pose;
  }

  State getState() const
  {
    State state;
    state.position = getPosition();
    state.velocity = getVelocity();
    state.orientation = getOrientation();
    state.gyro_bias = getGyroBias();
    state.accel_bias = getAccelBias();
    return state;
  }

  Eigen::Vector3d getPosition() const
  {
    return x_.segment(STATE::X, 3);
  }

  Eigen::Vector3d getVelocity() const
  {
    return x_.segment(STATE::VX, 3);
  }

  Eigen::Quaterniond getOrientation() const
  {
    return Eigen::Quaterniond(x_(STATE::QW), x_(STATE::QX), x_(STATE::QY), x_(STATE::QZ));
  }

  Eigen::Vector3d getGyroBias() const
  {
    return x_.segment(STATE::BGX, 3);
  }

  Eigen::Vector3d getAccelBias() const
  {
    return x_.segment(STATE::BAX, 3);
  }

  Eigen::VectorXd getX()
  {
    return x_;
  }

  Eigen::VectorXd getX() const
  {
    return x_;
  }

  Eigen::MatrixXd getCoveriance() const
  {
    return getCovariance();
  }

  Eigen::MatrixXd getCovariance() const
  {
    return P_;
  }

  bool capPositionCovariance(const double max_xy, const double max_z)
  {
    if (!(max_xy > 0.0) || !(max_z > 0.0) || !std::isfinite(max_xy) || !std::isfinite(max_z)) {
      return false;
    }
    return capErrorStateCovariance(max_xy, max_z, 0.0, 0.0, 0.0, 0.0);
  }

  bool capErrorStateCovariance(
    const double max_pos_xy,
    const double max_pos_z,
    const double max_vel_xy,
    const double max_vel_z,
    const double max_attitude_rp,
    const double max_attitude_yaw)
  {
    const double caps[9] = {
      max_pos_xy, max_pos_xy, max_pos_z,
      max_vel_xy, max_vel_xy, max_vel_z,
      max_attitude_rp, max_attitude_rp, max_attitude_yaw};
    for (const double cap : caps) {
      if (cap < 0.0 || !std::isfinite(cap)) {
        return false;
      }
    }
    if (max_pos_xy == 0.0 && max_pos_z == 0.0 && max_vel_xy == 0.0 && max_vel_z == 0.0 &&
      max_attitude_rp == 0.0 && max_attitude_yaw == 0.0)
    {
      return false;
    }

    EigenMatrixErrorState scale = EigenMatrixErrorState::Identity();
    bool changed = false;
    for (int i = 0; i < 9; ++i) {
      const double p = P_(i, i);
      if (caps[i] > 0.0 && std::isfinite(p) && p > caps[i]) {
        scale(i, i) = std::sqrt(caps[i] / p);
        changed = true;
      }
    }
    if (!changed) {
      return true;
    }

    P_ = scale * P_ * scale;
    P_ = 0.5 * (P_ + P_.transpose());
    return true;
  }

  int getNumState() const
  {
    return num_state_;
  }

private:
  static const int num_state_{16};
  static const int num_error_state_{15};

  typedef Eigen::Matrix<double, num_error_state_, num_error_state_> EigenMatrixErrorState;

  enum STATE
  {
    X  = 0, Y = 1, Z = 2,
    VX = 3, VY = 4, VZ = 5,
    QX = 6, QY = 7, QZ = 8, QW = 9,
    BGX = 10, BGY = 11, BGZ = 12,
    BAX = 13, BAY = 14, BAZ = 15,
  };
  enum ERROR_STATE
  {
    DX   = 0, DY = 1, DZ = 2,
    DVX  = 3, DVY = 4, DVZ = 5,
    DTHX = 6, DTHY = 7, DTHZ = 8,
    DBGX = 9, DBGY = 10, DBGZ = 11,
    DBAX = 12, DBAY = 13, DBAZ = 14,
  };

  void applyInitialBiasCovariances()
  {
    P_.block<3, 3>(ERROR_STATE::DBGX, ERROR_STATE::DBGX) =
      initial_gyro_bias_covariance_ * Eigen::Matrix3d::Identity();
    P_.block<3, 3>(ERROR_STATE::DBAX, ERROR_STATE::DBAX) =
      initial_accel_bias_covariance_ * Eigen::Matrix3d::Identity();
  }

  void applyErrorState(const Eigen::Ref<const Eigen::Matrix<double, num_error_state_, 1>> & dx)
  {
    // position / velocity
    x_.segment(STATE::X, 3) = x_.segment(STATE::X, 3) + dx.segment(ERROR_STATE::DX, 3);
    x_.segment(STATE::VX, 3) = x_.segment(STATE::VX, 3) + dx.segment(ERROR_STATE::DVX, 3);

    // orientation (small-angle update, right-multiplicative)
    const Eigen::Vector3d dtheta =
      Eigen::Vector3d(dx(ERROR_STATE::DTHX), dx(ERROR_STATE::DTHY), dx(ERROR_STATE::DTHZ));
    const double norm = dtheta.norm();
    Eigen::Quaterniond dq;
    if (norm < 1e-12) {
      dq = Eigen::Quaterniond::Identity();
    } else {
      dq = Eigen::Quaterniond(
        std::cos(norm / 2),
        std::sin(norm / 2) * dtheta.x() / norm,
        std::sin(norm / 2) * dtheta.y() / norm,
        std::sin(norm / 2) * dtheta.z() / norm);
    }

    Eigen::Quaterniond q(x_(STATE::QW), x_(STATE::QX), x_(STATE::QY), x_(STATE::QZ));
    q.normalize();
    const Eigen::Quaterniond q_new = (q * dq).normalized();
    x_.segment(STATE::QX, 4) = Eigen::Vector4d(q_new.x(), q_new.y(), q_new.z(), q_new.w());

    // gyro bias
    x_.segment(STATE::BGX, 3) = x_.segment(STATE::BGX, 3) + dx.segment(ERROR_STATE::DBGX, 3);
    // accel bias
    x_.segment(STATE::BAX, 3) = x_.segment(STATE::BAX, 3) + dx.segment(ERROR_STATE::DBAX, 3);
  }

  double previous_time_imu_;
  bool has_previous_time_imu_;
  double var_imu_w_;
  double var_imu_acc_;
  double var_imu_gyro_bias_;
  double var_imu_acc_bias_;
  bool use_continuous_process_noise_density_;
  bool use_second_order_state_transition_;
  bool use_second_order_process_noise_;
  double max_prediction_dt_sec_;
  double initial_gyro_bias_covariance_;
  double initial_accel_bias_covariance_;

  Eigen::Matrix<double, num_state_, 1> x_;
  EigenMatrixErrorState P_;

  Eigen::Vector3d gravity_{0.0, 0.0, 9.80665};

  double tau_gyro_bias_;
  double tau_acc_bias_;
};
}  // namespace core

using EKFEstimator = core::EKFEstimator;
}  // namespace kalman_filter_localization

// Backward-compatible global alias (original API).
using EKFEstimator = kalman_filter_localization::core::EKFEstimator;

#endif  // KALMAN_FILTER_LOCALIZATION__CORE__EKF_ESTIMATOR_HPP_

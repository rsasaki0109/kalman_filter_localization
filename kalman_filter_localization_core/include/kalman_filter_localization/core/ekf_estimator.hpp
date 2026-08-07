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
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <limits>

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
    kInvalidInput,
    kNumericalFailure,
  };

  enum class ObservationUpdateStatus : std::uint8_t
  {
    kUpdated = 0,
    kInvalidMeasurement,
    kInvalidVariance,
    kNumericalFailure,
  };

  enum class ObservationRejectReason : std::uint8_t
  {
    kNone = 0,
    kInvalidMeasurement,
    kInvalidCovariance,
    kInnovationNotPositiveDefinite,
    kNisGate,
    kNumericalInvariant,
  };

  struct ObservationUpdateDiagnostics3
  {
    ObservationUpdateStatus status{ObservationUpdateStatus::kInvalidMeasurement};
    ObservationRejectReason reason{ObservationRejectReason::kInvalidMeasurement};
    Eigen::Vector3d innovation{Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN())};
    Eigen::Matrix3d innovation_covariance{Eigen::Matrix3d::Constant(
        std::numeric_limits<double>::quiet_NaN())};
    double nis{std::numeric_limits<double>::quiet_NaN()};
    bool accepted{false};
  };

  struct ObservationUpdateDiagnostics1
  {
    ObservationUpdateStatus status{ObservationUpdateStatus::kInvalidMeasurement};
    ObservationRejectReason reason{ObservationRejectReason::kInvalidMeasurement};
    double innovation{std::numeric_limits<double>::quiet_NaN()};
    double innovation_variance{std::numeric_limits<double>::quiet_NaN()};
    double nis{std::numeric_limits<double>::quiet_NaN()};
    bool accepted{false};
  };

  enum class RobustLoss : std::uint8_t
  {
    kNone = 0,
    kHuber,
    kCauchy,
  };

  enum class PropagationModel : std::uint8_t
  {
    kLegacy = 0,
    kFast,
    kExact,
  };

  static constexpr int kErrorStateSize = 15;
  using ErrorStateVector = Eigen::Matrix<double, kErrorStateSize, 1>;
  using ErrorStateMatrix = Eigen::Matrix<double, kErrorStateSize, kErrorStateSize>;
  using ObservationJacobian3 = Eigen::Matrix<double, 3, kErrorStateSize>;
  using NoiseInputMatrix = Eigen::Matrix<double, kErrorStateSize, 12>;
  using ContinuousNoiseMatrix = Eigen::Matrix<double, 12, 12>;

  struct DiscreteErrorModel
  {
    ErrorStateMatrix transition{ErrorStateMatrix::Identity()};
    ErrorStateMatrix process_covariance{ErrorStateMatrix::Zero()};
  };

  struct Snapshot
  {
    State state{};
    ErrorStateMatrix covariance{ErrorStateMatrix::Identity()};
    double previous_time_imu{0.0};
    bool has_previous_time_imu{false};
    Eigen::Vector3d previous_gyro_measurement{Eigen::Vector3d::Zero()};
    Eigen::Vector3d previous_accel_measurement{Eigen::Vector3d::Zero()};
    bool has_previous_imu_measurement{false};
    bool gyro_bias_learning_enabled{true};
    bool accel_bias_learning_enabled{true};
  };

  static Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d & value)
  {
    Eigen::Matrix3d result;
    result <<
      0.0, -value.z(), value.y(),
      value.z(), 0.0, -value.x(),
      -value.y(), value.x(), 0.0;
    return result;
  }

  static Eigen::Matrix3d rightJacobianSO3(const Eigen::Vector3d & rotation_vector)
  {
    const double angle = rotation_vector.norm();
    const Eigen::Matrix3d skew = skewSymmetric(rotation_vector);
    if (angle < 1.0e-5) {
      return Eigen::Matrix3d::Identity() - 0.5 * skew + (1.0 / 6.0) * skew * skew;
    }
    const double angle_squared = angle * angle;
    return Eigen::Matrix3d::Identity() -
           ((1.0 - std::cos(angle)) / angle_squared) * skew +
           ((angle - std::sin(angle)) / (angle_squared * angle)) * skew * skew;
  }

  static ErrorStateMatrix continuousErrorStateJacobian(
    const State & state,
    const Eigen::Vector3d & gyro,
    const Eigen::Vector3d & linear_acceleration,
    const double tau_gyro_bias,
    const double tau_accel_bias)
  {
    const Eigen::Quaterniond orientation = state.orientation.normalized();
    const Eigen::Matrix3d world_from_body = orientation.toRotationMatrix();
    const Eigen::Vector3d corrected_gyro = gyro - state.gyro_bias;
    const Eigen::Vector3d corrected_accel = linear_acceleration - state.accel_bias;
    ErrorStateMatrix result = ErrorStateMatrix::Zero();
    result.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
    result.block<3, 3>(3, 6) = -world_from_body * skewSymmetric(corrected_accel);
    result.block<3, 3>(3, 12) = -world_from_body;
    result.block<3, 3>(6, 6) = -skewSymmetric(corrected_gyro);
    result.block<3, 3>(6, 9) = -Eigen::Matrix3d::Identity();
    if (tau_gyro_bias > 0.0 && std::isfinite(tau_gyro_bias)) {
      result.block<3, 3>(9, 9) =
        (-1.0 / tau_gyro_bias) * Eigen::Matrix3d::Identity();
    }
    if (tau_accel_bias > 0.0 && std::isfinite(tau_accel_bias)) {
      result.block<3, 3>(12, 12) =
        (-1.0 / tau_accel_bias) * Eigen::Matrix3d::Identity();
    }
    return result;
  }

  static ErrorStateMatrix secondOrderErrorStateTransition(
    const ErrorStateMatrix & continuous_jacobian, const double dt)
  {
    const ErrorStateMatrix scaled = continuous_jacobian * dt;
    return ErrorStateMatrix::Identity() + scaled + 0.5 * scaled * scaled;
  }

  static NoiseInputMatrix continuousNoiseInputJacobian(const State & state)
  {
    const Eigen::Matrix3d world_from_body =
      state.orientation.normalized().toRotationMatrix();
    NoiseInputMatrix result = NoiseInputMatrix::Zero();
    result.block<3, 3>(3, 0) = -world_from_body;
    result.block<3, 3>(6, 3) = -Eigen::Matrix3d::Identity();
    result.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity();
    result.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity();
    return result;
  }

  static ContinuousNoiseMatrix continuousNoiseCovariance(
    const double accelerometer_noise_density,
    const double gyroscope_noise_density,
    const double gyro_bias_driving_noise,
    const double accel_bias_driving_noise)
  {
    ContinuousNoiseMatrix result = ContinuousNoiseMatrix::Zero();
    result.block<3, 3>(0, 0) =
      accelerometer_noise_density * Eigen::Matrix3d::Identity();
    result.block<3, 3>(3, 3) =
      gyroscope_noise_density * Eigen::Matrix3d::Identity();
    result.block<3, 3>(6, 6) =
      gyro_bias_driving_noise * Eigen::Matrix3d::Identity();
    result.block<3, 3>(9, 9) =
      accel_bias_driving_noise * Eigen::Matrix3d::Identity();
    return result;
  }

  static DiscreteErrorModel exactDiscretizeErrorModel(
    const ErrorStateMatrix & continuous_jacobian,
    const ErrorStateMatrix & continuous_error_noise_covariance,
    const double dt)
  {
    using VanLoanMatrix = Eigen::Matrix<double, 2 * kErrorStateSize, 2 * kErrorStateSize>;
    VanLoanMatrix generator = VanLoanMatrix::Zero();
    generator.block<kErrorStateSize, kErrorStateSize>(0, 0) = continuous_jacobian;
    generator.block<kErrorStateSize, kErrorStateSize>(0, kErrorStateSize) =
      continuous_error_noise_covariance;
    generator.block<kErrorStateSize, kErrorStateSize>(kErrorStateSize, kErrorStateSize) =
      -continuous_jacobian.transpose();
    const VanLoanMatrix exponential = (generator * dt).exp();
    DiscreteErrorModel result;
    result.transition = exponential.block<kErrorStateSize, kErrorStateSize>(0, 0);
    result.process_covariance =
      exponential.block<kErrorStateSize, kErrorStateSize>(0, kErrorStateSize) *
      result.transition.transpose();
    result.process_covariance =
      0.5 * (result.process_covariance + result.process_covariance.transpose());
    return result;
  }

  static Eigen::Vector3d positionObservation(
    const State & state, ObservationJacobian3 * jacobian = nullptr)
  {
    if (jacobian != nullptr) {
      jacobian->setZero();
      jacobian->template block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    }
    return state.position;
  }

  static Eigen::Vector3d leverArmPositionObservation(
    const State & state, const Eigen::Vector3d & lever_arm_body,
    ObservationJacobian3 * jacobian = nullptr)
  {
    const Eigen::Matrix3d world_from_body = state.orientation.normalized().toRotationMatrix();
    if (jacobian != nullptr) {
      jacobian->setZero();
      jacobian->template block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
      jacobian->template block<3, 3>(0, 6) =
        -world_from_body * skewSymmetric(lever_arm_body);
    }
    return state.position + world_from_body * lever_arm_body;
  }

  static Eigen::Vector3d worldVelocityObservation(
    const State & state, ObservationJacobian3 * jacobian = nullptr)
  {
    if (jacobian != nullptr) {
      jacobian->setZero();
      jacobian->template block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
    }
    return state.velocity;
  }

  static double yawObservation(
    const State & state,
    Eigen::Matrix<double, 1, kErrorStateSize> * jacobian = nullptr)
  {
    const Eigen::Matrix3d rotation = state.orientation.normalized().toRotationMatrix();
    const double x = rotation(0, 0);
    const double y = rotation(1, 0);
    const double denominator = x * x + y * y;
    if (jacobian != nullptr) {
      jacobian->setZero();
      if (denominator > 1.0e-12) {
        (*jacobian)(0, 7) = (-x * rotation(1, 2) + y * rotation(0, 2)) /
          denominator;
        (*jacobian)(0, 8) = (x * rotation(1, 1) - y * rotation(0, 1)) /
          denominator;
      } else {
        jacobian->setConstant(std::numeric_limits<double>::quiet_NaN());
      }
    }
    return std::atan2(y, x);
  }

  static Eigen::Vector3d bodyVelocityObservation(
    const State & state, ObservationJacobian3 * jacobian = nullptr)
  {
    const Eigen::Matrix3d body_from_world =
      state.orientation.normalized().toRotationMatrix().transpose();
    const Eigen::Vector3d body_velocity = body_from_world * state.velocity;
    if (jacobian != nullptr) {
      jacobian->setZero();
      jacobian->template block<3, 3>(0, 3) = body_from_world;
      jacobian->template block<3, 3>(0, 6) = skewSymmetric(body_velocity);
    }
    return body_velocity;
  }

  static Eigen::Vector3d gyroBiasObservation(
    const State & state, ObservationJacobian3 * jacobian = nullptr)
  {
    if (jacobian != nullptr) {
      jacobian->setZero();
      jacobian->template block<3, 3>(0, 9) = Eigen::Matrix3d::Identity();
    }
    return state.gyro_bias;
  }

  static ObservationJacobian3 orientationObservationJacobian()
  {
    ObservationJacobian3 result = ObservationJacobian3::Zero();
    result.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();
    return result;
  }

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
    propagation_model_{PropagationModel::kLegacy},
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
    if (!std::isfinite(dt_imu) || !gyro.allFinite() || !linear_acceleration.allFinite()) {
      return PredictionUpdateStatus::kInvalidInput;
    }
    if (dt_imu <= 0.0) {
      return PredictionUpdateStatus::kNonPositiveDt;
    }
    if (dt_imu > max_prediction_dt_sec_) {
      return PredictionUpdateStatus::kDtTooLarge;
    }

    const Eigen::Matrix<double, num_state_, 1> state_before = x_;
    const EigenMatrixErrorState covariance_before = P_;
    Eigen::Vector3d integration_gyro = gyro;
    Eigen::Vector3d integration_acceleration = linear_acceleration;
    if (propagation_model_ != PropagationModel::kLegacy) {
      if (has_previous_imu_measurement_) {
        integration_gyro = 0.5 * (previous_gyro_measurement_ + gyro);
        integration_acceleration = 0.5 * (previous_accel_measurement_ + linear_acceleration);
      }
      previous_gyro_measurement_ = gyro;
      previous_accel_measurement_ = linear_acceleration;
      has_previous_imu_measurement_ = true;
    }
    const Eigen::Vector3d gyro_bias = x_.segment(STATE::BGX, 3);
    const Eigen::Vector3d accel_bias = x_.segment(STATE::BAX, 3);
    const Eigen::Vector3d unbiased_gyro = integration_gyro - gyro_bias;

    // Integrate angular velocity using the exponential map.
    const Eigen::Vector3d wdt = unbiased_gyro * dt_imu;
    const double wdt_norm = wdt.norm();
    const Eigen::Quaterniond quat_wdt =
      (wdt_norm > 0.0) ?
      Eigen::Quaterniond(Eigen::AngleAxisd(wdt_norm, wdt / wdt_norm)) :
      Eigen::Quaterniond::Identity();
    const Eigen::Vector3d acc = Eigen::Vector3d(
      integration_acceleration.x(),
      integration_acceleration.y(),
      integration_acceleration.z());
    const Eigen::Vector3d unbiased_acc = acc - accel_bias;

    // state
    Eigen::Quaterniond previous_quat(x_(STATE::QW), x_(STATE::QX), x_(STATE::QY), x_(STATE::QZ));
    previous_quat.normalize();
    const Eigen::Quaterniond half_rotation = wdt_norm > 0.0 ?
      Eigen::Quaterniond(Eigen::AngleAxisd(0.5 * wdt_norm, wdt / wdt_norm)) :
      Eigen::Quaterniond::Identity();
    const Eigen::Quaterniond acceleration_orientation =
      propagation_model_ == PropagationModel::kLegacy ? previous_quat :
      (previous_quat * half_rotation).normalized();
    const Eigen::Matrix3d rot_mat = acceleration_orientation.toRotationMatrix();
    const State linearization_state{
      x_.segment(STATE::X, 3), x_.segment(STATE::VX, 3), acceleration_orientation,
      gyro_bias, accel_bias};

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
    const EigenMatrixErrorState Fc = continuousErrorStateJacobian(
      linearization_state, integration_gyro, integration_acceleration,
      tau_gyro_bias_, tau_acc_bias_);
    DiscreteErrorModel exact_model;
    if (propagation_model_ == PropagationModel::kExact) {
      const NoiseInputMatrix exact_noise_input =
        continuousNoiseInputJacobian(linearization_state);
      const ContinuousNoiseMatrix exact_continuous_noise = continuousNoiseCovariance(
        var_imu_acc_, var_imu_w_, var_imu_gyro_bias_, var_imu_acc_bias_);
      exact_model = exactDiscretizeErrorModel(
        Fc, exact_noise_input * exact_continuous_noise * exact_noise_input.transpose(), dt_imu);
      F = exact_model.transition;
    } else if (use_second_order_state_transition_) {
      F = secondOrderErrorStateTransition(Fc, dt_imu);
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
    if (propagation_model_ == PropagationModel::kExact) {
      discrete_process_noise = exact_model.process_covariance;
    } else if (use_continuous_process_noise_density_ && use_second_order_process_noise_) {
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
    if (!checkNumericalInvariants()) {
      x_ = state_before;
      P_ = covariance_before;
      has_last_discrete_model_ = false;
      return PredictionUpdateStatus::kNumericalFailure;
    }
    last_discrete_model_.transition = F;
    last_discrete_model_.process_covariance = discrete_process_noise;
    last_prediction_dt_ = dt_imu;
    has_last_discrete_model_ = true;
    return PredictionUpdateStatus::kUpdated;
  }

  void resetImuTimeBase()
  {
    previous_time_imu_ = 0.0;
    has_previous_time_imu_ = false;
    has_previous_imu_measurement_ = false;
  }

  bool primeImuMeasurement(
    const Eigen::Vector3d & gyro, const Eigen::Vector3d & linear_acceleration)
  {
    if (!gyro.allFinite() || !linear_acceleration.allFinite()) {
      return false;
    }
    previous_gyro_measurement_ = gyro;
    previous_accel_measurement_ = linear_acceleration;
    has_previous_imu_measurement_ = true;
    return true;
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

    const ObservationJacobian3 H = orientationObservationJacobian();

    Eigen::Matrix<double, num_error_state_, 3> K;
    if (!computeKalmanGain(H, R, K)) {
      return ObservationUpdateStatus::kNumericalFailure;
    }
    const Eigen::Matrix<double, num_error_state_, 1> dx = K * innov;
    if (!finishObservationUpdate(dx, K, H, R)) {
      return ObservationUpdateStatus::kNumericalFailure;
    }
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
  ObservationUpdateDiagnostics3 observationUpdatePositionWithCovariance(
    const Eigen::Vector3d & measured_position,
    const Eigen::Matrix3d & measurement_covariance,
    const double max_nis = 0.0)
  {
    ObservationJacobian3 jacobian;
    const Eigen::Vector3d predicted = positionObservation(getState(), &jacobian);
    return performObservationUpdate3(
      measured_position, predicted, jacobian, measurement_covariance, max_nis);
  }

  ObservationUpdateDiagnostics3 observationUpdateLeverArmPositionWithCovariance(
    const Eigen::Vector3d & measured_antenna_position,
    const Eigen::Vector3d & antenna_lever_arm_body,
    const Eigen::Matrix3d & measurement_covariance,
    const double max_nis = 0.0)
  {
    if (!antenna_lever_arm_body.allFinite()) {
      return ObservationUpdateDiagnostics3{};
    }
    ObservationJacobian3 jacobian;
    const Eigen::Vector3d predicted = leverArmPositionObservation(
      getState(), antenna_lever_arm_body, &jacobian);
    return performObservationUpdate3(
      measured_antenna_position, predicted, jacobian, measurement_covariance, max_nis);
  }

  ObservationUpdateDiagnostics3 observationUpdateWorldVelocityWithCovariance(
    const Eigen::Vector3d & measured_velocity,
    const Eigen::Matrix3d & measurement_covariance,
    const double max_nis = 0.0)
  {
    ObservationJacobian3 jacobian;
    const Eigen::Vector3d predicted = worldVelocityObservation(getState(), &jacobian);
    return performObservationUpdate3(
      measured_velocity, predicted, jacobian, measurement_covariance, max_nis);
  }

  ObservationUpdateDiagnostics1 observationUpdateYawWithVariance(
    const double measured_yaw_rad, const double measurement_variance,
    const double max_nis = 0.0)
  {
    ObservationUpdateDiagnostics1 diagnostics;
    if (!std::isfinite(measured_yaw_rad) || !std::isfinite(measurement_variance) ||
      !(measurement_variance > 0.0) || !std::isfinite(max_nis))
    {
      if (std::isfinite(measured_yaw_rad)) {
        diagnostics.status = ObservationUpdateStatus::kInvalidVariance;
        diagnostics.reason = ObservationRejectReason::kInvalidCovariance;
      }
      return diagnostics;
    }
    Eigen::Matrix<double, 1, num_error_state_> jacobian;
    const double predicted = yawObservation(getState(), &jacobian);
    if (!std::isfinite(predicted) || !jacobian.allFinite()) {
      diagnostics.status = ObservationUpdateStatus::kNumericalFailure;
      diagnostics.reason = ObservationRejectReason::kNumericalInvariant;
      return diagnostics;
    }
    diagnostics.innovation = std::atan2(
      std::sin(measured_yaw_rad - predicted), std::cos(measured_yaw_rad - predicted));
    diagnostics.innovation_variance =
      (jacobian * P_ * jacobian.transpose())(0, 0) + measurement_variance;
    if (!(diagnostics.innovation_variance > 0.0) ||
      !std::isfinite(diagnostics.innovation_variance))
    {
      diagnostics.status = ObservationUpdateStatus::kNumericalFailure;
      diagnostics.reason = ObservationRejectReason::kInnovationNotPositiveDefinite;
      return diagnostics;
    }
    diagnostics.nis = diagnostics.innovation * diagnostics.innovation /
      diagnostics.innovation_variance;
    if (max_nis > 0.0 && diagnostics.nis > max_nis) {
      diagnostics.status = ObservationUpdateStatus::kUpdated;
      diagnostics.reason = ObservationRejectReason::kNisGate;
      return diagnostics;
    }
    Eigen::Matrix<double, 1, 1> covariance;
    covariance(0, 0) = measurement_variance;
    Eigen::Matrix<double, num_error_state_, 1> gain;
    if (!computeKalmanGain(jacobian, covariance, gain)) {
      diagnostics.status = ObservationUpdateStatus::kNumericalFailure;
      diagnostics.reason = ObservationRejectReason::kNumericalInvariant;
      return diagnostics;
    }
    const Eigen::Matrix<double, num_error_state_, 1> error_update =
      gain * diagnostics.innovation;
    if (!finishObservationUpdate(error_update, gain, jacobian, covariance)) {
      diagnostics.status = ObservationUpdateStatus::kNumericalFailure;
      diagnostics.reason = ObservationRejectReason::kNumericalInvariant;
      return diagnostics;
    }
    diagnostics.status = ObservationUpdateStatus::kUpdated;
    diagnostics.reason = ObservationRejectReason::kNone;
    diagnostics.accepted = true;
    return diagnostics;
  }

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
    return observationUpdatePositionWithCovariance(y, variance.asDiagonal()).status;
  }

  ObservationUpdateStatus observationUpdatePositionWithLeverArmWithStatus(
    const Eigen::Vector3d & antenna_position_world,
    const Eigen::Vector3d & antenna_lever_arm_body,
    const Eigen::Vector3d & variance)
  {
    return observationUpdateLeverArmPositionWithCovariance(
      antenna_position_world, antenna_lever_arm_body, variance.asDiagonal()).status;
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
    ObservationJacobian3 H;
    const Eigen::Vector3d predicted_bias = gyroBiasObservation(getState(), &H);
    const Eigen::Matrix3d R = variance.asDiagonal();
    Eigen::Matrix<double, num_error_state_, 3> K;
    if (!computeKalmanGain(H, R, K)) {
      return ObservationUpdateStatus::kNumericalFailure;
    }
    const Eigen::Matrix<double, num_error_state_, 1> dx =
      K * (measured_stationary_gyro - predicted_bias);
    if (!finishObservationUpdate(dx, K, H, R)) {
      return ObservationUpdateStatus::kNumericalFailure;
    }
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
    if (propagate_cross_state) {
      return observationUpdateWorldVelocityWithCovariance(y, variance.asDiagonal()).status;
    }

    Eigen::Matrix3d R;
    R <<
      variance.x(), 0, 0,
      0, variance.y(), 0,
      0, 0, variance.z();

    ObservationJacobian3 H;
    const Eigen::Vector3d predicted_velocity = worldVelocityObservation(getState(), &H);
    Eigen::Matrix<double, num_error_state_, 3> K =
      Eigen::Matrix<double, num_error_state_, 3>::Zero();

    const Eigen::Matrix3d P_vv = P_.block<3, 3>(ERROR_STATE::DVX, ERROR_STATE::DVX);
    const Eigen::Matrix3d innovation_covariance = P_vv + R;
    const Eigen::LDLT<Eigen::Matrix3d> decomposition(innovation_covariance);
    if (decomposition.info() != Eigen::Success || !decomposition.isPositive()) {
      return ObservationUpdateStatus::kNumericalFailure;
    }
    const Eigen::Vector3d absolute_pivots = decomposition.vectorD().cwiseAbs();
    if (!(absolute_pivots.minCoeff() > 1.0e-12 * absolute_pivots.maxCoeff())) {
      return ObservationUpdateStatus::kNumericalFailure;
    }
    K.block<3, 3>(ERROR_STATE::DVX, 0) = decomposition.solve(P_vv).transpose();

    const Eigen::Matrix<double, num_error_state_, 1> dx = K * (y - predicted_velocity);

    if (!finishObservationUpdate(dx, K, H, R)) {
      return ObservationUpdateStatus::kNumericalFailure;
    }
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

    ObservationJacobian3 full_jacobian;
    const Eigen::Vector3d body_velocity = bodyVelocityObservation(getState(), &full_jacobian);
    const Eigen::Matrix<double, 2, num_error_state_> H = full_jacobian.bottomRows<2>();

    const Eigen::Matrix2d R = variance.asDiagonal();
    Eigen::Matrix<double, num_error_state_, 2> K;
    if (!computeKalmanGain(H, R, K)) {
      return ObservationUpdateStatus::kNumericalFailure;
    }
    const Eigen::Vector2d predicted = body_velocity.tail<2>();
    const Eigen::Matrix<double, num_error_state_, 1> dx =
      K * (lateral_vertical_velocity - predicted);

    if (!finishObservationUpdate(dx, K, H, R)) {
      return ObservationUpdateStatus::kNumericalFailure;
    }
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

    ObservationJacobian3 H;
    const Eigen::Vector3d predicted = bodyVelocityObservation(getState(), &H);
    const Eigen::Matrix3d R = variance.asDiagonal();
    Eigen::Matrix<double, num_error_state_, 3> K;
    if (!computeKalmanGain(H, R, K)) {
      return ObservationUpdateStatus::kNumericalFailure;
    }
    const Eigen::Matrix<double, num_error_state_, 1> dx =
      K * (measured_body_velocity - predicted);

    if (!finishObservationUpdate(dx, K, H, R)) {
      return ObservationUpdateStatus::kNumericalFailure;
    }
    return ObservationUpdateStatus::kUpdated;
  }

  // Observe only forward body-frame velocity. A wheel-speed sensor does not measure
  // lateral or vertical velocity; those are separate non-holonomic constraints.
  ObservationUpdateStatus observationUpdateBodyForwardSpeedWithStatus(
    const double measured_forward_speed,
    const double variance,
    const bool propagate_cross_state = false)
  {
    if (!std::isfinite(measured_forward_speed)) {
      return ObservationUpdateStatus::kInvalidMeasurement;
    }
    if (!(variance > 0.0) || !std::isfinite(variance)) {
      return ObservationUpdateStatus::kInvalidVariance;
    }
    ObservationJacobian3 full_jacobian;
    const Eigen::Vector3d predicted = bodyVelocityObservation(getState(), &full_jacobian);
    const Eigen::Matrix<double, 1, num_error_state_> H = full_jacobian.topRows<1>();
    Eigen::Matrix<double, 1, 1> measurement_covariance;
    measurement_covariance(0, 0) = variance;
    if (propagate_cross_state) {
      Eigen::Matrix<double, num_error_state_, 1> gain;
      if (!computeKalmanGain(H, measurement_covariance, gain)) {
        return ObservationUpdateStatus::kNumericalFailure;
      }
      const Eigen::Matrix<double, num_error_state_, 1> error_update =
        gain * (measured_forward_speed - predicted.x());
      return finishObservationUpdate(error_update, gain, H, measurement_covariance) ?
             ObservationUpdateStatus::kUpdated :
             ObservationUpdateStatus::kNumericalFailure;
    }
    const Eigen::Matrix3d body_from_world =
      getOrientation().normalized().toRotationMatrix().transpose();
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
    if (!finishObservationUpdate(dx, K, H, measurement_covariance)) {
      return ObservationUpdateStatus::kNumericalFailure;
    }
    return ObservationUpdateStatus::kUpdated;
  }

  void setTauGyroBias(const double tau_gyro_bias)
  {
    tau_gyro_bias_ = tau_gyro_bias;
  }

  void setBiasLearningEnabled(
    const bool gyro_bias_learning_enabled,
    const bool accel_bias_learning_enabled)
  {
    gyro_bias_learning_enabled_ = gyro_bias_learning_enabled;
    accel_bias_learning_enabled_ = accel_bias_learning_enabled;
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

  void setPropagationModel(const PropagationModel model)
  {
    propagation_model_ = model;
    if (model == PropagationModel::kLegacy) {
      use_continuous_process_noise_density_ = false;
      use_second_order_state_transition_ = false;
      use_second_order_process_noise_ = false;
    } else {
      use_continuous_process_noise_density_ = true;
      use_second_order_state_transition_ = true;
      use_second_order_process_noise_ = true;
    }
  }

  PropagationModel getPropagationModel() const
  {
    return propagation_model_;
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

  bool setInitialErrorStateCovariance(
    const Eigen::Vector3d & position_variance,
    const Eigen::Vector3d & velocity_variance,
    const Eigen::Vector3d & attitude_variance,
    const Eigen::Vector3d & gyro_bias_variance,
    const Eigen::Vector3d & accel_bias_variance)
  {
    const Eigen::Vector3d variances[] = {
      position_variance, velocity_variance, attitude_variance,
      gyro_bias_variance, accel_bias_variance};
    for (const auto & variance : variances) {
      if (!variance.allFinite() || (variance.array() < 0.0).any()) {
        return false;
      }
    }
    P_.setZero();
    P_.block<3, 3>(ERROR_STATE::DX, ERROR_STATE::DX).diagonal() = position_variance;
    P_.block<3, 3>(ERROR_STATE::DVX, ERROR_STATE::DVX).diagonal() = velocity_variance;
    P_.block<3, 3>(ERROR_STATE::DTHX, ERROR_STATE::DTHX).diagonal() = attitude_variance;
    P_.block<3, 3>(ERROR_STATE::DBGX, ERROR_STATE::DBGX).diagonal() = gyro_bias_variance;
    P_.block<3, 3>(ERROR_STATE::DBAX, ERROR_STATE::DBAX).diagonal() = accel_bias_variance;
    return checkNumericalInvariants();
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

  Snapshot getSnapshot() const
  {
    Snapshot snapshot;
    snapshot.state = getState();
    snapshot.covariance = P_;
    snapshot.previous_time_imu = previous_time_imu_;
    snapshot.has_previous_time_imu = has_previous_time_imu_;
    snapshot.previous_gyro_measurement = previous_gyro_measurement_;
    snapshot.previous_accel_measurement = previous_accel_measurement_;
    snapshot.has_previous_imu_measurement = has_previous_imu_measurement_;
    snapshot.gyro_bias_learning_enabled = gyro_bias_learning_enabled_;
    snapshot.accel_bias_learning_enabled = accel_bias_learning_enabled_;
    return snapshot;
  }

  bool restoreSnapshot(const Snapshot & snapshot)
  {
    if (!snapshot.state.position.allFinite() || !snapshot.state.velocity.allFinite() ||
      !snapshot.state.orientation.coeffs().allFinite() ||
      !(snapshot.state.orientation.norm() > 0.0) || !snapshot.state.gyro_bias.allFinite() ||
      !snapshot.state.accel_bias.allFinite() || !snapshot.covariance.allFinite() ||
      !std::isfinite(snapshot.previous_time_imu) ||
      !snapshot.previous_gyro_measurement.allFinite() ||
      !snapshot.previous_accel_measurement.allFinite())
    {
      return false;
    }
    const Snapshot before = getSnapshot();
    setState(snapshot.state);
    P_ = 0.5 * (snapshot.covariance + snapshot.covariance.transpose());
    previous_time_imu_ = snapshot.previous_time_imu;
    has_previous_time_imu_ = snapshot.has_previous_time_imu;
    previous_gyro_measurement_ = snapshot.previous_gyro_measurement;
    previous_accel_measurement_ = snapshot.previous_accel_measurement;
    has_previous_imu_measurement_ = snapshot.has_previous_imu_measurement;
    gyro_bias_learning_enabled_ = snapshot.gyro_bias_learning_enabled;
    accel_bias_learning_enabled_ = snapshot.accel_bias_learning_enabled;
    if (!checkNumericalInvariants()) {
      setState(before.state);
      P_ = before.covariance;
      previous_time_imu_ = before.previous_time_imu;
      has_previous_time_imu_ = before.has_previous_time_imu;
      previous_gyro_measurement_ = before.previous_gyro_measurement;
      previous_accel_measurement_ = before.previous_accel_measurement;
      has_previous_imu_measurement_ = before.has_previous_imu_measurement;
      gyro_bias_learning_enabled_ = before.gyro_bias_learning_enabled;
      accel_bias_learning_enabled_ = before.accel_bias_learning_enabled;
      return false;
    }
    return true;
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

  bool gyroBiasLearningEnabled() const
  {
    return gyro_bias_learning_enabled_;
  }

  bool accelBiasLearningEnabled() const
  {
    return accel_bias_learning_enabled_;
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

  double getQuaternionNormError() const
  {
    return std::fabs(getOrientation().norm() - 1.0);
  }

  double getCovarianceSymmetryError() const
  {
    return (P_ - P_.transpose()).cwiseAbs().maxCoeff();
  }

  double getMinimumCovarianceEigenvalue() const
  {
    if (!P_.allFinite()) {
      return -std::numeric_limits<double>::infinity();
    }
    const Eigen::SelfAdjointEigenSolver<EigenMatrixErrorState> solver(
      0.5 * (P_ + P_.transpose()), Eigen::EigenvaluesOnly);
    if (solver.info() != Eigen::Success) {
      return -std::numeric_limits<double>::infinity();
    }
    return solver.eigenvalues().minCoeff();
  }

  bool checkNumericalInvariants(
    const double quaternion_tolerance = 1.0e-10,
    const double symmetry_tolerance = 1.0e-10,
    const double negative_eigenvalue_tolerance = 1.0e-9) const
  {
    return x_.allFinite() && P_.allFinite() &&
           getQuaternionNormError() <= quaternion_tolerance &&
           getCovarianceSymmetryError() <= symmetry_tolerance &&
           getMinimumCovarianceEigenvalue() >= -negative_eigenvalue_tolerance;
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

  // Returns the discrete transition and process-covariance matrices actually used
  // by the most recent successful predictionUpdateDt call. This lets downstream
  // consumers (e.g. an RTS smoother) reconstruct the linearized error dynamics
  // consistently with the running filter.
  bool getLastDiscreteModel(DiscreteErrorModel & model) const
  {
    if (!has_last_discrete_model_) {
      return false;
    }
    model = last_discrete_model_;
    return true;
  }

  double getLastPredictionDt() const
  {
    return last_prediction_dt_;
  }

  bool hasLastDiscreteModel() const
  {
    return has_last_discrete_model_;
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

  ObservationUpdateDiagnostics3 performObservationUpdate3(
    const Eigen::Vector3d & measured,
    const Eigen::Vector3d & predicted,
    const ObservationJacobian3 & jacobian,
    const Eigen::Matrix3d & measurement_covariance,
    const double max_nis)
  {
    ObservationUpdateDiagnostics3 diagnostics;
    if (!measured.allFinite() || !predicted.allFinite() || !jacobian.allFinite() ||
      !std::isfinite(max_nis))
    {
      return diagnostics;
    }

    diagnostics.innovation = measured - predicted;
    if (!measurement_covariance.allFinite() ||
      !measurement_covariance.isApprox(measurement_covariance.transpose(), 1.0e-12))
    {
      diagnostics.status = ObservationUpdateStatus::kInvalidVariance;
      diagnostics.reason = ObservationRejectReason::kInvalidCovariance;
      return diagnostics;
    }
    const Eigen::LDLT<Eigen::Matrix3d> covariance_decomposition(measurement_covariance);
    if (covariance_decomposition.info() != Eigen::Success ||
      !covariance_decomposition.isPositive())
    {
      diagnostics.status = ObservationUpdateStatus::kInvalidVariance;
      diagnostics.reason = ObservationRejectReason::kInvalidCovariance;
      return diagnostics;
    }
    const Eigen::Vector3d covariance_pivots =
      covariance_decomposition.vectorD().cwiseAbs();
    if (!(covariance_pivots.minCoeff() > 1.0e-12 * covariance_pivots.maxCoeff())) {
      diagnostics.status = ObservationUpdateStatus::kInvalidVariance;
      diagnostics.reason = ObservationRejectReason::kInvalidCovariance;
      return diagnostics;
    }

    diagnostics.innovation_covariance =
      jacobian * P_ * jacobian.transpose() + measurement_covariance;
    diagnostics.innovation_covariance = 0.5 *
      (diagnostics.innovation_covariance + diagnostics.innovation_covariance.transpose());
    const Eigen::LDLT<Eigen::Matrix3d> innovation_decomposition(
      diagnostics.innovation_covariance);
    if (innovation_decomposition.info() != Eigen::Success ||
      !innovation_decomposition.isPositive())
    {
      diagnostics.status = ObservationUpdateStatus::kNumericalFailure;
      diagnostics.reason = ObservationRejectReason::kInnovationNotPositiveDefinite;
      return diagnostics;
    }
    const Eigen::Vector3d absolute_pivots = innovation_decomposition.vectorD().cwiseAbs();
    if (!(absolute_pivots.minCoeff() > 1.0e-12 * absolute_pivots.maxCoeff())) {
      diagnostics.status = ObservationUpdateStatus::kNumericalFailure;
      diagnostics.reason = ObservationRejectReason::kInnovationNotPositiveDefinite;
      return diagnostics;
    }

    const Eigen::Vector3d whitened_innovation =
      innovation_decomposition.solve(diagnostics.innovation);
    diagnostics.nis = diagnostics.innovation.dot(whitened_innovation);
    if (innovation_decomposition.info() != Eigen::Success ||
      !whitened_innovation.allFinite() || !std::isfinite(diagnostics.nis) ||
      diagnostics.nis < -1.0e-12)
    {
      diagnostics.status = ObservationUpdateStatus::kNumericalFailure;
      diagnostics.reason = ObservationRejectReason::kNumericalInvariant;
      return diagnostics;
    }
    diagnostics.nis = std::max(0.0, diagnostics.nis);
    if (max_nis > 0.0 && diagnostics.nis > max_nis) {
      diagnostics.status = ObservationUpdateStatus::kUpdated;
      diagnostics.reason = ObservationRejectReason::kNisGate;
      return diagnostics;
    }

    const Eigen::Matrix<double, num_error_state_, 3> covariance_times_jacobian =
      P_ * jacobian.transpose();
    const Eigen::Matrix<double, num_error_state_, 3> gain =
      innovation_decomposition.solve(covariance_times_jacobian.transpose()).transpose();
    if (innovation_decomposition.info() != Eigen::Success || !gain.allFinite()) {
      diagnostics.status = ObservationUpdateStatus::kNumericalFailure;
      diagnostics.reason = ObservationRejectReason::kNumericalInvariant;
      return diagnostics;
    }
    const Eigen::Matrix<double, num_error_state_, 1> error_update =
      gain * diagnostics.innovation;
    if (!finishObservationUpdate(
        error_update, gain, jacobian, measurement_covariance))
    {
      diagnostics.status = ObservationUpdateStatus::kNumericalFailure;
      diagnostics.reason = ObservationRejectReason::kNumericalInvariant;
      return diagnostics;
    }
    diagnostics.status = ObservationUpdateStatus::kUpdated;
    diagnostics.reason = ObservationRejectReason::kNone;
    diagnostics.accepted = true;
    return diagnostics;
  }

  template<int MeasurementSize>
  bool computeKalmanGain(
    const Eigen::Matrix<double, MeasurementSize, num_error_state_> & jacobian,
    const Eigen::Matrix<double, MeasurementSize, MeasurementSize> & measurement_covariance,
    Eigen::Matrix<double, num_error_state_, MeasurementSize> & gain) const
  {
    const Eigen::Matrix<double, MeasurementSize, MeasurementSize> innovation_covariance =
      jacobian * P_ * jacobian.transpose() + measurement_covariance;
    const Eigen::LDLT<Eigen::Matrix<double, MeasurementSize, MeasurementSize>> decomposition(
      0.5 * (innovation_covariance + innovation_covariance.transpose()));
    if (decomposition.info() != Eigen::Success || !decomposition.isPositive()) {
      return false;
    }
    const auto absolute_pivots = decomposition.vectorD().cwiseAbs();
    if (!(absolute_pivots.minCoeff() > 1.0e-12 * absolute_pivots.maxCoeff())) {
      return false;
    }
    const Eigen::Matrix<double, num_error_state_, MeasurementSize> covariance_times_jacobian =
      P_ * jacobian.transpose();
    gain = decomposition.solve(covariance_times_jacobian.transpose()).transpose();
    return decomposition.info() == Eigen::Success && gain.allFinite();
  }

  template<int MeasurementSize>
  bool finishObservationUpdate(
    const Eigen::Matrix<double, num_error_state_, 1> & error_update,
    const Eigen::Matrix<double, num_error_state_, MeasurementSize> & gain,
    const Eigen::Matrix<double, MeasurementSize, num_error_state_> & jacobian,
    const Eigen::Matrix<double, MeasurementSize, MeasurementSize> & measurement_covariance)
  {
    const Eigen::Matrix<double, num_state_, 1> state_before = x_;
    const EigenMatrixErrorState covariance_before = P_;
    Eigen::Matrix<double, num_error_state_, MeasurementSize> effective_gain = gain;
    Eigen::Matrix<double, num_error_state_, 1> effective_error_update = error_update;
    if (!gyro_bias_learning_enabled_) {
      effective_gain.template block<3, MeasurementSize>(ERROR_STATE::DBGX, 0).setZero();
      effective_error_update.template segment<3>(ERROR_STATE::DBGX).setZero();
    }
    if (!accel_bias_learning_enabled_) {
      effective_gain.template block<3, MeasurementSize>(ERROR_STATE::DBAX, 0).setZero();
      effective_error_update.template segment<3>(ERROR_STATE::DBAX).setZero();
    }
    const EigenMatrixErrorState identity = EigenMatrixErrorState::Identity();
    const EigenMatrixErrorState joseph_factor = identity - effective_gain * jacobian;
    EigenMatrixErrorState updated_covariance =
      joseph_factor * P_ * joseph_factor.transpose() +
      effective_gain * measurement_covariance * effective_gain.transpose();
    applyErrorState(effective_error_update);
    EigenMatrixErrorState reset_jacobian = EigenMatrixErrorState::Identity();
    reset_jacobian.block<3, 3>(ERROR_STATE::DTHX, ERROR_STATE::DTHX) =
      rightJacobianSO3(effective_error_update.segment<3>(ERROR_STATE::DTHX));
    P_ = reset_jacobian * updated_covariance * reset_jacobian.transpose();
    P_ = 0.5 * (P_ + P_.transpose());
    if (!checkNumericalInvariants()) {
      x_ = state_before;
      P_ = covariance_before;
      return false;
    }
    return true;
  }

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
      dq = Eigen::Quaterniond(1.0, 0.5 * dtheta.x(), 0.5 * dtheta.y(), 0.5 * dtheta.z());
      dq.normalize();
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
  DiscreteErrorModel last_discrete_model_{};
  double last_prediction_dt_{0.0};
  bool has_last_discrete_model_{false};
  double var_imu_w_;
  double var_imu_acc_;
  double var_imu_gyro_bias_;
  double var_imu_acc_bias_;
  bool use_continuous_process_noise_density_;
  bool use_second_order_state_transition_;
  bool use_second_order_process_noise_;
  PropagationModel propagation_model_;
  Eigen::Vector3d previous_gyro_measurement_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d previous_accel_measurement_{Eigen::Vector3d::Zero()};
  bool has_previous_imu_measurement_{false};
  double max_prediction_dt_sec_;
  double initial_gyro_bias_covariance_;
  double initial_accel_bias_covariance_;
  bool gyro_bias_learning_enabled_{true};
  bool accel_bias_learning_enabled_{true};

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

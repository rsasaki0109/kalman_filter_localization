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

#include <gtest/gtest.h>

#include <limits>

#include <kalman_filter_localization/core/ekf_estimator.hpp>

TEST(EKFEstimatorCore, PredictionUpdateDtStatus)
{
  EKFEstimator ekf;
  const Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  const Eigen::Vector3d acc = Eigen::Vector3d::Zero();

  EXPECT_EQ(
    ekf.predictionUpdateDt(0.0, gyro, acc),
    EKFEstimator::PredictionUpdateStatus::kNonPositiveDt);
  EXPECT_EQ(
    ekf.predictionUpdateDt(-0.1, gyro, acc),
    EKFEstimator::PredictionUpdateStatus::kNonPositiveDt);
  EXPECT_EQ(
    ekf.predictionUpdateDt(1.0, gyro, acc),
    EKFEstimator::PredictionUpdateStatus::kDtTooLarge);
  EXPECT_EQ(
    ekf.predictionUpdateDt(0.01, gyro, acc),
    EKFEstimator::PredictionUpdateStatus::kUpdated);
}

TEST(EKFEstimatorCore, ObservationUpdateValidation)
{
  EKFEstimator ekf;

  const Eigen::Vector3d y_valid(1.0, 2.0, 3.0);
  const Eigen::Vector3d variance_valid(1.0, 1.0, 1.0);

  EXPECT_EQ(
    ekf.observationUpdateWithStatus(y_valid, variance_valid),
    EKFEstimator::ObservationUpdateStatus::kUpdated);

  const Eigen::Vector3d variance_bad(0.0, 1.0, 1.0);
  EXPECT_EQ(
    ekf.observationUpdateWithStatus(y_valid, variance_bad),
    EKFEstimator::ObservationUpdateStatus::kInvalidVariance);

  Eigen::Vector3d y_bad = y_valid;
  y_bad.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    ekf.observationUpdateWithStatus(y_bad, variance_valid),
    EKFEstimator::ObservationUpdateStatus::kInvalidMeasurement);
}

TEST(EKFEstimatorCore, ObservationUpdateVelocityValidationAndConvergence)
{
  EKFEstimator ekf;

  const Eigen::Vector3d y_valid(1.5, -2.0, 0.25);
  const Eigen::Vector3d variance_valid(1.0e-3, 1.0e-3, 1.0e-3);

  EXPECT_EQ(
    ekf.observationUpdateVelocityWithStatus(y_valid, variance_valid),
    EKFEstimator::ObservationUpdateStatus::kUpdated);

  const Eigen::Vector3d v_est = ekf.getVelocity();
  EXPECT_NEAR(v_est.x(), y_valid.x(), 1e-3);
  EXPECT_NEAR(v_est.y(), y_valid.y(), 1e-3);
  EXPECT_NEAR(v_est.z(), y_valid.z(), 1e-3);

  const Eigen::Vector3d variance_bad(0.0, 1.0, 1.0);
  EXPECT_EQ(
    ekf.observationUpdateVelocityWithStatus(y_valid, variance_bad),
    EKFEstimator::ObservationUpdateStatus::kInvalidVariance);

  Eigen::Vector3d y_bad = y_valid;
  y_bad.y() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    ekf.observationUpdateVelocityWithStatus(y_bad, variance_valid),
    EKFEstimator::ObservationUpdateStatus::kInvalidMeasurement);
}

TEST(EKFEstimatorCore, ObservationUpdateVelocityKeepsOrientationWhenDecoupled)
{
  EKFEstimator ekf;
  const Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  const Eigen::Vector3d acc(1.0, 2.0, 0.5);
  EXPECT_EQ(
    ekf.predictionUpdateDt(0.1, gyro, acc),
    EKFEstimator::PredictionUpdateStatus::kUpdated);

  const Eigen::Quaterniond q_before = ekf.getOrientation().normalized();
  const Eigen::Vector3d vel_meas(0.3, -0.2, 0.1);
  const Eigen::Vector3d vel_var(1.0e-2, 1.0e-2, 1.0e-2);

  EXPECT_EQ(
    ekf.observationUpdateVelocityWithStatus(vel_meas, vel_var),
    EKFEstimator::ObservationUpdateStatus::kUpdated);

  const Eigen::Quaterniond q_after = ekf.getOrientation().normalized();
  const double dot = std::abs(q_before.dot(q_after));
  const double angle_err = 2.0 * std::acos(std::min(1.0, std::max(-1.0, dot)));
  EXPECT_LT(angle_err, 1e-10);
}

TEST(EKFEstimatorCore, BodyVelocityConstraintReducesLateralAndVerticalVelocity)
{
  EKFEstimator ekf;
  EKFEstimator::State state;
  state.velocity = Eigen::Vector3d(5.0, 1.0, -0.5);
  ekf.setState(state);

  for (int i = 0; i < 10; ++i) {
    EXPECT_EQ(
      ekf.observationUpdateBodyVelocityConstraintWithStatus(
        Eigen::Vector2d::Zero(), Eigen::Vector2d(0.01, 0.01)),
      EKFEstimator::ObservationUpdateStatus::kUpdated);
  }

  const Eigen::Vector3d body_velocity =
    ekf.getOrientation().toRotationMatrix().transpose() * ekf.getVelocity();
  EXPECT_NEAR(body_velocity.y(), 0.0, 1.0e-3);
  EXPECT_NEAR(body_velocity.z(), 0.0, 1.0e-3);
  EXPECT_GT(body_velocity.x(), 4.9);
}

TEST(EKFEstimatorCore, BodyVelocityConstraintValidatesInputs)
{
  EKFEstimator ekf;
  EXPECT_EQ(
    ekf.observationUpdateBodyVelocityConstraintWithStatus(
      Eigen::Vector2d::Zero(), Eigen::Vector2d(0.0, 0.1)),
    EKFEstimator::ObservationUpdateStatus::kInvalidVariance);
  EXPECT_EQ(
    ekf.observationUpdateBodyVelocityConstraintWithStatus(
      Eigen::Vector2d(std::numeric_limits<double>::quiet_NaN(), 0.0),
      Eigen::Vector2d(0.1, 0.1)),
    EKFEstimator::ObservationUpdateStatus::kInvalidMeasurement);
}

TEST(EKFEstimatorCore, PredictionUpdateSubtractsGyroBias)
{
  EKFEstimator ekf;
  EKFEstimator::State state;
  state.gyro_bias = Eigen::Vector3d(0.0, 0.0, 0.1);
  ekf.setState(state);

  EXPECT_EQ(
    ekf.predictionUpdateDt(
      0.1, Eigen::Vector3d(0.0, 0.0, 0.1), Eigen::Vector3d::Zero()),
    EKFEstimator::PredictionUpdateStatus::kUpdated);

  const Eigen::Quaterniond q_after = ekf.getOrientation().normalized();
  const double dot = std::abs(Eigen::Quaterniond::Identity().dot(q_after));
  const double angle_err = 2.0 * std::acos(std::min(1.0, std::max(-1.0, dot)));
  EXPECT_LT(angle_err, 1e-10);
  EXPECT_NEAR(ekf.getGyroBias().z(), 0.1, 1e-4);
}

TEST(EKFEstimatorCore, PredictionUpdateSubtractsAccelBias)
{
  EKFEstimator ekf;
  ekf.setGravityZ(0.0);
  ekf.setTauAccBias(1.0e12);

  EKFEstimator::State state;
  state.accel_bias = Eigen::Vector3d(0.3, -0.2, 0.1);
  ekf.setState(state);

  EXPECT_EQ(
    ekf.predictionUpdateDt(0.1, Eigen::Vector3d::Zero(), state.accel_bias),
    EKFEstimator::PredictionUpdateStatus::kUpdated);

  EXPECT_LT(ekf.getPosition().norm(), 1e-12);
  EXPECT_LT(ekf.getVelocity().norm(), 1e-12);
  EXPECT_NEAR((ekf.getAccelBias() - state.accel_bias).norm(), 0.0, 1e-12);
}

TEST(EKFEstimatorCore, PredictionUpdateUsesBodyFrameAngularVelocity)
{
  EKFEstimator ekf;
  ekf.setGravityZ(0.0);
  constexpr double kHalfPi = 1.57079632679489661923;

  EKFEstimator::State state;
  state.orientation =
    Eigen::Quaterniond(Eigen::AngleAxisd(kHalfPi, Eigen::Vector3d::UnitZ()));
  ekf.setState(state);

  const Eigen::Vector3d gyro_meas(1.0, 0.0, 0.0);
  EXPECT_EQ(
    ekf.predictionUpdateDt(0.1, gyro_meas, Eigen::Vector3d::Zero()),
    EKFEstimator::PredictionUpdateStatus::kUpdated);

  const Eigen::Quaterniond q_expected =
    (state.orientation * Eigen::Quaterniond(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX())))
    .normalized();
  const Eigen::Quaterniond q_wrong =
    (Eigen::Quaterniond(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX())) * state.orientation)
    .normalized();
  const Eigen::Quaterniond q_actual = ekf.getOrientation().normalized();

  const double dot_expected = std::abs(q_expected.dot(q_actual));
  const double angle_expected = 2.0 * std::acos(std::min(1.0, std::max(-1.0, dot_expected)));
  const double dot_wrong = std::abs(q_wrong.dot(q_actual));
  const double angle_wrong = 2.0 * std::acos(std::min(1.0, std::max(-1.0, dot_wrong)));

  EXPECT_LT(angle_expected, 1e-10);
  EXPECT_GT(angle_wrong, 1.0e-2);
}

TEST(EKFEstimatorCore, InitialBiasCovarianceSettersUpdateCovariance)
{
  EKFEstimator ekf;

  const Eigen::MatrixXd p_default = ekf.getCovariance();
  EXPECT_NEAR(p_default(9, 9), 0.0, 1e-12);
  EXPECT_NEAR(p_default(12, 12), 0.0, 1e-12);

  EXPECT_TRUE(ekf.setInitialGyroBiasCovariance(0.25));
  EXPECT_TRUE(ekf.setInitialAccelBiasCovariance(0.5));

  const Eigen::MatrixXd p = ekf.getCovariance();
  EXPECT_NEAR(p(9, 9), 0.25, 1e-12);
  EXPECT_NEAR(p(10, 10), 0.25, 1e-12);
  EXPECT_NEAR(p(11, 11), 0.25, 1e-12);
  EXPECT_NEAR(p(12, 12), 0.5, 1e-12);
  EXPECT_NEAR(p(13, 13), 0.5, 1e-12);
  EXPECT_NEAR(p(14, 14), 0.5, 1e-12);

  EXPECT_FALSE(ekf.setInitialGyroBiasCovariance(-1.0));
  EXPECT_FALSE(ekf.setInitialAccelBiasCovariance(-1.0));
  const Eigen::MatrixXd p_after_invalid = ekf.getCovariance();
  EXPECT_NEAR(p_after_invalid(9, 9), 0.25, 1e-12);
  EXPECT_NEAR(p_after_invalid(12, 12), 0.5, 1e-12);
}

TEST(EKFEstimatorCore, CapPositionCovarianceScalesPositionBlock)
{
  EKFEstimator ekf;

  const Eigen::MatrixXd p_before = ekf.getCovariance();
  ASSERT_GT(p_before(0, 0), 1.0);
  ASSERT_GT(p_before(1, 1), 1.0);
  ASSERT_GT(p_before(2, 2), 2.0);

  EXPECT_TRUE(ekf.capPositionCovariance(1.0, 2.0));
  const Eigen::MatrixXd p = ekf.getCovariance();
  EXPECT_NEAR(p(0, 0), 1.0, 1e-12);
  EXPECT_NEAR(p(1, 1), 1.0, 1e-12);
  EXPECT_NEAR(p(2, 2), 2.0, 1e-12);

  EXPECT_FALSE(ekf.capPositionCovariance(0.0, 1.0));
  EXPECT_FALSE(ekf.capPositionCovariance(1.0, -1.0));
}

TEST(EKFEstimatorCore, CapErrorStateCovarianceScalesEnabledBlocks)
{
  EKFEstimator ekf;

  EXPECT_TRUE(ekf.capErrorStateCovariance(1.0, 2.0, 3.0, 4.0, 0.01, 0.02));

  const Eigen::MatrixXd p = ekf.getCovariance();
  EXPECT_NEAR(p(0, 0), 1.0, 1e-12);
  EXPECT_NEAR(p(1, 1), 1.0, 1e-12);
  EXPECT_NEAR(p(2, 2), 2.0, 1e-12);
  EXPECT_NEAR(p(3, 3), 3.0, 1e-12);
  EXPECT_NEAR(p(4, 4), 3.0, 1e-12);
  EXPECT_NEAR(p(5, 5), 4.0, 1e-12);
  EXPECT_NEAR(p(6, 6), 0.01, 1e-12);
  EXPECT_NEAR(p(7, 7), 0.01, 1e-12);
  EXPECT_NEAR(p(8, 8), 0.02, 1e-12);

  EXPECT_FALSE(ekf.capErrorStateCovariance(-1.0, 2.0, 0.0, 0.0, 0.0, 0.0));
  EXPECT_FALSE(ekf.capErrorStateCovariance(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
}

TEST(EKFEstimatorCore, OrientationObservationCanEstimateGyroBias)
{
  EKFEstimator ekf;
  ekf.setGravityZ(0.0);
  ekf.setVarImuGyroBias(1.0e-4);
  const Eigen::Vector3d gyro_meas(0.0, 0.0, 0.1);
  const Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  const Eigen::Quaterniond q_meas = Eigen::Quaterniond::Identity();
  const Eigen::Vector3d var_rpy(1.0e-4, 1.0e-4, 1.0e-4);

  for (int i = 0; i < 300; ++i) {
    EXPECT_EQ(
      ekf.predictionUpdateDt(0.01, gyro_meas, acc),
      EKFEstimator::PredictionUpdateStatus::kUpdated);
    EXPECT_EQ(
      ekf.observationUpdateOrientationWithStatus(q_meas, var_rpy),
      EKFEstimator::ObservationUpdateStatus::kUpdated);
  }

  const Eigen::Vector3d bias_est = ekf.getGyroBias();
  EXPECT_GT(bias_est.z(), 1.0e-3);

  const Eigen::Quaterniond q_est = ekf.getOrientation().normalized();
  const double dot = std::abs(q_est.dot(q_meas));
  const double angle_err = 2.0 * std::acos(std::min(1.0, std::max(-1.0, dot)));
  EXPECT_LT(angle_err, 2.0e-2);
}

TEST(EKFEstimatorCore, PositionObservationCanEstimateAccelBias)
{
  EKFEstimator ekf;
  ekf.setGravityZ(0.0);
  ekf.setTauAccBias(1.0e12);
  ekf.setVarImuAccBias(1.0e-4);
  ASSERT_TRUE(ekf.setInitialAccelBiasCovariance(1.0));

  const Eigen::Vector3d acc_meas(0.3, -0.2, 0.1);
  const Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  const Eigen::Vector3d pos_meas = Eigen::Vector3d::Zero();
  const Eigen::Vector3d pos_var(1.0e-4, 1.0e-4, 1.0e-4);

  for (int i = 0; i < 400; ++i) {
    EXPECT_EQ(
      ekf.predictionUpdateDt(0.01, gyro, acc_meas),
      EKFEstimator::PredictionUpdateStatus::kUpdated);
    EXPECT_EQ(
      ekf.observationUpdateWithStatus(pos_meas, pos_var),
      EKFEstimator::ObservationUpdateStatus::kUpdated);
  }

  const Eigen::Vector3d bias_est = ekf.getAccelBias();
  EXPECT_NEAR(bias_est.x(), acc_meas.x(), 5.0e-2);
  EXPECT_NEAR(bias_est.y(), acc_meas.y(), 5.0e-2);
  EXPECT_NEAR(bias_est.z(), acc_meas.z(), 5.0e-2);
  EXPECT_LT(ekf.getPosition().norm(), 5.0e-2);
  EXPECT_LT(ekf.getVelocity().norm(), 5.0e-2);
}

TEST(EKFEstimatorCore, ObservationUpdateOrientationValidationAndConvergence)
{
  EKFEstimator ekf;

  constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
  const Eigen::Quaterniond q_meas =
    Eigen::Quaterniond(Eigen::AngleAxisd(10.0 * kDeg2Rad, Eigen::Vector3d::UnitX()));
  const Eigen::Vector3d var_ok(1e-3, 1e-3, 1e-3);

  Eigen::Vector3d var_bad = var_ok;
  var_bad.x() = 0.0;
  EXPECT_EQ(
    ekf.observationUpdateOrientationWithStatus(q_meas, var_bad),
    EKFEstimator::ObservationUpdateStatus::kInvalidVariance);

  Eigen::Quaterniond q_bad = q_meas;
  q_bad.coeffs().setConstant(std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(
    ekf.observationUpdateOrientationWithStatus(q_bad, var_ok),
    EKFEstimator::ObservationUpdateStatus::kInvalidMeasurement);

  EXPECT_EQ(
    ekf.observationUpdateOrientationWithStatus(q_meas, var_ok),
    EKFEstimator::ObservationUpdateStatus::kUpdated);

  const Eigen::Quaterniond q_est = ekf.getOrientation().normalized();
  const double dot = std::abs(q_est.dot(q_meas.normalized()));
  const double angle_err = 2.0 * std::acos(std::min(1.0, std::max(-1.0, dot)));
  EXPECT_LT(angle_err, 1e-3);
}

TEST(EKFEstimatorCore, PredictionUpdateWithStatusTimeBaseAndReset)
{
  EKFEstimator ekf;
  const Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  const Eigen::Vector3d acc = Eigen::Vector3d::Zero();

  EXPECT_EQ(
    ekf.predictionUpdateWithStatus(10.0, gyro, acc),
    EKFEstimator::PredictionUpdateStatus::kSkippedNoTimeBase);

  EXPECT_EQ(
    ekf.predictionUpdateWithStatus(10.0, gyro, acc),
    EKFEstimator::PredictionUpdateStatus::kNonPositiveDt);

  EXPECT_EQ(
    ekf.predictionUpdateWithStatus(10.01, gyro, acc),
    EKFEstimator::PredictionUpdateStatus::kUpdated);

  ekf.resetImuTimeBase();
  EXPECT_EQ(
    ekf.predictionUpdateWithStatus(20.0, gyro, acc),
    EKFEstimator::PredictionUpdateStatus::kSkippedNoTimeBase);
}

TEST(EKFEstimatorCore, PredictionUpdateWithStatusRecoversAfterLargeDt)
{
  EKFEstimator ekf;
  const Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  const Eigen::Vector3d acc = Eigen::Vector3d::Zero();

  EXPECT_EQ(
    ekf.predictionUpdateWithStatus(0.0, gyro, acc),
    EKFEstimator::PredictionUpdateStatus::kSkippedNoTimeBase);
  EXPECT_EQ(
    ekf.predictionUpdateWithStatus(1.0, gyro, acc),
    EKFEstimator::PredictionUpdateStatus::kDtTooLarge);

  EXPECT_EQ(
    ekf.predictionUpdateWithStatus(1.01, gyro, acc),
    EKFEstimator::PredictionUpdateStatus::kUpdated);
}

TEST(EKFEstimatorCore, SetInitialXCheckedValidatesSize)
{
  EKFEstimator ekf;

  Eigen::VectorXd x_bad(ekf.getNumState() - 1);
  x_bad.setZero();
  EXPECT_FALSE(ekf.setInitialXChecked(x_bad));

  Eigen::VectorXd x_ok(ekf.getNumState());
  x_ok.setZero();
  // Last element is qw.
  x_ok(ekf.getNumState() - 1) = 1.0;
  EXPECT_TRUE(ekf.setInitialXChecked(x_ok));
}

TEST(EKFEstimatorCore, MaxPredictionDtSecConfig)
{
  EKFEstimator ekf;
  const Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  const Eigen::Vector3d acc = Eigen::Vector3d::Zero();

  EXPECT_DOUBLE_EQ(ekf.getMaxPredictionDtSec(), 0.5);
  EXPECT_FALSE(ekf.setMaxPredictionDtSec(0.0));
  EXPECT_FALSE(ekf.setMaxPredictionDtSec(-1.0));
  EXPECT_FALSE(ekf.setMaxPredictionDtSec(std::numeric_limits<double>::infinity()));
  EXPECT_TRUE(ekf.setMaxPredictionDtSec(1.5));
  EXPECT_DOUBLE_EQ(ekf.getMaxPredictionDtSec(), 1.5);

  EXPECT_EQ(
    ekf.predictionUpdateDt(1.0, gyro, acc),
    EKFEstimator::PredictionUpdateStatus::kUpdated);
}

TEST(EKFEstimatorCore, ContinuousNoiseDensityIsImuRateIndependent)
{
  EKFEstimator ekf_100_hz;
  EKFEstimator ekf_200_hz;
  ekf_100_hz.setUseContinuousProcessNoiseDensity(true);
  ekf_200_hz.setUseContinuousProcessNoiseDensity(true);
  ekf_100_hz.setVarImuAcc(0.4);
  ekf_200_hz.setVarImuAcc(0.4);
  ekf_100_hz.setVarImuGyro(0.2);
  ekf_200_hz.setVarImuGyro(0.2);

  const Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  const Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(
      ekf_100_hz.predictionUpdateDt(0.01, gyro, acceleration),
      EKFEstimator::PredictionUpdateStatus::kUpdated);
  }
  for (int i = 0; i < 200; ++i) {
    EXPECT_EQ(
      ekf_200_hz.predictionUpdateDt(0.005, gyro, acceleration),
      EKFEstimator::PredictionUpdateStatus::kUpdated);
  }

  const Eigen::MatrixXd covariance_100_hz = ekf_100_hz.getCovariance();
  const Eigen::MatrixXd covariance_200_hz = ekf_200_hz.getCovariance();
  const Eigen::Matrix3d velocity_covariance_100_hz = covariance_100_hz.block<3, 3>(3, 3);
  const Eigen::Matrix3d velocity_covariance_200_hz = covariance_200_hz.block<3, 3>(3, 3);
  const Eigen::Matrix3d attitude_covariance_100_hz = covariance_100_hz.block<3, 3>(6, 6);
  const Eigen::Matrix3d attitude_covariance_200_hz = covariance_200_hz.block<3, 3>(6, 6);
  EXPECT_TRUE(velocity_covariance_100_hz.isApprox(velocity_covariance_200_hz, 1.0e-9));
  EXPECT_TRUE(attitude_covariance_100_hz.isApprox(attitude_covariance_200_hz, 1.0e-9));
}

TEST(EKFEstimatorCore, SecondOrderTransitionReducesImuRateSensitivity)
{
  EKFEstimator ekf_100_hz;
  EKFEstimator ekf_200_hz;
  ekf_100_hz.setUseContinuousProcessNoiseDensity(true);
  ekf_200_hz.setUseContinuousProcessNoiseDensity(true);
  ekf_100_hz.setUseSecondOrderStateTransition(true);
  ekf_200_hz.setUseSecondOrderStateTransition(true);
  const Eigen::Vector3d gyro(0.02, -0.01, 0.1);
  const Eigen::Vector3d acceleration(0.3, -0.2, 9.80665);

  for (int i = 0; i < 100; ++i) {
    ASSERT_EQ(
      ekf_100_hz.predictionUpdateDt(0.01, gyro, acceleration),
      EKFEstimator::PredictionUpdateStatus::kUpdated);
  }
  for (int i = 0; i < 200; ++i) {
    ASSERT_EQ(
      ekf_200_hz.predictionUpdateDt(0.005, gyro, acceleration),
      EKFEstimator::PredictionUpdateStatus::kUpdated);
  }

  const Eigen::MatrixXd covariance_100_hz = ekf_100_hz.getCovariance();
  const Eigen::MatrixXd covariance_200_hz = ekf_200_hz.getCovariance();
  const double relative_difference =
    (covariance_100_hz - covariance_200_hz).norm() / covariance_200_hz.norm();
  EXPECT_LT(relative_difference, 2.0e-3);
}

TEST(EKFEstimatorCore, RobustLossContinuouslyInflatesMeasurementVariance)
{
  using RobustLoss = EKFEstimator::RobustLoss;
  EXPECT_DOUBLE_EQ(
    EKFEstimator::computeRobustVarianceScale(2.0, RobustLoss::kHuber, 2.5, 100.0), 1.0);
  EXPECT_DOUBLE_EQ(
    EKFEstimator::computeRobustVarianceScale(5.0, RobustLoss::kHuber, 2.5, 100.0), 2.0);
  EXPECT_DOUBLE_EQ(
    EKFEstimator::computeRobustVarianceScale(5.0, RobustLoss::kCauchy, 2.5, 100.0), 5.0);
  EXPECT_DOUBLE_EQ(
    EKFEstimator::computeRobustVarianceScale(1000.0, RobustLoss::kCauchy, 2.5, 20.0), 20.0);
  EXPECT_DOUBLE_EQ(
    EKFEstimator::computeRobustVarianceScale(5.0, RobustLoss::kNone, 2.5, 100.0), 1.0);
}

TEST(EKFEstimatorCore, LeverArmPositionUpdateReducesAntennaResidual)
{
  EKFEstimator ekf;
  EKFEstimator::State state;
  state.orientation = Eigen::Quaterniond(
    Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()));
  ekf.setState(state);
  const Eigen::Vector3d lever_arm(2.0, 0.0, 0.5);
  const Eigen::Vector3d antenna_measurement = lever_arm;
  const auto antenna_prediction = [&ekf, &lever_arm]() -> Eigen::Vector3d {
      const Eigen::Vector3d position = ekf.getPosition();
      const Eigen::Vector3d rotated_lever_arm = ekf.getOrientation() * lever_arm;
      return position + rotated_lever_arm;
    };
  const double error_before = (antenna_measurement - antenna_prediction()).norm();

  EXPECT_EQ(
    ekf.observationUpdatePositionWithLeverArmWithStatus(
      antenna_measurement, lever_arm, Eigen::Vector3d::Constant(0.01)),
    EKFEstimator::ObservationUpdateStatus::kUpdated);

  const double error_after = (antenna_measurement - antenna_prediction()).norm();
  EXPECT_LT(error_after, error_before);
}

TEST(EKFEstimatorCore, StationaryGyroObservationEstimatesGyroBias)
{
  EKFEstimator ekf;
  ASSERT_TRUE(ekf.setInitialGyroBiasCovariance(1.0));
  const Eigen::Vector3d stationary_gyro(0.01, -0.02, 0.03);
  for (int i = 0; i < 5; ++i) {
    EXPECT_EQ(
      ekf.observationUpdateGyroBiasWithStatus(
        stationary_gyro, Eigen::Vector3d::Constant(1.0e-5)),
      EKFEstimator::ObservationUpdateStatus::kUpdated);
  }
  EXPECT_TRUE(ekf.getGyroBias().isApprox(stationary_gyro, 1.0e-4));
  EXPECT_EQ(
    ekf.observationUpdateGyroBiasWithStatus(
      stationary_gyro, Eigen::Vector3d::Zero()),
    EKFEstimator::ObservationUpdateStatus::kInvalidVariance);
}

TEST(EKFEstimatorCore, CoupledProcessNoiseIsImuRateIndependentForPosition)
{
  EKFEstimator ekf_100_hz;
  EKFEstimator ekf_200_hz;
  for (EKFEstimator * ekf : {&ekf_100_hz, &ekf_200_hz}) {
    ekf->setUseContinuousProcessNoiseDensity(true);
    ekf->setUseSecondOrderStateTransition(true);
    ekf->setUseSecondOrderProcessNoise(true);
    ekf->setVarImuAcc(0.4);
    ekf->setVarImuGyro(0.2);
  }
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  for (int i = 0; i < 100; ++i) {
    ASSERT_EQ(
      ekf_100_hz.predictionUpdateDt(0.01, zero, zero),
      EKFEstimator::PredictionUpdateStatus::kUpdated);
  }
  for (int i = 0; i < 200; ++i) {
    ASSERT_EQ(
      ekf_200_hz.predictionUpdateDt(0.005, zero, zero),
      EKFEstimator::PredictionUpdateStatus::kUpdated);
  }
  const Eigen::Matrix3d position_covariance_100_hz =
    ekf_100_hz.getCovariance().block<3, 3>(0, 0);
  const Eigen::Matrix3d position_covariance_200_hz =
    ekf_200_hz.getCovariance().block<3, 3>(0, 0);
  EXPECT_TRUE(position_covariance_100_hz.isApprox(position_covariance_200_hz, 1.0e-9));
}

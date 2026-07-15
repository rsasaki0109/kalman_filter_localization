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
#include <random>

#include <kalman_filter_localization/core/ekf_estimator.hpp>

namespace
{
EKFEstimator::State perturbState(
  const EKFEstimator::State & state, const EKFEstimator::ErrorStateVector & error)
{
  EKFEstimator::State result = state;
  result.position += error.segment<3>(0);
  result.velocity += error.segment<3>(3);
  const Eigen::Vector3d angle = error.segment<3>(6);
  const double norm = angle.norm();
  const Eigen::Quaterniond increment = norm > 0.0 ?
    Eigen::Quaterniond(Eigen::AngleAxisd(norm, angle / norm)) :
    Eigen::Quaterniond::Identity();
  result.orientation = (state.orientation.normalized() * increment).normalized();
  result.gyro_bias += error.segment<3>(9);
  result.accel_bias += error.segment<3>(12);
  return result;
}

Eigen::Vector3d quaternionLog(Eigen::Quaterniond quaternion)
{
  quaternion.normalize();
  if (quaternion.w() < 0.0) {
    quaternion.coeffs() *= -1.0;
  }
  const Eigen::AngleAxisd angle_axis(quaternion);
  return angle_axis.axis() * angle_axis.angle();
}

EKFEstimator::ErrorStateVector stateError(
  const EKFEstimator::State & nominal, const EKFEstimator::State & truth)
{
  EKFEstimator::ErrorStateVector result;
  result.segment<3>(0) = truth.position - nominal.position;
  result.segment<3>(3) = truth.velocity - nominal.velocity;
  result.segment<3>(6) = quaternionLog(
    nominal.orientation.normalized().conjugate() * truth.orientation.normalized());
  result.segment<3>(9) = truth.gyro_bias - nominal.gyro_bias;
  result.segment<3>(12) = truth.accel_bias - nominal.accel_bias;
  return result;
}

EKFEstimator::State propagateState(
  const EKFEstimator::State & state, const double dt,
  const Eigen::Vector3d & gyro, const Eigen::Vector3d & acceleration,
  const double tau_gyro, const double tau_accel)
{
  EKFEstimator estimator;
  estimator.setState(state);
  estimator.setTauGyroBias(tau_gyro);
  estimator.setTauAccBias(tau_accel);
  EXPECT_EQ(
    estimator.predictionUpdateDt(dt, gyro, acceleration),
    EKFEstimator::PredictionUpdateStatus::kUpdated);
  return estimator.getState();
}

template<typename ObservationFunction>
EKFEstimator::ObservationJacobian3 numericalObservationJacobian(
  const EKFEstimator::State & state, ObservationFunction observation, const double epsilon)
{
  EKFEstimator::ObservationJacobian3 result;
  for (int column = 0; column < EKFEstimator::kErrorStateSize; ++column) {
    EKFEstimator::ErrorStateVector error = EKFEstimator::ErrorStateVector::Zero();
    error(column) = epsilon;
    const Eigen::Vector3d positive = observation(perturbState(state, error));
    const Eigen::Vector3d negative = observation(perturbState(state, -error));
    result.col(column) = (positive - negative) / (2.0 * epsilon);
  }
  return result;
}
}  // namespace

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

TEST(EKFEstimatorCore, PositionDiagnosticsUseFullInnovationCovariance)
{
  EKFEstimator ekf;
  const Eigen::Vector3d measured(2.0, -1.0, 0.5);
  Eigen::Matrix3d measurement_covariance;
  measurement_covariance <<
    2.0, 0.4, 0.0,
    0.4, 1.5, -0.2,
    0.0, -0.2, 0.8;
  const EKFEstimator::Snapshot before = ekf.getSnapshot();
  EKFEstimator::ObservationJacobian3 jacobian;
  const Eigen::Vector3d predicted = EKFEstimator::positionObservation(before.state, &jacobian);
  const Eigen::Vector3d innovation = measured - predicted;
  const Eigen::Matrix3d expected_innovation_covariance =
    jacobian * before.covariance * jacobian.transpose() + measurement_covariance;
  const double expected_nis = innovation.dot(
    expected_innovation_covariance.ldlt().solve(innovation));

  const auto diagnostics = ekf.observationUpdatePositionWithCovariance(
    measured, measurement_covariance);

  EXPECT_EQ(diagnostics.status, EKFEstimator::ObservationUpdateStatus::kUpdated);
  EXPECT_EQ(diagnostics.reason, EKFEstimator::ObservationRejectReason::kNone);
  EXPECT_TRUE(diagnostics.accepted);
  EXPECT_TRUE(diagnostics.innovation.isApprox(innovation, 1.0e-12));
  EXPECT_TRUE(diagnostics.innovation_covariance.isApprox(
      expected_innovation_covariance, 1.0e-12));
  EXPECT_NEAR(diagnostics.nis, expected_nis, 1.0e-12);
}

TEST(EKFEstimatorCore, CorrelatedPositionCovarianceChangesCrossAxisUpdate)
{
  EKFEstimator correlated;
  EKFEstimator diagonal;
  Eigen::Matrix3d correlated_covariance = Eigen::Matrix3d::Identity();
  correlated_covariance(0, 1) = 0.8;
  correlated_covariance(1, 0) = 0.8;
  const Eigen::Vector3d measured(1.0, 0.0, 0.0);

  ASSERT_TRUE(correlated.observationUpdatePositionWithCovariance(
      measured, correlated_covariance).accepted);
  ASSERT_TRUE(diagonal.observationUpdatePositionWithCovariance(
      measured, Eigen::Matrix3d::Identity()).accepted);

  EXPECT_GT(std::abs(correlated.getPosition().y()), 1.0e-3);
  EXPECT_NEAR(diagonal.getPosition().y(), 0.0, 1.0e-12);
}

TEST(EKFEstimatorCore, NisGateRejectsWithoutMutatingEstimator)
{
  EKFEstimator ekf;
  const EKFEstimator::Snapshot before = ekf.getSnapshot();

  const auto diagnostics = ekf.observationUpdatePositionWithCovariance(
    Eigen::Vector3d(100.0, 0.0, 0.0), Eigen::Matrix3d::Identity(), 10.0);

  EXPECT_EQ(diagnostics.status, EKFEstimator::ObservationUpdateStatus::kUpdated);
  EXPECT_EQ(diagnostics.reason, EKFEstimator::ObservationRejectReason::kNisGate);
  EXPECT_FALSE(diagnostics.accepted);
  EXPECT_GT(diagnostics.nis, 10.0);
  const EKFEstimator::Snapshot after = ekf.getSnapshot();
  EXPECT_TRUE(after.covariance.isApprox(before.covariance, 0.0));
  EXPECT_TRUE(after.state.position.isApprox(before.state.position, 0.0));
  EXPECT_TRUE(after.state.velocity.isApprox(before.state.velocity, 0.0));
  EXPECT_TRUE(after.state.orientation.coeffs().isApprox(
      before.state.orientation.coeffs(), 0.0));
}

TEST(EKFEstimatorCore, LeverArmDiagnosticsIncludeAttitudeUncertainty)
{
  EKFEstimator ekf;
  EKFEstimator::State state;
  state.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(
      0.4, Eigen::Vector3d::UnitZ()));
  ekf.setState(state);
  const Eigen::Vector3d lever_arm(1.2, -0.3, 0.5);
  EKFEstimator::ObservationJacobian3 jacobian;
  const Eigen::Vector3d predicted = EKFEstimator::leverArmPositionObservation(
    ekf.getState(), lever_arm, &jacobian);
  const Eigen::Vector3d measured = predicted + Eigen::Vector3d(0.2, -0.1, 0.05);
  const Eigen::Matrix3d measurement_covariance =
    Eigen::Vector3d(0.4, 0.5, 0.6).asDiagonal();
  const EKFEstimator::Snapshot before = ekf.getSnapshot();
  const Eigen::Matrix3d expected =
    jacobian * before.covariance * jacobian.transpose() + measurement_covariance;

  const auto diagnostics = ekf.observationUpdateLeverArmPositionWithCovariance(
    measured, lever_arm, measurement_covariance);

  ASSERT_TRUE(diagnostics.accepted);
  EXPECT_TRUE(diagnostics.innovation_covariance.isApprox(expected, 1.0e-12));
  EXPECT_GT((expected - before.covariance.block<3, 3>(0, 0) -
    measurement_covariance).norm(), 0.1);
}

TEST(EKFEstimatorCore, FullVelocityUpdateReportsNis)
{
  EKFEstimator ekf;
  const Eigen::Vector3d measured(1.0, -0.5, 0.25);
  Eigen::Matrix3d covariance;
  covariance <<
    0.5, 0.1, 0.0,
    0.1, 0.8, 0.05,
    0.0, 0.05, 0.4;
  const auto diagnostics = ekf.observationUpdateWorldVelocityWithCovariance(
    measured, covariance);

  EXPECT_TRUE(diagnostics.accepted);
  EXPECT_TRUE(std::isfinite(diagnostics.nis));
  EXPECT_GT(diagnostics.nis, 0.0);
}

TEST(EKFEstimatorCore, YawUpdateUsesActualRightErrorJacobianAndNis)
{
  EKFEstimator ekf;
  EKFEstimator::State state;
  state.orientation = (
    Eigen::Quaterniond(Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ())) *
    Eigen::Quaterniond(Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY())) *
    Eigen::Quaterniond(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()))).normalized();
  ekf.setState(state);
  Eigen::Matrix<double, 1, EKFEstimator::kErrorStateSize> jacobian;
  const double predicted = EKFEstimator::yawObservation(state, &jacobian);
  constexpr double epsilon = 1.0e-7;
  for (int axis = 0; axis < 3; ++axis) {
    EKFEstimator::State positive = state;
    EKFEstimator::State negative = state;
    positive.orientation = (state.orientation *
      Eigen::Quaterniond(Eigen::AngleAxisd(epsilon, Eigen::Vector3d::Unit(axis)))).normalized();
    negative.orientation = (state.orientation *
      Eigen::Quaterniond(Eigen::AngleAxisd(-epsilon, Eigen::Vector3d::Unit(axis)))).normalized();
    const double numerical = std::atan2(
      std::sin(EKFEstimator::yawObservation(positive) -
      EKFEstimator::yawObservation(negative)),
      std::cos(EKFEstimator::yawObservation(positive) -
      EKFEstimator::yawObservation(negative))) / (2.0 * epsilon);
    EXPECT_NEAR(jacobian(0, 6 + axis), numerical, 1.0e-8);
  }
  const double measurement = predicted + 0.2;
  const double variance = 0.05;
  const double expected_s =
    (jacobian * ekf.getSnapshot().covariance * jacobian.transpose())(0, 0) + variance;
  const auto diagnostics = ekf.observationUpdateYawWithVariance(measurement, variance);
  ASSERT_TRUE(diagnostics.accepted);
  EXPECT_NEAR(diagnostics.innovation_variance, expected_s, 1.0e-12);
  EXPECT_NEAR(diagnostics.nis, 0.2 * 0.2 / expected_s, 1.0e-12);
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

TEST(EKFEstimatorCore, BodyVelocityObservationConvergesToWheelSpeed)
{
  EKFEstimator estimator;
  EKFEstimator::State state;
  state.position = Eigen::Vector3d::Zero();
  state.velocity = Eigen::Vector3d(2.0, 1.0, -0.5);
  state.orientation = Eigen::Quaterniond::Identity();
  estimator.setState(state);

  const Eigen::Vector3d measured(8.0, 0.0, 0.0);
  const Eigen::Vector3d variance(0.01, 0.01, 0.01);
  for (int index = 0; index < 50; ++index) {
    EXPECT_EQ(
      estimator.observationUpdateBodyVelocityWithStatus(measured, variance),
      EKFEstimator::ObservationUpdateStatus::kUpdated);
  }
  const Eigen::Vector3d body_velocity =
    estimator.getOrientation().toRotationMatrix().transpose() * estimator.getVelocity();
  EXPECT_NEAR(body_velocity.x(), 8.0, 0.05);
  EXPECT_NEAR(body_velocity.y(), 0.0, 0.05);
  EXPECT_NEAR(body_velocity.z(), 0.0, 0.05);
}

TEST(EKFEstimatorCore, BodyVelocityObservationUsesBodyFrame)
{
  EKFEstimator estimator;
  EKFEstimator::State state;
  state.position = Eigen::Vector3d::Zero();
  state.velocity = Eigen::Vector3d(0.0, 4.0, 0.0);
  state.orientation = Eigen::Quaterniond(
    Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
  estimator.setState(state);

  EXPECT_EQ(
    estimator.observationUpdateBodyVelocityWithStatus(
      Eigen::Vector3d(4.0, 0.0, 0.0), Eigen::Vector3d::Constant(0.01)),
    EKFEstimator::ObservationUpdateStatus::kUpdated);
  EXPECT_NEAR(estimator.getVelocity().x(), 0.0, 1.0e-9);
  EXPECT_NEAR(estimator.getVelocity().y(), 4.0, 1.0e-9);
}

TEST(EKFEstimatorCore, BodyForwardSpeedDoesNotConstrainUnmeasuredAxes)
{
  EKFEstimator ekf;
  EKFEstimator::State state = ekf.getState();
  state.velocity = Eigen::Vector3d(5.0, 2.0, -1.0);
  ekf.setState(state);
  for (int index = 0; index < 20; ++index) {
    EXPECT_EQ(
      ekf.observationUpdateBodyForwardSpeedWithStatus(3.0, 1.0e-3),
      EKFEstimator::ObservationUpdateStatus::kUpdated);
  }
  EXPECT_NEAR(ekf.getVelocity().x(), 3.0, 1.0e-3);
  EXPECT_NEAR(ekf.getVelocity().y(), 2.0, 1.0e-9);
  EXPECT_NEAR(ekf.getVelocity().z(), -1.0, 1.0e-9);
  EXPECT_EQ(
    ekf.observationUpdateBodyForwardSpeedWithStatus(3.0, 0.0),
    EKFEstimator::ObservationUpdateStatus::kInvalidVariance);
}

TEST(EKFEstimatorCore, BodyForwardSpeedFullGainPropagatesCrossStateInformation)
{
  EKFEstimator decoupled;
  EKFEstimator full;
  EKFEstimator gated;
  for (int index = 0; index < 100; ++index) {
    const Eigen::Vector3d gyro(0.03, -0.02, 0.15);
    const Eigen::Vector3d acceleration(1.0, 0.4, 9.7);
    ASSERT_EQ(
      decoupled.predictionUpdateDt(0.01, gyro, acceleration),
      EKFEstimator::PredictionUpdateStatus::kUpdated);
    ASSERT_EQ(
      full.predictionUpdateDt(0.01, gyro, acceleration),
      EKFEstimator::PredictionUpdateStatus::kUpdated);
    ASSERT_EQ(
      gated.predictionUpdateDt(0.01, gyro, acceleration),
      EKFEstimator::PredictionUpdateStatus::kUpdated);
  }
  const Eigen::Quaterniond decoupled_orientation_before = decoupled.getOrientation();
  const Eigen::Quaterniond full_orientation_before = full.getOrientation();

  ASSERT_EQ(
    decoupled.observationUpdateBodyForwardSpeedWithStatus(3.0, 0.01, false),
    EKFEstimator::ObservationUpdateStatus::kUpdated);
  ASSERT_EQ(
    full.observationUpdateBodyForwardSpeedWithStatus(3.0, 0.01, true),
    EKFEstimator::ObservationUpdateStatus::kUpdated);
  gated.setBiasLearningEnabled(false, false);
  ASSERT_EQ(
    gated.observationUpdateBodyForwardSpeedWithStatus(3.0, 0.01, true),
    EKFEstimator::ObservationUpdateStatus::kUpdated);

  EXPECT_TRUE(decoupled.getOrientation().coeffs().isApprox(
      decoupled_orientation_before.coeffs(), 1.0e-12));
  EXPECT_GT(std::abs(full.getOrientation().dot(full_orientation_before)) < 1.0 ?
    2.0 * std::acos(std::abs(full.getOrientation().dot(full_orientation_before))) : 0.0,
    1.0e-6);
  EXPECT_TRUE(gated.getGyroBias().isZero(0.0));
  EXPECT_TRUE(gated.getAccelBias().isZero(0.0));
}

TEST(EKFEstimatorCore, BiasLearningGateSuppressesUnobservableAccelBiasCorrection)
{
  EKFEstimator enabled;
  EKFEstimator gated;
  for (EKFEstimator * estimator : {&enabled, &gated}) {
    estimator->setGravityZ(0.0);
    estimator->setTauAccBias(1.0e12);
    estimator->setVarImuAccBias(1.0e-4);
    ASSERT_TRUE(estimator->setInitialAccelBiasCovariance(1.0));
  }
  gated.setBiasLearningEnabled(true, false);
  const Eigen::Vector3d acceleration(0.3, -0.2, 0.1);
  for (int index = 0; index < 100; ++index) {
    for (EKFEstimator * estimator : {&enabled, &gated}) {
      ASSERT_EQ(estimator->predictionUpdateDt(
          0.01, Eigen::Vector3d::Zero(), acceleration),
        EKFEstimator::PredictionUpdateStatus::kUpdated);
      ASSERT_EQ(estimator->observationUpdateWithStatus(
          Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(1.0e-4)),
        EKFEstimator::ObservationUpdateStatus::kUpdated);
    }
  }
  EXPECT_GT(enabled.getAccelBias().norm(), 0.05);
  EXPECT_TRUE(gated.getAccelBias().isZero(0.0));
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

TEST(EKFEstimatorMath, ObservationJacobiansMatchFiniteDifferenceForOneHundredStates)
{
  std::mt19937 generator(0x5EEDu);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  std::uniform_real_distribution<double> position(-100.0, 100.0);
  std::uniform_real_distribution<double> velocity(-30.0, 30.0);
  constexpr double epsilon = 1.0e-6;

  for (int sample = 0; sample < 100; ++sample) {
    EKFEstimator::State state;
    state.position = Eigen::Vector3d(position(generator), position(generator), position(generator));
    state.velocity = Eigen::Vector3d(velocity(generator), velocity(generator), velocity(generator));
    Eigen::Vector3d axis(unit(generator), unit(generator), unit(generator));
    if (axis.norm() < 1.0e-6) {
      axis = Eigen::Vector3d::UnitX();
    }
    state.orientation = Eigen::Quaterniond(
      Eigen::AngleAxisd(2.8 * unit(generator), axis.normalized()));
    state.gyro_bias = 0.1 * Eigen::Vector3d(unit(generator), unit(generator), unit(generator));
    state.accel_bias = 0.5 * Eigen::Vector3d(unit(generator), unit(generator), unit(generator));
    const Eigen::Vector3d lever_arm(
      3.0 * unit(generator), 3.0 * unit(generator), 3.0 * unit(generator));

    EKFEstimator::ObservationJacobian3 analytic;
    (void)EKFEstimator::positionObservation(state, &analytic);
    const auto numeric_position = numericalObservationJacobian(
      state, [](const EKFEstimator::State & value) {
        return EKFEstimator::positionObservation(value);
      }, epsilon);
    EXPECT_LT((analytic - numeric_position).cwiseAbs().maxCoeff(), 2.0e-8) << sample;

    (void)EKFEstimator::leverArmPositionObservation(state, lever_arm, &analytic);
    const auto numeric_lever = numericalObservationJacobian(
      state, [&lever_arm](const EKFEstimator::State & value) {
        return EKFEstimator::leverArmPositionObservation(value, lever_arm);
      }, epsilon);
    EXPECT_LT((analytic - numeric_lever).cwiseAbs().maxCoeff(), 5.0e-8) << sample;

    (void)EKFEstimator::worldVelocityObservation(state, &analytic);
    const auto numeric_world_velocity = numericalObservationJacobian(
      state, [](const EKFEstimator::State & value) {
        return EKFEstimator::worldVelocityObservation(value);
      }, epsilon);
    EXPECT_LT((analytic - numeric_world_velocity).cwiseAbs().maxCoeff(), 2.0e-8) << sample;

    (void)EKFEstimator::bodyVelocityObservation(state, &analytic);
    const auto numeric_body_velocity = numericalObservationJacobian(
      state, [](const EKFEstimator::State & value) {
        return EKFEstimator::bodyVelocityObservation(value);
      }, epsilon);
    EXPECT_LT((analytic - numeric_body_velocity).cwiseAbs().maxCoeff(), 2.0e-7) << sample;
    // Forward wheel and NHC are respectively row 0 and rows 1--2 of this same model.

    (void)EKFEstimator::gyroBiasObservation(state, &analytic);
    const auto numeric_bias = numericalObservationJacobian(
      state, [](const EKFEstimator::State & value) {
        return EKFEstimator::gyroBiasObservation(value);
      }, epsilon);
    EXPECT_LT((analytic - numeric_bias).cwiseAbs().maxCoeff(), 2.0e-10) << sample;

    const Eigen::Quaterniond nominal_orientation = state.orientation.normalized();
    const auto numeric_orientation = numericalObservationJacobian(
      state, [&nominal_orientation](const EKFEstimator::State & value) {
        return quaternionLog(
          nominal_orientation.conjugate() * value.orientation.normalized());
      }, epsilon);
    EXPECT_LT(
      (EKFEstimator::orientationObservationJacobian() - numeric_orientation)
      .cwiseAbs().maxCoeff(), 2.0e-9) << sample;
  }
}

TEST(EKFEstimatorMath, SecondOrderTransitionMatchesFiniteDifferenceForOneHundredStates)
{
  std::mt19937 generator(0xC0FFEEu);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  std::uniform_real_distribution<double> velocity(-20.0, 20.0);
  constexpr double dt = 1.0e-3;
  constexpr double epsilon = 2.0e-6;
  constexpr double tau_gyro = 300.0;
  constexpr double tau_accel = 600.0;

  for (int sample = 0; sample < 100; ++sample) {
    EKFEstimator::State state;
    state.position = 50.0 * Eigen::Vector3d(unit(generator), unit(generator), unit(generator));
    state.velocity = Eigen::Vector3d(velocity(generator), velocity(generator), velocity(generator));
    Eigen::Vector3d axis(unit(generator), unit(generator), unit(generator));
    if (axis.norm() < 1.0e-6) {
      axis = Eigen::Vector3d::UnitY();
    }
    state.orientation = Eigen::Quaterniond(
      Eigen::AngleAxisd(2.8 * unit(generator), axis.normalized()));
    state.gyro_bias = 0.05 * Eigen::Vector3d(unit(generator), unit(generator), unit(generator));
    state.accel_bias = 0.2 * Eigen::Vector3d(unit(generator), unit(generator), unit(generator));
    const Eigen::Vector3d gyro =
      state.gyro_bias + 1.5 * Eigen::Vector3d(unit(generator), unit(generator), unit(generator));
    const Eigen::Vector3d acceleration =
      state.accel_bias + 12.0 * Eigen::Vector3d(unit(generator), unit(generator), unit(generator));

    const auto continuous = EKFEstimator::continuousErrorStateJacobian(
      state, gyro, acceleration, tau_gyro, tau_accel);
    const auto analytic = EKFEstimator::secondOrderErrorStateTransition(continuous, dt);
    const EKFEstimator::State nominal_next =
      propagateState(state, dt, gyro, acceleration, tau_gyro, tau_accel);
    EKFEstimator::ErrorStateMatrix numeric;
    for (int column = 0; column < EKFEstimator::kErrorStateSize; ++column) {
      EKFEstimator::ErrorStateVector error = EKFEstimator::ErrorStateVector::Zero();
      error(column) = epsilon;
      const EKFEstimator::State positive_next = propagateState(
        perturbState(state, error), dt, gyro, acceleration, tau_gyro, tau_accel);
      const EKFEstimator::State negative_next = propagateState(
        perturbState(state, -error), dt, gyro, acceleration, tau_gyro, tau_accel);
      numeric.col(column) =
        (stateError(nominal_next, positive_next) -
        stateError(nominal_next, negative_next)) / (2.0 * epsilon);
    }
    EXPECT_LT((analytic - numeric).cwiseAbs().maxCoeff(), 3.0e-5) << sample;
  }
}

TEST(EKFEstimatorMath, RightJacobianMatchesFiniteDifferenceForOneHundredRotations)
{
  std::mt19937 generator(0xBADC0DEu);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  constexpr double epsilon = 1.0e-7;
  for (int sample = 0; sample < 100; ++sample) {
    Eigen::Vector3d rotation(2.5 * unit(generator), 2.5 * unit(generator), 2.5 * unit(generator));
    if (rotation.norm() > 3.0) {
      rotation *= 3.0 / rotation.norm();
    }
    const double angle = rotation.norm();
    const Eigen::Quaterniond nominal = angle > 0.0 ?
      Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotation / angle)) :
      Eigen::Quaterniond::Identity();
    Eigen::Matrix3d numeric;
    for (int column = 0; column < 3; ++column) {
      Eigen::Vector3d positive_rotation = rotation;
      Eigen::Vector3d negative_rotation = rotation;
      positive_rotation(column) += epsilon;
      negative_rotation(column) -= epsilon;
      const auto exponential = [](const Eigen::Vector3d & value) {
          const double value_norm = value.norm();
          return value_norm > 0.0 ?
                 Eigen::Quaterniond(Eigen::AngleAxisd(value_norm, value / value_norm)) :
                 Eigen::Quaterniond::Identity();
        };
      const Eigen::Vector3d positive = quaternionLog(
        nominal.conjugate() * exponential(positive_rotation));
      const Eigen::Vector3d negative = quaternionLog(
        nominal.conjugate() * exponential(negative_rotation));
      numeric.col(column) = (positive - negative) / (2.0 * epsilon);
    }
    EXPECT_LT(
      (EKFEstimator::rightJacobianSO3(rotation) - numeric).cwiseAbs().maxCoeff(),
      2.0e-8) << sample;
  }
}

TEST(EKFEstimatorCore, NumericalInvariantsHoldAfterMixedPredictionAndUpdates)
{
  EKFEstimator estimator;
  estimator.setUseContinuousProcessNoiseDensity(true);
  estimator.setUseSecondOrderStateTransition(true);
  estimator.setUseSecondOrderProcessNoise(true);
  estimator.setVarImuAcc(0.03);
  estimator.setVarImuGyro(0.01);
  estimator.setVarImuGyroBias(1.0e-8);
  estimator.setVarImuAccBias(1.0e-7);
  EKFEstimator::State state;
  state.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()));
  state.velocity = Eigen::Vector3d(8.0, 1.0, -0.2);
  estimator.setState(state);

  for (int iteration = 0; iteration < 100; ++iteration) {
    const double phase = 0.03 * iteration;
    const Eigen::Vector3d gyro(0.01 * std::sin(phase), 0.02 * std::cos(phase), 0.05);
    const Eigen::Vector3d acceleration(
      0.2 * std::cos(phase), 0.1 * std::sin(phase), 9.80665);
    ASSERT_EQ(
      estimator.predictionUpdateDt(0.01, gyro, acceleration),
      EKFEstimator::PredictionUpdateStatus::kUpdated);
    if (iteration % 5 == 0) {
      ASSERT_EQ(
        estimator.observationUpdateVelocityWithStatus(
          estimator.getVelocity() + Eigen::Vector3d(0.02, -0.01, 0.01),
          Eigen::Vector3d::Constant(0.1), true),
        EKFEstimator::ObservationUpdateStatus::kUpdated);
    }
    if (iteration % 10 == 0) {
      ASSERT_EQ(
        estimator.observationUpdatePositionWithLeverArmWithStatus(
          estimator.getPosition() + estimator.getOrientation() * Eigen::Vector3d(1.2, 0.3, 0.8),
          Eigen::Vector3d(1.2, 0.3, 0.8), Eigen::Vector3d::Constant(0.2)),
        EKFEstimator::ObservationUpdateStatus::kUpdated);
    }
    EXPECT_TRUE(estimator.checkNumericalInvariants()) << iteration;
    EXPECT_LE(estimator.getQuaternionNormError(), 1.0e-12) << iteration;
    EXPECT_LE(estimator.getCovarianceSymmetryError(), 1.0e-12) << iteration;
    EXPECT_GE(estimator.getMinimumCovarianceEigenvalue(), -1.0e-10) << iteration;
  }
}

TEST(EKFEstimatorCore, InvalidPredictionDoesNotMutateStateOrCovariance)
{
  EKFEstimator estimator;
  const Eigen::VectorXd state_before = estimator.getX();
  const Eigen::MatrixXd covariance_before = estimator.getCovariance();
  Eigen::Vector3d invalid_gyro = Eigen::Vector3d::Zero();
  invalid_gyro.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    estimator.predictionUpdateDt(0.01, invalid_gyro, Eigen::Vector3d::Zero()),
    EKFEstimator::PredictionUpdateStatus::kInvalidInput);
  EXPECT_TRUE(estimator.getX().isApprox(state_before, 0.0));
  EXPECT_TRUE(estimator.getCovariance().isApprox(covariance_before, 0.0));
}

TEST(EKFEstimatorCore, IllConditionedMeasurementCovarianceIsRejectedWithoutMutation)
{
  EKFEstimator estimator;
  const Eigen::VectorXd state_before = estimator.getX();
  const Eigen::MatrixXd covariance_before = estimator.getCovariance();
  EXPECT_EQ(
    estimator.observationUpdateWithStatus(
      Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Vector3d(1.0e-20, 1.0e20, 1.0)),
    EKFEstimator::ObservationUpdateStatus::kInvalidVariance);
  EXPECT_TRUE(estimator.getX().isApprox(state_before, 0.0));
  EXPECT_TRUE(estimator.getCovariance().isApprox(covariance_before, 0.0));
}

TEST(EKFEstimatorMath, ExactDiscretizationHasSemigroupPropertyAndPsdNoise)
{
  EKFEstimator::State state;
  state.orientation = Eigen::Quaterniond(
    Eigen::AngleAxisd(0.7, Eigen::Vector3d(1.0, -2.0, 0.5).normalized()));
  state.gyro_bias = Eigen::Vector3d(0.01, -0.02, 0.005);
  state.accel_bias = Eigen::Vector3d(0.1, -0.05, 0.02);
  const Eigen::Vector3d gyro(0.2, -0.1, 0.3);
  const Eigen::Vector3d acceleration(2.0, -1.0, 9.5);
  constexpr double tau_gyro = 300.0;
  constexpr double tau_accel = 600.0;
  const auto continuous = EKFEstimator::continuousErrorStateJacobian(
    state, gyro, acceleration, tau_gyro, tau_accel);
  const auto noise_input = EKFEstimator::continuousNoiseInputJacobian(state);
  const auto noise = EKFEstimator::continuousNoiseCovariance(0.03, 0.01, 1.0e-8, 1.0e-7);
  const EKFEstimator::ErrorStateMatrix error_noise = noise_input * noise * noise_input.transpose();

  const auto first = EKFEstimator::exactDiscretizeErrorModel(continuous, error_noise, 0.013);
  const auto second = EKFEstimator::exactDiscretizeErrorModel(continuous, error_noise, 0.027);
  const auto whole = EKFEstimator::exactDiscretizeErrorModel(continuous, error_noise, 0.040);
  const EKFEstimator::ErrorStateMatrix composed_transition =
    second.transition * first.transition;
  const EKFEstimator::ErrorStateMatrix composed_noise =
    second.transition * first.process_covariance * second.transition.transpose() +
    second.process_covariance;
  EXPECT_TRUE(whole.transition.isApprox(composed_transition, 2.0e-13));
  EXPECT_TRUE(whole.process_covariance.isApprox(composed_noise, 2.0e-13));
  const Eigen::SelfAdjointEigenSolver<EKFEstimator::ErrorStateMatrix> eigen_solver(
    whole.process_covariance, Eigen::EigenvaluesOnly);
  ASSERT_EQ(eigen_solver.info(), Eigen::Success);
  EXPECT_GE(eigen_solver.eigenvalues().minCoeff(), -1.0e-13);

  EXPECT_NEAR(whole.transition(9, 9), std::exp(-0.040 / tau_gyro), 1.0e-14);
  EXPECT_NEAR(whole.transition(12, 12), std::exp(-0.040 / tau_accel), 1.0e-14);
  const double expected_gyro_bias_variance =
    1.0e-8 * tau_gyro * 0.5 * (1.0 - std::exp(-2.0 * 0.040 / tau_gyro));
  EXPECT_NEAR(whole.process_covariance(9, 9), expected_gyro_bias_variance, 1.0e-15);
}

TEST(EKFEstimatorMath, ExactCovarianceIsRateIndependentAtFiftyToFourHundredHertz)
{
  EKFEstimator::State state;
  state.orientation = Eigen::Quaterniond(
    Eigen::AngleAxisd(-0.5, Eigen::Vector3d(0.3, 0.4, -0.2).normalized()));
  const Eigen::Vector3d gyro(0.1, 0.2, -0.15);
  const Eigen::Vector3d acceleration(1.0, -0.5, 9.7);
  const auto continuous = EKFEstimator::continuousErrorStateJacobian(
    state, gyro, acceleration, 250.0, 500.0);
  const auto noise_input = EKFEstimator::continuousNoiseInputJacobian(state);
  const auto noise = EKFEstimator::continuousNoiseCovariance(0.04, 0.02, 2.0e-8, 3.0e-7);
  const EKFEstimator::ErrorStateMatrix error_noise = noise_input * noise * noise_input.transpose();
  const EKFEstimator::ErrorStateMatrix initial =
    0.2 * EKFEstimator::ErrorStateMatrix::Identity();
  EKFEstimator::ErrorStateMatrix reference;

  for (const int rate : {50, 100, 200, 400}) {
    const auto discrete = EKFEstimator::exactDiscretizeErrorModel(
      continuous, error_noise, 1.0 / static_cast<double>(rate));
    EKFEstimator::ErrorStateMatrix covariance = initial;
    for (int step = 0; step < rate; ++step) {
      covariance = discrete.transition * covariance * discrete.transition.transpose() +
        discrete.process_covariance;
    }
    covariance = 0.5 * (covariance + covariance.transpose());
    if (rate == 50) {
      reference = covariance;
    } else {
      EXPECT_TRUE(covariance.isApprox(reference, 2.0e-11)) << rate;
    }
    const Eigen::SelfAdjointEigenSolver<EKFEstimator::ErrorStateMatrix> eigen_solver(
      covariance, Eigen::EigenvaluesOnly);
    ASSERT_EQ(eigen_solver.info(), Eigen::Success);
    EXPECT_GE(eigen_solver.eigenvalues().minCoeff(), -1.0e-12) << rate;
  }
}

TEST(EKFEstimatorCore, ExactBackendCovarianceIsRateIndependent)
{
  Eigen::MatrixXd reference;
  for (const int rate : {50, 100, 200, 400}) {
    EKFEstimator estimator;
    estimator.setPropagationModel(EKFEstimator::PropagationModel::kExact);
    estimator.setVarImuAcc(0.04);
    estimator.setVarImuGyro(0.02);
    estimator.setVarImuGyroBias(2.0e-8);
    estimator.setVarImuAccBias(3.0e-7);
    estimator.setTauGyroBias(250.0);
    estimator.setTauAccBias(500.0);
    ASSERT_TRUE(estimator.primeImuMeasurement(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
    for (int step = 0; step < rate; ++step) {
      ASSERT_EQ(
        estimator.predictionUpdateDt(
          1.0 / static_cast<double>(rate),
          Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
        EKFEstimator::PredictionUpdateStatus::kUpdated);
    }
    if (rate == 50) {
      reference = estimator.getCovariance();
    } else {
      EXPECT_TRUE(estimator.getCovariance().isApprox(reference, 2.0e-9)) << rate;
    }
    EXPECT_TRUE(estimator.checkNumericalInvariants()) << rate;
  }
}

TEST(EKFEstimatorCore, MidpointMeanIntegrationImprovesTurningAcceleration)
{
  EKFEstimator legacy;
  EKFEstimator exact;
  exact.setPropagationModel(EKFEstimator::PropagationModel::kExact);
  ASSERT_TRUE(legacy.setGravityZ(0.0));
  ASSERT_TRUE(exact.setGravityZ(0.0));
  const Eigen::Vector3d gyro(0.0, 0.0, 1.0);
  const Eigen::Vector3d acceleration(2.0, 0.0, 0.0);
  ASSERT_TRUE(legacy.primeImuMeasurement(gyro, acceleration));
  ASSERT_TRUE(exact.primeImuMeasurement(gyro, acceleration));
  constexpr double dt = 0.02;
  constexpr int steps = 50;
  for (int step = 0; step < steps; ++step) {
    ASSERT_EQ(
      legacy.predictionUpdateDt(dt, gyro, acceleration),
      EKFEstimator::PredictionUpdateStatus::kUpdated);
    ASSERT_EQ(
      exact.predictionUpdateDt(dt, gyro, acceleration),
      EKFEstimator::PredictionUpdateStatus::kUpdated);
  }
  constexpr double duration = dt * steps;
  const Eigen::Vector3d expected_velocity(
    2.0 * std::sin(duration), 2.0 * (1.0 - std::cos(duration)), 0.0);
  const Eigen::Vector3d expected_position(
    2.0 * (1.0 - std::cos(duration)),
    2.0 * (duration - std::sin(duration)), 0.0);
  const double legacy_error =
    (legacy.getVelocity() - expected_velocity).norm() +
    (legacy.getPosition() - expected_position).norm();
  const double exact_error =
    (exact.getVelocity() - expected_velocity).norm() +
    (exact.getPosition() - expected_position).norm();
  EXPECT_LT(exact_error, 0.15 * legacy_error);
}

TEST(EKFEstimatorMath, ExactMonteCarloNeesIsConsistentAndBetterThanLegacy)
{
  EKFEstimator::State state;
  state.orientation = Eigen::Quaterniond(
    Eigen::AngleAxisd(0.8, Eigen::Vector3d(0.2, -0.7, 0.4).normalized()));
  const Eigen::Vector3d gyro(0.3, -0.2, 0.4);
  const Eigen::Vector3d acceleration(3.0, -2.0, 8.5);
  const auto continuous = EKFEstimator::continuousErrorStateJacobian(
    state, gyro, acceleration, 300.0, 600.0);
  const auto noise_input = EKFEstimator::continuousNoiseInputJacobian(state);
  const auto continuous_noise =
    EKFEstimator::continuousNoiseCovariance(0.05, 0.02, 2.0e-7, 5.0e-6);
  const EKFEstimator::ErrorStateMatrix error_noise =
    noise_input * continuous_noise * noise_input.transpose();
  constexpr double dt = 0.02;
  constexpr int steps = 50;
  const auto exact_step = EKFEstimator::exactDiscretizeErrorModel(
    continuous, error_noise, dt);

  const EKFEstimator::ErrorStateMatrix legacy_transition =
    EKFEstimator::ErrorStateMatrix::Identity() + continuous * dt;
  EKFEstimator::ContinuousNoiseMatrix legacy_discrete_noise =
    EKFEstimator::ContinuousNoiseMatrix::Zero();
  legacy_discrete_noise.block<3, 3>(0, 0) =
    0.05 * dt * dt * Eigen::Matrix3d::Identity();
  legacy_discrete_noise.block<3, 3>(3, 3) =
    0.02 * dt * dt * Eigen::Matrix3d::Identity();
  legacy_discrete_noise.block<3, 3>(6, 6) =
    2.0e-7 * dt * Eigen::Matrix3d::Identity();
  legacy_discrete_noise.block<3, 3>(9, 9) =
    5.0e-6 * dt * Eigen::Matrix3d::Identity();
  const EKFEstimator::ErrorStateMatrix legacy_process_noise =
    noise_input * legacy_discrete_noise * noise_input.transpose();

  EKFEstimator::ErrorStateMatrix truth_covariance =
    1.0e-6 * EKFEstimator::ErrorStateMatrix::Identity();
  EKFEstimator::ErrorStateMatrix exact_covariance = truth_covariance;
  EKFEstimator::ErrorStateMatrix legacy_covariance = truth_covariance;
  for (int step = 0; step < steps; ++step) {
    truth_covariance =
      exact_step.transition * truth_covariance * exact_step.transition.transpose() +
      exact_step.process_covariance;
    exact_covariance =
      exact_step.transition * exact_covariance * exact_step.transition.transpose() +
      exact_step.process_covariance;
    legacy_covariance =
      legacy_transition * legacy_covariance * legacy_transition.transpose() +
      legacy_process_noise;
  }
  truth_covariance = 0.5 * (truth_covariance + truth_covariance.transpose());
  exact_covariance = 0.5 * (exact_covariance + exact_covariance.transpose());
  legacy_covariance = 0.5 * (legacy_covariance + legacy_covariance.transpose());
  const Eigen::LLT<EKFEstimator::ErrorStateMatrix> truth_factorization(truth_covariance);
  const Eigen::LDLT<EKFEstimator::ErrorStateMatrix> exact_factorization(exact_covariance);
  const Eigen::LDLT<EKFEstimator::ErrorStateMatrix> legacy_factorization(legacy_covariance);
  ASSERT_EQ(truth_factorization.info(), Eigen::Success);
  ASSERT_EQ(exact_factorization.info(), Eigen::Success);
  ASSERT_EQ(legacy_factorization.info(), Eigen::Success);

  constexpr int trials = 4000;
  std::mt19937 generator(0x4E454553u);
  std::normal_distribution<double> normal(0.0, 1.0);
  double exact_nees_sum = 0.0;
  double legacy_nees_sum = 0.0;
  for (int trial = 0; trial < trials; ++trial) {
    EKFEstimator::ErrorStateVector standard_normal;
    for (int index = 0; index < EKFEstimator::kErrorStateSize; ++index) {
      standard_normal(index) = normal(generator);
    }
    const EKFEstimator::ErrorStateVector error =
      truth_factorization.matrixL() * standard_normal;
    exact_nees_sum += error.dot(exact_factorization.solve(error));
    legacy_nees_sum += error.dot(legacy_factorization.solve(error));
  }
  const double exact_mean_nees = exact_nees_sum / trials;
  const double legacy_mean_nees = legacy_nees_sum / trials;
  constexpr double expected_mean = EKFEstimator::kErrorStateSize;
  const double three_sigma_mean_tolerance =
    3.0 * std::sqrt(2.0 * expected_mean / trials);
  EXPECT_NEAR(exact_mean_nees, expected_mean, three_sigma_mean_tolerance);
  EXPECT_LT(
    std::fabs(exact_mean_nees - expected_mean),
    std::fabs(legacy_mean_nees - expected_mean));
}

TEST(EKFEstimatorMath, StateTruthNeesAndAllObservationNisHaveExpectedCoverage)
{
  constexpr int trials = 2000;
  std::mt19937 generator(20260716U);
  std::normal_distribution<double> normal(0.0, 1.0);
  int nees_covered = 0;
  int position_covered = 0;
  int velocity_covered = 0;
  int yaw_covered = 0;
  double nees_sum = 0.0;
  double position_nis_sum = 0.0;
  double velocity_nis_sum = 0.0;
  double yaw_nis_sum = 0.0;
  const Eigen::Vector3d state_variance = Eigen::Vector3d::Ones();
  const Eigen::Matrix3d measurement_covariance =
    Eigen::Vector3d::Constant(0.5).asDiagonal();

  for (int trial = 0; trial < trials; ++trial) {
    EKFEstimator::ErrorStateVector truth_error;
    for (int index = 0; index < EKFEstimator::kErrorStateSize; ++index) {
      truth_error(index) = normal(generator);
    }
    const double nees = truth_error.squaredNorm();
    nees_sum += nees;
    nees_covered += nees >= 6.262 && nees <= 27.488 ? 1 : 0;

    EKFEstimator position_filter;
    EKFEstimator velocity_filter;
    EKFEstimator yaw_filter;
    for (EKFEstimator * estimator : {&position_filter, &velocity_filter, &yaw_filter}) {
      ASSERT_TRUE(estimator->setInitialErrorStateCovariance(
          state_variance, state_variance, state_variance,
          state_variance, state_variance));
    }
    Eigen::Vector3d position_noise;
    Eigen::Vector3d velocity_noise;
    for (int axis = 0; axis < 3; ++axis) {
      position_noise(axis) = std::sqrt(0.5) * normal(generator);
      velocity_noise(axis) = std::sqrt(0.5) * normal(generator);
    }
    const auto position_diagnostics =
      position_filter.observationUpdatePositionWithCovariance(
      truth_error.segment<3>(0) + position_noise, measurement_covariance);
    const auto velocity_diagnostics =
      velocity_filter.observationUpdateWorldVelocityWithCovariance(
      truth_error.segment<3>(3) + velocity_noise, measurement_covariance);
    const double yaw_noise = std::sqrt(0.5) * normal(generator);
    const auto yaw_diagnostics = yaw_filter.observationUpdateYawWithVariance(
      truth_error(8) + yaw_noise, 0.5);
    ASSERT_TRUE(position_diagnostics.accepted);
    ASSERT_TRUE(velocity_diagnostics.accepted);
    ASSERT_TRUE(yaw_diagnostics.accepted);
    position_nis_sum += position_diagnostics.nis;
    velocity_nis_sum += velocity_diagnostics.nis;
    yaw_nis_sum += yaw_diagnostics.nis;
    position_covered +=
      position_diagnostics.nis >= 0.216 && position_diagnostics.nis <= 9.348 ? 1 : 0;
    velocity_covered +=
      velocity_diagnostics.nis >= 0.216 && velocity_diagnostics.nis <= 9.348 ? 1 : 0;
    yaw_covered +=
      yaw_diagnostics.nis >= 0.000982 && yaw_diagnostics.nis <= 5.024 ? 1 : 0;
  }

  EXPECT_NEAR(nees_sum / trials, 15.0, 0.5);
  EXPECT_NEAR(position_nis_sum / trials, 3.0, 0.2);
  EXPECT_NEAR(velocity_nis_sum / trials, 3.0, 0.2);
  EXPECT_NEAR(yaw_nis_sum / trials, 1.0, 0.1);
  for (const int covered : {nees_covered, position_covered, velocity_covered, yaw_covered}) {
    const double coverage = static_cast<double>(covered) / trials;
    EXPECT_GT(coverage, 0.92);
    EXPECT_LT(coverage, 0.98);
  }
}

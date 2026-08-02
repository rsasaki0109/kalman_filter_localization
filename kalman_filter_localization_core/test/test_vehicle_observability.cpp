// Copyright (c) 2026, Ryohei Sasaki
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//  * Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>

#include <limits>

#include <kalman_filter_localization/core/ekf_estimator.hpp>
#include <kalman_filter_localization/core/measurement_quality.hpp>
#include <kalman_filter_localization/core/vehicle_model.hpp>
#include <kalman_filter_localization/core/vehicle_observability.hpp>

using kalman_filter_localization::core::SlipTurnDetector;
using kalman_filter_localization::core::StationaryDetector;
using kalman_filter_localization::core::VehicleModelConfig;
using kalman_filter_localization::core::VehicleModelInput;
using kalman_filter_localization::core::GroundVehicleModel;
using kalman_filter_localization::core::PlanarVehicleModel;

TEST(VehicleObservability, StationaryDetectorRequiresContinuousDuration)
{
  StationaryDetector::Config config;
  config.minimum_duration_sec = 0.5;
  StationaryDetector detector(config);
  const Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  const Eigen::Vector3d accel(0.0, 0.0, 9.80665);

  EXPECT_EQ(detector.update(0.0, gyro, accel, 0.0, 9.80665),
    StationaryDetector::State::kCandidate);
  EXPECT_EQ(detector.update(0.49, gyro, accel, 0.0, 9.80665),
    StationaryDetector::State::kCandidate);
  EXPECT_EQ(detector.update(0.5, gyro, accel, 0.0, 9.80665),
    StationaryDetector::State::kStationary);
  EXPECT_EQ(detector.update(0.6, Eigen::Vector3d(0.1, 0.0, 0.0), accel, 0.0, 9.80665),
    StationaryDetector::State::kMoving);
}

TEST(VehicleObservability, SlipInjectionReducesNhcGainAndRecovers)
{
  SlipTurnDetector::Config config;
  config.wheel_innovation_threshold_mps = 0.5;
  config.maximum_variance_scale = 100.0;
  config.recovery_samples = 3U;
  SlipTurnDetector detector(config);

  for (int sample = 0; sample < 10; ++sample) {
    const auto result = detector.update(0.05, 0.1, 0.05);
    EXPECT_EQ(result.state, SlipTurnDetector::State::kTrusted);
    EXPECT_DOUBLE_EQ(result.variance_scale, 1.0);
  }
  const auto slip = detector.update(0.05, 0.1, 3.0);
  ASSERT_EQ(slip.state, SlipTurnDetector::State::kSlip);
  EXPECT_GE(slip.variance_scale, 36.0);

  EKFEstimator trusted_filter;
  EKFEstimator slip_filter;
  EKFEstimator::State state;
  state.velocity = Eigen::Vector3d(5.0, 1.0, 0.0);
  trusted_filter.setState(state);
  slip_filter.setState(state);
  ASSERT_EQ(trusted_filter.observationUpdateBodyVelocityConstraintWithStatus(
      Eigen::Vector2d::Zero(), Eigen::Vector2d(0.01, 0.01)),
    EKFEstimator::ObservationUpdateStatus::kUpdated);
  ASSERT_EQ(slip_filter.observationUpdateBodyVelocityConstraintWithStatus(
      Eigen::Vector2d::Zero(), Eigen::Vector2d(0.01, 0.01) * slip.variance_scale),
    EKFEstimator::ObservationUpdateStatus::kUpdated);
  EXPECT_GT(std::abs(slip_filter.getVelocity().y()),
    std::abs(trusted_filter.getVelocity().y()));

  EXPECT_EQ(detector.update(0.05, 0.1, 0.05).state, SlipTurnDetector::State::kSlip);
  EXPECT_EQ(detector.update(0.05, 0.1, 0.05).state, SlipTurnDetector::State::kSlip);
  const auto recovered = detector.update(0.05, 0.1, 0.05);
  EXPECT_EQ(recovered.state, SlipTurnDetector::State::kTrusted);
  EXPECT_DOUBLE_EQ(recovered.variance_scale, 1.0);
}

TEST(VehicleObservability, BiasLearningNeedsStationarityOrExcitation)
{
  const auto unobservable = kalman_filter_localization::core::evaluateBiasObservability(
    false, 0.01, 0.1, 0.1, 0.5);
  EXPECT_FALSE(unobservable.learn_gyro_bias);
  EXPECT_FALSE(unobservable.learn_accel_bias);

  const auto excited = kalman_filter_localization::core::evaluateBiasObservability(
    false, 0.2, 1.0, 0.1, 0.5);
  EXPECT_TRUE(excited.learn_gyro_bias);
  EXPECT_TRUE(excited.learn_accel_bias);

  const auto stationary = kalman_filter_localization::core::evaluateBiasObservability(
    true, 0.0, 0.0, 0.1, 0.5);
  EXPECT_TRUE(stationary.learn_gyro_bias);
  EXPECT_TRUE(stationary.learn_accel_bias);
}

TEST(VehicleObservability, GnssReacquisitionRequiresConsistencyAndBlendsGradually)
{
  kalman_filter_localization::core::GnssReacquisitionGate::Config config;
  config.outage_duration_sec = 1.0;
  config.required_consistent_updates = 3U;
  config.blending_updates = 3U;
  config.initial_variance_scale = 10.0;
  kalman_filter_localization::core::GnssReacquisitionGate gate(config);

  EXPECT_TRUE(gate.update(0.0, 0.1, 0.1).accepted);
  const auto first = gate.update(2.0, 1.0, 0.5);
  EXPECT_FALSE(first.accepted);
  EXPECT_EQ(first.reason,
    kalman_filter_localization::core::GnssReacquisitionGate::Reason::kWaitingForConsistency);
  EXPECT_FALSE(gate.update(2.1, 1.0, 0.5).accepted);
  const auto blended_first = gate.update(2.2, 1.0, 0.5);
  ASSERT_TRUE(blended_first.accepted);
  EXPECT_DOUBLE_EQ(blended_first.variance_scale, 10.0);
  const auto blended_second = gate.update(2.3, 1.0, 0.5);
  EXPECT_TRUE(blended_second.accepted);
  EXPECT_LT(blended_second.variance_scale, blended_first.variance_scale);
  EXPECT_GT(blended_second.variance_scale, 1.0);
  (void)gate.update(2.4, 1.0, 0.5);
  EXPECT_DOUBLE_EQ(gate.update(2.5, 1.0, 0.5).variance_scale, 1.0);
}

TEST(VehicleObservability, MeasurementQualityUsesCommonRejectReasons)
{
  using kalman_filter_localization::core::MeasurementRejectReason;
  EXPECT_EQ(kalman_filter_localization::core::evaluateMeasurementQuality(
      true, 1.0, 2.0, 5.0, 10.0).reason, MeasurementRejectReason::kNone);
  EXPECT_EQ(kalman_filter_localization::core::evaluateMeasurementQuality(
      false, 1.0, 2.0, 5.0, 10.0).reason,
    MeasurementRejectReason::kReceiverQuality);
  EXPECT_EQ(kalman_filter_localization::core::evaluateMeasurementQuality(
      true, 6.0, 2.0, 5.0, 10.0).reason,
    MeasurementRejectReason::kInnovationMagnitude);
  EXPECT_EQ(kalman_filter_localization::core::evaluateMeasurementQuality(
      true, 1.0, 11.0, 5.0, 10.0).reason, MeasurementRejectReason::kNis);
}

TEST(VehicleModel, GroundModelProvidesStablePluginContract)
{
  GroundVehicleModel model;
  VehicleModelConfig config;
  config.minimum_forward_speed_mps = 0.5;
  config.wheel_innovation_threshold_mps = 0.5;
  config.recovery_samples = 2U;
  ASSERT_TRUE(model.configure(config));
  EXPECT_EQ(model.name(), "ground_vehicle");

  VehicleModelInput input;
  input.time_sec = 1.0;
  input.body_velocity = Eigen::Vector3d(5.0, 0.0, 0.0);
  input.yaw_rate_radps = 0.05;
  input.lateral_acceleration_mps2 = 0.1;
  input.wheel_innovation_mps = 0.1;
  input.has_wheel_innovation = true;
  const auto trusted = model.evaluate(input);
  ASSERT_TRUE(trusted.valid);
  EXPECT_TRUE(trusted.apply_nonholonomic_constraint);
  EXPECT_TRUE(trusted.constrain_vertical_velocity);
  EXPECT_EQ(trusted.slip_state_text, "trusted");
  EXPECT_DOUBLE_EQ(trusted.nhc_variance_scale, 1.0);

  input.time_sec = 2.0;
  input.wheel_innovation_mps = 2.0;
  const auto slip = model.evaluate(input);
  ASSERT_TRUE(slip.valid);
  EXPECT_EQ(slip.slip_state_text, "slip");
  EXPECT_GT(slip.nhc_variance_scale, 1.0);

  input.body_velocity.x() = 0.1;
  const auto stopped = model.evaluate(input);
  ASSERT_TRUE(stopped.valid);
  EXPECT_FALSE(stopped.apply_nonholonomic_constraint);
}

TEST(VehicleModel, PlanarModelLeavesVerticalMotionUnconstrained)
{
  PlanarVehicleModel model;
  ASSERT_TRUE(model.configure(VehicleModelConfig{}));
  EXPECT_EQ(model.name(), "planar_vehicle");

  VehicleModelInput input;
  input.time_sec = 1.0;
  input.body_velocity = Eigen::Vector3d(2.0, 0.0, 0.2);
  input.yaw_rate_radps = 0.0;
  input.lateral_acceleration_mps2 = 0.0;
  const auto output = model.evaluate(input);
  ASSERT_TRUE(output.valid);
  EXPECT_TRUE(output.apply_nonholonomic_constraint);
  EXPECT_FALSE(output.constrain_vertical_velocity);
}

TEST(VehicleModel, InvalidInputIsRejectedWithoutStateMutation)
{
  GroundVehicleModel model;
  ASSERT_TRUE(model.configure(VehicleModelConfig{}));
  VehicleModelInput input;
  input.time_sec = 1.0;
  input.body_velocity = Eigen::Vector3d(2.0, 0.0, 0.0);
  input.yaw_rate_radps = 0.0;
  input.lateral_acceleration_mps2 = 0.0;
  input.wheel_innovation_mps = std::numeric_limits<double>::quiet_NaN();
  input.has_wheel_innovation = true;
  EXPECT_FALSE(model.evaluate(input).valid);
}

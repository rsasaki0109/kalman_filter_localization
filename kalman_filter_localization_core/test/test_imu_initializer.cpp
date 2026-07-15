// Copyright (c) 2026, Ryohei Sasaki
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
//  * Neither the name of the copyright holder nor the names of its
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

#include <cmath>

#include <kalman_filter_localization/core/ekf_estimator.hpp>
#include <kalman_filter_localization/core/imu_initializer.hpp>

namespace
{
using kalman_filter_localization::core::EKFEstimator;
using kalman_filter_localization::core::ImuStationaryInitializer;
using kalman_filter_localization::core::YawInitializer;
}

TEST(ImuStationaryInitializer, EstimatesGyroBiasAndRollPitchFromStationaryWindow)
{
  ImuStationaryInitializer::Config config;
  config.window_duration_sec = 1.0;
  config.minimum_samples = 90U;
  ImuStationaryInitializer initializer(config);
  const Eigen::Vector3d gyro_bias(0.003, -0.002, 0.001);
  const Eigen::Quaterniond world_from_body =
    Eigen::AngleAxisd(0.12, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(-0.08, Eigen::Vector3d::UnitY());
  const Eigen::Vector3d specific_force =
    world_from_body.conjugate() * Eigen::Vector3d(0.0, 0.0, config.gravity_mps2);
  for (int index = 0; index <= 100; ++index) {
    const double perturbation = 1.0e-4 * std::sin(0.3 * index);
    const auto status = initializer.addSample(
      index * 0.01,
      gyro_bias + Eigen::Vector3d(perturbation, -perturbation, perturbation),
      specific_force + Eigen::Vector3d(perturbation, perturbation, -perturbation));
    if (index < 95) {
      EXPECT_NE(status, ImuStationaryInitializer::Status::kInvalidSample);
    }
  }
  const auto & result = initializer.result();
  ASSERT_EQ(result.status, ImuStationaryInitializer::Status::kInitialized);
  EXPECT_LT((result.gyro_bias - gyro_bias).norm(), 1.0e-4);
  const Eigen::Vector3d aligned = result.roll_pitch_orientation * specific_force.normalized();
  EXPECT_LT((aligned - Eigen::Vector3d::UnitZ()).norm(), 1.0e-6);
  EXPECT_GT(result.confidence, 0.95);
}

TEST(ImuStationaryInitializer, ReportsMovingAndInvalidTimestampReasons)
{
  ImuStationaryInitializer::Config config;
  config.window_duration_sec = 0.2;
  config.minimum_samples = 10U;
  ImuStationaryInitializer initializer(config);
  for (int index = 0; index <= 20; ++index) {
    ASSERT_NE(
      initializer.addSample(
        index * 0.01, Eigen::Vector3d(0.0, 0.0, index % 2 == 0 ? 0.2 : -0.2),
        Eigen::Vector3d(0.0, 0.0, 9.80665)),
      ImuStationaryInitializer::Status::kInvalidSample);
  }
  EXPECT_EQ(initializer.result().status, ImuStationaryInitializer::Status::kMoving);
  EXPECT_EQ(
    initializer.addSample(0.1, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
    ImuStationaryInitializer::Status::kReverseTime);
}

TEST(YawInitializer, UsesExplicitDualAntennaDopplerExternalPriority)
{
  YawInitializer initializer;
  EXPECT_TRUE(initializer.offer(YawInitializer::Source::kExternalPose, 0.3, 0.1, 1.0));
  EXPECT_TRUE(initializer.offer(YawInitializer::Source::kDopplerOrCourse, 0.4, 0.05, 1.1));
  EXPECT_FALSE(initializer.offer(YawInitializer::Source::kExternalPose, 0.5, 0.01, 1.2));
  EXPECT_TRUE(initializer.offer(YawInitializer::Source::kDualAntenna, 3.5, 0.001, 1.3));
  EXPECT_EQ(initializer.result().source, YawInitializer::Source::kDualAntenna);
  EXPECT_NEAR(initializer.result().yaw_rad, 3.5 - 2.0 * M_PI, 1.0e-12);
}

TEST(EkfInitialCovariance, SetsAllErrorStateBlocksFromSensorAccuracy)
{
  EKFEstimator estimator;
  ASSERT_TRUE(estimator.setInitialErrorStateCovariance(
      Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Vector3d(4.0, 5.0, 6.0),
      Eigen::Vector3d(0.1, 0.2, 0.3), Eigen::Vector3d(0.01, 0.02, 0.03),
      Eigen::Vector3d(0.04, 0.05, 0.06)));
  const Eigen::MatrixXd covariance = estimator.getCovariance();
  EXPECT_TRUE(covariance.isDiagonal(0.0));
  EXPECT_DOUBLE_EQ(covariance(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(covariance(5, 5), 6.0);
  EXPECT_DOUBLE_EQ(covariance(8, 8), 0.3);
  EXPECT_DOUBLE_EQ(covariance(11, 11), 0.03);
  EXPECT_DOUBLE_EQ(covariance(14, 14), 0.06);
  EXPECT_FALSE(estimator.setInitialErrorStateCovariance(
      Eigen::Vector3d(-1.0, 1.0, 1.0), Eigen::Vector3d::Ones(),
      Eigen::Vector3d::Ones(), Eigen::Vector3d::Ones(), Eigen::Vector3d::Ones()));
}

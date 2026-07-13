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

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <limits>

#include <kalman_filter_localization/core/odometry.hpp>

namespace
{
Eigen::Matrix4d makeTransform(const double x, const double y, const double z, const double yaw)
{
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) =
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  transform.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);
  return transform;
}
}  // namespace

TEST(OdometryCore, ComposePoseWithRelativeOdomPureTranslation)
{
  const Eigen::Matrix4d previous_global = makeTransform(0.0, 0.0, 0.0, 0.0);
  const Eigen::Matrix4d previous_odom = makeTransform(5.0, 1.0, 0.0, 0.0);
  const Eigen::Matrix4d current_odom = makeTransform(8.0, -2.0, 0.0, 0.0);

  const Eigen::Matrix4d composed =
    kalman_filter_localization::core::composePoseWithRelativeOdom(
    previous_global, previous_odom, current_odom);

  EXPECT_NEAR(composed(0, 3), 3.0, 1e-9);
  EXPECT_NEAR(composed(1, 3), -3.0, 1e-9);
  EXPECT_NEAR(composed(2, 3), 0.0, 1e-9);
}

TEST(OdometryCore, ComposePoseWithRelativeOdomAppliesGlobalRotation)
{
  const double pi = std::acos(-1.0);
  const Eigen::Matrix4d previous_global = makeTransform(10.0, 1.0, 0.0, pi / 2.0);
  const Eigen::Matrix4d previous_odom = makeTransform(5.0, 0.0, 0.0, 0.0);
  const Eigen::Matrix4d current_odom = makeTransform(8.0, 0.0, 0.0, 0.0);

  const Eigen::Matrix4d composed =
    kalman_filter_localization::core::composePoseWithRelativeOdom(
    previous_global, previous_odom, current_odom);

  EXPECT_NEAR(composed(0, 3), 10.0, 1e-9);
  EXPECT_NEAR(composed(1, 3), 4.0, 1e-9);
  EXPECT_NEAR(composed(2, 3), 0.0, 1e-9);
}

TEST(OdometryCore, RemoveLeverArmUsesBodyOrientation)
{
  const double pi = std::acos(-1.0);
  const Eigen::Quaterniond world_from_body(
    Eigen::AngleAxisd(pi / 2.0, Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d body_position(10.0, 20.0, 2.0);
  const Eigen::Vector3d lever_arm_body(2.0, 0.0, 1.0);
  const Eigen::Vector3d antenna_position =
    body_position + world_from_body * lever_arm_body;

  const Eigen::Vector3d result =
    kalman_filter_localization::core::removeLeverArmFromPosition(
    antenna_position, world_from_body, lever_arm_body);

  EXPECT_TRUE(result.isApprox(body_position, 1.0e-12));
}

TEST(OdometryCore, StationaryImuRequiresAllSignalsToBeQuiet)
{
  using kalman_filter_localization::core::isStationaryImu;
  const Eigen::Vector3d quiet_gyro(0.001, -0.002, 0.003);
  const Eigen::Vector3d gravity(0.01, -0.02, 9.80);
  EXPECT_TRUE(isStationaryImu(quiet_gyro, gravity, 0.05, 9.80665, 0.02, 0.2, 0.3));
  EXPECT_FALSE(isStationaryImu(
    Eigen::Vector3d(0.0, 0.0, 0.1), gravity, 0.05, 9.80665, 0.02, 0.2, 0.3));
  EXPECT_FALSE(isStationaryImu(
    quiet_gyro, Eigen::Vector3d(3.0, 0.0, 9.8), 0.05, 9.80665, 0.02, 0.2, 0.3));
  EXPECT_FALSE(isStationaryImu(quiet_gyro, gravity, 2.0, 9.80665, 0.02, 0.2, 0.3));
}

TEST(OdometryCore, ExtrapolatePositionUsesConstantVelocity)
{
  const Eigen::Vector3d result =
    kalman_filter_localization::core::extrapolatePositionConstantVelocity(
    Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Vector3d(4.0, -2.0, 0.5), 0.25);
  EXPECT_TRUE(result.isApprox(Eigen::Vector3d(2.0, 1.5, 3.125), 1.0e-12));
}

TEST(OdometryCore, SanitizeMeasurementVarianceFallsBackAndClamps)
{
  const Eigen::Vector3d result =
    kalman_filter_localization::core::sanitizeMeasurementVariance(
    Eigen::Vector3d(1.0e-6, std::numeric_limits<double>::quiet_NaN(), 100.0),
    Eigen::Vector3d(0.1, 0.2, 0.3),
    Eigen::Vector3d(0.01, 0.01, 0.02),
    Eigen::Vector3d(10.0, 10.0, 5.0));
  EXPECT_TRUE(result.isApprox(Eigen::Vector3d(0.01, 0.2, 5.0), 1.0e-12));
}

TEST(OdometryCore, WheelSpeedScaleFactorUsesFilteredMedian)
{
  using kalman_filter_localization::core::medianWheelSpeedScaleFactor;
  const std::vector<double> samples{1.02, 50.0, 0.98, 1.00, 0.99};
  EXPECT_NEAR(medianWheelSpeedScaleFactor(samples, 1.0, 0.8, 1.2), 0.995, 1.0e-12);
  EXPECT_DOUBLE_EQ(medianWheelSpeedScaleFactor({50.0}, 1.0, 0.8, 1.2), 1.0);
  EXPECT_DOUBLE_EQ(medianWheelSpeedScaleFactor({}, 0.97, 0.8, 1.2), 0.97);
}

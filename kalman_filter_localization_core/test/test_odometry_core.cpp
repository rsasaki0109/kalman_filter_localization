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

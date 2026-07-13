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
#ifndef KALMAN_FILTER_LOCALIZATION__CORE__ODOMETRY_HPP_
#define KALMAN_FILTER_LOCALIZATION__CORE__ODOMETRY_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace kalman_filter_localization
{
namespace core
{
// Compose global pose from previous/current odom and previous global pose.
inline Eigen::Matrix4d composePoseWithRelativeOdom(
  const Eigen::Matrix4d & previous_global_pose,
  const Eigen::Matrix4d & previous_odom_pose,
  const Eigen::Matrix4d & current_odom_pose)
{
  return previous_global_pose * previous_odom_pose.inverse() * current_odom_pose;
}

// Convert a GNSS antenna position into the body/IMU origin position.
inline Eigen::Vector3d removeLeverArmFromPosition(
  const Eigen::Vector3d & antenna_position_world,
  const Eigen::Quaterniond & world_from_body,
  const Eigen::Vector3d & antenna_lever_arm_body)
{
  return antenna_position_world -
         world_from_body.normalized() * antenna_lever_arm_body;
}

inline bool isStationaryImu(
  const Eigen::Vector3d & angular_velocity,
  const Eigen::Vector3d & linear_acceleration,
  const double estimated_speed,
  const double gravity_mps2,
  const double max_angular_velocity,
  const double max_acceleration_error,
  const double max_speed)
{
  return angular_velocity.allFinite() && linear_acceleration.allFinite() &&
         std::isfinite(estimated_speed) &&
         angular_velocity.norm() <= max_angular_velocity &&
         std::fabs(linear_acceleration.norm() - gravity_mps2) <= max_acceleration_error &&
         estimated_speed <= max_speed;
}

inline Eigen::Vector3d extrapolatePositionConstantVelocity(
  const Eigen::Vector3d & position,
  const Eigen::Vector3d & velocity,
  const double duration_sec)
{
  return position + velocity * duration_sec;
}

inline Eigen::Vector3d sanitizeMeasurementVariance(
  const Eigen::Vector3d & candidate,
  const Eigen::Vector3d & fallback,
  const Eigen::Vector3d & minimum,
  const Eigen::Vector3d & maximum)
{
  Eigen::Vector3d result;
  for (int index = 0; index < 3; ++index) {
    result(index) =
      std::isfinite(candidate(index)) && candidate(index) > 0.0 ?
      candidate(index) : fallback(index);
    result(index) = std::max(result(index), minimum(index));
    if (maximum(index) > 0.0) {
      result(index) = std::min(result(index), maximum(index));
    }
  }
  return result;
}
}  // namespace core
}  // namespace kalman_filter_localization

#endif  // KALMAN_FILTER_LOCALIZATION__CORE__ODOMETRY_HPP_

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

#ifndef KALMAN_FILTER_LOCALIZATION__CORE__IMU_INITIALIZER_HPP_
#define KALMAN_FILTER_LOCALIZATION__CORE__IMU_INITIALIZER_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>

namespace kalman_filter_localization
{
namespace core
{

class ImuStationaryInitializer
{
public:
  enum class Status : std::uint8_t
  {
    kCollecting = 0,
    kInitialized,
    kMoving,
    kInvalidSample,
    kReverseTime,
  };

  struct Config
  {
    double window_duration_sec{1.0};
    std::size_t minimum_samples{50U};
    double gravity_mps2{9.80665};
    double max_gyro_std_radps{0.01};
    double max_accel_std_mps2{0.1};
    double max_accel_norm_error_mps2{0.3};
  };

  struct Result
  {
    Status status{Status::kCollecting};
    Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond roll_pitch_orientation{Eigen::Quaterniond::Identity()};
    double confidence{0.0};
    double gyro_std_radps{std::numeric_limits<double>::quiet_NaN()};
    double accel_std_mps2{std::numeric_limits<double>::quiet_NaN()};
    double accel_norm_error_mps2{std::numeric_limits<double>::quiet_NaN()};
    std::size_t sample_count{0U};
  };

  ImuStationaryInitializer() = default;

  explicit ImuStationaryInitializer(const Config & config)
  : config_(config)
  {
  }

  bool setConfig(const Config & config)
  {
    if (!(config.window_duration_sec > 0.0) || config.minimum_samples < 2U ||
      !(config.gravity_mps2 > 0.0) || !(config.max_gyro_std_radps > 0.0) ||
      !(config.max_accel_std_mps2 > 0.0) || !(config.max_accel_norm_error_mps2 > 0.0) ||
      !std::isfinite(config.window_duration_sec) || !std::isfinite(config.gravity_mps2) ||
      !std::isfinite(config.max_gyro_std_radps) ||
      !std::isfinite(config.max_accel_std_mps2) ||
      !std::isfinite(config.max_accel_norm_error_mps2))
    {
      return false;
    }
    config_ = config;
    reset();
    return true;
  }

  void reset()
  {
    samples_.clear();
    result_ = Result{};
  }

  Status addSample(
    const double time, const Eigen::Vector3d & gyro, const Eigen::Vector3d & acceleration)
  {
    if (!std::isfinite(time) || !gyro.allFinite() || !acceleration.allFinite()) {
      result_.status = Status::kInvalidSample;
      return result_.status;
    }
    if (!samples_.empty() && !(time > samples_.back().time)) {
      result_.status = Status::kReverseTime;
      return result_.status;
    }
    samples_.push_back({time, gyro, acceleration});
    while (samples_.size() > 1U &&
      time - samples_.front().time > config_.window_duration_sec)
    {
      samples_.pop_front();
    }
    return evaluate();
  }

  const Result & result() const {return result_;}

private:
  struct Sample
  {
    double time;
    Eigen::Vector3d gyro;
    Eigen::Vector3d acceleration;
  };

  static double confidenceTerm(const double value, const double limit)
  {
    return std::max(0.0, std::min(1.0, 1.0 - value / limit));
  }

  Status evaluate()
  {
    result_.sample_count = samples_.size();
    if (samples_.size() < config_.minimum_samples ||
      samples_.back().time - samples_.front().time < config_.window_duration_sec * 0.95)
    {
      result_.status = Status::kCollecting;
      return result_.status;
    }
    Eigen::Vector3d gyro_mean = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_mean = Eigen::Vector3d::Zero();
    for (const auto & sample : samples_) {
      gyro_mean += sample.gyro;
      accel_mean += sample.acceleration;
    }
    gyro_mean /= static_cast<double>(samples_.size());
    accel_mean /= static_cast<double>(samples_.size());
    double gyro_squared_error = 0.0;
    double accel_squared_error = 0.0;
    for (const auto & sample : samples_) {
      gyro_squared_error += (sample.gyro - gyro_mean).squaredNorm();
      accel_squared_error += (sample.acceleration - accel_mean).squaredNorm();
    }
    const double denominator = static_cast<double>(samples_.size() - 1U);
    result_.gyro_std_radps = std::sqrt(gyro_squared_error / denominator);
    result_.accel_std_mps2 = std::sqrt(accel_squared_error / denominator);
    result_.accel_norm_error_mps2 = std::fabs(accel_mean.norm() - config_.gravity_mps2);
    result_.confidence = std::min(
      confidenceTerm(result_.gyro_std_radps, config_.max_gyro_std_radps),
      std::min(
        confidenceTerm(result_.accel_std_mps2, config_.max_accel_std_mps2),
        confidenceTerm(
          result_.accel_norm_error_mps2, config_.max_accel_norm_error_mps2)));
    if (result_.gyro_std_radps > config_.max_gyro_std_radps ||
      result_.accel_std_mps2 > config_.max_accel_std_mps2 ||
      result_.accel_norm_error_mps2 > config_.max_accel_norm_error_mps2 ||
      !(accel_mean.norm() > 0.0))
    {
      result_.status = Status::kMoving;
      return result_.status;
    }
    result_.gyro_bias = gyro_mean;
    result_.roll_pitch_orientation = Eigen::Quaterniond::FromTwoVectors(
      accel_mean.normalized(), Eigen::Vector3d::UnitZ()).normalized();
    result_.status = Status::kInitialized;
    return result_.status;
  }

  Config config_{};
  std::deque<Sample> samples_;
  Result result_{};
};

class YawInitializer
{
public:
  enum class Source : std::uint8_t
  {
    kNone = 0,
    kExternalPose = 1,
    kDopplerOrCourse = 2,
    kDualAntenna = 3,
  };

  struct Result
  {
    Source source{Source::kNone};
    double yaw_rad{0.0};
    double variance_rad2{std::numeric_limits<double>::infinity()};
    double stamp{std::numeric_limits<double>::quiet_NaN()};
  };

  bool offer(
    const Source source, const double yaw_rad, const double variance_rad2, const double stamp)
  {
    if (source == Source::kNone || !std::isfinite(yaw_rad) || !(variance_rad2 > 0.0) ||
      !std::isfinite(variance_rad2) || !std::isfinite(stamp))
    {
      return false;
    }
    if (static_cast<std::uint8_t>(source) < static_cast<std::uint8_t>(result_.source)) {
      return false;
    }
    result_ = {source, std::atan2(std::sin(yaw_rad), std::cos(yaw_rad)), variance_rad2, stamp};
    return true;
  }

  const Result & result() const {return result_;}
  void reset() {result_ = Result{};}

private:
  Result result_{};
};

}  // namespace core
}  // namespace kalman_filter_localization

#endif  // KALMAN_FILTER_LOCALIZATION__CORE__IMU_INITIALIZER_HPP_

// Copyright (c) 2020, Ryohei Sasaki
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

#ifndef KALMAN_FILTER_LOCALIZATION__CORE__VEHICLE_OBSERVABILITY_HPP_
#define KALMAN_FILTER_LOCALIZATION__CORE__VEHICLE_OBSERVABILITY_HPP_

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace kalman_filter_localization
{
namespace core
{

class StationaryDetector
{
public:
  enum class State : std::uint8_t {kMoving = 0, kCandidate, kStationary, kInvalid};

  struct Config
  {
    double max_angular_velocity_radps{0.02};
    double max_acceleration_norm_error_mps2{0.2};
    double max_speed_mps{0.3};
    double minimum_duration_sec{0.5};
  };

  StationaryDetector() = default;
  explicit StationaryDetector(const Config & config)
  : config_(config) {}

  State update(
    const double time_sec, const Eigen::Vector3d & angular_velocity,
    const Eigen::Vector3d & acceleration, const double speed_mps, const double gravity_mps2)
  {
    if (!std::isfinite(time_sec) || !angular_velocity.allFinite() ||
      !acceleration.allFinite() || !std::isfinite(speed_mps) ||
      !(gravity_mps2 > 0.0) || !validConfig())
    {
      state_ = State::kInvalid;
      has_candidate_start_ = false;
      return state_;
    }
    const bool stationary_sample =
      angular_velocity.norm() <= config_.max_angular_velocity_radps &&
      std::fabs(acceleration.norm() - gravity_mps2) <=
      config_.max_acceleration_norm_error_mps2 &&
      std::fabs(speed_mps) <= config_.max_speed_mps;
    if (!stationary_sample) {
      state_ = State::kMoving;
      has_candidate_start_ = false;
      return state_;
    }
    if (!has_candidate_start_ || time_sec < candidate_start_sec_) {
      candidate_start_sec_ = time_sec;
      has_candidate_start_ = true;
    }
    state_ = time_sec - candidate_start_sec_ >= config_.minimum_duration_sec ?
      State::kStationary : State::kCandidate;
    return state_;
  }

  void reset()
  {
    state_ = State::kMoving;
    has_candidate_start_ = false;
    candidate_start_sec_ = 0.0;
  }

  State state() const {return state_;}
  bool isStationary() const {return state_ == State::kStationary;}

private:
  bool validConfig() const
  {
    return config_.max_angular_velocity_radps >= 0.0 &&
           config_.max_acceleration_norm_error_mps2 >= 0.0 &&
           config_.max_speed_mps >= 0.0 && config_.minimum_duration_sec >= 0.0;
  }

  Config config_{};
  State state_{State::kMoving};
  bool has_candidate_start_{false};
  double candidate_start_sec_{0.0};
};

class SlipTurnDetector
{
public:
  enum class State : std::uint8_t {kTrusted = 0, kTurning, kSlip, kInvalid};

  struct Config
  {
    double yaw_rate_threshold_radps{0.5};
    double lateral_acceleration_threshold_mps2{1.5};
    double wheel_innovation_threshold_mps{1.0};
    double exit_ratio{0.7};
    double maximum_variance_scale{100.0};
    std::size_t recovery_samples{5U};
  };

  struct Result
  {
    State state{State::kInvalid};
    double variance_scale{1.0};
    double severity{std::numeric_limits<double>::quiet_NaN()};
  };

  SlipTurnDetector() = default;
  explicit SlipTurnDetector(const Config & config)
  : config_(config) {}

  Result update(
    const double yaw_rate_radps, const double lateral_acceleration_mps2,
    const double wheel_velocity_innovation_mps = 0.0)
  {
    if (!std::isfinite(yaw_rate_radps) || !std::isfinite(lateral_acceleration_mps2) ||
      !std::isfinite(wheel_velocity_innovation_mps) || !validConfig())
    {
      state_ = State::kInvalid;
      recovery_count_ = 0U;
      return Result{state_, config_.maximum_variance_scale,
        std::numeric_limits<double>::quiet_NaN()};
    }
    const double yaw_ratio = std::fabs(yaw_rate_radps) / config_.yaw_rate_threshold_radps;
    const double lateral_ratio =
      std::fabs(lateral_acceleration_mps2) / config_.lateral_acceleration_threshold_mps2;
    const double wheel_ratio =
      std::fabs(wheel_velocity_innovation_mps) / config_.wheel_innovation_threshold_mps;
    const double turn_severity = std::max(yaw_ratio, lateral_ratio);
    const double severity = std::max(turn_severity, wheel_ratio);
    const State requested = wheel_ratio >= 1.0 ? State::kSlip :
      (turn_severity >= 1.0 ? State::kTurning : State::kTrusted);
    if (requested != State::kTrusted) {
      state_ = requested;
      recovery_count_ = 0U;
    } else if (state_ == State::kTrusted || severity <= config_.exit_ratio) {
      ++recovery_count_;
      if (state_ == State::kTrusted || recovery_count_ >= config_.recovery_samples) {
        state_ = State::kTrusted;
        recovery_count_ = 0U;
      }
    } else {
      recovery_count_ = 0U;
    }
    const double scale = state_ == State::kTrusted ? 1.0 :
      std::min(config_.maximum_variance_scale, std::max(1.0, severity * severity));
    return Result{state_, scale, severity};
  }

  void reset() {state_ = State::kTrusted; recovery_count_ = 0U;}
  State state() const {return state_;}

private:
  bool validConfig() const
  {
    return config_.yaw_rate_threshold_radps > 0.0 &&
           config_.lateral_acceleration_threshold_mps2 > 0.0 &&
           config_.wheel_innovation_threshold_mps > 0.0 &&
           config_.exit_ratio >= 0.0 && config_.exit_ratio < 1.0 &&
           config_.maximum_variance_scale >= 1.0 && config_.recovery_samples > 0U;
  }

  Config config_{};
  State state_{State::kTrusted};
  std::size_t recovery_count_{0U};
};

struct BiasObservabilityDecision
{
  bool learn_gyro_bias{false};
  bool learn_accel_bias{false};
};

inline BiasObservabilityDecision evaluateBiasObservability(
  const bool stationary, const double angular_rate_radps,
  const double horizontal_specific_force_mps2, const double minimum_angular_excitation_radps,
  const double minimum_acceleration_excitation_mps2)
{
  if (!std::isfinite(angular_rate_radps) || !std::isfinite(horizontal_specific_force_mps2)) {
    return BiasObservabilityDecision{};
  }
  return BiasObservabilityDecision{
    stationary || std::fabs(angular_rate_radps) >= minimum_angular_excitation_radps,
    stationary || std::fabs(horizontal_specific_force_mps2) >=
    minimum_acceleration_excitation_mps2};
}

class GnssReacquisitionGate
{
public:
  enum class State : std::uint8_t {kTracking = 0, kCandidate, kBlending, kInvalid};
  enum class Reason : std::uint8_t
  {
    kNone = 0, kWaitingForConsistency, kPositionInnovation, kVelocityInnovation, kInvalidInput
  };

  struct Config
  {
    double outage_duration_sec{1.0};
    std::size_t required_consistent_updates{3U};
    std::size_t blending_updates{5U};
    double initial_variance_scale{20.0};
    double maximum_position_innovation_m{30.0};
    double maximum_velocity_innovation_mps{10.0};
  };

  struct Result
  {
    State state{State::kInvalid};
    Reason reason{Reason::kInvalidInput};
    bool accepted{false};
    double variance_scale{1.0};
    std::size_t consistent_updates{0U};
  };

  GnssReacquisitionGate() = default;
  explicit GnssReacquisitionGate(const Config & config)
  : config_(config) {}

  Result update(
    const double time_sec, const double position_innovation_m,
    const double velocity_innovation_mps = 0.0)
  {
    if (!std::isfinite(time_sec) || !std::isfinite(position_innovation_m) ||
      !std::isfinite(velocity_innovation_mps) || !validConfig())
    {
      return Result{State::kInvalid, Reason::kInvalidInput, false, 1.0, 0U};
    }
    if (has_last_time_ && time_sec - last_time_sec_ > config_.outage_duration_sec) {
      state_ = State::kCandidate;
      consistent_updates_ = 0U;
      blending_updates_remaining_ = config_.blending_updates;
    }
    last_time_sec_ = time_sec;
    has_last_time_ = true;
    if (state_ == State::kCandidate) {
      if (position_innovation_m > config_.maximum_position_innovation_m) {
        consistent_updates_ = 0U;
        return Result{state_, Reason::kPositionInnovation, false,
          config_.initial_variance_scale, consistent_updates_};
      }
      if (velocity_innovation_mps > config_.maximum_velocity_innovation_mps) {
        consistent_updates_ = 0U;
        return Result{state_, Reason::kVelocityInnovation, false,
          config_.initial_variance_scale, consistent_updates_};
      }
      ++consistent_updates_;
      if (consistent_updates_ < config_.required_consistent_updates) {
        return Result{state_, Reason::kWaitingForConsistency, false,
          config_.initial_variance_scale, consistent_updates_};
      }
      state_ = config_.blending_updates > 0U ? State::kBlending : State::kTracking;
    }
    if (state_ == State::kBlending) {
      const double fraction = config_.blending_updates > 0U ?
        static_cast<double>(blending_updates_remaining_) /
        static_cast<double>(config_.blending_updates) : 0.0;
      const double scale = 1.0 + (config_.initial_variance_scale - 1.0) * fraction;
      if (blending_updates_remaining_ > 0U) {
        --blending_updates_remaining_;
      }
      if (blending_updates_remaining_ == 0U) {
        state_ = State::kTracking;
      }
      return Result{State::kBlending, Reason::kNone, true, scale, consistent_updates_};
    }
    return Result{state_, Reason::kNone, true, 1.0, consistent_updates_};
  }

  void reset()
  {
    state_ = State::kTracking;
    has_last_time_ = false;
    last_time_sec_ = 0.0;
    consistent_updates_ = 0U;
    blending_updates_remaining_ = 0U;
  }

private:
  bool validConfig() const
  {
    return config_.outage_duration_sec > 0.0 &&
           config_.required_consistent_updates > 0U &&
           config_.initial_variance_scale >= 1.0 &&
           config_.maximum_position_innovation_m > 0.0 &&
           config_.maximum_velocity_innovation_mps > 0.0;
  }

  Config config_{};
  State state_{State::kTracking};
  bool has_last_time_{false};
  double last_time_sec_{0.0};
  std::size_t consistent_updates_{0U};
  std::size_t blending_updates_remaining_{0U};
};

}  // namespace core
}  // namespace kalman_filter_localization

#endif  // KALMAN_FILTER_LOCALIZATION__CORE__VEHICLE_OBSERVABILITY_HPP_

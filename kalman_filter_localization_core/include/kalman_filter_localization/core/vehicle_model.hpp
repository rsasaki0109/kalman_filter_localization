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
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef KALMAN_FILTER_LOCALIZATION__CORE__VEHICLE_MODEL_HPP_
#define KALMAN_FILTER_LOCALIZATION__CORE__VEHICLE_MODEL_HPP_

#include <Eigen/Core>

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include <kalman_filter_localization/core/vehicle_observability.hpp>

namespace kalman_filter_localization
{
namespace core
{

// Configuration shared by built-in and external vehicle-model plugins. Values
// are physical quantities in SI units; a plugin may ignore fields it does not
// use, but it must reject a configuration it cannot interpret safely.
struct VehicleModelConfig
{
  double minimum_forward_speed_mps{0.5};
  double yaw_rate_threshold_radps{0.5};
  double lateral_acceleration_threshold_mps2{1.5};
  double wheel_innovation_threshold_mps{1.0};
  double maximum_variance_scale{100.0};
  std::uint32_t recovery_samples{5U};
};

struct VehicleModelInput
{
  double time_sec{std::numeric_limits<double>::quiet_NaN()};
  Eigen::Vector3d body_velocity{Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN())};
  double yaw_rate_radps{std::numeric_limits<double>::quiet_NaN()};
  double lateral_acceleration_mps2{std::numeric_limits<double>::quiet_NaN()};
  double wheel_innovation_mps{0.0};
  bool has_wheel_innovation{false};
};

struct VehicleModelOutput
{
  bool valid{false};
  bool apply_nonholonomic_constraint{false};
  bool constrain_vertical_velocity{true};
  std::uint8_t slip_state{static_cast<std::uint8_t>(SlipTurnDetector::State::kInvalid)};
  std::string slip_state_text{"invalid"};
  double nhc_variance_scale{1.0};
  double slip_severity{std::numeric_limits<double>::quiet_NaN()};
};

// Stable ROS-free ABI contract for vehicle-specific motion constraints. A ROS
// layer may load implementations through pluginlib, while simulation and
// downstream applications can use the same interface without ROS headers.
class VehicleModel
{
public:
  virtual ~VehicleModel() = default;

  virtual std::string name() const = 0;
  virtual bool configure(const VehicleModelConfig & config) = 0;
  virtual VehicleModelOutput evaluate(const VehicleModelInput & input) = 0;
  virtual void reset() = 0;
};

class GroundVehicleModel : public VehicleModel
{
public:
  std::string name() const override {return "ground_vehicle";}

  bool configure(const VehicleModelConfig & config) override
  {
    if (!std::isfinite(config.minimum_forward_speed_mps) ||
      config.minimum_forward_speed_mps < 0.0 ||
      !std::isfinite(config.yaw_rate_threshold_radps) ||
      config.yaw_rate_threshold_radps <= 0.0 ||
      !std::isfinite(config.lateral_acceleration_threshold_mps2) ||
      config.lateral_acceleration_threshold_mps2 <= 0.0 ||
      !std::isfinite(config.wheel_innovation_threshold_mps) ||
      config.wheel_innovation_threshold_mps <= 0.0 ||
      !std::isfinite(config.maximum_variance_scale) ||
      config.maximum_variance_scale < 1.0 || config.recovery_samples == 0U)
    {
      return false;
    }
    minimum_forward_speed_mps_ = config.minimum_forward_speed_mps;
    SlipTurnDetector::Config slip_config;
    slip_config.yaw_rate_threshold_radps = config.yaw_rate_threshold_radps;
    slip_config.lateral_acceleration_threshold_mps2 =
      config.lateral_acceleration_threshold_mps2;
    slip_config.wheel_innovation_threshold_mps = config.wheel_innovation_threshold_mps;
    slip_config.maximum_variance_scale = config.maximum_variance_scale;
    slip_config.recovery_samples = config.recovery_samples;
    slip_detector_ = SlipTurnDetector(slip_config);
    configured_ = true;
    return true;
  }

  VehicleModelOutput evaluate(const VehicleModelInput & input) override
  {
    VehicleModelOutput output;
    if (!configured_ || !std::isfinite(input.time_sec) || !input.body_velocity.allFinite() ||
      !std::isfinite(input.yaw_rate_radps) ||
      !std::isfinite(input.lateral_acceleration_mps2) ||
      (input.has_wheel_innovation && !std::isfinite(input.wheel_innovation_mps)))
    {
      return output;
    }
    const auto result = slip_detector_.update(
      input.yaw_rate_radps, input.lateral_acceleration_mps2,
      input.has_wheel_innovation ? input.wheel_innovation_mps : 0.0);
    output.valid = true;
    output.apply_nonholonomic_constraint =
      std::fabs(input.body_velocity.x()) >= minimum_forward_speed_mps_;
    output.constrain_vertical_velocity = true;
    output.slip_state = static_cast<std::uint8_t>(result.state);
    output.slip_state_text = slipStateText(result.state);
    output.nhc_variance_scale = result.variance_scale;
    output.slip_severity = result.severity;
    return output;
  }

  void reset() override {slip_detector_.reset();}

protected:
  static const char * slipStateText(const SlipTurnDetector::State state)
  {
    switch (state) {
      case SlipTurnDetector::State::kTrusted:
        return "trusted";
      case SlipTurnDetector::State::kTurning:
        return "turning";
      case SlipTurnDetector::State::kSlip:
        return "slip";
      case SlipTurnDetector::State::kInvalid:
        return "invalid";
    }
    return "unknown";
  }

  double minimum_forward_speed_mps_{0.5};
  SlipTurnDetector slip_detector_;
  bool configured_{false};
};

// A planar wheeled-robot model keeps the lateral no-slip constraint but does
// not force the vertical body velocity to zero. This is useful for platforms
// whose suspension/terrain motion makes vertical NHC overconfident.
class PlanarVehicleModel : public GroundVehicleModel
{
public:
  std::string name() const override {return "planar_vehicle";}

  VehicleModelOutput evaluate(const VehicleModelInput & input) override
  {
    auto output = GroundVehicleModel::evaluate(input);
    output.constrain_vertical_velocity = false;
    return output;
  }
};

}  // namespace core
}  // namespace kalman_filter_localization

#endif  // KALMAN_FILTER_LOCALIZATION__CORE__VEHICLE_MODEL_HPP_

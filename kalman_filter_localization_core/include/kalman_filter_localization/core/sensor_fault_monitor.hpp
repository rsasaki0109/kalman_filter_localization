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
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
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

#ifndef KALMAN_FILTER_LOCALIZATION__CORE__SENSOR_FAULT_MONITOR_HPP_
#define KALMAN_FILTER_LOCALIZATION__CORE__SENSOR_FAULT_MONITOR_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace kalman_filter_localization
{
namespace core
{

class SensorFaultMonitor
{
public:
  enum class Sensor : std::uint8_t
  {
    kImu = 0,
    kGnss = 1,
    kWheel = 2,
    kOdom = 3,
    kCount = 4,
  };

  struct Config
  {
    std::size_t trip_count{5};
    double hold_sec{5.0};
  };

  struct State
  {
    bool isolated{false};
    std::uint32_t consecutive_failures{0};
    std::uint64_t fault_events{0};
    double last_fault_time_sec{std::numeric_limits<double>::quiet_NaN()};
  };

  SensorFaultMonitor()
  {
    (void)setConfig(Config{});
  }

  bool setConfig(const Config & config)
  {
    if (config.trip_count < 1 || !(config.hold_sec >= 0.0) ||
      !std::isfinite(config.hold_sec))
    {
      return false;
    }
    config_ = config;
    return true;
  }

  const Config & config() const
  {
    return config_;
  }

  const State & state(const Sensor sensor) const
  {
    return states_[index(sensor)];
  }

  bool allows(const Sensor sensor, const double time_sec) const
  {
    const State & current = state(sensor);
    if (!current.isolated) {
      return true;
    }
    if (!std::isfinite(time_sec) || !std::isfinite(current.last_fault_time_sec)) {
      return false;
    }
    return time_sec - current.last_fault_time_sec >= config_.hold_sec;
  }

  void observe(const Sensor sensor, const bool healthy, const double time_sec)
  {
    State & current = states_[index(sensor)];
    if (healthy) {
      if (current.isolated && allows(sensor, time_sec)) {
        current.isolated = false;
        current.consecutive_failures = 0;
      } else if (!current.isolated) {
        current.consecutive_failures = 0;
      }
      return;
    }

    if (current.isolated) {
      if (std::isfinite(time_sec)) {
        current.last_fault_time_sec = time_sec;
      }
      return;
    }
    ++current.consecutive_failures;
    if (current.consecutive_failures >= config_.trip_count) {
      current.isolated = true;
      ++current.fault_events;
      current.last_fault_time_sec = time_sec;
    }
  }

  void reset()
  {
    for (State & current : states_) {
      current = State{};
    }
  }

private:
  static constexpr std::size_t kSensorCount = 4U;

  static constexpr std::size_t index(const Sensor sensor)
  {
    return static_cast<std::size_t>(sensor);
  }

  Config config_{};
  State states_[kSensorCount]{};
};

}  // namespace core
}  // namespace kalman_filter_localization

#endif  // KALMAN_FILTER_LOCALIZATION__CORE__SENSOR_FAULT_MONITOR_HPP_

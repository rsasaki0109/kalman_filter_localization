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

#include <gtest/gtest.h>

#include <kalman_filter_localization/core/sensor_fault_monitor.hpp>

namespace
{
using Sensor = kalman_filter_localization::core::SensorFaultMonitor::Sensor;
}

TEST(SensorFaultMonitor, TripsHoldsAndRecoversAfterHealthyProbe)
{
  kalman_filter_localization::core::SensorFaultMonitor monitor;
  kalman_filter_localization::core::SensorFaultMonitor::Config config;
  config.trip_count = 2;
  config.hold_sec = 5.0;
  ASSERT_TRUE(monitor.setConfig(config));

  monitor.observe(Sensor::kGnss, false, 10.0);
  EXPECT_TRUE(monitor.allows(Sensor::kGnss, 10.0));
  monitor.observe(Sensor::kGnss, false, 11.0);

  EXPECT_TRUE(monitor.state(Sensor::kGnss).isolated);
  EXPECT_EQ(monitor.state(Sensor::kGnss).fault_events, 1U);
  EXPECT_FALSE(monitor.allows(Sensor::kGnss, 15.9));
  EXPECT_TRUE(monitor.allows(Sensor::kGnss, 16.0));

  monitor.observe(Sensor::kGnss, true, 16.0);
  EXPECT_FALSE(monitor.state(Sensor::kGnss).isolated);
  EXPECT_EQ(monitor.state(Sensor::kGnss).consecutive_failures, 0U);
}

TEST(SensorFaultMonitor, SensorsAreIndependentAndResettable)
{
  kalman_filter_localization::core::SensorFaultMonitor monitor;
  kalman_filter_localization::core::SensorFaultMonitor::Config config;
  config.trip_count = 1;
  config.hold_sec = 0.0;
  ASSERT_TRUE(monitor.setConfig(config));

  monitor.observe(Sensor::kWheel, false, 1.0);
  EXPECT_TRUE(monitor.state(Sensor::kWheel).isolated);
  EXPECT_FALSE(monitor.state(Sensor::kOdom).isolated);
  monitor.reset();
  EXPECT_FALSE(monitor.state(Sensor::kWheel).isolated);
  EXPECT_EQ(monitor.state(Sensor::kWheel).fault_events, 0U);
}

TEST(SensorFaultMonitor, RejectsInvalidConfiguration)
{
  kalman_filter_localization::core::SensorFaultMonitor monitor;
  kalman_filter_localization::core::SensorFaultMonitor::Config config;
  config.trip_count = 0;
  EXPECT_FALSE(monitor.setConfig(config));
  config.trip_count = 3;
  config.hold_sec = -1.0;
  EXPECT_FALSE(monitor.setConfig(config));
}

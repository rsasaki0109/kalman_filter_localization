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

#include <cstdint>

#include <algorithm>
#include <cmath>
#include <vector>

#include <kalman_filter_localization/core/eskf_replay.hpp>

namespace
{
using kalman_filter_localization::core::EKFEstimator;
using kalman_filter_localization::core::EskfReplay;

void configure(EKFEstimator & estimator)
{
  estimator.setPropagationModel(EKFEstimator::PropagationModel::kExact);
  estimator.setVarImuAcc(0.02);
  estimator.setVarImuGyro(0.001);
  estimator.setVarImuGyroBias(1.0e-7);
  estimator.setVarImuAccBias(1.0e-6);
  estimator.setTauGyroBias(300.0);
  estimator.setTauAccBias(300.0);
  estimator.setMaxPredictionDtSec(2.0);
}

EskfReplay::MeasurementFunction positionUpdate(
  const Eigen::Vector3d & position, const Eigen::Vector3d & variance)
{
  return [position, variance](EKFEstimator & estimator) {
           return estimator.observationUpdateWithStatus(position, variance);
         };
}
}  // namespace

TEST(EskfReplay, DelayedReverseOrderMatchesNoDelayStateAndCovariance)
{
  EKFEstimator reference;
  EKFEstimator replayed;
  configure(reference);
  configure(replayed);
  EskfReplay replay(replayed, 10.0);

  const Eigen::Vector3d acceleration(0.4, 0.1, 9.80665);
  const Eigen::Vector3d gyro(0.0, 0.0, 0.08);
  ASSERT_TRUE(reference.primeImuMeasurement(gyro, acceleration));
  ASSERT_EQ(replay.addImu(0.0, gyro, acceleration), EskfReplay::Status::kInitialized);

  struct Measurement
  {
    double time;
    std::uint64_t id;
    Eigen::Vector3d position;
  };
  std::vector<Measurement> measurements;
  for (int index = 1; index <= 200; ++index) {
    const double time = index * 0.01;
    ASSERT_EQ(
      reference.predictionUpdateDt(0.01, gyro, acceleration),
      EKFEstimator::PredictionUpdateStatus::kUpdated);
    ASSERT_EQ(replay.addImu(time, gyro, acceleration), EskfReplay::Status::kApplied);
    if (index % 25 == 0) {
      const Eigen::Vector3d measurement(
        0.2 * time * time + 0.01 * index, 0.05 * time, 0.0);
      const Eigen::Vector3d variance(0.04, 0.04, 0.09);
      ASSERT_EQ(
        reference.observationUpdateWithStatus(measurement, variance),
        EKFEstimator::ObservationUpdateStatus::kUpdated);
      measurements.push_back({time, static_cast<std::uint64_t>(index), measurement});
    }
  }

  const Eigen::Vector3d variance(0.04, 0.04, 0.09);
  std::reverse(measurements.begin(), measurements.end());
  for (const auto & measurement : measurements) {
    ASSERT_EQ(
      replay.applyMeasurement(
        measurement.time, 3.0, measurement.id,
        positionUpdate(measurement.position, variance)),
      EskfReplay::Status::kApplied);
  }

  EXPECT_LT((reference.getX() - replayed.getX()).norm(), 1.0e-9);
  EXPECT_LT((reference.getCovariance() - replayed.getCovariance()).norm(), 1.0e-9);
  EXPECT_EQ(replay.counters().rewind_count, measurements.size());
  EXPECT_TRUE(replayed.checkNumericalInvariants());
}

TEST(EskfReplay, InterpolatesImuAtMeasurementTime)
{
  EKFEstimator reference;
  EKFEstimator replayed;
  configure(reference);
  configure(replayed);
  EskfReplay replay(replayed, 2.0);
  const Eigen::Vector3d acceleration0(0.0, 0.0, 9.80665);
  const Eigen::Vector3d acceleration1(2.0, 0.0, 9.80665);
  const Eigen::Vector3d gyro0 = Eigen::Vector3d::Zero();
  const Eigen::Vector3d gyro1(0.0, 0.0, 0.2);
  ASSERT_TRUE(reference.primeImuMeasurement(gyro0, acceleration0));
  ASSERT_EQ(replay.addImu(0.0, gyro0, acceleration0), EskfReplay::Status::kInitialized);

  const Eigen::Vector3d gyro_mid = 0.5 * (gyro0 + gyro1);
  const Eigen::Vector3d acceleration_mid = 0.5 * (acceleration0 + acceleration1);
  ASSERT_EQ(
    reference.predictionUpdateDt(0.5, gyro_mid, acceleration_mid),
    EKFEstimator::PredictionUpdateStatus::kUpdated);
  const Eigen::Vector3d measured_position(0.12, -0.03, 0.01);
  const Eigen::Vector3d variance(0.02, 0.02, 0.02);
  ASSERT_EQ(
    reference.observationUpdateWithStatus(measured_position, variance),
    EKFEstimator::ObservationUpdateStatus::kUpdated);
  ASSERT_EQ(
    reference.predictionUpdateDt(0.5, gyro1, acceleration1),
    EKFEstimator::PredictionUpdateStatus::kUpdated);

  ASSERT_EQ(replay.addImu(1.0, gyro1, acceleration1), EskfReplay::Status::kApplied);
  ASSERT_EQ(
    replay.applyMeasurement(
      0.5, 1.2, 42U, positionUpdate(measured_position, variance)),
    EskfReplay::Status::kApplied);
  EXPECT_LT((reference.getX() - replayed.getX()).norm(), 1.0e-10);
  EXPECT_LT((reference.getCovariance() - replayed.getCovariance()).norm(), 1.0e-10);
}

TEST(EskfReplay, EnforcesTimestampPoliciesAndBoundedHistory)
{
  EKFEstimator estimator;
  configure(estimator);
  EskfReplay replay(estimator, 0.25);
  const Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  const Eigen::Vector3d acceleration(0.0, 0.0, 9.80665);
  ASSERT_EQ(replay.addImu(0.0, gyro, acceleration), EskfReplay::Status::kInitialized);
  for (int index = 1; index <= 100; ++index) {
    ASSERT_EQ(
      replay.addImu(index * 0.01, gyro, acceleration), EskfReplay::Status::kApplied);
  }
  EXPECT_LE(replay.imuHistorySize(), 27U);

  const Eigen::Vector3d variance = Eigen::Vector3d::Ones();
  EXPECT_EQ(
    replay.applyMeasurement(
      0.1, 1.1, 1U, positionUpdate(Eigen::Vector3d::Zero(), variance)),
    EskfReplay::Status::kTooOld);
  EXPECT_EQ(
    replay.applyMeasurement(
      1.6, 1.6, 2U, positionUpdate(Eigen::Vector3d::Zero(), variance)),
    EskfReplay::Status::kFuture);
  ASSERT_EQ(
    replay.applyMeasurement(
      0.9, 1.1, 3U, positionUpdate(Eigen::Vector3d::Zero(), variance)),
    EskfReplay::Status::kApplied);
  EXPECT_EQ(
    replay.applyMeasurement(
      0.9, 1.2, 3U, positionUpdate(Eigen::Vector3d::Zero(), variance)),
    EskfReplay::Status::kDuplicate);
  EXPECT_EQ(replay.addImu(0.95, gyro, acceleration), EskfReplay::Status::kReverseImu);
  EXPECT_EQ(replay.counters().too_old, 1U);
  EXPECT_EQ(replay.counters().future, 1U);
  EXPECT_EQ(replay.counters().duplicate, 1U);
  EXPECT_EQ(replay.counters().reverse_imu, 1U);
  EXPECT_DOUBLE_EQ(replay.lastTimingTrace().sensor_time, 0.9);
  EXPECT_DOUBLE_EQ(replay.lastTimingTrace().arrival_time, 1.2);
  EXPECT_DOUBLE_EQ(replay.lastTimingTrace().filter_time_before, 1.0);
  EXPECT_DOUBLE_EQ(replay.lastTimingTrace().apply_time, 0.9);
  EXPECT_EQ(
    replay.applyMeasurement(
      1.02, 1.01, 4U, positionUpdate(Eigen::Vector3d::Zero(), variance)),
    EskfReplay::Status::kQueuedFuture);
  EXPECT_EQ(replay.counters().future_queued, 1U);
  EXPECT_EQ(replay.addImu(1.03, gyro, acceleration), EskfReplay::Status::kApplied);
  EXPECT_EQ(replay.counters().measurements_applied, 2U);
}

TEST(EskfReplay, QueuesMeasurementThatArrivesBeforeFirstImu)
{
  EKFEstimator estimator;
  configure(estimator);
  EskfReplay replay(estimator, 1.0);
  const Eigen::Vector3d measured_position(1.0, -0.5, 0.2);
  const Eigen::Vector3d variance = Eigen::Vector3d::Constant(0.1);
  EXPECT_EQ(
    replay.applyMeasurement(
      0.0, 0.01, 1U, positionUpdate(measured_position, variance)),
    EskfReplay::Status::kQueuedFuture);
  EXPECT_EQ(
    replay.addImu(
      0.01, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 9.80665)),
    EskfReplay::Status::kInitialized);
  EXPECT_EQ(replay.counters().measurements_applied, 1U);
  EXPECT_GT(estimator.getPosition().x(), 0.0);
}

TEST(EskfReplay, DelayJitterDropAndReorderingSweepMatchesNoDelay)
{
  const std::vector<double> delay_sweep{0.0, 0.1, 0.25, 0.5};
  for (const double delay : delay_sweep) {
    EKFEstimator reference;
    EKFEstimator delayed;
    configure(reference);
    configure(delayed);
    reference.setPropagationModel(EKFEstimator::PropagationModel::kFast);
    delayed.setPropagationModel(EKFEstimator::PropagationModel::kFast);
    EskfReplay replay(delayed, 2.0);
    const Eigen::Vector3d gyro(0.0, 0.0, 0.04);
    const Eigen::Vector3d acceleration(0.3, -0.05, 9.80665);
    const Eigen::Vector3d variance(0.03, 0.03, 0.06);
    ASSERT_TRUE(reference.primeImuMeasurement(gyro, acceleration));

    struct Arrival
    {
      double time;
      bool imu;
      double sensor_time;
      std::uint64_t id;
      Eigen::Vector3d position;
    };
    std::vector<Arrival> arrivals;
    for (int index = 0; index <= 170; ++index) {
      const double time = index * 0.01;
      arrivals.push_back({time, true, time, 0U, Eigen::Vector3d::Zero()});
      if (index > 0) {
        ASSERT_EQ(
          reference.predictionUpdateDt(0.01, gyro, acceleration),
          EKFEstimator::PredictionUpdateStatus::kUpdated);
      }
      if (index > 0 && index <= 100 && index % 10 == 0 && index % 40 != 0) {
        const Eigen::Vector3d position(
          0.15 * time * time + 0.002 * index,
          -0.025 * time,
          0.01 * std::sin(time));
        ASSERT_EQ(
          reference.observationUpdateWithStatus(position, variance),
          EKFEstimator::ObservationUpdateStatus::kUpdated);
        const double jitter = delay > 0.0 ? 0.07 * (1.0 + std::sin(2.3 * index)) : 0.0;
        arrivals.push_back(
          {time + delay + jitter, false, time, static_cast<std::uint64_t>(index), position});
      }
    }
    std::sort(
      arrivals.begin(), arrivals.end(),
      [](const Arrival & left, const Arrival & right) {
        if (left.time != right.time) {
          return left.time < right.time;
        }
        return left.imu && !right.imu;
      });

    double previous_output_time = -1.0;
    for (const Arrival & arrival : arrivals) {
      if (arrival.imu) {
        const auto status = replay.addImu(arrival.sensor_time, gyro, acceleration);
        ASSERT_TRUE(
          status == EskfReplay::Status::kInitialized || status == EskfReplay::Status::kApplied);
        EXPECT_GT(replay.latestTime(), previous_output_time);
        previous_output_time = replay.latestTime();
      } else {
        ASSERT_EQ(
          replay.applyMeasurement(
            arrival.sensor_time, arrival.time, arrival.id,
            positionUpdate(arrival.position, variance)),
          EskfReplay::Status::kApplied);
        EXPECT_DOUBLE_EQ(replay.latestTime(), previous_output_time);
      }
    }
    EXPECT_LT((reference.getX() - delayed.getX()).norm(), 1.0e-8) << "delay=" << delay;
    EXPECT_LT((reference.getCovariance() - delayed.getCovariance()).norm(), 1.0e-8) <<
      "delay=" << delay;
  }
}

TEST(EkfSnapshot, InvalidRestoreDoesNotMutateEstimator)
{
  EKFEstimator estimator;
  const auto before = estimator.getSnapshot();
  auto invalid = before;
  invalid.covariance(0, 0) = -1000.0;
  EXPECT_FALSE(estimator.restoreSnapshot(invalid));
  EXPECT_LT((estimator.getX() - EKFEstimator().getX()).norm(), 1.0e-15);
  EXPECT_LT((estimator.getCovariance() - before.covariance).norm(), 1.0e-15);
}

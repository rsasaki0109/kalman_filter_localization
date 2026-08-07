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
#include <random>
#include <vector>

#include <kalman_filter_localization/core/fixed_lag_smoother.hpp>

namespace
{
using kalman_filter_localization::core::EKFEstimator;
using kalman_filter_localization::core::FixedLagSmoother;

void configure(EKFEstimator & estimator)
{
  estimator.setPropagationModel(EKFEstimator::PropagationModel::kExact);
  estimator.setVarImuAcc(0.5);
  estimator.setVarImuGyro(0.01);
  estimator.setVarImuGyroBias(1.0e-6);
  estimator.setVarImuAccBias(1.0e-5);
  estimator.setTauGyroBias(300.0);
  estimator.setTauAccBias(300.0);
  estimator.setMaxPredictionDtSec(2.0);
}

EKFEstimator::State makeState(const Eigen::Vector3d & position)
{
  EKFEstimator::State state;
  state.position = position;
  state.velocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  state.orientation = Eigen::Quaterniond::Identity();
  state.gyro_bias = Eigen::Vector3d::Zero();
  state.accel_bias = Eigen::Vector3d::Zero();
  return state;
}

FixedLagSmoother::MeasurementFunction positionUpdate(
  const Eigen::Vector3d & position, const Eigen::Vector3d & variance)
{
  return [position, variance](EKFEstimator & estimator) {
           return estimator.observationUpdateWithStatus(position, variance);
         };
}
}  // namespace

// The filter alone should see a strictly larger covariance than the smoother for
// the same node. In a straight-line run with only position measurements, the RTS
// backward pass re-conditions past nodes and must never increase their trace.
TEST(FixedLagSmoother, SmoothedCovarianceIsNoLargerThanFilterCovariance)
{
  EKFEstimator estimator;
  configure(estimator);
  FixedLagSmoother smoother(estimator, -1.0);

  const Eigen::Vector3d variance(0.1, 0.1, 0.1);
  for (int i = 0; i <= 40; ++i) {
    const double t = 0.1 * i;
    const Eigen::Vector3d position(1.0 * t, 0.0, 0.0);
    ASSERT_EQ(
      smoother.addImu(t, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 9.80665)),
      i == 0 ? FixedLagSmoother::Status::kInitialized : FixedLagSmoother::Status::kApplied);
    if (i > 0 && i % 10 == 0) {
      ASSERT_EQ(
        smoother.applyMeasurement(
          t, static_cast<std::uint64_t>(i),
          positionUpdate(position, variance)),
        FixedLagSmoother::Status::kApplied);
    }
  }

  const auto smoothed = smoother.smoothAll();
  ASSERT_EQ(smoothed.size(), smoother.nodeCount());
  const auto & nodes = smoother.nodes();
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    const double filter_trace = nodes[i].covariance.trace();
    const double smoothed_trace = smoothed[i].covariance.trace();
    EXPECT_LE(smoothed_trace, filter_trace + 1.0e-9)
      << "node " << i << " filter=" << filter_trace << " smoothed=" << smoothed_trace;
  }
}

// A fixed-lag window that covers the entire dataset must reproduce the batch
// smoother output exactly (identical states and covariances, time-aligned).
TEST(FixedLagSmoother, InfiniteLagMatchesBatch)
{
  EKFEstimator estimator;
  configure(estimator);
  FixedLagSmoother smoother(estimator, -1.0);

  const Eigen::Vector3d variance(0.2, 0.2, 0.2);
  for (int i = 0; i <= 50; ++i) {
    const double t = 0.1 * i;
    const Eigen::Vector3d position(1.0 * t, 0.5 * t, 0.0);
    ASSERT_EQ(
      smoother.addImu(t, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 9.80665)),
      i == 0 ? FixedLagSmoother::Status::kInitialized : FixedLagSmoother::Status::kApplied);
    if (i > 0 && i % 5 == 0) {
      ASSERT_EQ(
        smoother.applyMeasurement(
          t, static_cast<std::uint64_t>(i),
          positionUpdate(position, variance)),
        FixedLagSmoother::Status::kApplied);
    }
  }
  const std::vector<FixedLagSmoother::SmoothedNode> batch = smoother.smoothAll();
  ASSERT_FALSE(batch.empty());

  // Now rerun with a huge lag: nothing should be emitted before finalize().
  EKFEstimator estimator2;
  configure(estimator2);
  FixedLagSmoother fixed(estimator2, 1.0e6);
  for (int i = 0; i <= 50; ++i) {
    const double t = 0.1 * i;
    const Eigen::Vector3d position(1.0 * t, 0.5 * t, 0.0);
    fixed.addImu(t, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 9.80665));
    if (i > 0 && i % 5 == 0) {
      fixed.applyMeasurement(t, static_cast<std::uint64_t>(i), positionUpdate(position, variance));
    }
  }
  EXPECT_EQ(fixed.smoothedCount(), 0U);
  ASSERT_EQ(fixed.finalize(), batch.size());
  std::vector<FixedLagSmoother::SmoothedNode> final_nodes;
  FixedLagSmoother::SmoothedNode out;
  while (fixed.popSmoothed(out)) {
    final_nodes.push_back(out);
  }
  ASSERT_EQ(final_nodes.size(), batch.size());
  for (std::size_t i = 0; i < batch.size(); ++i) {
    EXPECT_DOUBLE_EQ(final_nodes[i].time, batch[i].time);
    EXPECT_TRUE(final_nodes[i].state.position.isApprox(batch[i].state.position, 1.0e-9))
      << "node " << i;
    EXPECT_TRUE(final_nodes[i].covariance.isApprox(batch[i].covariance, 1.0e-9))
      << "node " << i;
  }
}

// With real measurements, smoothing must reduce position error against ground
// truth in the aggregate: the batch oracle is at least as good as the filter.
TEST(FixedLagSmoother, SmootherReducesErrorOnNoisyMeasurements)
{
  EKFEstimator estimator;
  configure(estimator);
  FixedLagSmoother smoother(estimator, -1.0);

  std::mt19937 generator(42);
  std::normal_distribution<double> noise(0.0, 0.3);

  const double measurement_noise_variance = 0.3 * 0.3;
  for (int i = 0; i <= 60; ++i) {
    const double t = 0.1 * i;
    const Eigen::Vector3d true_position(1.0 * t, 0.0, 0.0);
    ASSERT_EQ(
      smoother.addImu(t, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 9.80665)),
      i == 0 ? FixedLagSmoother::Status::kInitialized : FixedLagSmoother::Status::kApplied);
    if (i > 0 && i % 3 == 0) {
      const Eigen::Vector3d measured = true_position +
        Eigen::Vector3d(noise(generator), noise(generator), noise(generator));
      ASSERT_EQ(
        smoother.applyMeasurement(
          t, static_cast<std::uint64_t>(i),
          positionUpdate(measured, Eigen::Vector3d::Constant(measurement_noise_variance))),
        FixedLagSmoother::Status::kApplied);
    }
  }

  const auto smoothed = smoother.smoothAll();
  const auto & nodes = smoother.nodes();
  ASSERT_EQ(smoothed.size(), nodes.size());

  double filter_error_sum = 0.0;
  double smoothed_error_sum = 0.0;
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    const double t = nodes[i].time;
    const Eigen::Vector3d true_position(1.0 * t, 0.0, 0.0);
    filter_error_sum += (nodes[i].state.position - true_position).norm();
    smoothed_error_sum += (smoothed[i].state.position - true_position).norm();
  }
  EXPECT_LT(smoothed_error_sum, filter_error_sum);
}

// A fixed-lag window emits the oldest node once it leaves the window; emissions
// are in time order and each emitted covariance is consistent with the batch one
// for nodes that have enough future data (here the window covers everything, so
// emitted nodes equal the batch nodes they correspond to).
TEST(FixedLagSmoother, FixedLagEmitsInTimeOrder)
{
  EKFEstimator estimator;
  configure(estimator);
  FixedLagSmoother smoother(estimator, 1.0);

  const Eigen::Vector3d variance(0.2, 0.2, 0.2);
  for (int i = 0; i <= 40; ++i) {
    const double t = 0.1 * i;
    const Eigen::Vector3d position(1.0 * t, 0.0, 0.0);
    const auto status = smoother.addImu(
      t, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 9.80665));
    ASSERT_TRUE(
      status == FixedLagSmoother::Status::kInitialized ||
      status == FixedLagSmoother::Status::kApplied ||
      status == FixedLagSmoother::Status::kEmitted);
    if (i > 0 && i % 5 == 0) {
      ASSERT_EQ(
        smoother.applyMeasurement(
          t, static_cast<std::uint64_t>(i),
          positionUpdate(position, variance)),
        FixedLagSmoother::Status::kApplied);
    }
  }
  EXPECT_GT(smoother.smoothedCount(), 0U);
  double previous_time = -1.0;
  FixedLagSmoother::SmoothedNode out;
  while (smoother.popSmoothed(out)) {
    EXPECT_GT(out.time, previous_time);
    previous_time = out.time;
  }
}

// Reverse-stamped IMU and duplicate measurements must be rejected and counted.
TEST(FixedLagSmoother, RejectsReverseImuAndDuplicateMeasurements)
{
  EKFEstimator estimator;
  configure(estimator);
  FixedLagSmoother smoother(estimator, -1.0);

  ASSERT_EQ(
    smoother.addImu(0.0, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 9.80665)),
    FixedLagSmoother::Status::kInitialized);
  ASSERT_EQ(
    smoother.addImu(0.1, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 9.80665)),
    FixedLagSmoother::Status::kApplied);
  ASSERT_EQ(
    smoother.addImu(0.05, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 9.80665)),
    FixedLagSmoother::Status::kReverseImu);

  const Eigen::Vector3d variance(0.1, 0.1, 0.1);
  const Eigen::Vector3d position(0.1, 0.0, 0.0);
  ASSERT_EQ(
    smoother.applyMeasurement(0.1, 7U, positionUpdate(position, variance)),
    FixedLagSmoother::Status::kApplied);
  ASSERT_EQ(
    smoother.applyMeasurement(0.1, 7U, positionUpdate(position, variance)),
    FixedLagSmoother::Status::kDuplicate);
  EXPECT_EQ(smoother.counters().duplicate, 1U);
  EXPECT_EQ(smoother.counters().reverse_imu, 1U);
}

// Subsampling (node_subsample > 1) must keep the smoother near the exact
// per-IMU result: composed transition/Qd over a subsample interval reproduces
// the same smoothed trajectory within a small tolerance while shrinking the
// window and the per-step backward-pass cost.
TEST(FixedLagSmoother, SubsampledMatchesPerImu)
{
  const Eigen::Vector3d variance(0.2, 0.2, 0.2);

  EKFEstimator estimator_full;
  configure(estimator_full);
  FixedLagSmoother full(estimator_full, -1.0);

  EKFEstimator estimator_sub;
  configure(estimator_sub);
  FixedLagSmoother sub(estimator_sub, -1.0);
  ASSERT_TRUE(sub.setNodeSubsample(4U));

  for (int i = 0; i <= 200; ++i) {
    const double t = 0.01 * i;
    const Eigen::Vector3d true_position(0.5 * t, 0.1 * t, 0.0);
    const auto full_status = full.addImu(
      t, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 9.80665));
    const auto sub_status = sub.addImu(
      t, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 9.80665));
    ASSERT_TRUE(
      full_status == FixedLagSmoother::Status::kInitialized ||
      full_status == FixedLagSmoother::Status::kApplied);
    ASSERT_TRUE(
      sub_status == FixedLagSmoother::Status::kInitialized ||
      sub_status == FixedLagSmoother::Status::kApplied ||
      sub_status == FixedLagSmoother::Status::kEmitted);
    if (i > 0 && i % 20 == 0) {
      ASSERT_EQ(
        full.applyMeasurement(
          t, static_cast<std::uint64_t>(i),
          positionUpdate(true_position, variance)),
        FixedLagSmoother::Status::kApplied);
      ASSERT_EQ(
        sub.applyMeasurement(
          t, static_cast<std::uint64_t>(i),
          positionUpdate(true_position, variance)),
        FixedLagSmoother::Status::kApplied);
    }
  }

  const auto full_smoothed = full.smoothAll();
  const auto sub_smoothed = sub.smoothAll();
  ASSERT_FALSE(full_smoothed.empty());
  ASSERT_FALSE(sub_smoothed.empty());
  EXPECT_LE(sub.nodeCount(), full.nodeCount());
  EXPECT_GT(full.nodeCount(), 100U);

  auto mean_error = [](const std::vector<FixedLagSmoother::SmoothedNode> & traj) {
      double sum = 0.0;
      for (const auto & node : traj) {
        const Eigen::Vector3d truth(0.5 * node.time, 0.1 * node.time, 0.0);
        sum += (node.state.position - truth).norm();
      }
      return traj.empty() ? 0.0 : sum / static_cast<double>(traj.size());
    };
  const double full_error = mean_error(full_smoothed);
  const double sub_error = mean_error(sub_smoothed);
  // The subsampled oracle must stay close to the per-IMU oracle.
  EXPECT_LT(sub_error, full_error + 0.05);
  EXPECT_LT(sub_error, 0.10);
}

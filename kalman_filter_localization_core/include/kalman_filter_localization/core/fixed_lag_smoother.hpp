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

#ifndef KALMAN_FILTER_LOCALIZATION__CORE__FIXED_LAG_SMOOTHER_HPP_
#define KALMAN_FILTER_LOCALIZATION__CORE__FIXED_LAG_SMOOTHER_HPP_

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <unordered_set>
#include <utility>
#include <vector>

#include <kalman_filter_localization/core/ekf_estimator.hpp>

namespace kalman_filter_localization
{
namespace core
{

// Offline/optional fixed-lag Rauch-Tung-Striebel (RTS) smoother over the ESKF.
//
// The smoother records, for every IMU prediction step, the nominal state right
// after propagation (before any measurement update), the post-update state and
// covariance, and the discrete transition / process-covariance pair (Phi, Qd)
// that the filter actually used (see EKFEstimator::getLastDiscreteModel). A
// backward RTS pass then re-conditions every stored node on all measurements up
// to the latest one, which is exactly the "offline oracle" the EKF error
// analysis in plan.md Phase 7 asks for.
//
// Two operation modes:
//  - Fixed-lag (lag_sec > 0): the retained window is bounded in time. When the
//    oldest node falls out of the lag window it is smoothed against the current
//    window, emitted through popSmoothed(), and dropped. Memory is O(lag).
//  - Batch (lag_sec <= 0): all nodes are kept until finalize(), which emits the
//    whole smoothed trajectory in one backward pass.
//
// Measurements are expected in non-decreasing sensor time order (in-order, as an
// offline bag replay would produce). Out-of-window, duplicate, or reverse-stamped
// events are counted and rejected; delayed out-of-order rewind is the job of
// EskfReplay in the live pipeline, not of this oracle.
class FixedLagSmoother
{
public:
  using MeasurementFunction =
    std::function<EKFEstimator::ObservationUpdateStatus(EKFEstimator &)>;

  enum class Status : std::uint8_t
  {
    kInitialized = 0,
    kApplied,
    kEmitted,
    kQueuedFuture,
    kTooOld,
    kFuture,
    kDuplicate,
    kReverseImu,
    kInvalidInput,
    kUpdateRejected,
    kNumericalFailure,
  };

  struct Node
  {
    double time{std::numeric_limits<double>::quiet_NaN()};
    EKFEstimator::State predicted_state{};
    EKFEstimator::ErrorStateMatrix predicted_covariance{
      EKFEstimator::ErrorStateMatrix::Identity()};
    EKFEstimator::State state{};
    EKFEstimator::ErrorStateMatrix covariance{
      EKFEstimator::ErrorStateMatrix::Identity()};
    EKFEstimator::ErrorStateMatrix transition{
      EKFEstimator::ErrorStateMatrix::Identity()};
    EKFEstimator::ErrorStateMatrix process_covariance{
      EKFEstimator::ErrorStateMatrix::Zero()};
  };

  struct SmoothedNode
  {
    double time{std::numeric_limits<double>::quiet_NaN()};
    EKFEstimator::State state{};
    EKFEstimator::ErrorStateMatrix covariance{
      EKFEstimator::ErrorStateMatrix::Identity()};
  };

  struct Counters
  {
    std::uint64_t imu_samples{0};
    std::uint64_t measurements_applied{0};
    std::uint64_t smoothed_emitted{0};
    std::uint64_t too_old{0};
    std::uint64_t future{0};
    std::uint64_t duplicate{0};
    std::uint64_t reverse_imu{0};
    std::uint64_t invalid_input{0};
    std::uint64_t update_rejected{0};
    std::uint64_t numerical_failure{0};
  };

  explicit FixedLagSmoother(
    EKFEstimator & estimator, const double lag_sec,
    const double max_future_wait_sec = 0.5, const std::size_t node_subsample = 1)
  : estimator_(estimator), lag_sec_(lag_sec), max_future_wait_sec_(max_future_wait_sec),
    node_subsample_(std::max<std::size_t>(node_subsample, 1U))
  {
    if (!(max_future_wait_sec_ >= 0.0) || !std::isfinite(max_future_wait_sec_)) {
      max_future_wait_sec_ = 0.5;
    }
  }

  bool setLagDuration(const double lag_sec)
  {
    if (!std::isfinite(lag_sec)) {
      return false;
    }
    lag_sec_ = lag_sec;
    pruneToLag();
    return true;
  }

  // Records one smoother node every `subsample`-th IMU sample, composing the
  // per-step discrete transition / process covariance over the interval. This
  // shrinks the RTS window (and thus the per-step backward-pass cost) by the
  // same factor so the smoother can run in real time at IMU rate.
  bool setNodeSubsample(const std::size_t subsample)
  {
    if (subsample == 0U) {
      return false;
    }
    node_subsample_ = subsample;
    return true;
  }

  std::size_t nodeSubsample() const {return node_subsample_;}

  double lagDuration() const {return lag_sec_;}

  void reset()
  {
    nodes_.clear();
    measurements_.clear();
    pending_measurements_.clear();
    measurement_ids_.clear();
    emitted_.clear();
    last_imu_time_ = std::numeric_limits<double>::quiet_NaN();
    subsample_counter_ = 0U;
    accumulated_transition_ = EKFEstimator::ErrorStateMatrix::Identity();
    accumulated_process_covariance_ = EKFEstimator::ErrorStateMatrix::Zero();
    counters_ = Counters{};
  }

  // Feeds one IMU sample. A `correction` (e.g. wheel/NHC) is applied right after
  // the prediction, before any queued measurement, mirroring EskfReplay.
  Status addImu(
    const double sensor_time, const Eigen::Vector3d & gyro,
    const Eigen::Vector3d & acceleration, MeasurementFunction correction = MeasurementFunction{})
  {
    if (!std::isfinite(sensor_time) || !gyro.allFinite() || !acceleration.allFinite()) {
      ++counters_.invalid_input;
      return Status::kInvalidInput;
    }
    if (nodes_.empty()) {
      if (!estimator_.primeImuMeasurement(gyro, acceleration)) {
        ++counters_.invalid_input;
        return Status::kInvalidInput;
      }
      last_imu_time_ = sensor_time;
      subsample_counter_ = 0U;
      accumulated_transition_ = EKFEstimator::ErrorStateMatrix::Identity();
      accumulated_process_covariance_ = EKFEstimator::ErrorStateMatrix::Zero();
      if (correction && correction(estimator_) != EKFEstimator::ObservationUpdateStatus::kUpdated) {
        ++counters_.update_rejected;
        return Status::kUpdateRejected;
      }
      // Apply any measurements that arrived before the first IMU so node0's
      // covariance reflects the early updates. Otherwise node0 keeps the large
      // initial covariance while node1's predicted covariance is already small,
      // which blows up the RTS smoothing gain (P0 * Phi^T * inv(P1|0)).
      if (!applyPendingMeasurements(sensor_time)) {
        ++counters_.numerical_failure;
        return Status::kNumericalFailure;
      }
      Node node;
      node.time = sensor_time;
      node.predicted_state = estimator_.getState();
      node.predicted_covariance = estimator_.getCovariance();
      node.state = estimator_.getState();
      node.covariance = estimator_.getCovariance();
      node.transition = EKFEstimator::ErrorStateMatrix::Identity();
      node.process_covariance = EKFEstimator::ErrorStateMatrix::Zero();
      nodes_.push_back(std::move(node));
      ++counters_.imu_samples;
      return Status::kInitialized;
    }
    if (!(sensor_time > last_imu_time_)) {
      ++counters_.reverse_imu;
      return Status::kReverseImu;
    }
    const double dt = sensor_time - last_imu_time_;
    last_imu_time_ = sensor_time;
    const auto prediction_status = estimator_.predictionUpdateDt(dt, gyro, acceleration);
    if (prediction_status != EKFEstimator::PredictionUpdateStatus::kUpdated) {
      if (prediction_status == EKFEstimator::PredictionUpdateStatus::kNumericalFailure) {
        ++counters_.numerical_failure;
        return Status::kNumericalFailure;
      }
      ++counters_.invalid_input;
      return Status::kInvalidInput;
    }
    EKFEstimator::DiscreteErrorModel model;
    if (!estimator_.getLastDiscreteModel(model)) {
      ++counters_.numerical_failure;
      return Status::kNumericalFailure;
    }
    // Compose the discrete transition / process covariance over the current
    // subsample interval so a single RTS step covers several IMU steps.
    accumulated_transition_ = model.transition * accumulated_transition_;
    accumulated_process_covariance_ =
      model.transition * accumulated_process_covariance_ * model.transition.transpose() +
      model.process_covariance;
    ++subsample_counter_;
    if (subsample_counter_ < node_subsample_) {
      ++counters_.imu_samples;
      return Status::kApplied;
    }
    subsample_counter_ = 0U;
    if (correction && correction(estimator_) != EKFEstimator::ObservationUpdateStatus::kUpdated) {
      estimator_.restoreSnapshot(previousSnapshot());
      ++counters_.update_rejected;
      return Status::kUpdateRejected;
    }
    Node node;
    node.time = sensor_time;
    node.predicted_state = estimator_.getState();
    node.predicted_covariance = estimator_.getCovariance();
    node.transition = accumulated_transition_;
    node.process_covariance = accumulated_process_covariance_;
    accumulated_transition_ = EKFEstimator::ErrorStateMatrix::Identity();
    accumulated_process_covariance_ = EKFEstimator::ErrorStateMatrix::Zero();
    if (!applyPendingMeasurements(sensor_time)) {
      estimator_.restoreSnapshot(previousSnapshot());
      ++counters_.numerical_failure;
      return Status::kNumericalFailure;
    }
    node.state = estimator_.getState();
    node.covariance = estimator_.getCovariance();
    nodes_.push_back(std::move(node));
    ++counters_.imu_samples;
    pruneToLag();
    return emittedWasAppended() ? Status::kEmitted : Status::kApplied;
  }

  // Feeds one measurement whose update function is applied to the estimator at
  // its sensor time. In-order measurements whose time is at most the latest node
  // time are applied immediately; later ones are queued up to max_future_wait.
  Status applyMeasurement(
    const double sensor_time, const std::uint64_t measurement_id,
    MeasurementFunction update)
  {
    if (!std::isfinite(sensor_time) || !update) {
      ++counters_.invalid_input;
      return Status::kInvalidInput;
    }
    if (measurement_ids_.count(measurement_id) != 0U) {
      ++counters_.duplicate;
      return Status::kDuplicate;
    }
    if (nodes_.empty()) {
      MeasurementEvent event{sensor_time, measurement_id, std::move(update)};
      const auto position = std::lower_bound(
        pending_measurements_.begin(), pending_measurements_.end(), event,
        [](const MeasurementEvent & left, const MeasurementEvent & right) {
          return std::tie(left.time, left.id) < std::tie(right.time, right.id);
        });
      pending_measurements_.insert(position, std::move(event));
      measurement_ids_.insert(measurement_id);
      ++counters_.future;
      return Status::kQueuedFuture;
    }
    constexpr double kTimeTolerance = 1.0e-12;
    if (sensor_time < nodes_.front().time - kTimeTolerance) {
      ++counters_.too_old;
      return Status::kTooOld;
    }
    if (sensor_time > last_imu_time_ + kTimeTolerance) {
      if (sensor_time - last_imu_time_ <= max_future_wait_sec_) {
        MeasurementEvent event{sensor_time, measurement_id, std::move(update)};
        const auto position = std::lower_bound(
          pending_measurements_.begin(), pending_measurements_.end(), event,
          [](const MeasurementEvent & left, const MeasurementEvent & right) {
            return std::tie(left.time, left.id) < std::tie(right.time, right.id);
          });
        pending_measurements_.insert(position, std::move(event));
        measurement_ids_.insert(measurement_id);
        ++counters_.future;
        return Status::kQueuedFuture;
      }
      ++counters_.future;
      return Status::kFuture;
    }
    if (std::fabs(sensor_time - nodes_.back().time) <= kTimeTolerance) {
      // Measurement exactly at the latest node boundary: apply now so the node
      // records the post-measurement state, keeping the RTS relationship
      // predicted (pure propagation) -> state (after measurements).
      if (!applyEventNow(update)) {
        ++counters_.update_rejected;
        return Status::kUpdateRejected;
      }
      measurement_ids_.insert(measurement_id);
      ++counters_.measurements_applied;
      nodes_.back().state = estimator_.getState();
      nodes_.back().covariance = estimator_.getCovariance();
      return Status::kApplied;
    }
    // Measurement between node boundaries: defer to the next node so the
    // predicted state of that node stays measurement-free (required by RTS).
    {
      MeasurementEvent event{sensor_time, measurement_id, std::move(update)};
      const auto position = std::lower_bound(
        pending_measurements_.begin(), pending_measurements_.end(), event,
        [](const MeasurementEvent & left, const MeasurementEvent & right) {
          return std::tie(left.time, left.id) < std::tie(right.time, right.id);
        });
      pending_measurements_.insert(position, std::move(event));
      measurement_ids_.insert(measurement_id);
      return Status::kQueuedFuture;
    }
  }

  // After all data has been fed, emits the smoothed estimate for every retained
  // node (batch mode) in time order and clears the history. Returns how many
  // nodes were emitted.
  std::size_t finalize()
  {
    if (nodes_.size() < 2U) {
      std::size_t count = nodes_.size();
      for (const auto & node : nodes_) {
        emitted_.push_back({node.time, node.state, node.covariance});
      }
      nodes_.clear();
      counters_.smoothed_emitted += count;
      return count;
    }
    const std::vector<SmoothedNode> smoothed = runBackwardPass();
    emitted_.insert(emitted_.end(), smoothed.begin(), smoothed.end());
    counters_.smoothed_emitted += smoothed.size();
    const std::size_t count = smoothed.size();
    nodes_.clear();
    measurements_.clear();
    pending_measurements_.clear();
    measurement_ids_.clear();
    return count;
  }

  // Batch access: returns the full smoothed trajectory without clearing state.
  std::vector<SmoothedNode> smoothAll() const
  {
    if (nodes_.size() < 2U) {
      std::vector<SmoothedNode> result;
      result.reserve(nodes_.size());
      for (const auto & node : nodes_) {
        result.push_back({node.time, node.state, node.covariance});
      }
      return result;
    }
    return runBackwardPass();
  }

  bool popSmoothed(SmoothedNode & out)
  {
    if (emitted_.empty()) {
      return false;
    }
    out = emitted_.front();
    emitted_.erase(emitted_.begin());
    return true;
  }

  std::size_t smoothedCount() const {return emitted_.size();}

  double oldestTime() const
  {
    return nodes_.empty() ? std::numeric_limits<double>::quiet_NaN() :
           nodes_.front().time;
  }

  double latestTime() const
  {
    return nodes_.empty() ? std::numeric_limits<double>::quiet_NaN() :
           nodes_.back().time;
  }

  std::size_t nodeCount() const {return nodes_.size();}
  const std::vector<Node> & nodes() const {return nodes_;}
  const Counters & counters() const {return counters_;}

  std::size_t estimatedHistoryMemoryBytes() const
  {
    return nodes_.size() * sizeof(Node) +
           measurements_.size() * sizeof(MeasurementEvent) +
           pending_measurements_.size() * sizeof(MeasurementEvent);
  }

private:
  struct MeasurementEvent
  {
    double time;
    std::uint64_t id;
    MeasurementFunction update;
  };

  static constexpr int kDx = 0;
  static constexpr int kDv = 3;
  static constexpr int kDth = 6;
  static constexpr int kDbg = 9;
  static constexpr int kDba = 12;

  EKFEstimator::Snapshot previousSnapshot() const
  {
    return estimator_.getSnapshot();
  }

  static Eigen::Vector3d logSO3(const Eigen::Quaterniond & q)
  {
    Eigen::Quaterniond normalized = q.normalized();
    if (normalized.w() < 0.0) {
      normalized.coeffs() *= -1.0;
    }
    const Eigen::AngleAxisd axis_angle(normalized);
    const double angle = axis_angle.angle();
    if (angle < 1.0e-9) {
      return Eigen::Vector3d::Zero();
    }
    return axis_angle.axis() * angle;
  }

  static EKFEstimator::ErrorStateVector stateDifference(
    const EKFEstimator::State & a, const EKFEstimator::State & b)
  {
    EKFEstimator::ErrorStateVector delta =
      EKFEstimator::ErrorStateVector::Zero();
    delta.template segment<3>(kDx) =
      a.position - b.position;
    delta.template segment<3>(kDv) =
      a.velocity - b.velocity;
    delta.template segment<3>(kDth) =
      logSO3(b.orientation.conjugate() * a.orientation);
    delta.template segment<3>(kDbg) =
      a.gyro_bias - b.gyro_bias;
    delta.template segment<3>(kDba) =
      a.accel_bias - b.accel_bias;
    return delta;
  }

  static EKFEstimator::State injectState(
    const EKFEstimator::State & state,
    const EKFEstimator::ErrorStateVector & delta)
  {
    EKFEstimator::State result = state;
    result.position += delta.template segment<3>(kDx);
    result.velocity += delta.template segment<3>(kDv);
    const Eigen::Vector3d dtheta =
      delta.template segment<3>(kDth);
    const double norm = dtheta.norm();
    Eigen::Quaterniond dq;
    if (norm < 1.0e-12) {
      dq = Eigen::Quaterniond(1.0, 0.5 * dtheta.x(), 0.5 * dtheta.y(), 0.5 * dtheta.z());
      dq.normalize();
    } else {
      dq = Eigen::Quaterniond(
        std::cos(norm / 2.0),
        std::sin(norm / 2.0) * dtheta.x() / norm,
        std::sin(norm / 2.0) * dtheta.y() / norm,
        std::sin(norm / 2.0) * dtheta.z() / norm);
    }
    result.orientation = (result.orientation.normalized() * dq).normalized();
    result.gyro_bias += delta.template segment<3>(kDbg);
    result.accel_bias += delta.template segment<3>(kDba);
    return result;
  }

  std::vector<SmoothedNode> runBackwardPass() const
  {
    const std::size_t n = nodes_.size();
    std::vector<SmoothedNode> result(n);
    result[n - 1].time = nodes_[n - 1].time;
    result[n - 1].state = nodes_[n - 1].state;
    result[n - 1].covariance = nodes_[n - 1].covariance;
    for (std::size_t k = n - 1; k-- > 0U; ) {
      const Node & node = nodes_[k];
      const Node & next = nodes_[k + 1];
      const Eigen::Matrix<double, EKFEstimator::kErrorStateSize, 1> smoothed_delta =
        stateDifference(result[k + 1].state, next.predicted_state);
      const Eigen::Matrix<double, EKFEstimator::kErrorStateSize, EKFEstimator::kErrorStateSize>
      covariance_times_transition = node.covariance * next.transition.transpose();
      const Eigen::LDLT<EKFEstimator::ErrorStateMatrix> decomposition(
        0.5 * (next.predicted_covariance + next.predicted_covariance.transpose()));
      if (decomposition.info() != Eigen::Success || !decomposition.isPositive()) {
        result[k].time = node.time;
        result[k].state = node.state;
        result[k].covariance = node.covariance;
        continue;
      }
      const Eigen::Matrix<double, EKFEstimator::kErrorStateSize, EKFEstimator::kErrorStateSize>
      gain = decomposition.solve(covariance_times_transition.transpose()).transpose();
      const Eigen::Matrix<double, EKFEstimator::kErrorStateSize, 1> error_update =
        gain * smoothed_delta;
      if (!error_update.allFinite()) {
        result[k].time = node.time;
        result[k].state = node.state;
        result[k].covariance = node.covariance;
        continue;
      }
      result[k].time = node.time;
      result[k].state = injectState(node.state, error_update);
      EKFEstimator::ErrorStateMatrix updated =
        node.covariance + gain * (result[k + 1].covariance - next.predicted_covariance) *
        gain.transpose();
      updated = 0.5 * (updated + updated.transpose());
      if (!updated.allFinite()) {
        result[k].covariance = node.covariance;
      } else {
        result[k].covariance = updated;
      }
    }
    return result;
  }

  bool applyEventNow(const MeasurementFunction & update)
  {
    const auto status = update(estimator_);
    return status == EKFEstimator::ObservationUpdateStatus::kUpdated;
  }

  bool applyPendingMeasurements(const double filter_time)
  {
    while (!pending_measurements_.empty() &&
      pending_measurements_.front().time <= filter_time + 1.0e-12)
    {
      MeasurementEvent event = std::move(pending_measurements_.front());
      pending_measurements_.erase(pending_measurements_.begin());
      if (!applyEventNow(event.update)) {
        measurement_ids_.erase(event.id);
        return false;
      }
      measurement_ids_.erase(event.id);
      ++counters_.measurements_applied;
    }
    return true;
  }

  void pruneToLag()
  {
    if (!(lag_sec_ > 0.0) || nodes_.size() < 2U) {
      return;
    }
    bool emitted_any = false;
    while (nodes_.size() > 1U &&
      nodes_.back().time - nodes_.front().time > lag_sec_)
    {
      const std::vector<SmoothedNode> smoothed = runBackwardPass();
      emitted_.push_back(smoothed.front());
      ++counters_.smoothed_emitted;
      const double oldest = nodes_.front().time;
      nodes_.erase(nodes_.begin());
      measurements_.erase(
        std::remove_if(
          measurements_.begin(), measurements_.end(),
          [this, oldest](const MeasurementEvent & event) {
            if (event.time <= oldest) {
              measurement_ids_.erase(event.id);
              return true;
            }
            return false;
          }),
        measurements_.end());
      emitted_any = true;
    }
    last_emitted_ = emitted_any;
  }

  bool emittedWasAppended() const
  {
    return last_emitted_;
  }

  EKFEstimator & estimator_;
  double lag_sec_;
  double max_future_wait_sec_;
  std::size_t node_subsample_{1};
  double last_imu_time_{std::numeric_limits<double>::quiet_NaN()};
  std::size_t subsample_counter_{0};
  EKFEstimator::ErrorStateMatrix accumulated_transition_{
    EKFEstimator::ErrorStateMatrix::Identity()};
  EKFEstimator::ErrorStateMatrix accumulated_process_covariance_{
    EKFEstimator::ErrorStateMatrix::Zero()};
  bool last_emitted_{false};
  std::vector<Node> nodes_;
  std::vector<MeasurementEvent> measurements_;
  std::vector<MeasurementEvent> pending_measurements_;
  std::unordered_set<std::uint64_t> measurement_ids_;
  std::vector<SmoothedNode> emitted_;
  Counters counters_{};
};

}  // namespace core
}  // namespace kalman_filter_localization

#endif  // KALMAN_FILTER_LOCALIZATION__CORE__FIXED_LAG_SMOOTHER_HPP_

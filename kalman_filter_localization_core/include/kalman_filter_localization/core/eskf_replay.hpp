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

#ifndef KALMAN_FILTER_LOCALIZATION__CORE__ESKF_REPLAY_HPP_
#define KALMAN_FILTER_LOCALIZATION__CORE__ESKF_REPLAY_HPP_

#include <Eigen/Core>

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

class EskfReplay
{
public:
  using MeasurementFunction = std::function<EKFEstimator::ObservationUpdateStatus(EKFEstimator &)>;

  enum class Status : std::uint8_t
  {
    kApplied = 0,
    kInitialized,
    kQueuedFuture,
    kTooOld,
    kFuture,
    kDuplicate,
    kReverseImu,
    kInvalidInput,
    kUpdateRejected,
    kNumericalFailure,
  };

  struct Counters
  {
    std::uint64_t imu_samples{0};
    std::uint64_t measurements_applied{0};
    std::uint64_t rewind_count{0};
    std::uint64_t repropagated_imu_samples{0};
    std::uint64_t too_old{0};
    std::uint64_t future{0};
    std::uint64_t future_queued{0};
    std::uint64_t duplicate{0};
    std::uint64_t reverse_imu{0};
    std::uint64_t invalid_input{0};
    std::uint64_t update_rejected{0};
    std::uint64_t measurement_update_rejected{0};
    std::uint64_t imu_correction_rejected{0};
    std::uint64_t numerical_failure{0};
  };

  struct TimingTrace
  {
    double sensor_time{std::numeric_limits<double>::quiet_NaN()};
    double arrival_time{std::numeric_limits<double>::quiet_NaN()};
    double filter_time_before{std::numeric_limits<double>::quiet_NaN()};
    double apply_time{std::numeric_limits<double>::quiet_NaN()};
  };

  explicit EskfReplay(
    EKFEstimator & estimator, const double history_duration_sec = 1.0,
    const double max_future_wait_sec = 0.5)
  : estimator_(estimator), history_duration_sec_(history_duration_sec),
    max_future_wait_sec_(max_future_wait_sec)
  {
    if (!(max_future_wait_sec_ >= 0.0) || !std::isfinite(max_future_wait_sec_)) {
      max_future_wait_sec_ = 0.5;
    }
  }

  bool setHistoryDuration(const double duration_sec)
  {
    if (!(duration_sec > 0.0) || !std::isfinite(duration_sec)) {
      return false;
    }
    history_duration_sec_ = duration_sec;
    pruneHistory();
    return true;
  }

  void reset()
  {
    imu_samples_.clear();
    imu_snapshots_.clear();
    measurements_.clear();
    pending_measurements_.clear();
    measurement_ids_.clear();
    anchor_snapshot_ = estimator_.getSnapshot();
    last_trace_ = TimingTrace{};
  }

  Status addImu(
    const double sensor_time, const Eigen::Vector3d & gyro,
    const Eigen::Vector3d & acceleration, MeasurementFunction correction = MeasurementFunction{})
  {
    if (!std::isfinite(sensor_time) || !gyro.allFinite() || !acceleration.allFinite()) {
      ++counters_.invalid_input;
      return Status::kInvalidInput;
    }
    if (imu_samples_.empty()) {
      if (!estimator_.primeImuMeasurement(gyro, acceleration)) {
        ++counters_.invalid_input;
        return Status::kInvalidInput;
      }
      if (correction && correction(estimator_) != EKFEstimator::ObservationUpdateStatus::kUpdated) {
        ++counters_.update_rejected;
        ++counters_.imu_correction_rejected;
        return Status::kUpdateRejected;
      }
      imu_samples_.push_back({sensor_time, gyro, acceleration, std::move(correction)});
      imu_snapshots_.push_back(estimator_.getSnapshot());
      anchor_snapshot_ = estimator_.getSnapshot();
      ++counters_.imu_samples;
      applyPendingMeasurements(sensor_time);
      return Status::kInitialized;
    }
    if (!(sensor_time > imu_samples_.back().time)) {
      ++counters_.reverse_imu;
      return Status::kReverseImu;
    }
    const double dt = sensor_time - imu_samples_.back().time;
    const auto prediction_status = estimator_.predictionUpdateDt(dt, gyro, acceleration);
    if (prediction_status != EKFEstimator::PredictionUpdateStatus::kUpdated) {
      if (prediction_status == EKFEstimator::PredictionUpdateStatus::kNumericalFailure) {
        ++counters_.numerical_failure;
        return Status::kNumericalFailure;
      }
      ++counters_.invalid_input;
      return Status::kInvalidInput;
    }
    if (correction && correction(estimator_) != EKFEstimator::ObservationUpdateStatus::kUpdated) {
      estimator_.restoreSnapshot(imu_snapshots_.back());
      ++counters_.update_rejected;
      ++counters_.imu_correction_rejected;
      return Status::kUpdateRejected;
    }
    imu_samples_.push_back({sensor_time, gyro, acceleration, std::move(correction)});
    imu_snapshots_.push_back(estimator_.getSnapshot());
    ++counters_.imu_samples;
    applyPendingMeasurements(sensor_time);
    pruneHistory();
    return Status::kApplied;
  }

  Status applyMeasurement(
    const double sensor_time, const double arrival_time, const std::uint64_t measurement_id,
    MeasurementFunction update)
  {
    last_trace_ = {sensor_time, arrival_time, latestTime(), sensor_time};
    if (!std::isfinite(sensor_time) || !std::isfinite(arrival_time) || !update) {
      ++counters_.invalid_input;
      return Status::kInvalidInput;
    }
    if (measurement_ids_.count(measurement_id) != 0U) {
      ++counters_.duplicate;
      return Status::kDuplicate;
    }
    if (imu_samples_.empty()) {
      MeasurementEvent event{sensor_time, measurement_id, std::move(update)};
      const auto position = std::lower_bound(
        pending_measurements_.begin(), pending_measurements_.end(), event,
        [](const MeasurementEvent & left, const MeasurementEvent & right) {
          return std::tie(left.time, left.id) < std::tie(right.time, right.id);
        });
      pending_measurements_.insert(position, std::move(event));
      measurement_ids_.insert(measurement_id);
      ++counters_.future_queued;
      return Status::kQueuedFuture;
    }
    constexpr double kTimeTolerance = 1.0e-12;
    if (sensor_time < imu_samples_.front().time - kTimeTolerance) {
      ++counters_.too_old;
      return Status::kTooOld;
    }
    if (sensor_time > imu_samples_.back().time + kTimeTolerance) {
      if (sensor_time - imu_samples_.back().time <= max_future_wait_sec_) {
        MeasurementEvent event{sensor_time, measurement_id, std::move(update)};
        const auto position = std::lower_bound(
          pending_measurements_.begin(), pending_measurements_.end(), event,
          [](const MeasurementEvent & left, const MeasurementEvent & right) {
            return std::tie(left.time, left.id) < std::tie(right.time, right.id);
          });
        pending_measurements_.insert(position, std::move(event));
        measurement_ids_.insert(measurement_id);
        ++counters_.future_queued;
        return Status::kQueuedFuture;
      }
      ++counters_.future;
      return Status::kFuture;
    }

    MeasurementEvent event{sensor_time, measurement_id, std::move(update)};
    const auto position = std::lower_bound(
      measurements_.begin(), measurements_.end(), event,
      [](const MeasurementEvent & left, const MeasurementEvent & right) {
        return std::tie(left.time, left.id) < std::tie(right.time, right.id);
      });
    measurements_.insert(position, event);
    measurement_ids_.insert(measurement_id);

    const EKFEstimator::Snapshot current = estimator_.getSnapshot();
    const RebuildResult rebuild_result = rebuild(sensor_time);
    if (rebuild_result != RebuildResult::kSuccess) {
      estimator_.restoreSnapshot(current);
      removeMeasurement(measurement_id);
      if (rebuild_result == RebuildResult::kUpdateRejected) {
        return Status::kUpdateRejected;
      }
      ++counters_.numerical_failure;
      return Status::kNumericalFailure;
    }
    ++counters_.measurements_applied;
    ++counters_.rewind_count;
    return Status::kApplied;
  }

  double oldestTime() const
  {
    return imu_samples_.empty() ? std::numeric_limits<double>::quiet_NaN() :
           imu_samples_.front().time;
  }

  double latestTime() const
  {
    return imu_samples_.empty() ? std::numeric_limits<double>::quiet_NaN() :
           imu_samples_.back().time;
  }

  std::size_t imuHistorySize() const {return imu_samples_.size();}
  std::size_t measurementHistorySize() const {return measurements_.size();}
  std::size_t estimatedHistoryMemoryBytes() const
  {
    return imu_samples_.size() * sizeof(ImuSample) +
           measurements_.size() * sizeof(MeasurementEvent) +
           pending_measurements_.size() * sizeof(MeasurementEvent);
  }
  const Counters & counters() const {return counters_;}
  const TimingTrace & lastTimingTrace() const {return last_trace_;}

private:
  enum class RebuildResult : std::uint8_t
  {
    kSuccess = 0,
    kUpdateRejected,
    kNumericalFailure,
  };

  struct ImuSample
  {
    double time;
    Eigen::Vector3d gyro;
    Eigen::Vector3d acceleration;
    MeasurementFunction correction;
  };

  struct MeasurementEvent
  {
    double time;
    std::uint64_t id;
    MeasurementFunction update;
  };

  static Eigen::Vector3d interpolate(
    const Eigen::Vector3d & left, const Eigen::Vector3d & right, const double ratio)
  {
    return left + ratio * (right - left);
  }

  RebuildResult rebuild(const double from_time)
  {
    std::size_t start_index = 0;
    while (start_index + 1U < imu_samples_.size() &&
      imu_samples_[start_index + 1U].time < from_time)
    {
      ++start_index;
    }
    const EKFEstimator::Snapshot & start_snapshot =
      start_index == 0U ? anchor_snapshot_ : imu_snapshots_[start_index];
    if (!estimator_.restoreSnapshot(start_snapshot)) {
      return RebuildResult::kNumericalFailure;
    }
    std::size_t measurement_index = 0;
    if (start_index == 0U) {
      while (measurement_index < measurements_.size() &&
        measurements_[measurement_index].time <= imu_samples_.front().time)
      {
        if (!applyEvent(measurements_[measurement_index])) {
          return RebuildResult::kUpdateRejected;
        }
        ++measurement_index;
      }
      imu_snapshots_[0] = estimator_.getSnapshot();
    } else {
      while (measurement_index < measurements_.size() &&
        measurements_[measurement_index].time <= imu_samples_[start_index].time)
      {
        ++measurement_index;
      }
    }

    for (std::size_t index = start_index + 1U; index < imu_samples_.size(); ++index) {
      const ImuSample & previous = imu_samples_[index - 1];
      const ImuSample & current = imu_samples_[index];
      double cursor_time = previous.time;
      while (measurement_index < measurements_.size() &&
        measurements_[measurement_index].time < current.time)
      {
        const MeasurementEvent & event = measurements_[measurement_index];
        if (event.time > cursor_time) {
          const double ratio = (event.time - previous.time) / (current.time - previous.time);
          if (!predict(
              event.time - cursor_time,
              interpolate(previous.gyro, current.gyro, ratio),
              interpolate(previous.acceleration, current.acceleration, ratio)))
          {
            return RebuildResult::kNumericalFailure;
          }
          cursor_time = event.time;
        }
        if (!applyEvent(event)) {
          return RebuildResult::kUpdateRejected;
        }
        ++measurement_index;
      }
      if (current.time > cursor_time &&
        !predict(current.time - cursor_time, current.gyro, current.acceleration))
      {
        return RebuildResult::kNumericalFailure;
      }
      if (current.correction &&
        current.correction(estimator_) != EKFEstimator::ObservationUpdateStatus::kUpdated)
      {
        ++counters_.update_rejected;
        ++counters_.imu_correction_rejected;
        return RebuildResult::kUpdateRejected;
      }
      while (measurement_index < measurements_.size() &&
        measurements_[measurement_index].time <= current.time)
      {
        if (!applyEvent(measurements_[measurement_index])) {
          return RebuildResult::kUpdateRejected;
        }
        ++measurement_index;
      }
      imu_snapshots_[index] = estimator_.getSnapshot();
      ++counters_.repropagated_imu_samples;
    }
    return measurement_index == measurements_.size() ? RebuildResult::kSuccess :
           RebuildResult::kNumericalFailure;
  }

  bool predict(
    const double dt, const Eigen::Vector3d & gyro, const Eigen::Vector3d & acceleration)
  {
    return estimator_.predictionUpdateDt(dt, gyro, acceleration) ==
           EKFEstimator::PredictionUpdateStatus::kUpdated;
  }

  bool applyEvent(const MeasurementEvent & event)
  {
    const auto status = event.update(estimator_);
    if (status != EKFEstimator::ObservationUpdateStatus::kUpdated) {
      ++counters_.update_rejected;
      ++counters_.measurement_update_rejected;
      return false;
    }
    return true;
  }

  void removeMeasurement(const std::uint64_t id)
  {
    measurements_.erase(
      std::remove_if(
        measurements_.begin(), measurements_.end(),
        [id](const MeasurementEvent & event) {return event.id == id;}),
      measurements_.end());
    measurement_ids_.erase(id);
  }

  void applyPendingMeasurements(const double filter_time)
  {
    while (!pending_measurements_.empty() &&
      pending_measurements_.front().time <= filter_time)
    {
      MeasurementEvent event = std::move(pending_measurements_.front());
      pending_measurements_.erase(pending_measurements_.begin());
      const auto position = std::lower_bound(
        measurements_.begin(), measurements_.end(), event,
        [](const MeasurementEvent & left, const MeasurementEvent & right) {
          return std::tie(left.time, left.id) < std::tie(right.time, right.id);
        });
      measurements_.insert(position, event);
      const EKFEstimator::Snapshot current = estimator_.getSnapshot();
      const RebuildResult result = rebuild(event.time);
      if (result == RebuildResult::kSuccess) {
        ++counters_.measurements_applied;
        ++counters_.rewind_count;
      } else {
        estimator_.restoreSnapshot(current);
        removeMeasurement(event.id);
        if (result == RebuildResult::kNumericalFailure) {
          ++counters_.numerical_failure;
        }
      }
    }
  }

  void pruneHistory()
  {
    if (imu_samples_.size() < 2U || !(history_duration_sec_ > 0.0)) {
      return;
    }
    const double cutoff = imu_samples_.back().time - history_duration_sec_;
    std::size_t new_anchor = 0;
    while (new_anchor + 1U < imu_samples_.size() &&
      imu_samples_[new_anchor + 1U].time <= cutoff)
    {
      ++new_anchor;
    }
    if (new_anchor == 0U) {
      return;
    }
    anchor_snapshot_ = imu_snapshots_[new_anchor];
    const double anchor_time = imu_samples_[new_anchor].time;
    imu_samples_.erase(imu_samples_.begin(), imu_samples_.begin() + new_anchor);
    imu_snapshots_.erase(imu_snapshots_.begin(), imu_snapshots_.begin() + new_anchor);
    measurements_.erase(
      std::remove_if(
        measurements_.begin(), measurements_.end(),
        [this, anchor_time](const MeasurementEvent & event) {
          if (event.time <= anchor_time) {
            measurement_ids_.erase(event.id);
            return true;
          }
          return false;
        }),
      measurements_.end());
  }

  EKFEstimator & estimator_;
  double history_duration_sec_;
  double max_future_wait_sec_;
  EKFEstimator::Snapshot anchor_snapshot_{};
  std::vector<ImuSample> imu_samples_;
  std::vector<EKFEstimator::Snapshot> imu_snapshots_;
  std::vector<MeasurementEvent> measurements_;
  std::vector<MeasurementEvent> pending_measurements_;
  std::unordered_set<std::uint64_t> measurement_ids_;
  Counters counters_{};
  TimingTrace last_trace_{};
};

}  // namespace core
}  // namespace kalman_filter_localization

#endif  // KALMAN_FILTER_LOCALIZATION__CORE__ESKF_REPLAY_HPP_

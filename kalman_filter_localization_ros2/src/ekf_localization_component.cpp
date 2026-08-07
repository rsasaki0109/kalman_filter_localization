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
#include <kalman_filter_localization/ekf_localization_component.hpp>

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>

#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <array>
#include <cinttypes>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <rclcpp/qos.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <pluginlib/class_loader.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <kalman_filter_localization/core/ekf_estimator.hpp>
#include <kalman_filter_localization/core/eskf_replay.hpp>
#include <kalman_filter_localization/core/fixed_lag_smoother.hpp>
#include <kalman_filter_localization/core/imu_initializer.hpp>
#include <kalman_filter_localization/core/measurement_quality.hpp>
#include <kalman_filter_localization/core/odometry.hpp>
#include <kalman_filter_localization/core/sensor_fault_monitor.hpp>
#include <kalman_filter_localization/core/vehicle_model.hpp>
#include <kalman_filter_localization/core/vehicle_observability.hpp>
#include <kalman_filter_localization_msgs/msg/estimator_status.hpp>
#include <kalman_filter_localization_msgs/msg/measurement_quality.hpp>
#include <kalman_filter_localization_msgs/msg/observability_status.hpp>
#include <kalman_filter_localization_msgs/msg/replay_timing.hpp>

namespace kalman_filter_localization
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kWgs84SemiMajorAxisM = 6378137.0;
constexpr double kWgs84Flattening = 1.0 / 298.257223563;
constexpr double kWgs84EccentricitySquared =
  2.0 * kWgs84Flattening - kWgs84Flattening * kWgs84Flattening;

double degToRad(double angle_deg)
{
  return angle_deg * kPi / 180.0;
}

double wrapToPi(double angle_rad)
{
  while (angle_rad > kPi) {
    angle_rad -= 2.0 * kPi;
  }
  while (angle_rad < -kPi) {
    angle_rad += 2.0 * kPi;
  }
  return angle_rad;
}

double getYawRadFromQuaternion(const Eigen::Quaterniond & q_in)
{
  const Eigen::Quaterniond q = q_in.normalized();
  const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  return std::atan2(siny_cosp, cosy_cosp);
}

Eigen::Vector3d getRpyRadFromQuaternion(const Eigen::Quaterniond & q_in)
{
  const Eigen::Quaterniond q = q_in.normalized();
  const double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
  const double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
  const double pitch =
    std::abs(sinp) >= 1.0 ? std::copysign(kPi / 2.0, sinp) : std::asin(sinp);

  const double yaw = getYawRadFromQuaternion(q);
  return Eigen::Vector3d(roll, pitch, yaw);
}

double stampToSec(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

builtin_interfaces::msg::Time stampFromSec(const double sec)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(std::floor(sec));
  stamp.nanosec = static_cast<uint32_t>((sec - std::floor(sec)) * 1e9);
  return stamp;
}

Eigen::Matrix3d odometryPositionCovariance(
  const std::array<double, 36> & pose_covariance,
  const Eigen::Vector3d & fallback_variance)
{
  const Eigen::Matrix3d fallback = fallback_variance.asDiagonal();
  Eigen::Matrix3d covariance;
  covariance <<
    pose_covariance[0], pose_covariance[1], pose_covariance[2],
    pose_covariance[6], pose_covariance[7], pose_covariance[8],
    pose_covariance[12], pose_covariance[13], pose_covariance[14];
  covariance = 0.5 * (covariance + covariance.transpose());
  if (!covariance.allFinite() || (covariance.diagonal().array() <= 0.0).any()) {
    return fallback;
  }
  const Eigen::LDLT<Eigen::Matrix3d> ldlt(covariance);
  if (ldlt.info() != Eigen::Success || (ldlt.vectorD().array() <= 0.0).any()) {
    return fallback;
  }
  return covariance;
}

double positiveCovarianceOrFallback(const double value, const double fallback)
{
  return std::isfinite(value) && value > 0.0 ? value : fallback;
}

bool isValidLatitudeLongitude(double latitude_deg, double longitude_deg)
{
  return std::isfinite(latitude_deg) && std::isfinite(longitude_deg) &&
         latitude_deg >= -90.0 && latitude_deg <= 90.0 &&
         longitude_deg >= -180.0 && longitude_deg <= 180.0;
}

Eigen::Vector3d geodeticToEcef(double latitude_deg, double longitude_deg, double height_m)
{
  const double lat = degToRad(latitude_deg);
  const double lon = degToRad(longitude_deg);
  const double sin_lat = std::sin(lat);
  const double cos_lat = std::cos(lat);
  const double sin_lon = std::sin(lon);
  const double cos_lon = std::cos(lon);
  const double n = kWgs84SemiMajorAxisM /
    std::sqrt(1.0 - kWgs84EccentricitySquared * sin_lat * sin_lat);

  return Eigen::Vector3d(
    (n + height_m) * cos_lat * cos_lon,
    (n + height_m) * cos_lat * sin_lon,
    (n * (1.0 - kWgs84EccentricitySquared) + height_m) * sin_lat);
}

Eigen::Vector3d ecefToEnu(
  const Eigen::Vector3d & ecef,
  const Eigen::Vector3d & origin_ecef,
  double origin_latitude_deg,
  double origin_longitude_deg)
{
  const double lat0 = degToRad(origin_latitude_deg);
  const double lon0 = degToRad(origin_longitude_deg);
  const double sin_lat0 = std::sin(lat0);
  const double cos_lat0 = std::cos(lat0);
  const double sin_lon0 = std::sin(lon0);
  const double cos_lon0 = std::cos(lon0);
  const Eigen::Vector3d d = ecef - origin_ecef;

  return Eigen::Vector3d(
    -sin_lon0 * d.x() + cos_lon0 * d.y(),
    -sin_lat0 * cos_lon0 * d.x() - sin_lat0 * sin_lon0 * d.y() + cos_lat0 * d.z(),
    cos_lat0 * cos_lon0 * d.x() + cos_lat0 * sin_lon0 * d.y() + sin_lat0 * d.z());
}

const char * measurementRejectReasonText(const core::MeasurementRejectReason reason)
{
  switch (reason) {
    case core::MeasurementRejectReason::kNone:
      return "none";
    case core::MeasurementRejectReason::kInvalidInput:
      return "invalid_input";
    case core::MeasurementRejectReason::kReceiverQuality:
      return "receiver_quality";
    case core::MeasurementRejectReason::kInnovationMagnitude:
      return "innovation_magnitude";
    case core::MeasurementRejectReason::kNis:
      return "nis";
    case core::MeasurementRejectReason::kReacquisitionConsistency:
      return "reacquisition_consistency";
    case core::MeasurementRejectReason::kNumericalFailure:
      return "numerical_failure";
  }
  return "unknown";
}

const char * measurementSourceText(const int source)
{
  switch (source) {
    case 1:
      return "gnss_position";
    case 2:
      return "velocity";
    case 3:
      return "course_yaw";
    case 4:
      return "odometry";
    case 5:
      return "nhc";
    case 6:
      return "zupt";
    case 7:
      return "zihr";
    default:
      return "unknown";
  }
}

const char * stationaryStateText(const core::StationaryDetector::State state)
{
  switch (state) {
    case core::StationaryDetector::State::kMoving:
      return "moving";
    case core::StationaryDetector::State::kCandidate:
      return "candidate";
    case core::StationaryDetector::State::kStationary:
      return "stationary";
    case core::StationaryDetector::State::kInvalid:
      return "invalid";
  }
  return "unknown";
}

const char * slipStateText(const core::SlipTurnDetector::State state)
{
  switch (state) {
    case core::SlipTurnDetector::State::kTrusted:
      return "trusted";
    case core::SlipTurnDetector::State::kTurning:
      return "turning";
    case core::SlipTurnDetector::State::kSlip:
      return "slip";
    case core::SlipTurnDetector::State::kInvalid:
      return "invalid";
  }
  return "unknown";
}

const char * replayStatusText(const core::EskfReplay::Status status)
{
  switch (status) {
    case core::EskfReplay::Status::kApplied:
      return "applied";
    case core::EskfReplay::Status::kInitialized:
      return "initialized";
    case core::EskfReplay::Status::kQueuedFuture:
      return "queued_future";
    case core::EskfReplay::Status::kTooOld:
      return "too_old";
    case core::EskfReplay::Status::kFuture:
      return "future";
    case core::EskfReplay::Status::kDuplicate:
      return "duplicate";
    case core::EskfReplay::Status::kReverseImu:
      return "reverse_imu";
    case core::EskfReplay::Status::kInvalidInput:
      return "invalid_input";
    case core::EskfReplay::Status::kUpdateRejected:
      return "update_rejected";
    case core::EskfReplay::Status::kNumericalFailure:
      return "numerical_failure";
  }
  return "unknown";
}
}  // namespace

struct EkfLocalizationComponent::Impl
{
  struct BufferedInput
  {
    std::int64_t stamp_nanoseconds{0};
    int priority{0};
    std::uint64_t sequence{0};
    double arrival_time{std::numeric_limits<double>::quiet_NaN()};
    std::function<void()> process;
  };

  struct BufferedInputLater
  {
    bool operator()(const BufferedInput & left, const BufferedInput & right) const
    {
      return std::tie(left.stamp_nanoseconds, left.priority, left.sequence) >
             std::tie(right.stamp_nanoseconds, right.priority, right.sequence);
    }
  };

  struct StateSnapshot
  {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpy{Eigen::Vector3d::Zero()};
  };

  struct ImuReplayPlan
  {
    bool initialized{false};
    bool apply_zupt{false};
    bool apply_zihr{false};
    bool apply_nhc{false};
    bool constrain_nhc_vertical{true};
    double nhc_variance_scale{1.0};
    bool learn_gyro_bias{true};
    bool learn_accel_bias{true};
  };

  struct UpdateDeltaExtra
  {
    double raw_innovation_norm{std::numeric_limits<double>::quiet_NaN()};
    double used_innovation_norm{std::numeric_limits<double>::quiet_NaN()};
    double raw_nis{std::numeric_limits<double>::quiet_NaN()};
    double used_nis{std::numeric_limits<double>::quiet_NaN()};
    double variance_scale{std::numeric_limits<double>::quiet_NaN()};
    double kalman_gain_norm{std::numeric_limits<double>::quiet_NaN()};
    double kalman_gain_position_norm{std::numeric_limits<double>::quiet_NaN()};
    double kalman_gain_velocity_norm{std::numeric_limits<double>::quiet_NaN()};
    double kalman_gain_attitude_norm{std::numeric_limits<double>::quiet_NaN()};
    double predicted_dx_norm{std::numeric_limits<double>::quiet_NaN()};
    double predicted_dx_xy_norm{std::numeric_limits<double>::quiet_NaN()};
    double predicted_dtheta_norm{std::numeric_limits<double>::quiet_NaN()};
    Eigen::Vector3d covariance_position_diag{
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
    Eigen::Vector3d covariance_velocity_diag{
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
    Eigen::Vector3d covariance_attitude_diag{
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
    Eigen::Vector3d used_variance{
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  };

  explicit Impl(EkfLocalizationComponent & node)
  : node_(node),
    clock_(RCL_ROS_TIME),
    tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
    listener_(tfbuffer_),
    vehicle_model_loader_("kalman_filter_localization",
      "kalman_filter_localization::core::VehicleModel")
  {
  }

  ~Impl()
  {
    RCLCPP_INFO(
      node_.get_logger(),
      "input_counts initial_pose=%" PRIu64 " imu=%" PRIu64 " odom=%" PRIu64
      " gnss_pose=%" PRIu64 " gnss_navsatfix=%" PRIu64 " gnss_doppler=%" PRIu64
      " wheel=%" PRIu64 " published_pose=%" PRIu64 " reorder_late=%" PRIu64
      " reorder_buffered=%zu",
      received_initial_pose_count_, received_imu_count_, received_odom_count_,
      received_gnss_pose_count_, received_gnss_navsatfix_count_,
      received_gnss_doppler_count_, received_wheel_count_, published_pose_count_,
      reorder_late_input_count_, input_reorder_queue_.size());
    if (replay_) {
      const auto & counters = replay_->counters();
      RCLCPP_INFO(
        node_.get_logger(),
        "replay_counts imu=%" PRIu64 " measurements=%" PRIu64 " rewinds=%" PRIu64
        " repropagated_imu=%" PRIu64 " too_old=%" PRIu64 " future=%" PRIu64
        " duplicate=%" PRIu64 " reverse_imu=%" PRIu64 " rejected=%" PRIu64
        " measurement_rejected=%" PRIu64 " imu_correction_rejected=%" PRIu64
        " future_queued=%" PRIu64 " numerical_failure=%" PRIu64,
        counters.imu_samples, counters.measurements_applied, counters.rewind_count,
        counters.repropagated_imu_samples, counters.too_old, counters.future,
        counters.duplicate, counters.reverse_imu, counters.update_rejected,
        counters.measurement_update_rejected, counters.imu_correction_rejected,
        counters.future_queued,
        counters.numerical_failure);
      const double callback_mean_us = imu_callback_count_ > 0U ?
        imu_callback_total_us_ / static_cast<double>(imu_callback_count_) : 0.0;
      const double wall_duration_sec = has_imu_wall_start_ ?
        std::chrono::duration<double>(
        std::chrono::steady_clock::now() - imu_wall_start_).count() : 0.0;
      const double sensor_duration_sec = has_imu_sensor_start_ ?
        last_imu_sensor_time_ - first_imu_sensor_time_ : 0.0;
      RCLCPP_INFO(
        node_.get_logger(),
        "performance rtf=%.6f imu_callback_mean_us=%.3f imu_callback_max_us=%.3f "
        "history_memory_bytes=%zu history_imu=%zu history_measurements=%zu rewinds=%" PRIu64,
        wall_duration_sec > 0.0 ? sensor_duration_sec / wall_duration_sec : 0.0,
        callback_mean_us, imu_callback_max_us_, replay_->estimatedHistoryMemoryBytes(),
        replay_->imuHistorySize(), replay_->measurementHistorySize(), counters.rewind_count);
    }
  }

  static std::int64_t stampToNanoseconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
           static_cast<std::int64_t>(stamp.nanosec);
  }

  double filterTimeForDebug() const
  {
    if (enable_measurement_replay_ && replay_ && replay_->imuHistorySize() > 0U) {
      return replay_->latestTime();
    }
    return has_previous_time_imu_ ? previous_time_imu_ :
           std::numeric_limits<double>::quiet_NaN();
  }

  void publishTypedReplayTiming(
    const std::uint8_t source, const core::EskfReplay::Status status,
    const core::EskfReplay::TimingTrace & trace)
  {
    if (!typed_replay_timing_pub_) {
      return;
    }
    kalman_filter_localization_msgs::msg::ReplayTiming message;
    message.header.stamp = node_.now();
    message.header.frame_id = reference_frame_id_;
    message.sequence = ++replay_timing_sequence_;
    message.source_id = source;
    message.source = measurementSourceText(static_cast<int>(source));
    message.status = static_cast<std::uint8_t>(status);
    message.status_text = replayStatusText(status);
    message.sensor_time = trace.sensor_time;
    message.arrival_time = trace.arrival_time;
    message.filter_time_before = trace.filter_time_before;
    message.apply_time = trace.apply_time;
    message.applied = status == core::EskfReplay::Status::kApplied ||
      status == core::EskfReplay::Status::kInitialized;
    typed_replay_timing_pub_->publish(message);
  }

  void publishEstimatorStatus()
  {
    if (!estimator_status_pub_) {
      return;
    }
    const double filter_time = filterTimeForDebug();
    const double latest_imu_time = has_latest_imu_stamp_ ?
      latest_imu_stamp_.seconds() : std::numeric_limits<double>::quiet_NaN();
    const auto covariance = ekf_.getCovariance();
    const auto isFresh = [filter_time](
      const bool has_stamp, const double stamp, const double timeout) {
        if (!has_stamp || !std::isfinite(filter_time) || !std::isfinite(stamp)) {
          return false;
        }
        const double age = filter_time - stamp;
        return age >= -0.5 && age <= std::max(0.5, timeout);
      };
    const bool gnss_available = !use_gnss_ || isFresh(
      has_latest_gnss_, latest_gnss_time_, std::max(gnss_doppler_fallback_timeout_sec_, 1.0));
    const bool wheel_available = !use_wheel_speed_ || isFresh(
      has_latest_wheel_speed_, latest_wheel_speed_time_, std::max(max_imu_dt_sec_, 1.0));
    const auto imu_fault_state = sensor_fault_monitor_.state(
      core::SensorFaultMonitor::Sensor::kImu);
    const auto gnss_fault_state = sensor_fault_monitor_.state(
      core::SensorFaultMonitor::Sensor::kGnss);
    const auto wheel_fault_state = sensor_fault_monitor_.state(
      core::SensorFaultMonitor::Sensor::kWheel);
    const auto odom_fault_state = sensor_fault_monitor_.state(
      core::SensorFaultMonitor::Sensor::kOdom);
    const bool imu_isolated = imu_fault_state.isolated;
    const bool gnss_isolated = gnss_fault_state.isolated;
    const bool wheel_isolated = wheel_fault_state.isolated;
    const bool odom_isolated = odom_fault_state.isolated;
    const bool wheel_aid_available = use_wheel_speed_ && !wheel_isolated && wheel_available;
    const bool odom_aid_available = use_odom_ && !odom_isolated;
    const bool vehicle_aid_available = wheel_aid_available || odom_aid_available;
    const bool numerical_ok = ekf_.checkNumericalInvariants();
    const bool initialized = initial_pose_received_ && has_received_input_ &&
      (!enable_stationary_initialization_ || stationary_initialization_complete_);

    using StatusMessage = kalman_filter_localization_msgs::msg::EstimatorStatus;
    StatusMessage message;
    message.header.stamp = has_latest_imu_stamp_ ? latest_imu_stamp_ : node_.now();
    message.header.frame_id = reference_frame_id_;
    message.initialized = initialized;
    message.stationary =
      stationary_detector_.state() == core::StationaryDetector::State::kStationary;
    message.gnss_available = !use_gnss_ || (!gnss_isolated && gnss_available);
    message.wheel_available = !use_wheel_speed_ || (!wheel_isolated && wheel_available);
    message.replay_enabled = enable_measurement_replay_;
    message.numerical_invariant_ok = numerical_ok;
    message.sensor_fault_isolation_enabled = enable_sensor_fault_isolation_;
    message.imu_isolated = imu_isolated;
    message.gnss_isolated = gnss_isolated;
    message.wheel_isolated = wheel_isolated;
    message.odom_isolated = odom_isolated;
    message.filter_time = filter_time;
    message.latest_imu_time = latest_imu_time;
    message.input_age_sec = std::isfinite(filter_time) && std::isfinite(latest_imu_time) ?
      std::max(0.0, filter_time - latest_imu_time) : std::numeric_limits<double>::quiet_NaN();
    message.position_stddev_m = covariance.rows() >= 3 && covariance.cols() >= 3 ?
      std::sqrt(std::max(0.0, covariance.block<3, 3>(0, 0).diagonal().maxCoeff())) :
      std::numeric_limits<double>::quiet_NaN();
    message.velocity_stddev_mps = covariance.rows() >= 6 && covariance.cols() >= 6 ?
      std::sqrt(std::max(0.0, covariance.block<3, 3>(3, 3).diagonal().maxCoeff())) :
      std::numeric_limits<double>::quiet_NaN();
    message.yaw_stddev_rad = covariance.rows() > 8 && covariance.cols() > 8 ?
      std::sqrt(std::max(0.0, covariance(8, 8))) : std::numeric_limits<double>::quiet_NaN();
    message.wheel_scale_factor = wheel_speed_scale_factor_;
    message.received_imu = received_imu_count_;
    message.received_gnss = received_gnss_pose_count_ + received_gnss_navsatfix_count_;
    message.received_gnss_velocity = received_gnss_doppler_count_;
    message.received_wheel = received_wheel_count_;
    message.received_odom = received_odom_count_;
    message.published_pose = published_pose_count_;
    message.accepted_measurements = accepted_measurement_count_;
    message.rejected_measurements = rejected_measurement_count_;
    message.numerical_failures = numerical_failure_count_;
    message.late_input_count = reorder_late_input_count_;
    message.buffered_input_count = input_reorder_queue_.size();
    message.isolated_measurements = isolated_measurement_count_;
    message.imu_fault_events = imu_fault_state.fault_events;
    message.gnss_fault_events = gnss_fault_state.fault_events;
    message.wheel_fault_events = wheel_fault_state.fault_events;
    message.odom_fault_events = odom_fault_state.fault_events;
    if (replay_) {
      message.rewind_count = replay_->counters().rewind_count;
    }

    if (!initial_pose_received_ || !has_received_input_) {
      message.health = StatusMessage::HEALTH_UNKNOWN;
      message.mode = StatusMessage::MODE_UNINITIALIZED;
      message.health_text = "unknown";
      message.mode_text = "uninitialized";
      message.summary = initial_pose_received_ ? "waiting_for_sensor_input" :
        "waiting_for_initial_pose";
    } else if (enable_stationary_initialization_ && !stationary_initialization_complete_) {
      message.health = StatusMessage::HEALTH_DEGRADED;
      message.mode = StatusMessage::MODE_INITIALIZING;
      message.health_text = "degraded";
      message.mode_text = "initializing";
      message.summary = "waiting_for_stationary_initialization";
    } else if (imu_isolated) {
      message.health = StatusMessage::HEALTH_FAULT;
      message.mode = StatusMessage::MODE_FAULT;
      message.health_text = "fault";
      message.mode_text = "fault";
      message.summary = "imu_sensor_isolated";
    } else if (!numerical_ok) {
      message.health = StatusMessage::HEALTH_FAULT;
      message.mode = StatusMessage::MODE_FAULT;
      message.health_text = "fault";
      message.mode_text = "fault";
      message.summary = "numerical_invariant_failure";
    } else if (message.stationary) {
      message.health = StatusMessage::HEALTH_OK;
      message.mode = StatusMessage::MODE_STATIONARY;
      message.health_text = "ok";
      message.mode_text = "stationary";
      message.summary = "vehicle_stationary";
    } else if (gnss_position_reacquisition_updates_remaining_ > 0) {
      message.health = StatusMessage::HEALTH_DEGRADED;
      message.mode = StatusMessage::MODE_REACQUISITION;
      message.health_text = "degraded";
      message.mode_text = "reacquisition";
      message.summary = "validating_gnss_reacquisition";
    } else if (use_gnss_ && (gnss_isolated || !gnss_available)) {
      message.health = vehicle_aid_available ? StatusMessage::HEALTH_DEGRADED :
        StatusMessage::HEALTH_FAULT;
      message.mode = StatusMessage::MODE_OUTAGE;
      message.health_text = message.health == StatusMessage::HEALTH_FAULT ? "fault" : "degraded";
      message.mode_text = "outage";
      message.summary = gnss_isolated ? "gnss_sensor_isolated" :
        (vehicle_aid_available ? "gnss_outage_dead_reckoning" :
        "gnss_outage_without_vehicle_aid");
    } else if (use_odom_ && odom_isolated) {
      message.health = (wheel_aid_available || (use_gnss_ && !gnss_isolated)) ?
        StatusMessage::HEALTH_DEGRADED :
        StatusMessage::HEALTH_FAULT;
      message.mode = StatusMessage::MODE_OUTAGE;
      message.health_text = message.health == StatusMessage::HEALTH_FAULT ? "fault" : "degraded";
      message.mode_text = "outage";
      message.summary = "odom_sensor_isolated";
    } else if (use_wheel_speed_) {
      message.health = wheel_available && !wheel_isolated ? StatusMessage::HEALTH_OK :
        StatusMessage::HEALTH_DEGRADED;
      message.mode = StatusMessage::MODE_URBAN;
      message.health_text = wheel_available && !wheel_isolated ? "ok" : "degraded";
      message.mode_text = "urban";
      message.summary = wheel_isolated ? "wheel_sensor_isolated" :
        (wheel_available ? "gnss_and_wheel_fusion" : "wheel_input_stale");
    } else {
      message.health = StatusMessage::HEALTH_OK;
      message.mode = StatusMessage::MODE_OPEN_SKY;
      message.health_text = "ok";
      message.mode_text = "open_sky";
      message.summary = "gnss_imu_fusion";
    }
    estimator_status_pub_->publish(message);

    if (diagnostics_pub_) {
      diagnostic_msgs::msg::DiagnosticArray diagnostics;
      diagnostics.header = message.header;
      diagnostic_msgs::msg::DiagnosticStatus diagnostic;
      diagnostic.name = node_.get_fully_qualified_name() + std::string(": estimator");
      diagnostic.hardware_id = "kalman_filter_localization";
      diagnostic.level = message.health == StatusMessage::HEALTH_OK ?
        diagnostic_msgs::msg::DiagnosticStatus::OK :
        (message.health == StatusMessage::HEALTH_FAULT ?
        diagnostic_msgs::msg::DiagnosticStatus::ERROR :
        diagnostic_msgs::msg::DiagnosticStatus::WARN);
      diagnostic.message = message.summary;
      const auto addKeyValue = [&diagnostic](const std::string & key, const std::string & value) {
          diagnostic_msgs::msg::KeyValue item;
          item.key = key;
          item.value = value;
          diagnostic.values.push_back(item);
        };
      addKeyValue("health", message.health_text);
      addKeyValue("mode", message.mode_text);
      addKeyValue("gnss_available", message.gnss_available ? "true" : "false");
      addKeyValue("wheel_available", message.wheel_available ? "true" : "false");
      addKeyValue("sensor_fault_isolation_enabled",
        message.sensor_fault_isolation_enabled ? "true" : "false");
      addKeyValue("imu_isolated", message.imu_isolated ? "true" : "false");
      addKeyValue("gnss_isolated", message.gnss_isolated ? "true" : "false");
      addKeyValue("wheel_isolated", message.wheel_isolated ? "true" : "false");
      addKeyValue("odom_isolated", message.odom_isolated ? "true" : "false");
      addKeyValue("replay_enabled", message.replay_enabled ? "true" : "false");
      addKeyValue("accepted_measurements", std::to_string(message.accepted_measurements));
      addKeyValue("rejected_measurements", std::to_string(message.rejected_measurements));
      addKeyValue("numerical_failures", std::to_string(message.numerical_failures));
      addKeyValue("rewinds", std::to_string(message.rewind_count));
      diagnostics.status.push_back(diagnostic);
      diagnostics_pub_->publish(diagnostics);
    }
  }

  bool sensorInputAllowed(
    const core::SensorFaultMonitor::Sensor sensor, const double time_sec)
  {
    if (!enable_sensor_fault_isolation_ || sensor_fault_monitor_.allows(sensor, time_sec)) {
      return true;
    }
    ++isolated_measurement_count_;
    return false;
  }

  void recordSensorOutcome(
    const core::SensorFaultMonitor::Sensor sensor, const bool healthy, const double time_sec)
  {
    if (enable_sensor_fault_isolation_) {
      sensor_fault_monitor_.observe(sensor, healthy, time_sec);
    }
  }

  void publishInputTiming(
    const std::int64_t stamp_nanoseconds, const int priority, const double arrival_time,
    const double filter_time_before, const double apply_time)
  {
    if (!debug_replay_timing_pub_) {
      return;
    }
    std_msgs::msg::Float64MultiArray message;
    message.data = {
      static_cast<double>(priority), static_cast<double>(stamp_nanoseconds) * 1.0e-9,
      arrival_time, filter_time_before, apply_time, node_.now().seconds()};
    debug_replay_timing_pub_->publish(message);
  }

  void drainBufferedInputs(bool drain_all)
  {
    const std::int64_t window_nanoseconds = static_cast<std::int64_t>(
      input_reorder_window_sec_ * 1.0e9);
    const std::int64_t cutoff = latest_buffered_stamp_nanoseconds_ - window_nanoseconds;
    while (!input_reorder_queue_.empty()) {
      const BufferedInput & next = input_reorder_queue_.top();
      if (!drain_all && next.stamp_nanoseconds >= cutoff) {
        break;
      }
      BufferedInput input = next;
      input_reorder_queue_.pop();
      last_processed_stamp_nanoseconds_ = input.stamp_nanoseconds;
      last_processed_priority_ = input.priority;
      has_processed_buffered_input_ = true;
      const double filter_time_before = filterTimeForDebug();
      input.process();
      publishInputTiming(
        input.stamp_nanoseconds, input.priority, input.arrival_time,
        filter_time_before, filterTimeForDebug());
      ++reorder_processed_input_count_;
      if (drain_all && input.priority == 10 && input_reorder_drain_sleep_usec_ > 0) {
        std::this_thread::sleep_for(
          std::chrono::microseconds(input_reorder_drain_sleep_usec_));
      }
    }
  }

  void enqueueInput(
    const builtin_interfaces::msg::Time & stamp, int priority,
    std::function<void()> process)
  {
    const double arrival_time = node_.now().seconds();
    const std::int64_t stamp_nanoseconds = stampToNanoseconds(stamp);
    if (!(input_reorder_window_sec_ > 0.0)) {
      const double filter_time_before = filterTimeForDebug();
      process();
      publishInputTiming(
        stamp_nanoseconds, priority, arrival_time, filter_time_before, filterTimeForDebug());
      return;
    }
    if (has_processed_buffered_input_ &&
      std::tie(stamp_nanoseconds, priority) <
      std::tie(last_processed_stamp_nanoseconds_, last_processed_priority_))
    {
      ++reorder_late_input_count_;
      RCLCPP_ERROR(
        node_.get_logger(),
        "late input violates deterministic ordering: stamp_ns=%" PRId64
        " priority=%d last_stamp_ns=%" PRId64 " last_priority=%d",
        stamp_nanoseconds, priority, last_processed_stamp_nanoseconds_,
        last_processed_priority_);
      return;
    }
    latest_buffered_stamp_nanoseconds_ = std::max(
      latest_buffered_stamp_nanoseconds_, stamp_nanoseconds);
    input_reorder_queue_.push(
      BufferedInput{
        stamp_nanoseconds, priority, next_buffered_input_sequence_++, arrival_time, process});
    drainBufferedInputs(false);
  }

  bool applyInitialCovarianceTo(
    core::EKFEstimator & estimator,
    const Eigen::Vector3d & position_variance,
    const Eigen::Vector3d & attitude_variance)
  {
    return estimator.setInitialErrorStateCovariance(
      position_variance,
      Eigen::Vector3d(
        initial_velocity_variance_xy_, initial_velocity_variance_xy_,
        initial_velocity_variance_z_),
      attitude_variance,
      Eigen::Vector3d::Constant(initial_imu_gyro_bias_covariance_),
      Eigen::Vector3d::Constant(initial_imu_acc_bias_covariance_));
  }

  bool applyInitialCovariance(
    const Eigen::Vector3d & position_variance,
    const Eigen::Vector3d & attitude_variance)
  {
    return applyInitialCovarianceTo(ekf_, position_variance, attitude_variance);
  }

  bool applyConfiguredInitialCovarianceTo(core::EKFEstimator & estimator)
  {
    return applyInitialCovarianceTo(
      estimator,
      Eigen::Vector3d(
        initial_position_variance_xy_, initial_position_variance_xy_,
        initial_position_variance_z_),
      Eigen::Vector3d(
        initial_attitude_variance_rp_, initial_attitude_variance_rp_,
        initial_attitude_variance_yaw_));
  }

  bool applyConfiguredInitialCovariance()
  {
    return applyConfiguredInitialCovarianceTo(ekf_);
  }

  // Applies the process-model, noise, bias, gravity, and initial-covariance
  // settings to an estimator. The node's main EKF and the fixed-lag smoother run
  // with identical dynamics so the oracle sees the same model as the filter.
  void configureEstimator(
    core::EKFEstimator & estimator,
    const core::EKFEstimator::PropagationModel propagation_model)
  {
    estimator.setVarImuGyro(var_imu_w_);
    estimator.setVarImuAcc(var_imu_acc_);
    estimator.setPropagationModel(propagation_model);
    estimator.setVarImuGyroBias(var_imu_gyro_bias_);
    if (!estimator.setInitialGyroBiasCovariance(initial_imu_gyro_bias_covariance_)) {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter initial_imu_gyro_bias_covariance=%f. fallback to default=0.0",
        initial_imu_gyro_bias_covariance_);
    }
    estimator.setTauGyroBias(tau_gyro_bias_sec_);
    estimator.setVarImuAccBias(var_imu_acc_bias_);
    if (!estimator.setInitialAccelBiasCovariance(initial_imu_acc_bias_covariance_)) {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter initial_imu_acc_bias_covariance=%f. fallback to default=0.0",
        initial_imu_acc_bias_covariance_);
    }
    estimator.setTauAccBias(tau_acc_bias_sec_);
    if (!estimator.setMaxPredictionDtSec(max_imu_dt_sec_)) {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter max_imu_dt_sec=%f. fallback to default=%f",
        max_imu_dt_sec_, estimator.getMaxPredictionDtSec());
    }
    if (!estimator.setGravityZ(gravity_mps2_)) {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter gravity_mps2=%f. fallback to default=%f",
        gravity_mps2_, estimator.getGravityZ());
    }
    if (!applyConfiguredInitialCovarianceTo(estimator)) {
      throw std::invalid_argument("failed to apply configured initial covariance");
    }
  }

  void init()
  {
    node_.declare_parameter("reference_frame_id", "map");
    node_.get_parameter("reference_frame_id", reference_frame_id_);
    node_.declare_parameter("robot_frame_id", "base_link");
    node_.get_parameter("robot_frame_id", robot_frame_id_);
    node_.declare_parameter("initial_pose_topic", node_.get_name() + std::string("/initial_pose"));
    node_.get_parameter("initial_pose_topic", initial_pose_topic_);
    node_.declare_parameter("initial_pose_covariance_topic", std::string());
    node_.get_parameter("initial_pose_covariance_topic", initial_pose_covariance_topic_);
    node_.declare_parameter("imu_topic", node_.get_name() + std::string("/imu"));
    node_.get_parameter("imu_topic", imu_topic_);
    node_.declare_parameter("odom_topic", node_.get_name() + std::string("/odom"));
    node_.get_parameter("odom_topic", odom_topic_);
    node_.declare_parameter("odom_input_mode", "relative");
    node_.get_parameter("odom_input_mode", odom_input_mode_);
    node_.declare_parameter("gnss_pose_topic", node_.get_name() + std::string("/gnss_pose"));
    node_.get_parameter("gnss_pose_topic", gnss_pose_topic_);
    node_.declare_parameter("gnss_input_type", "pose");
    node_.get_parameter("gnss_input_type", gnss_input_type_);
    node_.declare_parameter("gnss_navsatfix_topic", node_.get_name() + std::string("/gnss/fix"));
    node_.get_parameter("gnss_navsatfix_topic", gnss_navsatfix_topic_);
    node_.declare_parameter("gnss_navsatfix_use_first_fix_as_origin", true);
    node_.get_parameter(
      "gnss_navsatfix_use_first_fix_as_origin", gnss_navsatfix_use_first_fix_as_origin_);
    node_.declare_parameter("gnss_navsatfix_use_position_covariance", false);
    node_.get_parameter(
      "gnss_navsatfix_use_position_covariance", gnss_navsatfix_use_position_covariance_);
    node_.declare_parameter("gnss_navsatfix_min_variance_xy", 1.0e-4);
    node_.get_parameter("gnss_navsatfix_min_variance_xy", gnss_navsatfix_min_variance_xy_);
    node_.declare_parameter("gnss_navsatfix_min_variance_z", 1.0e-4);
    node_.get_parameter("gnss_navsatfix_min_variance_z", gnss_navsatfix_min_variance_z_);
    node_.declare_parameter("gnss_navsatfix_max_variance_xy", 100.0);
    node_.get_parameter("gnss_navsatfix_max_variance_xy", gnss_navsatfix_max_variance_xy_);
    node_.declare_parameter("gnss_navsatfix_max_variance_z", 100.0);
    node_.get_parameter("gnss_navsatfix_max_variance_z", gnss_navsatfix_max_variance_z_);
    node_.declare_parameter(
      "gnss_navsatfix_origin_latitude", std::numeric_limits<double>::quiet_NaN());
    node_.get_parameter("gnss_navsatfix_origin_latitude", gnss_navsatfix_origin_latitude_);
    node_.declare_parameter(
      "gnss_navsatfix_origin_longitude", std::numeric_limits<double>::quiet_NaN());
    node_.get_parameter("gnss_navsatfix_origin_longitude", gnss_navsatfix_origin_longitude_);
    node_.declare_parameter(
      "gnss_navsatfix_origin_altitude", std::numeric_limits<double>::quiet_NaN());
    node_.get_parameter("gnss_navsatfix_origin_altitude", gnss_navsatfix_origin_altitude_);
    node_.declare_parameter("gnss_lever_arm_x", 0.0);
    node_.get_parameter("gnss_lever_arm_x", gnss_lever_arm_body_.x());
    node_.declare_parameter("gnss_lever_arm_y", 0.0);
    node_.get_parameter("gnss_lever_arm_y", gnss_lever_arm_body_.y());
    node_.declare_parameter("gnss_lever_arm_z", 0.0);
    node_.get_parameter("gnss_lever_arm_z", gnss_lever_arm_body_.z());
    node_.declare_parameter("compensate_gnss_delay", false);
    node_.get_parameter("compensate_gnss_delay", compensate_gnss_delay_);
    node_.declare_parameter("gnss_time_offset_sec", 0.0);
    node_.get_parameter("gnss_time_offset_sec", gnss_time_offset_sec_);
    node_.declare_parameter("max_gnss_delay_compensation_sec", 0.5);
    node_.get_parameter(
      "max_gnss_delay_compensation_sec", max_gnss_delay_compensation_sec_);

    node_.declare_parameter("pub_period", 10);
    node_.get_parameter("pub_period", pub_period_);
    node_.declare_parameter("input_qos_depth", 1);
    node_.get_parameter("input_qos_depth", input_qos_depth_);
    node_.declare_parameter("input_reorder_window_sec", 0.0);
    node_.get_parameter("input_reorder_window_sec", input_reorder_window_sec_);
    node_.declare_parameter("input_reorder_drain_sleep_usec", 0);
    node_.get_parameter("input_reorder_drain_sleep_usec", input_reorder_drain_sleep_usec_);
    node_.declare_parameter("output_qos_depth", 10);
    node_.get_parameter("output_qos_depth", output_qos_depth_);
    node_.declare_parameter("enable_measurement_replay", false);
    node_.get_parameter("enable_measurement_replay", enable_measurement_replay_);
    node_.declare_parameter("measurement_history_duration_sec", 1.0);
    node_.get_parameter(
      "measurement_history_duration_sec", measurement_history_duration_sec_);
    node_.declare_parameter("max_future_measurement_wait_sec", 0.5);
    node_.get_parameter(
      "max_future_measurement_wait_sec", max_future_measurement_wait_sec_);
    node_.declare_parameter("enable_fixed_lag_smoothing", false);
    node_.get_parameter("enable_fixed_lag_smoothing", enable_fixed_lag_smoothing_);
    node_.declare_parameter("fixed_lag_duration_sec", 5.0);
    node_.get_parameter("fixed_lag_duration_sec", fixed_lag_duration_sec_);
    node_.declare_parameter("fixed_lag_node_subsample", 1);
    node_.get_parameter("fixed_lag_node_subsample", fixed_lag_node_subsample_);
    node_.declare_parameter("var_imu_w", 0.01);
    node_.get_parameter("var_imu_w", var_imu_w_);
    node_.declare_parameter("var_imu_acc", 0.01);
    node_.get_parameter("var_imu_acc", var_imu_acc_);
    node_.declare_parameter("use_continuous_process_noise_density", false);
    node_.get_parameter(
      "use_continuous_process_noise_density", use_continuous_process_noise_density_);
    node_.declare_parameter("use_second_order_state_transition", false);
    node_.get_parameter(
      "use_second_order_state_transition", use_second_order_state_transition_);
    node_.declare_parameter("use_second_order_process_noise", false);
    node_.get_parameter("use_second_order_process_noise", use_second_order_process_noise_);
    node_.declare_parameter("propagation_model", "");
    node_.get_parameter("propagation_model", propagation_model_name_);
    node_.declare_parameter("var_imu_gyro_bias", 0.0);
    node_.get_parameter("var_imu_gyro_bias", var_imu_gyro_bias_);
    node_.declare_parameter("initial_imu_gyro_bias_covariance", 0.0);
    node_.get_parameter("initial_imu_gyro_bias_covariance", initial_imu_gyro_bias_covariance_);
    node_.declare_parameter("tau_gyro_bias_sec", 3600.0);
    node_.get_parameter("tau_gyro_bias_sec", tau_gyro_bias_sec_);
    node_.declare_parameter("var_imu_acc_bias", 0.0);
    node_.get_parameter("var_imu_acc_bias", var_imu_acc_bias_);
    node_.declare_parameter("initial_imu_acc_bias_covariance", 0.0);
    node_.get_parameter("initial_imu_acc_bias_covariance", initial_imu_acc_bias_covariance_);
    node_.declare_parameter("initial_position_variance_xy", 100.0);
    node_.get_parameter("initial_position_variance_xy", initial_position_variance_xy_);
    node_.declare_parameter("initial_position_variance_z", 100.0);
    node_.get_parameter("initial_position_variance_z", initial_position_variance_z_);
    node_.declare_parameter("initial_velocity_variance_xy", 100.0);
    node_.get_parameter("initial_velocity_variance_xy", initial_velocity_variance_xy_);
    node_.declare_parameter("initial_velocity_variance_z", 100.0);
    node_.get_parameter("initial_velocity_variance_z", initial_velocity_variance_z_);
    node_.declare_parameter("initial_attitude_variance_rp", 100.0);
    node_.get_parameter("initial_attitude_variance_rp", initial_attitude_variance_rp_);
    node_.declare_parameter("initial_attitude_variance_yaw", 100.0);
    node_.get_parameter("initial_attitude_variance_yaw", initial_attitude_variance_yaw_);
    node_.declare_parameter("enable_stationary_initialization", false);
    node_.get_parameter("enable_stationary_initialization", enable_stationary_initialization_);
    node_.declare_parameter("stationary_initialization_window_sec", 1.0);
    node_.get_parameter(
      "stationary_initialization_window_sec", stationary_initialization_window_sec_);
    node_.declare_parameter("stationary_initialization_min_samples", 50);
    node_.get_parameter(
      "stationary_initialization_min_samples", stationary_initialization_min_samples_);
    node_.declare_parameter("stationary_initialization_max_gyro_std_radps", 0.01);
    node_.get_parameter(
      "stationary_initialization_max_gyro_std_radps",
      stationary_initialization_max_gyro_std_radps_);
    node_.declare_parameter("stationary_initialization_max_accel_std_mps2", 0.1);
    node_.get_parameter(
      "stationary_initialization_max_accel_std_mps2",
      stationary_initialization_max_accel_std_mps2_);
    node_.declare_parameter("stationary_initialization_max_accel_norm_error_mps2", 0.3);
    node_.get_parameter(
      "stationary_initialization_max_accel_norm_error_mps2",
      stationary_initialization_max_accel_norm_error_mps2_);
    node_.declare_parameter(
      "initial_dual_antenna_yaw_rad", std::numeric_limits<double>::quiet_NaN());
    node_.get_parameter("initial_dual_antenna_yaw_rad", initial_dual_antenna_yaw_rad_);
    node_.declare_parameter("initial_dual_antenna_yaw_variance", 0.01);
    node_.get_parameter(
      "initial_dual_antenna_yaw_variance", initial_dual_antenna_yaw_variance_);
    node_.declare_parameter("tau_acc_bias_sec", 3600.0);
    node_.get_parameter("tau_acc_bias_sec", tau_acc_bias_sec_);
    node_.declare_parameter("use_imu_orientation", false);
    node_.get_parameter("use_imu_orientation", use_imu_orientation_);
    node_.declare_parameter("use_imu_orientation_covariance", true);
    node_.get_parameter("use_imu_orientation_covariance", use_imu_orientation_covariance_);
    node_.declare_parameter("var_imu_orientation_rpy", 0.01);
    node_.get_parameter("var_imu_orientation_rpy", var_imu_orientation_rpy_);
    node_.declare_parameter("use_flat_ground", false);
    node_.get_parameter("use_flat_ground", use_flat_ground_);
    node_.declare_parameter("var_flat_ground_rp", 0.03);
    node_.get_parameter("var_flat_ground_rp", var_flat_ground_rp_);
    node_.declare_parameter("use_nonholonomic_constraint", false);
    node_.get_parameter("use_nonholonomic_constraint", use_nonholonomic_constraint_);
    node_.declare_parameter("var_nhc_lateral_velocity", 0.05);
    node_.get_parameter("var_nhc_lateral_velocity", var_nhc_lateral_velocity_);
    node_.declare_parameter("var_nhc_vertical_velocity", 0.02);
    node_.get_parameter("var_nhc_vertical_velocity", var_nhc_vertical_velocity_);
    node_.declare_parameter("min_nhc_forward_speed_mps", 0.5);
    node_.get_parameter("min_nhc_forward_speed_mps", min_nhc_forward_speed_mps_);
    node_.declare_parameter("nhc_adaptive_yaw_rate_radps", 0.5);
    node_.get_parameter("nhc_adaptive_yaw_rate_radps", nhc_adaptive_yaw_rate_radps_);
    node_.declare_parameter("nhc_adaptive_lateral_accel_mps2", 1.5);
    node_.get_parameter(
      "nhc_adaptive_lateral_accel_mps2", nhc_adaptive_lateral_accel_mps2_);
    node_.declare_parameter("max_nhc_variance_scale", 100.0);
    node_.get_parameter("max_nhc_variance_scale", max_nhc_variance_scale_);
    node_.declare_parameter(
      "vehicle_model_plugin", "kalman_filter_localization/GroundVehicleModel");
    node_.get_parameter("vehicle_model_plugin", vehicle_model_plugin_);
    node_.declare_parameter("nhc_slip_wheel_innovation_mps", 1.0);
    node_.get_parameter("nhc_slip_wheel_innovation_mps", nhc_slip_wheel_innovation_mps_);
    node_.declare_parameter("nhc_recovery_samples", 5);
    node_.get_parameter("nhc_recovery_samples", nhc_recovery_samples_);
    node_.declare_parameter("use_wheel_speed", false);
    node_.get_parameter("use_wheel_speed", use_wheel_speed_);
    node_.declare_parameter("wheel_speed_topic", "/wheel_speed");
    node_.get_parameter("wheel_speed_topic", wheel_speed_topic_);
    node_.declare_parameter("wheel_speed_scale_factor", 1.0);
    node_.get_parameter("wheel_speed_scale_factor", wheel_speed_scale_factor_);
    node_.declare_parameter("wheel_speed_use_nonholonomic_constraints", false);
    node_.get_parameter(
      "wheel_speed_use_nonholonomic_constraints",
      wheel_speed_use_nonholonomic_constraints_);
    node_.declare_parameter("wheel_speed_propagate_cross_state", false);
    node_.get_parameter(
      "wheel_speed_propagate_cross_state", wheel_speed_propagate_cross_state_);
    node_.declare_parameter("wheel_speed_nhc_only_during_gnss_outage", false);
    node_.get_parameter(
      "wheel_speed_nhc_only_during_gnss_outage",
      wheel_speed_nhc_only_during_gnss_outage_);
    node_.declare_parameter("wheel_vertical_nhc_gnss_available_variance_scale", 1.0);
    node_.get_parameter(
      "wheel_vertical_nhc_gnss_available_variance_scale",
      wheel_vertical_nhc_gnss_available_variance_scale_);
    node_.declare_parameter("wheel_speed_gnss_outage_threshold_sec", 1.0);
    node_.get_parameter(
      "wheel_speed_gnss_outage_threshold_sec",
      wheel_speed_gnss_outage_threshold_sec_);
    node_.declare_parameter("estimate_wheel_speed_scale_factor", false);
    node_.get_parameter(
      "estimate_wheel_speed_scale_factor", estimate_wheel_speed_scale_factor_);
    node_.declare_parameter("wheel_scale_window_size", 100);
    node_.get_parameter("wheel_scale_window_size", wheel_scale_window_size_);
    node_.declare_parameter("wheel_scale_min_samples", 20);
    node_.get_parameter("wheel_scale_min_samples", wheel_scale_min_samples_);
    node_.declare_parameter("wheel_scale_min_speed_mps", 2.78);
    node_.get_parameter("wheel_scale_min_speed_mps", wheel_scale_min_speed_mps_);
    node_.declare_parameter("wheel_scale_max_yaw_rate_radps", 0.0873);
    node_.get_parameter("wheel_scale_max_yaw_rate_radps", wheel_scale_max_yaw_rate_radps_);
    node_.declare_parameter("wheel_scale_min_factor", 0.8);
    node_.get_parameter("wheel_scale_min_factor", wheel_scale_min_factor_);
    node_.declare_parameter("wheel_scale_max_factor", 1.2);
    node_.get_parameter("wheel_scale_max_factor", wheel_scale_max_factor_);
    node_.declare_parameter("wheel_scale_max_sample_age_sec", 0.2);
    node_.get_parameter("wheel_scale_max_sample_age_sec", wheel_scale_max_sample_age_sec_);
    node_.declare_parameter("wheel_scale_reference", "gnss");
    node_.get_parameter("wheel_scale_reference", wheel_scale_reference_);
    node_.declare_parameter("var_wheel_speed", 0.04);
    node_.get_parameter("var_wheel_speed", var_wheel_speed_);
    node_.declare_parameter("var_wheel_lateral_velocity", 0.05);
    node_.get_parameter("var_wheel_lateral_velocity", var_wheel_lateral_velocity_);
    node_.declare_parameter("var_wheel_vertical_velocity", 0.02);
    node_.get_parameter("var_wheel_vertical_velocity", var_wheel_vertical_velocity_);
    node_.declare_parameter("max_wheel_speed_innovation_mps", 5.0);
    node_.get_parameter("max_wheel_speed_innovation_mps", max_wheel_speed_innovation_mps_);
    node_.declare_parameter("use_zupt", false);
    node_.get_parameter("use_zupt", use_zupt_);
    node_.declare_parameter("zupt_max_angular_velocity_radps", 0.02);
    node_.get_parameter("zupt_max_angular_velocity_radps", zupt_max_angular_velocity_radps_);
    node_.declare_parameter("zupt_max_acceleration_error_mps2", 0.2);
    node_.get_parameter("zupt_max_acceleration_error_mps2", zupt_max_acceleration_error_mps2_);
    node_.declare_parameter("zupt_max_speed_mps", 0.3);
    node_.get_parameter("zupt_max_speed_mps", zupt_max_speed_mps_);
    node_.declare_parameter("zupt_min_stationary_duration_sec", 0.5);
    node_.get_parameter("zupt_min_stationary_duration_sec", zupt_min_stationary_duration_sec_);
    node_.declare_parameter("var_zupt_velocity", 0.01);
    node_.get_parameter("var_zupt_velocity", var_zupt_velocity_);
    node_.declare_parameter("use_zihr", false);
    node_.get_parameter("use_zihr", use_zihr_);
    node_.declare_parameter("var_zihr_gyro", 1.0e-5);
    node_.get_parameter("var_zihr_gyro", var_zihr_gyro_);
    node_.declare_parameter("enable_bias_observability_gate", false);
    node_.get_parameter("enable_bias_observability_gate", enable_bias_observability_gate_);
    node_.declare_parameter("bias_min_angular_excitation_radps", 0.1);
    node_.get_parameter(
      "bias_min_angular_excitation_radps", bias_min_angular_excitation_radps_);
    node_.declare_parameter("bias_min_acceleration_excitation_mps2", 0.5);
    node_.get_parameter(
      "bias_min_acceleration_excitation_mps2", bias_min_acceleration_excitation_mps2_);
    node_.declare_parameter("use_gnss_course_yaw", false);
    node_.get_parameter("use_gnss_course_yaw", use_gnss_course_yaw_);
    node_.declare_parameter("var_gnss_course_yaw", 0.05);
    node_.get_parameter("var_gnss_course_yaw", var_gnss_course_yaw_);
    node_.declare_parameter("min_gnss_course_distance_m", 1.0);
    node_.get_parameter("min_gnss_course_distance_m", min_gnss_course_distance_m_);
    node_.declare_parameter("min_gnss_course_speed_mps", 0.5);
    node_.get_parameter("min_gnss_course_speed_mps", min_gnss_course_speed_mps_);
    node_.declare_parameter("max_gnss_course_dt_sec", 1.0);
    node_.get_parameter("max_gnss_course_dt_sec", max_gnss_course_dt_sec_);
    node_.declare_parameter("max_gnss_course_dyaw_rad", kPi);
    node_.get_parameter("max_gnss_course_dyaw_rad", max_gnss_course_dyaw_rad_);
    node_.declare_parameter("use_gnss_velocity", false);
    node_.get_parameter("use_gnss_velocity", use_gnss_velocity_);
    node_.declare_parameter("use_gnss_doppler_velocity", false);
    node_.get_parameter("use_gnss_doppler_velocity", use_gnss_doppler_velocity_);
    node_.declare_parameter(
      "gnss_doppler_velocity_topic", node_.get_name() + std::string("/gnss/velocity"));
    node_.get_parameter("gnss_doppler_velocity_topic", gnss_doppler_velocity_topic_);
    node_.declare_parameter("use_gnss_doppler_velocity_covariance", true);
    node_.get_parameter(
      "use_gnss_doppler_velocity_covariance", use_gnss_doppler_velocity_covariance_);
    node_.declare_parameter("use_gnss_doppler_course_yaw", false);
    node_.get_parameter("use_gnss_doppler_course_yaw", use_gnss_doppler_course_yaw_);
    node_.declare_parameter("min_gnss_doppler_course_speed_mps", 1.0);
    node_.get_parameter(
      "min_gnss_doppler_course_speed_mps", min_gnss_doppler_course_speed_mps_);
    node_.declare_parameter("propagate_gnss_velocity_cross_state", true);
    node_.get_parameter(
      "propagate_gnss_velocity_cross_state", propagate_gnss_velocity_cross_state_);
    node_.declare_parameter("var_gnss_velocity_xy", 0.25);
    node_.get_parameter("var_gnss_velocity_xy", var_gnss_velocity_xy_);
    node_.declare_parameter("var_gnss_velocity_z", 0.5);
    node_.get_parameter("var_gnss_velocity_z", var_gnss_velocity_z_);
    node_.declare_parameter("min_gnss_velocity_distance_m", 0.0);
    node_.get_parameter("min_gnss_velocity_distance_m", min_gnss_velocity_distance_m_);
    node_.declare_parameter("max_gnss_velocity_dt_sec", 1.0);
    node_.get_parameter("max_gnss_velocity_dt_sec", max_gnss_velocity_dt_sec_);
    node_.declare_parameter("gnss_doppler_fallback_timeout_sec", 1.0);
    node_.get_parameter(
      "gnss_doppler_fallback_timeout_sec", gnss_doppler_fallback_timeout_sec_);
    node_.declare_parameter("gnss_position_velocity_correlation_scale", 2.0);
    node_.get_parameter(
      "gnss_position_velocity_correlation_scale",
      gnss_position_velocity_correlation_scale_);
    node_.declare_parameter("gnss_position_velocity_propagate_covariance", true);
    node_.get_parameter(
      "gnss_position_velocity_propagate_covariance",
      gnss_position_velocity_propagate_covariance_);
    node_.declare_parameter("max_gnss_velocity_innovation_mps", 5.0);
    node_.get_parameter("max_gnss_velocity_innovation_mps", max_gnss_velocity_innovation_mps_);
    node_.declare_parameter("max_gnss_velocity_nis", 0.0);
    node_.get_parameter("max_gnss_velocity_nis", max_gnss_velocity_nis_);
    node_.declare_parameter("max_imu_dt_sec", 0.5);
    node_.get_parameter("max_imu_dt_sec", max_imu_dt_sec_);
    node_.declare_parameter("gravity_mps2", 9.80665);
    node_.get_parameter("gravity_mps2", gravity_mps2_);
    node_.declare_parameter("var_gnss_xy", 0.1);
    node_.get_parameter("var_gnss_xy", var_gnss_xy_);
    node_.declare_parameter("var_gnss_z", 0.15);
    node_.get_parameter("var_gnss_z", var_gnss_z_);
    node_.declare_parameter("max_gnss_position_innovation_m", 0.0);
    node_.get_parameter("max_gnss_position_innovation_m", max_gnss_position_innovation_m_);
    node_.declare_parameter("max_gnss_position_innovation_clip_m", 0.0);
    node_.get_parameter(
      "max_gnss_position_innovation_clip_m", max_gnss_position_innovation_clip_m_);
    node_.declare_parameter("max_gnss_position_nis", 0.0);
    node_.get_parameter("max_gnss_position_nis", max_gnss_position_nis_);
    node_.declare_parameter("gnss_position_nis_adaptive_threshold", 0.0);
    node_.get_parameter(
      "gnss_position_nis_adaptive_threshold", gnss_position_nis_adaptive_threshold_);
    node_.declare_parameter("max_gnss_position_variance_scale", 1.0);
    node_.get_parameter("max_gnss_position_variance_scale", max_gnss_position_variance_scale_);
    node_.declare_parameter("gnss_position_robust_loss", "none");
    node_.get_parameter("gnss_position_robust_loss", gnss_position_robust_loss_name_);
    node_.declare_parameter("gnss_position_robust_tuning", 2.5);
    node_.get_parameter("gnss_position_robust_tuning", gnss_position_robust_tuning_);
    node_.declare_parameter("max_gnss_position_robust_variance_scale", 100.0);
    node_.get_parameter(
      "max_gnss_position_robust_variance_scale",
      max_gnss_position_robust_variance_scale_);
    node_.declare_parameter("gnss_position_reacquisition_dt_sec", 0.0);
    node_.get_parameter("gnss_position_reacquisition_dt_sec", gnss_position_reacquisition_dt_sec_);
    node_.declare_parameter("gnss_position_reacquisition_variance_scale", 1.0);
    node_.get_parameter(
      "gnss_position_reacquisition_variance_scale",
      gnss_position_reacquisition_variance_scale_);
    node_.declare_parameter("gnss_position_reacquisition_update_count", 1);
    node_.get_parameter(
      "gnss_position_reacquisition_update_count", gnss_position_reacquisition_update_count_);
    node_.declare_parameter("gnss_position_reacquisition_consistent_count", 3);
    node_.get_parameter(
      "gnss_position_reacquisition_consistent_count",
      gnss_position_reacquisition_consistent_count_);
    node_.declare_parameter("gnss_position_reacquisition_covariance_cap_xy", 0.0);
    node_.get_parameter(
      "gnss_position_reacquisition_covariance_cap_xy",
      gnss_position_reacquisition_covariance_cap_xy_);
    node_.declare_parameter("gnss_position_reacquisition_covariance_cap_z", 0.0);
    node_.get_parameter(
      "gnss_position_reacquisition_covariance_cap_z",
      gnss_position_reacquisition_covariance_cap_z_);
    node_.declare_parameter("gnss_position_reacquisition_velocity_covariance_cap_xy", 0.0);
    node_.get_parameter(
      "gnss_position_reacquisition_velocity_covariance_cap_xy",
      gnss_position_reacquisition_velocity_covariance_cap_xy_);
    node_.declare_parameter("gnss_position_reacquisition_velocity_covariance_cap_z", 0.0);
    node_.get_parameter(
      "gnss_position_reacquisition_velocity_covariance_cap_z",
      gnss_position_reacquisition_velocity_covariance_cap_z_);
    node_.declare_parameter("gnss_position_reacquisition_attitude_covariance_cap_rp", 0.0);
    node_.get_parameter(
      "gnss_position_reacquisition_attitude_covariance_cap_rp",
      gnss_position_reacquisition_attitude_covariance_cap_rp_);
    node_.declare_parameter("gnss_position_reacquisition_attitude_covariance_cap_yaw", 0.0);
    node_.get_parameter(
      "gnss_position_reacquisition_attitude_covariance_cap_yaw",
      gnss_position_reacquisition_attitude_covariance_cap_yaw_);
    node_.declare_parameter("gnss_position_reacquisition_reset_position", false);
    node_.get_parameter(
      "gnss_position_reacquisition_reset_position",
      gnss_position_reacquisition_reset_position_);
    node_.declare_parameter("gnss_position_reacquisition_reset_velocity", false);
    node_.get_parameter(
      "gnss_position_reacquisition_reset_velocity",
      gnss_position_reacquisition_reset_velocity_);
    node_.declare_parameter("gnss_position_innovation_adaptive_threshold_m", 0.0);
    node_.get_parameter(
      "gnss_position_innovation_adaptive_threshold_m",
      gnss_position_innovation_adaptive_threshold_m_);
    node_.declare_parameter("max_gnss_position_innovation_variance_scale", 1.0);
    node_.get_parameter(
      "max_gnss_position_innovation_variance_scale",
      max_gnss_position_innovation_variance_scale_);
    node_.declare_parameter("max_gnss_course_yaw_nis", 0.0);
    node_.get_parameter("max_gnss_course_yaw_nis", max_gnss_course_yaw_nis_);
    node_.declare_parameter("gnss_course_yaw_nis_adaptive_threshold", 0.0);
    node_.get_parameter(
      "gnss_course_yaw_nis_adaptive_threshold", gnss_course_yaw_nis_adaptive_threshold_);
    node_.declare_parameter("max_gnss_course_yaw_variance_scale", 1.0);
    node_.get_parameter(
      "max_gnss_course_yaw_variance_scale", max_gnss_course_yaw_variance_scale_);
    node_.declare_parameter("var_odom_xyz", 0.2);
    node_.get_parameter("var_odom_xyz", var_odom_xyz_);
    node_.declare_parameter("use_gnss", true);
    node_.get_parameter("use_gnss", use_gnss_);
    node_.declare_parameter("use_odom", false);
    node_.get_parameter("use_odom", use_odom_);
    node_.declare_parameter("enable_sensor_fault_isolation", false);
    node_.get_parameter("enable_sensor_fault_isolation", enable_sensor_fault_isolation_);
    node_.declare_parameter("sensor_fault_trip_count", 5);
    node_.get_parameter("sensor_fault_trip_count", sensor_fault_trip_count_);
    node_.declare_parameter("sensor_fault_hold_sec", 5.0);
    node_.get_parameter("sensor_fault_hold_sec", sensor_fault_hold_sec_);
    node_.declare_parameter("publish_debug_topics", false);
    node_.get_parameter("publish_debug_topics", publish_debug_topics_);
    node_.declare_parameter("output_stamp_source", "latest_input");
    node_.get_parameter("output_stamp_source", output_stamp_source_);
    node_.declare_parameter("output_publish_mode", "timer");
    node_.get_parameter("output_publish_mode", output_publish_mode_);
    node_.declare_parameter(
      "output_odometry_topic", node_.get_name() + std::string("/current_odometry"));
    node_.get_parameter("output_odometry_topic", output_odometry_topic_);
    node_.declare_parameter("publish_tf", false);
    node_.get_parameter("publish_tf", publish_tf_);
    if (
      output_stamp_source_ != "latest_input" &&
      output_stamp_source_ != "imu" &&
      output_stamp_source_ != "ros_time")
    {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter output_stamp_source='%s'. fallback to default='latest_input'",
        output_stamp_source_.c_str());
      output_stamp_source_ = "latest_input";
    }
    if (input_qos_depth_ < 1) {
      RCLCPP_WARN(
        node_.get_logger(), "invalid parameter input_qos_depth=%d. fallback to default=1",
        input_qos_depth_);
      input_qos_depth_ = 1;
    }
    if (!std::isfinite(input_reorder_window_sec_) || input_reorder_window_sec_ < 0.0) {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter input_reorder_window_sec=%f. fallback to disabled",
        input_reorder_window_sec_);
      input_reorder_window_sec_ = 0.0;
    }
    if (!(measurement_history_duration_sec_ > 0.0) ||
      !std::isfinite(measurement_history_duration_sec_))
    {
      throw std::invalid_argument("measurement_history_duration_sec must be finite and positive");
    }
    if (!(max_future_measurement_wait_sec_ >= 0.0) ||
      !std::isfinite(max_future_measurement_wait_sec_))
    {
      throw std::invalid_argument("max_future_measurement_wait_sec must be finite and nonnegative");
    }
    if (compensate_gnss_delay_) {
      RCLCPP_WARN(
        node_.get_logger(),
        "compensate_gnss_delay constant-velocity extrapolation is deprecated; use "
        "enable_measurement_replay");
    }
    if (enable_measurement_replay_ && compensate_gnss_delay_) {
      RCLCPP_WARN(
        node_.get_logger(),
        "measurement replay supersedes deprecated GNSS delay extrapolation");
      compensate_gnss_delay_ = false;
    }
    if (input_reorder_drain_sleep_usec_ < 0) {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter input_reorder_drain_sleep_usec=%d. fallback to 0",
        input_reorder_drain_sleep_usec_);
      input_reorder_drain_sleep_usec_ = 0;
    }
    if (output_qos_depth_ < 1) {
      RCLCPP_WARN(
        node_.get_logger(), "invalid parameter output_qos_depth=%d. fallback to 10",
        output_qos_depth_);
      output_qos_depth_ = 10;
    }
    if (output_publish_mode_ != "timer" && output_publish_mode_ != "imu") {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter output_publish_mode='%s'. fallback to default='timer'",
        output_publish_mode_.c_str());
      output_publish_mode_ = "timer";
    }
    if (gnss_input_type_ != "pose" && gnss_input_type_ != "navsatfix") {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter gnss_input_type='%s'. fallback to default='pose'",
        gnss_input_type_.c_str());
      gnss_input_type_ = "pose";
    }
    if (odom_input_mode_ != "relative" && odom_input_mode_ != "absolute") {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter odom_input_mode='%s'. fallback to default='relative'",
        odom_input_mode_.c_str());
      odom_input_mode_ = "relative";
    }
    if (enable_sensor_fault_isolation_) {
      core::SensorFaultMonitor::Config sensor_fault_config;
      sensor_fault_config.trip_count = sensor_fault_trip_count_ > 0 ?
        static_cast<std::size_t>(sensor_fault_trip_count_) : 0U;
      sensor_fault_config.hold_sec = sensor_fault_hold_sec_;
      if (!sensor_fault_monitor_.setConfig(sensor_fault_config)) {
        RCLCPP_WARN(
          node_.get_logger(),
          "invalid sensor fault isolation parameters; disabling fault isolation");
        enable_sensor_fault_isolation_ = false;
      }
    }
    if (gnss_position_robust_loss_name_ == "huber") {
      gnss_position_robust_loss_ = core::EKFEstimator::RobustLoss::kHuber;
    } else if (gnss_position_robust_loss_name_ == "cauchy") {
      gnss_position_robust_loss_ = core::EKFEstimator::RobustLoss::kCauchy;
    } else if (gnss_position_robust_loss_name_ != "none") {
      RCLCPP_WARN(node_.get_logger(), "invalid GNSS robust loss; fallback to none");
      gnss_position_robust_loss_name_ = "none";
    }
    if (!(gnss_position_robust_tuning_ > 0.0) ||
      !(max_gnss_position_robust_variance_scale_ >= 1.0))
    {
      RCLCPP_WARN(node_.get_logger(), "invalid GNSS robust loss parameters; disabling robust loss");
      gnss_position_robust_loss_ = core::EKFEstimator::RobustLoss::kNone;
    }
    if (!gnss_lever_arm_body_.allFinite()) {
      RCLCPP_WARN(node_.get_logger(), "invalid GNSS lever arm; fallback to zero");
      gnss_lever_arm_body_.setZero();
    }
    if (gnss_navsatfix_use_position_covariance_ &&
      (!(gnss_navsatfix_min_variance_xy_ > 0.0) ||
      !(gnss_navsatfix_min_variance_z_ > 0.0) ||
      !(gnss_navsatfix_max_variance_xy_ >= gnss_navsatfix_min_variance_xy_) ||
      !(gnss_navsatfix_max_variance_z_ >= gnss_navsatfix_min_variance_z_)))
    {
      RCLCPP_WARN(node_.get_logger(), "invalid NavSatFix covariance limits; using fixed variance");
      gnss_navsatfix_use_position_covariance_ = false;
    }
    if (compensate_gnss_delay_ &&
      (!std::isfinite(gnss_time_offset_sec_) ||
      !(max_gnss_delay_compensation_sec_ > 0.0) ||
      !std::isfinite(max_gnss_delay_compensation_sec_)))
    {
      RCLCPP_WARN(node_.get_logger(), "invalid GNSS delay parameters; disabling compensation");
      compensate_gnss_delay_ = false;
    }
    if (use_nonholonomic_constraint_) {
      if (!(var_nhc_lateral_velocity_ > 0.0) || !std::isfinite(var_nhc_lateral_velocity_)) {
        RCLCPP_WARN(node_.get_logger(), "invalid var_nhc_lateral_velocity; fallback to 0.05");
        var_nhc_lateral_velocity_ = 0.05;
      }
      if (!(var_nhc_vertical_velocity_ > 0.0) || !std::isfinite(var_nhc_vertical_velocity_)) {
        RCLCPP_WARN(node_.get_logger(), "invalid var_nhc_vertical_velocity; fallback to 0.02");
        var_nhc_vertical_velocity_ = 0.02;
      }
      if (!(min_nhc_forward_speed_mps_ >= 0.0) || !std::isfinite(min_nhc_forward_speed_mps_)) {
        RCLCPP_WARN(node_.get_logger(), "invalid min_nhc_forward_speed_mps; fallback to 0.5");
        min_nhc_forward_speed_mps_ = 0.5;
      }
      if (!(max_nhc_variance_scale_ >= 1.0) || !std::isfinite(max_nhc_variance_scale_)) {
        RCLCPP_WARN(node_.get_logger(), "invalid max_nhc_variance_scale; fallback to 100.0");
        max_nhc_variance_scale_ = 100.0;
      }
      if (!(nhc_adaptive_yaw_rate_radps_ > 0.0) ||
        !std::isfinite(nhc_adaptive_yaw_rate_radps_))
      {
        nhc_adaptive_yaw_rate_radps_ = 0.5;
      }
      if (!(nhc_adaptive_lateral_accel_mps2_ > 0.0) ||
        !std::isfinite(nhc_adaptive_lateral_accel_mps2_))
      {
        nhc_adaptive_lateral_accel_mps2_ = 1.5;
      }
      if (!(nhc_slip_wheel_innovation_mps_ > 0.0) ||
        !std::isfinite(nhc_slip_wheel_innovation_mps_))
      {
        nhc_slip_wheel_innovation_mps_ = 1.0;
      }
      if (nhc_recovery_samples_ < 1) {
        nhc_recovery_samples_ = 5;
      }
    }
    if (use_wheel_speed_ &&
      (!(wheel_speed_scale_factor_ > 0.0) || !std::isfinite(wheel_speed_scale_factor_) ||
      !(var_wheel_speed_ > 0.0) || !std::isfinite(var_wheel_speed_) ||
      !(var_wheel_lateral_velocity_ > 0.0) || !std::isfinite(var_wheel_lateral_velocity_) ||
      !(var_wheel_vertical_velocity_ > 0.0) || !std::isfinite(var_wheel_vertical_velocity_) ||
      !(max_wheel_speed_innovation_mps_ >= 0.0) ||
      !std::isfinite(max_wheel_speed_innovation_mps_) ||
      !(wheel_speed_gnss_outage_threshold_sec_ >= 0.0) ||
      !std::isfinite(wheel_speed_gnss_outage_threshold_sec_) ||
      !(wheel_vertical_nhc_gnss_available_variance_scale_ >= 1.0) ||
      !std::isfinite(wheel_vertical_nhc_gnss_available_variance_scale_)))
    {
      RCLCPP_WARN(node_.get_logger(), "invalid wheel-speed parameters; disabling wheel speed");
      use_wheel_speed_ = false;
    }
    if (estimate_wheel_speed_scale_factor_ &&
      (wheel_scale_window_size_ < 1 || wheel_scale_min_samples_ < 1 ||
      wheel_scale_min_samples_ > wheel_scale_window_size_ ||
      !(wheel_scale_min_speed_mps_ > 0.0) || !(wheel_scale_max_yaw_rate_radps_ >= 0.0) ||
      !(wheel_scale_min_factor_ > 0.0) ||
      !(wheel_scale_max_factor_ >= wheel_scale_min_factor_) ||
      !(wheel_scale_max_sample_age_sec_ >= 0.0)))
    {
      RCLCPP_WARN(node_.get_logger(), "invalid automatic wheel-scale parameters; disabling it");
      estimate_wheel_speed_scale_factor_ = false;
    }
    if (wheel_scale_reference_ != "gnss" && wheel_scale_reference_ != "odom" &&
      wheel_scale_reference_ != "either")
    {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid wheel_scale_reference='%s'. fallback to 'gnss'",
        wheel_scale_reference_.c_str());
      wheel_scale_reference_ = "gnss";
    }
    if (use_zupt_ || use_zihr_) {
      if (!(zupt_max_angular_velocity_radps_ >= 0.0) ||
        !(zupt_max_acceleration_error_mps2_ >= 0.0) ||
        !(zupt_max_speed_mps_ >= 0.0) ||
        !(zupt_min_stationary_duration_sec_ >= 0.0))
      {
        RCLCPP_WARN(node_.get_logger(),
            "invalid stationary detector parameters; disabling updates");
        use_zupt_ = false;
        use_zihr_ = false;
      }
    }
    if (use_zupt_ && (!(var_zupt_velocity_ > 0.0) || !std::isfinite(var_zupt_velocity_))) {
      RCLCPP_WARN(node_.get_logger(), "invalid var_zupt_velocity; disabling ZUPT");
      use_zupt_ = false;
    }
    if (use_zihr_ && (!(var_zihr_gyro_ > 0.0) || !std::isfinite(var_zihr_gyro_))) {
      RCLCPP_WARN(node_.get_logger(), "invalid var_zihr_gyro; disabling ZIHR");
      use_zihr_ = false;
    }
    if (!(bias_min_angular_excitation_radps_ >= 0.0) ||
      !std::isfinite(bias_min_angular_excitation_radps_) ||
      !(bias_min_acceleration_excitation_mps2_ >= 0.0) ||
      !std::isfinite(bias_min_acceleration_excitation_mps2_))
    {
      RCLCPP_WARN(node_.get_logger(), "invalid bias observability thresholds; disabling gate");
      enable_bias_observability_gate_ = false;
    }
    if (gnss_input_type_ == "navsatfix" && !gnss_navsatfix_use_first_fix_as_origin_) {
      if (isValidLatitudeLongitude(
          gnss_navsatfix_origin_latitude_, gnss_navsatfix_origin_longitude_))
      {
        const double origin_altitude = std::isfinite(gnss_navsatfix_origin_altitude_) ?
          gnss_navsatfix_origin_altitude_ : 0.0;
        setGnssNavSatFixOrigin(
          gnss_navsatfix_origin_latitude_, gnss_navsatfix_origin_longitude_, origin_altitude);
      } else {
        RCLCPP_WARN(
          node_.get_logger(),
          "invalid NavSatFix origin parameters. fallback to first fix as origin");
        gnss_navsatfix_use_first_fix_as_origin_ = true;
      }
    }
    if (use_imu_orientation_ && use_flat_ground_) {
      RCLCPP_WARN(
        node_.get_logger(),
        "both use_imu_orientation and use_flat_ground are true. "
        "use_flat_ground will be ignored.");
    }
    if (use_gnss_course_yaw_) {
      if (!(max_gnss_course_dt_sec_ > 0.0) || !std::isfinite(max_gnss_course_dt_sec_)) {
        RCLCPP_WARN(
          node_.get_logger(),
          "invalid parameter max_gnss_course_dt_sec=%f. fallback to 1.0",
          max_gnss_course_dt_sec_);
        max_gnss_course_dt_sec_ = 1.0;
      }
      if (!(min_gnss_course_distance_m_ >= 0.0) || !std::isfinite(min_gnss_course_distance_m_)) {
        RCLCPP_WARN(
          node_.get_logger(),
          "invalid parameter min_gnss_course_distance_m=%f. fallback to 1.0",
          min_gnss_course_distance_m_);
        min_gnss_course_distance_m_ = 1.0;
      }
      if (!(min_gnss_course_speed_mps_ >= 0.0) || !std::isfinite(min_gnss_course_speed_mps_)) {
        RCLCPP_WARN(
          node_.get_logger(),
          "invalid parameter min_gnss_course_speed_mps=%f. fallback to 0.5",
          min_gnss_course_speed_mps_);
        min_gnss_course_speed_mps_ = 0.5;
      }
      if (!(var_gnss_course_yaw_ > 0.0) || !std::isfinite(var_gnss_course_yaw_)) {
        RCLCPP_WARN(
          node_.get_logger(),
          "invalid parameter var_gnss_course_yaw=%f. fallback to 0.05",
          var_gnss_course_yaw_);
        var_gnss_course_yaw_ = 0.05;
      }
      if (!(max_gnss_course_dyaw_rad_ > 0.0) || max_gnss_course_dyaw_rad_ > kPi ||
        !std::isfinite(max_gnss_course_dyaw_rad_))
      {
        RCLCPP_WARN(
          node_.get_logger(),
          "invalid parameter max_gnss_course_dyaw_rad=%f. fallback to M_PI",
          max_gnss_course_dyaw_rad_);
        max_gnss_course_dyaw_rad_ = kPi;
      }
    }
    if (use_gnss_velocity_ || use_gnss_doppler_velocity_) {
      if (!(var_gnss_velocity_xy_ > 0.0) || !std::isfinite(var_gnss_velocity_xy_)) {
        RCLCPP_WARN(
          node_.get_logger(),
          "invalid parameter var_gnss_velocity_xy=%f. fallback to 0.25",
          var_gnss_velocity_xy_);
        var_gnss_velocity_xy_ = 0.25;
      }
      if (!(var_gnss_velocity_z_ > 0.0) || !std::isfinite(var_gnss_velocity_z_)) {
        RCLCPP_WARN(
          node_.get_logger(),
          "invalid parameter var_gnss_velocity_z=%f. fallback to 0.5",
          var_gnss_velocity_z_);
        var_gnss_velocity_z_ = 0.5;
      }
      if (!(max_gnss_velocity_dt_sec_ > 0.0) || !std::isfinite(max_gnss_velocity_dt_sec_)) {
        RCLCPP_WARN(
          node_.get_logger(),
          "invalid parameter max_gnss_velocity_dt_sec=%f. fallback to 1.0",
          max_gnss_velocity_dt_sec_);
        max_gnss_velocity_dt_sec_ = 1.0;
      }
      if (!(gnss_doppler_fallback_timeout_sec_ >= 0.0) ||
        !std::isfinite(gnss_doppler_fallback_timeout_sec_))
      {
        gnss_doppler_fallback_timeout_sec_ = 1.0;
      }
      if (!(gnss_position_velocity_correlation_scale_ >= 1.0) ||
        !std::isfinite(gnss_position_velocity_correlation_scale_))
      {
        gnss_position_velocity_correlation_scale_ = 2.0;
      }
      if (!(min_gnss_velocity_distance_m_ >= 0.0) ||
        !std::isfinite(min_gnss_velocity_distance_m_))
      {
        RCLCPP_WARN(
          node_.get_logger(),
          "invalid parameter min_gnss_velocity_distance_m=%f. fallback to 0.0",
          min_gnss_velocity_distance_m_);
        min_gnss_velocity_distance_m_ = 0.0;
      }
      if (!(max_gnss_velocity_innovation_mps_ > 0.0) ||
        !std::isfinite(max_gnss_velocity_innovation_mps_))
      {
        RCLCPP_WARN(
          node_.get_logger(),
          "invalid parameter max_gnss_velocity_innovation_mps=%f. fallback to 5.0",
          max_gnss_velocity_innovation_mps_);
        max_gnss_velocity_innovation_mps_ = 5.0;
      }
    }
    if (
      !(min_gnss_doppler_course_speed_mps_ >= 0.0) ||
      !std::isfinite(min_gnss_doppler_course_speed_mps_))
    {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter min_gnss_doppler_course_speed_mps=%f. fallback to 1.0",
        min_gnss_doppler_course_speed_mps_);
      min_gnss_doppler_course_speed_mps_ = 1.0;
    }

    if (!std::isfinite(gravity_mps2_) || gravity_mps2_ < 0.0) {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter gravity_mps2=%f. fallback to default=9.80665",
        gravity_mps2_);
      gravity_mps2_ = 9.80665;
    }
    if (!(max_gnss_position_nis_ >= 0.0) || !std::isfinite(max_gnss_position_nis_)) {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter max_gnss_position_nis=%f. disabling GNSS position NIS gate",
        max_gnss_position_nis_);
      max_gnss_position_nis_ = 0.0;
    }
    if (
      !(max_gnss_position_innovation_m_ >= 0.0) ||
      !std::isfinite(max_gnss_position_innovation_m_))
    {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter max_gnss_position_innovation_m=%f. disabling GNSS position "
        "innovation gate",
        max_gnss_position_innovation_m_);
      max_gnss_position_innovation_m_ = 0.0;
    }
    if (
      !(max_gnss_position_innovation_clip_m_ >= 0.0) ||
      !std::isfinite(max_gnss_position_innovation_clip_m_))
    {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter max_gnss_position_innovation_clip_m=%f. disabling GNSS position "
        "innovation clipping",
        max_gnss_position_innovation_clip_m_);
      max_gnss_position_innovation_clip_m_ = 0.0;
    }
    if (
      !(gnss_position_nis_adaptive_threshold_ >= 0.0) ||
      !std::isfinite(gnss_position_nis_adaptive_threshold_))
    {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter gnss_position_nis_adaptive_threshold=%f. disabling adaptive GNSS "
        "position covariance",
        gnss_position_nis_adaptive_threshold_);
      gnss_position_nis_adaptive_threshold_ = 0.0;
    }
    if (
      !(max_gnss_position_variance_scale_ >= 1.0) ||
      !std::isfinite(max_gnss_position_variance_scale_))
    {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter max_gnss_position_variance_scale=%f. fallback to 1.0",
        max_gnss_position_variance_scale_);
      max_gnss_position_variance_scale_ = 1.0;
    }
    if (
      !(gnss_position_reacquisition_dt_sec_ >= 0.0) ||
      !std::isfinite(gnss_position_reacquisition_dt_sec_))
    {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter gnss_position_reacquisition_dt_sec=%f. disabling GNSS "
        "reacquisition scaling",
        gnss_position_reacquisition_dt_sec_);
      gnss_position_reacquisition_dt_sec_ = 0.0;
    }
    if (
      !(gnss_position_reacquisition_variance_scale_ >= 1.0) ||
      !std::isfinite(gnss_position_reacquisition_variance_scale_))
    {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter gnss_position_reacquisition_variance_scale=%f. fallback to 1.0",
        gnss_position_reacquisition_variance_scale_);
      gnss_position_reacquisition_variance_scale_ = 1.0;
    }
    if (gnss_position_reacquisition_update_count_ < 1) {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter gnss_position_reacquisition_update_count=%d. fallback to 1",
        gnss_position_reacquisition_update_count_);
      gnss_position_reacquisition_update_count_ = 1;
    }
    if (gnss_position_reacquisition_consistent_count_ < 1) {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter gnss_position_reacquisition_consistent_count=%d. fallback to 3",
        gnss_position_reacquisition_consistent_count_);
      gnss_position_reacquisition_consistent_count_ = 3;
    }
    if (gnss_position_reacquisition_reset_position_ ||
      gnss_position_reacquisition_reset_velocity_)
    {
      RCLCPP_WARN(
        node_.get_logger(),
        "GNSS reacquisition hard reset is disabled; using consistency-gated gradual blending");
      gnss_position_reacquisition_reset_position_ = false;
      gnss_position_reacquisition_reset_velocity_ = false;
    }
    if (
      !(gnss_position_reacquisition_covariance_cap_xy_ >= 0.0) ||
      !std::isfinite(gnss_position_reacquisition_covariance_cap_xy_))
    {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter gnss_position_reacquisition_covariance_cap_xy=%f. disabling "
        "covariance cap",
        gnss_position_reacquisition_covariance_cap_xy_);
      gnss_position_reacquisition_covariance_cap_xy_ = 0.0;
    }
    if (
      !(gnss_position_reacquisition_covariance_cap_z_ >= 0.0) ||
      !std::isfinite(gnss_position_reacquisition_covariance_cap_z_))
    {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter gnss_position_reacquisition_covariance_cap_z=%f. disabling "
        "covariance cap",
        gnss_position_reacquisition_covariance_cap_z_);
      gnss_position_reacquisition_covariance_cap_z_ = 0.0;
    }
    if (
      !(gnss_position_reacquisition_velocity_covariance_cap_xy_ >= 0.0) ||
      !std::isfinite(gnss_position_reacquisition_velocity_covariance_cap_xy_))
    {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter gnss_position_reacquisition_velocity_covariance_cap_xy=%f. "
        "disabling covariance cap",
        gnss_position_reacquisition_velocity_covariance_cap_xy_);
      gnss_position_reacquisition_velocity_covariance_cap_xy_ = 0.0;
    }
    if (
      !(gnss_position_reacquisition_velocity_covariance_cap_z_ >= 0.0) ||
      !std::isfinite(gnss_position_reacquisition_velocity_covariance_cap_z_))
    {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter gnss_position_reacquisition_velocity_covariance_cap_z=%f. "
        "disabling covariance cap",
        gnss_position_reacquisition_velocity_covariance_cap_z_);
      gnss_position_reacquisition_velocity_covariance_cap_z_ = 0.0;
    }
    if (
      !(gnss_position_reacquisition_attitude_covariance_cap_rp_ >= 0.0) ||
      !std::isfinite(gnss_position_reacquisition_attitude_covariance_cap_rp_))
    {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter gnss_position_reacquisition_attitude_covariance_cap_rp=%f. "
        "disabling covariance cap",
        gnss_position_reacquisition_attitude_covariance_cap_rp_);
      gnss_position_reacquisition_attitude_covariance_cap_rp_ = 0.0;
    }
    if (
      !(gnss_position_reacquisition_attitude_covariance_cap_yaw_ >= 0.0) ||
      !std::isfinite(gnss_position_reacquisition_attitude_covariance_cap_yaw_))
    {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter gnss_position_reacquisition_attitude_covariance_cap_yaw=%f. "
        "disabling covariance cap",
        gnss_position_reacquisition_attitude_covariance_cap_yaw_);
      gnss_position_reacquisition_attitude_covariance_cap_yaw_ = 0.0;
    }
    if (
      !(gnss_position_innovation_adaptive_threshold_m_ >= 0.0) ||
      !std::isfinite(gnss_position_innovation_adaptive_threshold_m_))
    {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter gnss_position_innovation_adaptive_threshold_m=%f. disabling "
        "innovation-norm GNSS position covariance",
        gnss_position_innovation_adaptive_threshold_m_);
      gnss_position_innovation_adaptive_threshold_m_ = 0.0;
    }
    if (
      !(max_gnss_position_innovation_variance_scale_ >= 1.0) ||
      !std::isfinite(max_gnss_position_innovation_variance_scale_))
    {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter max_gnss_position_innovation_variance_scale=%f. fallback to 1.0",
        max_gnss_position_innovation_variance_scale_);
      max_gnss_position_innovation_variance_scale_ = 1.0;
    }
    if (!(max_gnss_course_yaw_nis_ >= 0.0) || !std::isfinite(max_gnss_course_yaw_nis_)) {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter max_gnss_course_yaw_nis=%f. disabling GNSS course-yaw NIS gate",
        max_gnss_course_yaw_nis_);
      max_gnss_course_yaw_nis_ = 0.0;
    }
    if (
      !(gnss_course_yaw_nis_adaptive_threshold_ >= 0.0) ||
      !std::isfinite(gnss_course_yaw_nis_adaptive_threshold_))
    {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter gnss_course_yaw_nis_adaptive_threshold=%f. disabling adaptive "
        "GNSS course-yaw covariance",
        gnss_course_yaw_nis_adaptive_threshold_);
      gnss_course_yaw_nis_adaptive_threshold_ = 0.0;
    }
    if (
      !(max_gnss_course_yaw_variance_scale_ >= 1.0) ||
      !std::isfinite(max_gnss_course_yaw_variance_scale_))
    {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter max_gnss_course_yaw_variance_scale=%f. fallback to 1.0",
        max_gnss_course_yaw_variance_scale_);
      max_gnss_course_yaw_variance_scale_ = 1.0;
    }
    if (!(var_imu_gyro_bias_ >= 0.0) || !std::isfinite(var_imu_gyro_bias_)) {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter var_imu_gyro_bias=%f. fallback to default=0.0",
        var_imu_gyro_bias_);
      var_imu_gyro_bias_ = 0.0;
    }
    if (!(tau_gyro_bias_sec_ > 0.0) || !std::isfinite(tau_gyro_bias_sec_)) {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter tau_gyro_bias_sec=%f. fallback to default=3600.0",
        tau_gyro_bias_sec_);
      tau_gyro_bias_sec_ = 3600.0;
    }
    if (!(initial_imu_gyro_bias_covariance_ >= 0.0) ||
      !std::isfinite(initial_imu_gyro_bias_covariance_))
    {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter initial_imu_gyro_bias_covariance=%f. fallback to default=0.0",
        initial_imu_gyro_bias_covariance_);
      initial_imu_gyro_bias_covariance_ = 0.0;
    }
    if (!(var_imu_acc_bias_ >= 0.0) || !std::isfinite(var_imu_acc_bias_)) {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter var_imu_acc_bias=%f. fallback to default=0.0",
        var_imu_acc_bias_);
      var_imu_acc_bias_ = 0.0;
    }
    if (!(tau_acc_bias_sec_ > 0.0) || !std::isfinite(tau_acc_bias_sec_)) {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter tau_acc_bias_sec=%f. fallback to default=3600.0",
        tau_acc_bias_sec_);
      tau_acc_bias_sec_ = 3600.0;
    }
    if (!(initial_imu_acc_bias_covariance_ >= 0.0) ||
      !std::isfinite(initial_imu_acc_bias_covariance_))
    {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter initial_imu_acc_bias_covariance=%f. fallback to default=0.0",
        initial_imu_acc_bias_covariance_);
      initial_imu_acc_bias_covariance_ = 0.0;
    }
    const double initial_variances[] = {
      initial_position_variance_xy_, initial_position_variance_z_,
      initial_velocity_variance_xy_, initial_velocity_variance_z_,
      initial_attitude_variance_rp_, initial_attitude_variance_yaw_};
    for (const double variance : initial_variances) {
      if (!(variance >= 0.0) || !std::isfinite(variance)) {
        throw std::invalid_argument("all initial state variances must be finite and nonnegative");
      }
    }
    core::ImuStationaryInitializer::Config initializer_config;
    initializer_config.window_duration_sec = stationary_initialization_window_sec_;
    initializer_config.minimum_samples = stationary_initialization_min_samples_ > 0 ?
      static_cast<std::size_t>(stationary_initialization_min_samples_) : 0U;
    initializer_config.gravity_mps2 = gravity_mps2_;
    initializer_config.max_gyro_std_radps = stationary_initialization_max_gyro_std_radps_;
    initializer_config.max_accel_std_mps2 = stationary_initialization_max_accel_std_mps2_;
    initializer_config.max_accel_norm_error_mps2 =
      stationary_initialization_max_accel_norm_error_mps2_;
    if (!stationary_initializer_.setConfig(initializer_config)) {
      throw std::invalid_argument("invalid stationary initialization parameters");
    }
    core::StationaryDetector::Config stationary_config;
    stationary_config.max_angular_velocity_radps = zupt_max_angular_velocity_radps_;
    stationary_config.max_acceleration_norm_error_mps2 =
      zupt_max_acceleration_error_mps2_;
    stationary_config.max_speed_mps = zupt_max_speed_mps_;
    stationary_config.minimum_duration_sec = zupt_min_stationary_duration_sec_;
    stationary_detector_ = core::StationaryDetector(stationary_config);
    try {
      vehicle_model_ = vehicle_model_loader_.createSharedInstance(vehicle_model_plugin_);
    } catch (const pluginlib::PluginlibException & exception) {
      throw std::invalid_argument(
              std::string("failed to load vehicle_model_plugin='") + vehicle_model_plugin_ +
              "': " + exception.what());
    }
    if (!vehicle_model_) {
      throw std::invalid_argument("vehicle model plugin returned a null instance");
    }
    core::VehicleModelConfig vehicle_model_config;
    vehicle_model_config.minimum_forward_speed_mps = min_nhc_forward_speed_mps_;
    vehicle_model_config.yaw_rate_threshold_radps = nhc_adaptive_yaw_rate_radps_;
    vehicle_model_config.lateral_acceleration_threshold_mps2 =
      nhc_adaptive_lateral_accel_mps2_;
    vehicle_model_config.wheel_innovation_threshold_mps = nhc_slip_wheel_innovation_mps_;
    vehicle_model_config.maximum_variance_scale = max_nhc_variance_scale_;
    vehicle_model_config.recovery_samples = nhc_recovery_samples_ > 0 ?
      static_cast<std::uint32_t>(nhc_recovery_samples_) : 1U;
    if (!vehicle_model_->configure(vehicle_model_config)) {
      throw std::invalid_argument(
              std::string("vehicle model plugin rejected configuration: ") +
              vehicle_model_->name());
    }
    RCLCPP_INFO(
      node_.get_logger(), "vehicle model: plugin='%s' model='%s'",
      vehicle_model_plugin_.c_str(), vehicle_model_->name().c_str());
    // Private vehicle model / stationary detector for the fixed-lag smoothing
    // forward track so per-IMU corrections run independently of the main filter.
    if (enable_fixed_lag_smoothing_) {
      try {
        smoother_vehicle_model_ =
          vehicle_model_loader_.createSharedInstance(vehicle_model_plugin_);
      } catch (const pluginlib::PluginlibException & exception) {
        throw std::invalid_argument(
                std::string("failed to load smoother vehicle_model_plugin='") +
                vehicle_model_plugin_ + "': " + exception.what());
      }
      if (!smoother_vehicle_model_ ||
        !smoother_vehicle_model_->configure(vehicle_model_config))
      {
        throw std::invalid_argument(
                std::string("smoother vehicle model plugin rejected configuration: ") +
                (smoother_vehicle_model_ ? smoother_vehicle_model_->name() : std::string("null")));
      }
      smoother_stationary_detector_ = core::StationaryDetector(stationary_config);
    }
    core::GnssReacquisitionGate::Config reacquisition_config;
    reacquisition_config.outage_duration_sec = gnss_position_reacquisition_dt_sec_ > 0.0 ?
      gnss_position_reacquisition_dt_sec_ : 1.0;
    reacquisition_config.required_consistent_updates =
      gnss_position_reacquisition_consistent_count_ > 0 ?
      static_cast<std::size_t>(gnss_position_reacquisition_consistent_count_) : 1U;
    reacquisition_config.blending_updates = gnss_position_reacquisition_update_count_ > 0 ?
      static_cast<std::size_t>(gnss_position_reacquisition_update_count_) : 1U;
    reacquisition_config.initial_variance_scale =
      std::max(1.0, gnss_position_reacquisition_variance_scale_);
    reacquisition_config.maximum_position_innovation_m =
      max_gnss_position_innovation_m_ > 0.0 ? max_gnss_position_innovation_m_ : 30.0;
    reacquisition_config.maximum_velocity_innovation_mps =
      max_gnss_velocity_innovation_mps_ > 0.0 ? max_gnss_velocity_innovation_mps_ : 10.0;
    gnss_reacquisition_gate_ = core::GnssReacquisitionGate(reacquisition_config);
    if (std::isfinite(initial_dual_antenna_yaw_rad_) &&
      (!(initial_dual_antenna_yaw_variance_ > 0.0) ||
      !std::isfinite(initial_dual_antenna_yaw_variance_)))
    {
      throw std::invalid_argument("initial dual-antenna yaw variance must be finite and positive");
    }

    ekf_.setVarImuGyro(var_imu_w_);
    ekf_.setVarImuAcc(var_imu_acc_);
    const bool all_legacy_flags_disabled =
      !use_continuous_process_noise_density_ && !use_second_order_state_transition_ &&
      !use_second_order_process_noise_;
    const bool all_legacy_flags_enabled =
      use_continuous_process_noise_density_ && use_second_order_state_transition_ &&
      use_second_order_process_noise_;
    if (propagation_model_name_.empty()) {
      if (!all_legacy_flags_disabled && !all_legacy_flags_enabled) {
        throw std::invalid_argument(
                "deprecated propagation booleans must be either all false or all true");
      }
      propagation_model_name_ = all_legacy_flags_enabled ? "fast" : "legacy";
      RCLCPP_WARN(
        node_.get_logger(),
        "deprecated propagation booleans selected propagation_model='%s'; set the string "
        "parameter explicitly", propagation_model_name_.c_str());
    }
    core::EKFEstimator::PropagationModel propagation_model;
    if (propagation_model_name_ == "legacy") {
      if (!all_legacy_flags_disabled) {
        throw std::invalid_argument(
                "propagation_model='legacy' contradicts enabled deprecated propagation flags");
      }
      propagation_model = core::EKFEstimator::PropagationModel::kLegacy;
    } else if (propagation_model_name_ == "fast") {
      if (!all_legacy_flags_disabled && !all_legacy_flags_enabled) {
        throw std::invalid_argument(
                "propagation_model='fast' contradicts mixed deprecated propagation flags");
      }
      propagation_model = core::EKFEstimator::PropagationModel::kFast;
    } else if (propagation_model_name_ == "exact") {
      if (!all_legacy_flags_disabled) {
        throw std::invalid_argument(
                "propagation_model='exact' contradicts enabled deprecated propagation flags");
      }
      propagation_model = core::EKFEstimator::PropagationModel::kExact;
    } else {
      throw std::invalid_argument(
              "propagation_model must be one of: legacy, fast, exact");
    }
    configureEstimator(ekf_, propagation_model);
    configureEstimator(smoother_ekf_, propagation_model);
    if (enable_fixed_lag_smoothing_) {
      if (!(fixed_lag_duration_sec_ > 0.0) || !std::isfinite(fixed_lag_duration_sec_)) {
        throw std::invalid_argument("fixed_lag_duration_sec must be finite and positive");
      }
      if (!enable_measurement_replay_) {
        RCLCPP_WARN(
          node_.get_logger(),
          "enable_fixed_lag_smoothing requires enable_measurement_replay=true; "
          "smoother output is disabled without the in-order sensor-time measurement feed");
      }
      smoother_ = std::make_unique<core::FixedLagSmoother>(
        smoother_ekf_, fixed_lag_duration_sec_, max_future_measurement_wait_sec_,
        static_cast<std::size_t>(std::max(fixed_lag_node_subsample_, 1)));
    }
    var_gnss_ << var_gnss_xy_, var_gnss_xy_, var_gnss_z_;
    var_gnss_velocity_ << var_gnss_velocity_xy_, var_gnss_velocity_xy_, var_gnss_velocity_z_;
    var_odom_ << var_odom_xyz_, var_odom_xyz_, var_odom_xyz_;
    replay_ = std::make_unique<core::EskfReplay>(
      ekf_, measurement_history_duration_sec_, max_future_measurement_wait_sec_);

    // Setup Publisher
    const std::string output_pose_name = node_.get_name() + std::string("/current_pose");
    current_pose_pub_ =
      node_.create_publisher<geometry_msgs::msg::PoseStamped>(
      output_pose_name, rclcpp::QoS(output_qos_depth_));
    current_odometry_pub_ =
      node_.create_publisher<nav_msgs::msg::Odometry>(output_odometry_topic_, 10);
    if (enable_fixed_lag_smoothing_) {
      const std::string smoothed_pose_name = node_.get_name() + std::string("/smoothed_pose");
      smoothed_pose_pub_ =
        node_.create_publisher<geometry_msgs::msg::PoseStamped>(
        smoothed_pose_name, rclcpp::QoS(output_qos_depth_));
    }
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    }
    const std::string output_gyro_bias_name =
      node_.get_name() + std::string("/current_gyro_bias");
    current_gyro_bias_pub_ =
      node_.create_publisher<geometry_msgs::msg::Vector3Stamped>(output_gyro_bias_name, 10);
    const std::string output_accel_bias_name =
      node_.get_name() + std::string("/current_accel_bias");
    current_accel_bias_pub_ =
      node_.create_publisher<geometry_msgs::msg::Vector3Stamped>(output_accel_bias_name, 10);
    estimator_status_pub_ =
      node_.create_publisher<kalman_filter_localization_msgs::msg::EstimatorStatus>(
      node_.get_name() + std::string("/status"), 10);
    diagnostics_pub_ =
      node_.create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      node_.get_name() + std::string("/diagnostics"), 10);
    typed_measurement_quality_pub_ =
      node_.create_publisher<kalman_filter_localization_msgs::msg::MeasurementQuality>(
      node_.get_name() + std::string("/debug/measurement_quality_typed"), 50);
    typed_observability_pub_ =
      node_.create_publisher<kalman_filter_localization_msgs::msg::ObservabilityStatus>(
      node_.get_name() + std::string("/debug/observability_typed"), 50);
    typed_replay_timing_pub_ =
      node_.create_publisher<kalman_filter_localization_msgs::msg::ReplayTiming>(
      node_.get_name() + std::string("/debug/replay_timing_typed"), 50);
    if (publish_debug_topics_) {
      const std::string debug_prefix = node_.get_name() + std::string("/debug/");
      debug_gnss_position_pub_ =
        node_.create_publisher<std_msgs::msg::Float64MultiArray>(
        debug_prefix + std::string("gnss_position"), 10);
      debug_gnss_course_yaw_pub_ =
        node_.create_publisher<std_msgs::msg::Float64MultiArray>(
        debug_prefix + std::string("gnss_course_yaw"), 10);
      debug_update_delta_pub_ =
        node_.create_publisher<std_msgs::msg::Float64MultiArray>(
        debug_prefix + std::string("update_delta"), 10);
      debug_replay_timing_pub_ =
        node_.create_publisher<std_msgs::msg::Float64MultiArray>(
        debug_prefix + std::string("replay_timing"), 10);
      debug_initialization_pub_ =
        node_.create_publisher<std_msgs::msg::Float64MultiArray>(
        debug_prefix + std::string("initialization"), 10);
      debug_observability_pub_ =
        node_.create_publisher<std_msgs::msg::Float64MultiArray>(
        debug_prefix + std::string("observability"), 10);
      debug_measurement_quality_pub_ =
        node_.create_publisher<std_msgs::msg::Float64MultiArray>(
        debug_prefix + std::string("measurement_quality"), 10);
    }

    // Setup Subscriber
    auto process_initial_pose =
      [this](
      const geometry_msgs::msg::PoseStamped::SharedPtr msg,
      const bool use_message_covariance,
      const std::array<double, 3> & position_variance,
      const std::array<double, 3> & attitude_variance) -> void
      {
        RCLCPP_INFO(node_.get_logger(), "received initial pose");
        initial_pose_received_ = true;
        current_pose_ = *msg;

        core::EKFEstimator::State state;
        state.position = Eigen::Vector3d(
          current_pose_.pose.position.x,
          current_pose_.pose.position.y,
          current_pose_.pose.position.z);
        state.velocity = Eigen::Vector3d::Zero();
        state.orientation = Eigen::Quaterniond(
          current_pose_.pose.orientation.w,
          current_pose_.pose.orientation.x,
          current_pose_.pose.orientation.y,
          current_pose_.pose.orientation.z);
        ekf_.setState(state);
        if (smoother_) {
          smoother_ekf_.setState(state);
          smoother_->reset();
        }
        const bool covariance_applied = use_message_covariance ?
          applyInitialCovariance(
          Eigen::Vector3d(
            position_variance[0], position_variance[1], position_variance[2]),
          Eigen::Vector3d(
            attitude_variance[0], attitude_variance[1], attitude_variance[2])) :
          applyConfiguredInitialCovariance();
        if (!covariance_applied) {
          RCLCPP_WARN(node_.get_logger(), "initial pose covariance was rejected; using EKF state");
        }
        stationary_initializer_.reset();
        stationary_detector_.reset();
        if (vehicle_model_) {
          vehicle_model_->reset();
        }
        gnss_reacquisition_gate_.reset();
        stationary_initialization_complete_ = !enable_stationary_initialization_;
        yaw_initializer_.reset();
        (void)yaw_initializer_.offer(
          core::YawInitializer::Source::kExternalPose,
          getYawRadFromQuaternion(state.orientation.normalized()),
          std::max(
            use_message_covariance ? attitude_variance[2] : initial_attitude_variance_yaw_,
            1.0e-12),
          stampToSec(msg->header.stamp));
        if (std::isfinite(initial_dual_antenna_yaw_rad_)) {
          (void)yaw_initializer_.offer(
            core::YawInitializer::Source::kDualAntenna,
            initial_dual_antenna_yaw_rad_, initial_dual_antenna_yaw_variance_,
            stampToSec(msg->header.stamp));
        }

        // Reset IMU dt integration base on re-initialization.
        has_previous_time_imu_ = false;
        previous_time_imu_ = 0.0;
        replay_->reset();
        sensor_fault_monitor_.reset();

        // Reset odom baseline too.
        current_pose_odom_ = current_pose_;
        has_previous_odom_ = false;
        previous_odom_mat_ = Eigen::Matrix4d::Identity();

        // Reset GNSS-derived baselines on re-initialization.
        has_course_base_gnss_ = false;
        has_previous_velocity_gnss_ = false;
        has_latest_gnss_doppler_ = false;
        has_latest_gnss_ = false;
        latest_gnss_time_ = std::numeric_limits<double>::quiet_NaN();
        has_reacquisition_previous_position_ = false;
        has_stationary_start_time_ = false;
      };
    auto initial_pose_callback =
      [this, process_initial_pose](
      const geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void
      {
        ++received_initial_pose_count_;
        enqueueInput(
          msg->header.stamp, 0,
          [process_initial_pose, msg]() {
            process_initial_pose(msg, false, std::array<double, 3>{}, std::array<double, 3>{});
          });
      };
    auto initial_pose_covariance_callback =
      [this, process_initial_pose](
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) -> void
      {
        ++received_initial_pose_count_;
        enqueueInput(
          msg->header.stamp, 0,
          [this, process_initial_pose, msg]() {
            auto pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
            pose->header = msg->header;
            pose->pose = msg->pose.pose;
            const std::array<double, 3> position_variance = {
              positiveCovarianceOrFallback(
                msg->pose.covariance[0], initial_position_variance_xy_),
              positiveCovarianceOrFallback(
                msg->pose.covariance[7], initial_position_variance_xy_),
              positiveCovarianceOrFallback(
                msg->pose.covariance[14], initial_position_variance_z_)};
            const std::array<double, 3> attitude_variance = {
              positiveCovarianceOrFallback(
                msg->pose.covariance[21], initial_attitude_variance_rp_),
              positiveCovarianceOrFallback(
                msg->pose.covariance[28], initial_attitude_variance_rp_),
              positiveCovarianceOrFallback(
                msg->pose.covariance[35], initial_attitude_variance_yaw_)};
            process_initial_pose(pose, true, position_variance, attitude_variance);
          });
      };

    auto process_imu =
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) -> void
      {
        if (!initial_pose_received_) {
          return;
        }
        const double sensor_time = stampToSec(msg->header.stamp);
        if (!sensorInputAllowed(core::SensorFaultMonitor::Sensor::kImu, sensor_time)) {
          return;
        }
        if (!std::isfinite(msg->angular_velocity.x) ||
          !std::isfinite(msg->angular_velocity.y) ||
          !std::isfinite(msg->angular_velocity.z) ||
          !std::isfinite(msg->linear_acceleration.x) ||
          !std::isfinite(msg->linear_acceleration.y) ||
          !std::isfinite(msg->linear_acceleration.z))
        {
          recordSensorOutcome(core::SensorFaultMonitor::Sensor::kImu, false, sensor_time);
          RCLCPP_WARN_THROTTLE(
            node_.get_logger(), clock_, 5000, "skip IMU sample with non-finite data");
          return;
        }
        sensor_msgs::msg::Imu transformed_msg;
        try {
          geometry_msgs::msg::Vector3Stamped acc_in;
          geometry_msgs::msg::Vector3Stamped acc_out;
          geometry_msgs::msg::Vector3Stamped w_in;
          geometry_msgs::msg::Vector3Stamped w_out;
          acc_in.vector.x = msg->linear_acceleration.x;
          acc_in.vector.y = msg->linear_acceleration.y;
          acc_in.vector.z = msg->linear_acceleration.z;
          w_in.vector.x = msg->angular_velocity.x;
          w_in.vector.y = msg->angular_velocity.y;
          w_in.vector.z = msg->angular_velocity.z;
          const tf2::TimePoint time_point = tf2::TimePoint(
            std::chrono::seconds(msg->header.stamp.sec) +
            std::chrono::nanoseconds(msg->header.stamp.nanosec));
          const geometry_msgs::msg::TransformStamped transform =
            tfbuffer_.lookupTransform(
            robot_frame_id_,
            msg->header.frame_id,
            time_point);
          tf2::doTransform(acc_in, acc_out, transform);
          tf2::doTransform(w_in, w_out, transform);
          transformed_msg.header.stamp = msg->header.stamp;
          transformed_msg.header.frame_id = robot_frame_id_;
          transformed_msg.angular_velocity.x = w_out.vector.x;
          transformed_msg.angular_velocity.y = w_out.vector.y;
          transformed_msg.angular_velocity.z = w_out.vector.z;
          transformed_msg.linear_acceleration.x = acc_out.vector.x;
          transformed_msg.linear_acceleration.y = acc_out.vector.y;
          transformed_msg.linear_acceleration.z = acc_out.vector.z;
          if (use_imu_orientation_) {
            // Transform orientation from msg frame to robot_frame_id_.
            // Assumes Imu::orientation represents the orientation of the sensor/body frame in the
            // world frame (world <- body).
            const Eigen::Quaterniond q_world_src(
              msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
            const Eigen::Quaterniond q_robot_src(
              transform.transform.rotation.w,
              transform.transform.rotation.x,
              transform.transform.rotation.y,
              transform.transform.rotation.z);
            const Eigen::Quaterniond q_world_robot =
              (q_world_src * q_robot_src.conjugate()).normalized();
            transformed_msg.orientation.x = q_world_robot.x();
            transformed_msg.orientation.y = q_world_robot.y();
            transformed_msg.orientation.z = q_world_robot.z();
            transformed_msg.orientation.w = q_world_robot.w();
            transformed_msg.orientation_covariance = msg->orientation_covariance;
          }
          const auto callback_start = std::chrono::steady_clock::now();
          const double sensor_time = stampToSec(transformed_msg.header.stamp);
          if (!has_imu_wall_start_) {
            imu_wall_start_ = callback_start;
            has_imu_wall_start_ = true;
          }
          if (!has_imu_sensor_start_) {
            first_imu_sensor_time_ = sensor_time;
            has_imu_sensor_start_ = true;
          }
          last_imu_sensor_time_ = sensor_time;
          const bool prediction_ok = predictUpdate(transformed_msg);
          recordSensorOutcome(
            core::SensorFaultMonitor::Sensor::kImu, prediction_ok, sensor_time);
          const double callback_us = std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - callback_start).count();
          imu_callback_total_us_ += callback_us;
          imu_callback_max_us_ = std::max(imu_callback_max_us_, callback_us);
          ++imu_callback_count_;
          if (output_publish_mode_ == "imu") {
            broadcastPose();
          }
        } catch (tf2::TransformException & e) {
          recordSensorOutcome(core::SensorFaultMonitor::Sensor::kImu, false, sensor_time);
          RCLCPP_ERROR(node_.get_logger(), "%s", e.what());
          return;
        } catch (std::runtime_error & e) {
          recordSensorOutcome(core::SensorFaultMonitor::Sensor::kImu, false, sensor_time);
          RCLCPP_ERROR(node_.get_logger(), "%s", e.what());
          return;
        }
      };
    auto imu_callback =
      [this, process_imu](const sensor_msgs::msg::Imu::SharedPtr msg) -> void
      {
        ++received_imu_count_;
        enqueueInput(msg->header.stamp, 10, [process_imu, msg]() {process_imu(msg);});
      };

    auto process_odom =
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) -> void
      {
        if (!initial_pose_received_ || !use_odom_) {
          return;
        }
        const double sensor_time = stampToSec(msg->header.stamp);
        if (!sensorInputAllowed(core::SensorFaultMonitor::Sensor::kOdom, sensor_time)) {
          return;
        }
        const auto & position = msg->pose.pose.position;
        const auto & orientation = msg->pose.pose.orientation;
        if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
          !std::isfinite(position.z) || !std::isfinite(orientation.x) ||
          !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
          !std::isfinite(orientation.w))
        {
          recordSensorOutcome(core::SensorFaultMonitor::Sensor::kOdom, false, sensor_time);
          RCLCPP_WARN_THROTTLE(
            node_.get_logger(), clock_, 5000, "skip odometry sample with non-finite pose");
          return;
        }
        if (estimate_wheel_speed_scale_factor_ && wheel_scale_reference_ != "gnss" &&
          std::isfinite(msg->twist.twist.linear.x))
        {
          updateWheelSpeedScaleFactor(
            std::fabs(msg->twist.twist.linear.x), sensor_time);
        }
        if (odom_input_mode_ == "absolute") {
          geometry_msgs::msg::PoseStamped pose;
          pose.header = msg->header;
          pose.pose = msg->pose.pose;
          const Eigen::Matrix3d covariance = odometryPositionCovariance(
            msg->pose.covariance, var_odom_);
          const Eigen::Vector3d position(
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
          if (enable_measurement_replay_) {
            (void)applyReplayMeasurement(
              msg->header.stamp, 4U,
              [position, covariance](core::EKFEstimator & estimator) {
                return estimator.observationUpdatePositionWithCovariance(
                  position, covariance).status;
              });
          } else {
            measurementUpdate(pose, covariance, false, Eigen::Vector3d::Zero());
          }
          recordSensorOutcome(core::SensorFaultMonitor::Sensor::kOdom, true, sensor_time);
          return;
        }
        Eigen::Affine3d affine;
        tf2::fromMsg(msg->pose.pose, affine);
        const Eigen::Matrix4d odom_mat = affine.matrix();
        if (!has_previous_odom_) {
          current_pose_odom_ = current_pose_;
          previous_odom_mat_ = odom_mat;
          has_previous_odom_ = true;
          recordSensorOutcome(core::SensorFaultMonitor::Sensor::kOdom, true, sensor_time);
          return;
        }

        Eigen::Affine3d current_affine;
        tf2::fromMsg(current_pose_odom_.pose, current_affine);
        const Eigen::Matrix4d current_global_mat = current_affine.matrix();
        const Eigen::Matrix4d current_trans =
          core::composePoseWithRelativeOdom(current_global_mat, previous_odom_mat_, odom_mat);

        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;
        pose.pose.position.x = current_trans(0, 3);
        pose.pose.position.y = current_trans(1, 3);
        pose.pose.position.z = current_trans(2, 3);
        if (enable_measurement_replay_) {
          const Eigen::Vector3d position(
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
          (void)applyReplayMeasurement(
            msg->header.stamp, 4U,
            [position, this](core::EKFEstimator & estimator) {
              return estimator.observationUpdateWithStatus(position, var_odom_);
            });
        } else {
          measurementUpdate(pose, var_odom_);
        }

        current_pose_odom_ = current_pose_;
        previous_odom_mat_ = odom_mat;
        recordSensorOutcome(core::SensorFaultMonitor::Sensor::kOdom, true, sensor_time);
      };
    auto odom_callback =
      [this, process_odom](const nav_msgs::msg::Odometry::SharedPtr msg) -> void
      {
        ++received_odom_count_;
        enqueueInput(msg->header.stamp, 50, [process_odom, msg]() {process_odom(msg);});
      };

    auto process_gnss_pose =
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void
      {
        handleGnssPose(*msg, var_gnss_.asDiagonal());
      };
    auto gnss_pose_callback =
      [this, process_gnss_pose](
      const geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void
      {
        ++received_gnss_pose_count_;
        enqueueInput(
          msg->header.stamp, 20,
          [process_gnss_pose, msg]() {process_gnss_pose(msg);});
      };

    auto process_gnss_navsatfix =
      [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) -> void
      {
        if (!initial_pose_received_ || !use_gnss_) {
          return;
        }
        geometry_msgs::msg::PoseStamped pose_msg;
        if (convertNavSatFixToPose(*msg, pose_msg)) {
          handleGnssPose(pose_msg, getNavSatFixCovariance(*msg));
        }
      };
    auto gnss_navsatfix_callback =
      [this, process_gnss_navsatfix](
      const sensor_msgs::msg::NavSatFix::SharedPtr msg) -> void
      {
        ++received_gnss_navsatfix_count_;
        enqueueInput(
          msg->header.stamp, 20,
          [process_gnss_navsatfix, msg]() {process_gnss_navsatfix(msg);});
      };

    auto process_gnss_doppler_velocity =
      [this](const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) -> void
      {
        if (!initial_pose_received_ || !use_gnss_ || !use_gnss_doppler_velocity_) {
          return;
        }
        const double sensor_time = stampToSec(msg->header.stamp);
        if (!sensorInputAllowed(core::SensorFaultMonitor::Sensor::kGnss, sensor_time)) {
          return;
        }
        if (!msg->header.frame_id.empty() && msg->header.frame_id != reference_frame_id_) {
          RCLCPP_WARN_THROTTLE(
            node_.get_logger(), clock_, 5000,
            "skip GNSS Doppler velocity in frame '%s'; expected '%s'",
            msg->header.frame_id.c_str(), reference_frame_id_.c_str());
          return;
        }
        const Eigen::Vector3d velocity(
          msg->twist.twist.linear.x,
          msg->twist.twist.linear.y,
          msg->twist.twist.linear.z);
        latest_gnss_doppler_time_ = stampToSec(msg->header.stamp);
        has_latest_gnss_doppler_ = std::isfinite(latest_gnss_doppler_time_);
        latest_gnss_time_ = latest_gnss_doppler_time_ + gnss_time_offset_sec_;
        has_latest_gnss_ = std::isfinite(latest_gnss_time_);
        Eigen::Matrix3d velocity_covariance = var_gnss_velocity_.asDiagonal();
        if (use_gnss_doppler_velocity_covariance_) {
          Eigen::Matrix3d candidate;
          for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
              candidate(row, column) = msg->twist.covariance[6 * row + column];
            }
          }
          candidate = 0.5 * (candidate + candidate.transpose());
          const Eigen::LDLT<Eigen::Matrix3d> decomposition(candidate);
          if (candidate.allFinite() && decomposition.info() == Eigen::Success &&
            decomposition.isPositive() && (candidate.diagonal().array() > 0.0).all())
          {
            velocity_covariance = candidate;
          } else {
            RCLCPP_WARN_THROTTLE(
              node_.get_logger(), clock_, 5000,
              "invalid GNSS Doppler velocity covariance; using configured variances");
          }
        }
        const Eigen::Vector3d variance = velocity_covariance.diagonal();
        const double horizontal_speed = velocity.head<2>().norm();
        if (horizontal_speed >= min_gnss_doppler_course_speed_mps_) {
          const double yaw_variance = std::max(
            (variance.x() + variance.y()) /
            (horizontal_speed * horizontal_speed), 1.0e-12);
          (void)yaw_initializer_.offer(
            core::YawInitializer::Source::kDopplerOrCourse,
            std::atan2(velocity.y(), velocity.x()), yaw_variance,
            stampToSec(msg->header.stamp));
        }
        if (enable_measurement_replay_) {
          (void)applyReplayMeasurement(
            msg->header.stamp, 2U,
            [this, velocity, velocity_covariance, variance](core::EKFEstimator & estimator) {
              return propagate_gnss_velocity_cross_state_ ?
                     estimator.observationUpdateWorldVelocityWithCovariance(
                velocity, velocity_covariance).status :
                     estimator.observationUpdateVelocityWithStatus(velocity, variance, false);
            });
          return;
        }
        updateVelocityMeasurement(
          velocity, velocity_covariance, "GNSS Doppler", msg->header.stamp);
        if (use_gnss_doppler_course_yaw_) {
          updateYawFromDopplerVelocity(velocity, velocity_covariance, msg->header.stamp);
        }
      };
    auto gnss_doppler_velocity_callback =
      [this, process_gnss_doppler_velocity](
      const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) -> void
      {
        ++received_gnss_doppler_count_;
        enqueueInput(
          msg->header.stamp, 30,
          [process_gnss_doppler_velocity, msg]() {process_gnss_doppler_velocity(msg);});
      };

    auto process_wheel_speed =
      [this](const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) -> void
      {
        if (!initial_pose_received_ || !use_wheel_speed_) {
          return;
        }
        const double sensor_time = stampToSec(msg->header.stamp);
        if (!sensorInputAllowed(core::SensorFaultMonitor::Sensor::kWheel, sensor_time)) {
          return;
        }
        const double speed = msg->twist.twist.linear.x * wheel_speed_scale_factor_;
        if (!std::isfinite(speed)) {
          recordSensorOutcome(core::SensorFaultMonitor::Sensor::kWheel, false, sensor_time);
          RCLCPP_WARN_THROTTLE(
            node_.get_logger(), clock_, 5000, "skip non-finite wheel speed");
          return;
        }
        latest_raw_wheel_speed_mps_ = msg->twist.twist.linear.x;
        latest_wheel_speed_time_ =
          msg->header.stamp.sec + msg->header.stamp.nanosec * 1.0e-9;
        has_latest_wheel_speed_ = std::isfinite(latest_raw_wheel_speed_mps_) &&
          std::isfinite(latest_wheel_speed_time_);
        const bool gnss_outage = has_previous_gnss_position_time_ &&
          latest_wheel_speed_time_ - previous_gnss_position_time_ >
          wheel_speed_gnss_outage_threshold_sec_;
        const bool use_wheel_nhc = wheel_speed_use_nonholonomic_constraints_ &&
          (!wheel_speed_nhc_only_during_gnss_outage_ || gnss_outage);
        const double wheel_vertical_variance = var_wheel_vertical_velocity_ *
          (gnss_outage ? 1.0 : wheel_vertical_nhc_gnss_available_variance_scale_);
        if (enable_measurement_replay_) {
          const auto replay_status = applyReplayMeasurement(
            msg->header.stamp, 3U,
            [this, speed, use_wheel_nhc, wheel_vertical_variance](
              core::EKFEstimator & estimator)
            {
              const Eigen::Vector3d predicted_body_velocity =
              estimator.getOrientation().toRotationMatrix().transpose() *
              estimator.getVelocity();
              if (max_wheel_speed_innovation_mps_ > 0.0 &&
              std::fabs(speed - predicted_body_velocity.x()) >
              max_wheel_speed_innovation_mps_)
              {
                return core::EKFEstimator::ObservationUpdateStatus::kInvalidMeasurement;
              }
              if (!use_wheel_nhc) {
                return estimator.observationUpdateBodyForwardSpeedWithStatus(
                  speed, var_wheel_speed_, wheel_speed_propagate_cross_state_);
              }
              return estimator.observationUpdateBodyVelocityWithStatus(
                Eigen::Vector3d(speed, 0.0, 0.0),
                Eigen::Vector3d(
                  var_wheel_speed_, var_wheel_lateral_velocity_,
                  wheel_vertical_variance));
            });
          recordSensorOutcome(
            core::SensorFaultMonitor::Sensor::kWheel,
            replay_status == core::EskfReplay::Status::kApplied ||
            replay_status == core::EskfReplay::Status::kQueuedFuture,
            sensor_time);
          return;
        }
        const Eigen::Vector3d predicted_body_velocity =
          ekf_.getOrientation().toRotationMatrix().transpose() * ekf_.getVelocity();
        if (max_wheel_speed_innovation_mps_ > 0.0 &&
          std::fabs(speed - predicted_body_velocity.x()) > max_wheel_speed_innovation_mps_)
        {
          recordSensorOutcome(core::SensorFaultMonitor::Sensor::kWheel, false, sensor_time);
          RCLCPP_WARN_THROTTLE(
            node_.get_logger(), clock_, 5000,
            "skip wheel-speed innovation: measured=%f predicted=%f", speed,
            predicted_body_velocity.x());
          return;
        }
        auto status = core::EKFEstimator::ObservationUpdateStatus::kInvalidMeasurement;
        if (!use_wheel_nhc) {
          status = ekf_.observationUpdateBodyForwardSpeedWithStatus(
            speed, var_wheel_speed_, wheel_speed_propagate_cross_state_);
        } else {
          status = ekf_.observationUpdateBodyVelocityWithStatus(
            Eigen::Vector3d(speed, 0.0, 0.0),
            Eigen::Vector3d(
              var_wheel_speed_, var_wheel_lateral_velocity_, wheel_vertical_variance));
        }
        if (status != core::EKFEstimator::ObservationUpdateStatus::kUpdated) {
          recordSensorOutcome(core::SensorFaultMonitor::Sensor::kWheel, false, sensor_time);
          RCLCPP_WARN_THROTTLE(
            node_.get_logger(), clock_, 5000, "skip invalid wheel-speed update");
        } else {
          recordSensorOutcome(core::SensorFaultMonitor::Sensor::kWheel, true, sensor_time);
        }
      };
    auto wheel_speed_callback =
      [this, process_wheel_speed](
      const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) -> void
      {
        ++received_wheel_count_;
        enqueueInput(
          msg->header.stamp, 40,
          [process_wheel_speed, msg]() {process_wheel_speed(msg);});
      };

    sub_initial_pose_ =
      node_.create_subscription<geometry_msgs::msg::PoseStamped>(
      initial_pose_topic_, rclcpp::QoS(input_qos_depth_),
      initial_pose_callback);
    if (!initial_pose_covariance_topic_.empty()) {
      sub_initial_pose_covariance_ =
        node_.create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        initial_pose_covariance_topic_, rclcpp::QoS(input_qos_depth_),
        initial_pose_covariance_callback);
      RCLCPP_INFO(
        node_.get_logger(),
        "initial pose covariance input: '%s'",
        initial_pose_covariance_topic_.c_str());
    }
    rclcpp::SensorDataQoS imu_qos;
    imu_qos.keep_last(input_qos_depth_);
    sub_imu_ =
      node_.create_subscription<sensor_msgs::msg::Imu>(imu_topic_, imu_qos, imu_callback);
    sub_odom_ =
      node_.create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(input_qos_depth_),
      odom_callback);
    if (gnss_input_type_ == "navsatfix") {
      rclcpp::SensorDataQoS gnss_qos;
      gnss_qos.keep_last(input_qos_depth_);
      sub_gnss_navsatfix_ =
        node_.create_subscription<sensor_msgs::msg::NavSatFix>(
        gnss_navsatfix_topic_, gnss_qos,
        gnss_navsatfix_callback);
      RCLCPP_INFO(
        node_.get_logger(),
        "GNSS input: NavSatFix '%s' -> ENU PoseStamped in frame '%s'",
        gnss_navsatfix_topic_.c_str(), reference_frame_id_.c_str());
    } else {
      sub_gnss_pose_ =
        node_.create_subscription<geometry_msgs::msg::PoseStamped>(
        gnss_pose_topic_, rclcpp::QoS(input_qos_depth_),
        gnss_pose_callback);
    }
    if (use_gnss_doppler_velocity_) {
      rclcpp::SensorDataQoS velocity_qos;
      velocity_qos.keep_last(input_qos_depth_);
      sub_gnss_doppler_velocity_ =
        node_.create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        gnss_doppler_velocity_topic_, velocity_qos, gnss_doppler_velocity_callback);
      RCLCPP_INFO(
        node_.get_logger(), "GNSS Doppler velocity input: '%s' in frame '%s'",
        gnss_doppler_velocity_topic_.c_str(), reference_frame_id_.c_str());
    }
    if (use_wheel_speed_) {
      rclcpp::SensorDataQoS wheel_qos;
      wheel_qos.keep_last(input_qos_depth_);
      sub_wheel_speed_ =
        node_.create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        wheel_speed_topic_, wheel_qos, wheel_speed_callback);
      RCLCPP_INFO(
        node_.get_logger(), "wheel-speed input: '%s'", wheel_speed_topic_.c_str());
    }
    drain_input_buffer_service_ = node_.create_service<std_srvs::srv::Trigger>(
      node_.get_name() + std::string("/drain_input_buffer"),
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        (void)request;
        drainBufferedInputs(true);
        response->success = input_reorder_queue_.empty() && reorder_late_input_count_ == 0;
        response->message =
        std::string("processed=") + std::to_string(reorder_processed_input_count_) +
        std::string(" late=") + std::to_string(reorder_late_input_count_) +
        std::string(" buffered=") + std::to_string(input_reorder_queue_.size());
      });
    if (output_publish_mode_ == "timer") {
      const std::chrono::milliseconds period(pub_period_);
      timer_ = node_.create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() {broadcastPose();});
    }
  }

  core::EskfReplay::MeasurementFunction makeImuReplayCorrection(
    const sensor_msgs::msg::Imu & imu_msg, const double sensor_time)
  {
    return makeImuCorrection(imu_msg, sensor_time, stationary_detector_, *vehicle_model_);
  }

  // Builds a per-IMU correction function (orientation / flat-ground / NHC /
  // ZUPT / ZIHR / bias-observability gating) against a caller-provided
  // stationary detector and vehicle model. The main replay track uses the
  // node's shared detector/model; the fixed-lag smoothing track uses its own
  // private instances so the two forward runs stay independent.
  core::EskfReplay::MeasurementFunction makeImuCorrection(
    const sensor_msgs::msg::Imu & imu_msg, const double sensor_time,
    core::StationaryDetector & stationary_detector,
    core::VehicleModel & vehicle_model)
  {
    const auto plan = std::make_shared<ImuReplayPlan>();
    return [this, imu_msg, sensor_time, plan, &stationary_detector,
             &vehicle_model](core::EKFEstimator & estimator) {
             using UpdateStatus = core::EKFEstimator::ObservationUpdateStatus;
             UpdateStatus status = UpdateStatus::kUpdated;
             const Eigen::Vector3d angular_velocity(
               imu_msg.angular_velocity.x, imu_msg.angular_velocity.y,
               imu_msg.angular_velocity.z);
             const Eigen::Vector3d acceleration(
               imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y,
               imu_msg.linear_acceleration.z);
             if (!plan->initialized) {
               const auto stationary_state = stationary_detector.update(
                 sensor_time, angular_velocity, acceleration,
                 estimator.getVelocity().norm(), gravity_mps2_);
               plan->apply_zupt = use_zupt_ &&
                 stationary_state == core::StationaryDetector::State::kStationary;
               plan->apply_zihr = use_zihr_ &&
                 stationary_state == core::StationaryDetector::State::kStationary;
               const Eigen::Vector3d body_velocity =
                 estimator.getOrientation().normalized().toRotationMatrix().transpose() *
                 estimator.getVelocity();
               core::VehicleModelInput vehicle_model_input;
               vehicle_model_input.time_sec = sensor_time;
               vehicle_model_input.body_velocity = body_velocity;
               vehicle_model_input.yaw_rate_radps = imu_msg.angular_velocity.z;
               vehicle_model_input.lateral_acceleration_mps2 = imu_msg.linear_acceleration.y;
               const auto vehicle_model_output = vehicle_model.evaluate(vehicle_model_input);
               plan->apply_nhc = vehicle_model_output.valid &&
                 vehicle_model_output.apply_nonholonomic_constraint &&
                 std::isfinite(vehicle_model_output.nhc_variance_scale) &&
                 vehicle_model_output.nhc_variance_scale >= 1.0;
               plan->constrain_nhc_vertical = vehicle_model_output.constrain_vertical_velocity;
               plan->nhc_variance_scale = plan->apply_nhc ? std::min(
                 max_nhc_variance_scale_, vehicle_model_output.nhc_variance_scale) :
                 max_nhc_variance_scale_;
               if (enable_bias_observability_gate_) {
                 const auto bias_decision = core::evaluateBiasObservability(
                   stationary_state == core::StationaryDetector::State::kStationary,
                   angular_velocity.norm(), acceleration.head<2>().norm(),
                   bias_min_angular_excitation_radps_,
                   bias_min_acceleration_excitation_mps2_);
                 plan->learn_gyro_bias = bias_decision.learn_gyro_bias;
                 plan->learn_accel_bias = bias_decision.learn_accel_bias;
               }
               plan->initialized = true;
             }
             estimator.setBiasLearningEnabled(
               plan->learn_gyro_bias, plan->learn_accel_bias);
             if (use_imu_orientation_) {
               const Eigen::Quaterniond measurement(
                 imu_msg.orientation.w, imu_msg.orientation.x,
                 imu_msg.orientation.y, imu_msg.orientation.z);
               Eigen::Vector3d variance = Eigen::Vector3d::Constant(var_imu_orientation_rpy_);
               if (use_imu_orientation_covariance_ && imu_msg.orientation_covariance[0] >= 0.0) {
                 const Eigen::Vector3d candidate(
                   imu_msg.orientation_covariance[0], imu_msg.orientation_covariance[4],
                   imu_msg.orientation_covariance[8]);
                 if (candidate.allFinite() && (candidate.array() > 0.0).all()) {
                   variance = candidate;
                 }
               }
               status = estimator.observationUpdateOrientationWithStatus(measurement, variance);
               if (status != UpdateStatus::kUpdated) {
                 return status;
               }
             } else if (use_flat_ground_) {
               const Eigen::Quaterniond estimate = estimator.getOrientation().normalized();
               const double yaw = getYawRadFromQuaternion(estimate);
               const Eigen::Quaterniond level(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
               status = estimator.observationUpdateOrientationWithStatus(
                 level, Eigen::Vector3d(var_flat_ground_rp_, var_flat_ground_rp_, 1.0e6));
               if (status != UpdateStatus::kUpdated) {
                 return status;
               }
             }

             if (use_nonholonomic_constraint_ && plan->apply_nhc) {
               const Eigen::Vector3d body_velocity =
                 estimator.getOrientation().normalized().toRotationMatrix().transpose() *
                 estimator.getVelocity();
               if (body_velocity.allFinite() &&
                 std::fabs(body_velocity.x()) >= min_nhc_forward_speed_mps_)
               {
                 status = estimator.observationUpdateBodyVelocityConstraintWithStatus(
                   Eigen::Vector2d::Zero(),
                   Eigen::Vector2d(
                     var_nhc_lateral_velocity_ * plan->nhc_variance_scale,
                     plan->constrain_nhc_vertical ?
                     var_nhc_vertical_velocity_ * plan->nhc_variance_scale : 1.0e6));
                 if (status != UpdateStatus::kUpdated) {
                   return status;
                 }
               }
             }

             if (plan->apply_zupt) {
               status = estimator.observationUpdateVelocityWithStatus(
                 Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(var_zupt_velocity_), true);
               if (status != UpdateStatus::kUpdated) {
                 return status;
               }
             }
             if (plan->apply_zihr) {
               status = estimator.observationUpdateGyroBiasWithStatus(
                 angular_velocity, Eigen::Vector3d::Constant(var_zihr_gyro_));
             }
             return status;
           };
  }

  void feedSmootherMeasurement(
    const double sensor_time, const std::uint64_t measurement_id,
    const core::EskfReplay::Status replay_status,
    const core::EskfReplay::MeasurementFunction & update)
  {
    if (!smoother_ || !enable_fixed_lag_smoothing_) {
      return;
    }
    if (replay_status != core::EskfReplay::Status::kApplied &&
      replay_status != core::EskfReplay::Status::kQueuedFuture)
    {
      return;
    }
    (void)smoother_->applyMeasurement(sensor_time, measurement_id, update);
  }

  core::EskfReplay::Status applyReplayMeasurement(
    const builtin_interfaces::msg::Time & stamp, const std::uint8_t source,
    core::EskfReplay::MeasurementFunction update, const double time_offset_sec = 0.0)
  {
    const double sensor_time = stampToSec(stamp) + time_offset_sec;
    const double arrival_time = node_.now().seconds();
    const std::uint64_t stamp_bits = static_cast<std::uint64_t>(stampToNanoseconds(stamp));
    const std::uint64_t measurement_id =
      (static_cast<std::uint64_t>(source) << 56U) ^ stamp_bits;
    const auto smoother_update = update;
    const auto status = replay_->applyMeasurement(
      sensor_time, arrival_time, measurement_id, std::move(update));
    feedSmootherMeasurement(sensor_time, measurement_id, status, smoother_update);
    if (debug_replay_timing_pub_) {
      const auto & trace = replay_->lastTimingTrace();
      std_msgs::msg::Float64MultiArray message;
      message.data = {
        static_cast<double>(source), trace.sensor_time, trace.arrival_time,
        trace.filter_time_before, trace.apply_time, static_cast<double>(status)};
      debug_replay_timing_pub_->publish(message);
    }
    publishTypedReplayTiming(source, status, replay_->lastTimingTrace());
    if (status != core::EskfReplay::Status::kApplied &&
      status != core::EskfReplay::Status::kQueuedFuture)
    {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), clock_, 5000,
        "measurement replay rejected source=%u sensor=%.9f filter=%.9f status=%d",
        static_cast<unsigned int>(source), sensor_time, replay_->latestTime(),
        static_cast<int>(status));
    }
    return status;
  }

  bool predictUpdate(const sensor_msgs::msg::Imu & imu_msg)
  {
    has_received_input_ = true;
    current_stamp_ = imu_msg.header.stamp;
    latest_imu_stamp_ = imu_msg.header.stamp;
    has_latest_imu_stamp_ = true;

    const double current_time_imu = imu_msg.header.stamp.sec +
      imu_msg.header.stamp.nanosec * 1e-9;

    const Eigen::Vector3d gyro(
      imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z);
    const Eigen::Vector3d linear_acceleration(
      imu_msg.linear_acceleration.x,
      imu_msg.linear_acceleration.y,
      imu_msg.linear_acceleration.z);
    latest_abs_yaw_rate_radps_ = std::fabs(gyro.z());

    if (enable_stationary_initialization_ && !stationary_initialization_complete_) {
      const auto initialization_status = stationary_initializer_.addSample(
        current_time_imu, gyro, linear_acceleration);
      const auto & result = stationary_initializer_.result();
      const bool invalid_initialization_sample =
        initialization_status == core::ImuStationaryInitializer::Status::kInvalidSample ||
        initialization_status == core::ImuStationaryInitializer::Status::kReverseTime;
      if (debug_initialization_pub_) {
        std_msgs::msg::Float64MultiArray message;
        message.data = {
          static_cast<double>(initialization_status), result.confidence,
          result.gyro_std_radps, result.accel_std_mps2, result.accel_norm_error_mps2,
          static_cast<double>(result.sample_count), result.gyro_bias.x(),
          result.gyro_bias.y(), result.gyro_bias.z()};
        debug_initialization_pub_->publish(message);
      }
      if (initialization_status == core::ImuStationaryInitializer::Status::kInitialized) {
        core::EKFEstimator::State state = ekf_.getState();
        const auto & yaw_result = yaw_initializer_.result();
        const double yaw = yaw_result.source == core::YawInitializer::Source::kNone ? 0.0 :
          yaw_result.yaw_rad;
        state.orientation = (
          Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())) *
          result.roll_pitch_orientation).normalized();
        state.velocity.setZero();
        state.gyro_bias = result.gyro_bias;
        ekf_.setState(state);
        if (smoother_) {
          smoother_ekf_.setState(state);
          smoother_->reset();
        }
        has_previous_time_imu_ = false;
        previous_time_imu_ = 0.0;
        replay_->reset();
        stationary_initialization_complete_ = true;
        RCLCPP_INFO(
          node_.get_logger(),
          "stationary initialization complete: samples=%zu confidence=%.3f "
          "gyro_bias=[%.6f %.6f %.6f] yaw_source=%d",
          result.sample_count, result.confidence, result.gyro_bias.x(), result.gyro_bias.y(),
          result.gyro_bias.z(), static_cast<int>(yaw_result.source));
      } else if (initialization_status == core::ImuStationaryInitializer::Status::kMoving) {
        RCLCPP_WARN_THROTTLE(
          node_.get_logger(), clock_, 5000,
          "stationary initialization waiting: moving gyro_std=%.6f accel_std=%.6f "
          "gravity_error=%.6f confidence=%.3f",
          result.gyro_std_radps, result.accel_std_mps2,
          result.accel_norm_error_mps2, result.confidence);
      } else if (invalid_initialization_sample) {
        RCLCPP_ERROR_THROTTLE(
          node_.get_logger(), clock_, 5000,
          "stationary initialization rejected IMU sample: status=%d",
          static_cast<int>(initialization_status));
      }
      return !invalid_initialization_sample;
    }

    if (enable_measurement_replay_) {
      const bool first_sample = replay_->imuHistorySize() == 0U;
      const auto status = replay_->addImu(
        current_time_imu, gyro, linear_acceleration,
        first_sample ? core::EskfReplay::MeasurementFunction{} :
        makeImuReplayCorrection(imu_msg, current_time_imu));
      if (smoother_ && enable_fixed_lag_smoothing_) {
        const bool smoother_first = smoother_->counters().imu_samples == 0U;
        const auto smoother_status = smoother_->addImu(
          current_time_imu, gyro, linear_acceleration,
          smoother_first ? core::EskfReplay::MeasurementFunction{} :
          makeImuCorrection(
            imu_msg, current_time_imu, smoother_stationary_detector_,
            *smoother_vehicle_model_));
        if (smoother_status != core::FixedLagSmoother::Status::kApplied &&
          smoother_status != core::FixedLagSmoother::Status::kInitialized &&
          smoother_status != core::FixedLagSmoother::Status::kEmitted)
        {
          RCLCPP_WARN_THROTTLE(
            node_.get_logger(), clock_, 5000,
            "smoother IMU rejected: status=%d imu_samples=%llu time=%.9f latest=%.9f "
            "max_dt=%f",
            static_cast<int>(smoother_status),
            static_cast<std::uint64_t>(smoother_->counters().imu_samples),
            current_time_imu, smoother_->latestTime(),
            smoother_ekf_.getMaxPredictionDtSec());
        }
      }
      if (status == core::EskfReplay::Status::kInitialized ||
        status == core::EskfReplay::Status::kApplied)
      {
        previous_time_imu_ = current_time_imu;
        has_previous_time_imu_ = true;
        return true;
      }
      if (status == core::EskfReplay::Status::kReverseImu) {
        RCLCPP_WARN_THROTTLE(
          node_.get_logger(), clock_, 5000,
          "skip duplicate or reverse IMU stamp in replay history");
      } else {
        RCLCPP_ERROR_THROTTLE(
          node_.get_logger(), clock_, 5000,
          "skip IMU update rejected by replay engine: status=%d", static_cast<int>(status));
      }
      return false;
    }

    if (!has_previous_time_imu_) {
      previous_time_imu_ = current_time_imu;
      has_previous_time_imu_ = true;
      (void)ekf_.primeImuMeasurement(
        Eigen::Vector3d(
          imu_msg.angular_velocity.x,
          imu_msg.angular_velocity.y,
          imu_msg.angular_velocity.z),
        Eigen::Vector3d(
          imu_msg.linear_acceleration.x,
          imu_msg.linear_acceleration.y,
          imu_msg.linear_acceleration.z));
      return true;
    }
    const double dt_imu = current_time_imu - previous_time_imu_;
    // Always advance the time base to allow recovery after large/invalid dt.
    previous_time_imu_ = current_time_imu;

    const auto status = ekf_.predictionUpdateDt(dt_imu, gyro, linear_acceleration);
    if (status == core::EKFEstimator::PredictionUpdateStatus::kNonPositiveDt) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), clock_, 5000,
        "skip EKF prediction update due to non-positive IMU dt: %f [sec]", dt_imu);
      return false;
    }
    if (status == core::EKFEstimator::PredictionUpdateStatus::kDtTooLarge) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), clock_, 5000,
        "skip EKF prediction update due to too large IMU dt: %f [sec]", dt_imu);
      return false;
    }
    if (status != core::EKFEstimator::PredictionUpdateStatus::kUpdated) {
      RCLCPP_ERROR_THROTTLE(
        node_.get_logger(), clock_, 5000,
        "skip EKF prediction update due to invalid input or numerical invariant failure");
      return false;
    }

    const auto stationary_state = stationary_detector_.update(
      current_time_imu, gyro, linear_acceleration,
      ekf_.getVelocity().norm(), gravity_mps2_);
    bool learn_gyro_bias = true;
    bool learn_accel_bias = true;
    if (enable_bias_observability_gate_) {
      const auto bias_decision = core::evaluateBiasObservability(
        stationary_state == core::StationaryDetector::State::kStationary,
        gyro.norm(), linear_acceleration.head<2>().norm(),
        bias_min_angular_excitation_radps_, bias_min_acceleration_excitation_mps2_);
      learn_gyro_bias = bias_decision.learn_gyro_bias;
      learn_accel_bias = bias_decision.learn_accel_bias;
    }
    ekf_.setBiasLearningEnabled(learn_gyro_bias, learn_accel_bias);

    if (use_imu_orientation_) {
      const Eigen::Quaterniond q_meas(
        imu_msg.orientation.w,
        imu_msg.orientation.x,
        imu_msg.orientation.y,
        imu_msg.orientation.z);

      Eigen::Vector3d var_rpy_rad2(
        var_imu_orientation_rpy_, var_imu_orientation_rpy_, var_imu_orientation_rpy_);
      if (use_imu_orientation_covariance_ && imu_msg.orientation_covariance[0] >= 0.0) {
        const Eigen::Vector3d from_msg(
          imu_msg.orientation_covariance[0],
          imu_msg.orientation_covariance[4],
          imu_msg.orientation_covariance[8]);
        if (from_msg.allFinite() && (from_msg.array() > 0.0).all()) {
          var_rpy_rad2 = from_msg;
        }
      }

      const auto obs_status = ekf_.observationUpdateOrientationWithStatus(q_meas, var_rpy_rad2);
      if (obs_status == core::EKFEstimator::ObservationUpdateStatus::kInvalidMeasurement) {
        RCLCPP_WARN_THROTTLE(
          node_.get_logger(), clock_, 5000,
          "skip EKF orientation update due to invalid quaternion measurement");
      } else if (obs_status == core::EKFEstimator::ObservationUpdateStatus::kInvalidVariance) {
        RCLCPP_WARN_THROTTLE(
          node_.get_logger(), clock_, 5000,
          "skip EKF orientation update due to invalid orientation variance");
      } else if (obs_status == core::EKFEstimator::ObservationUpdateStatus::kNumericalFailure) {
        RCLCPP_ERROR_THROTTLE(
          node_.get_logger(), clock_, 5000,
          "skip EKF orientation update due to innovation or covariance numerical failure");
      }
    } else if (use_flat_ground_) {
      // Pseudo-observation: ground vehicles typically have small roll/pitch. This helps prevent
      // attitude drift when roll/pitch are weakly observable (e.g. gravity-compensated accel).
      const Eigen::Quaterniond q_est = ekf_.getOrientation().normalized();
      const double yaw_rad = getYawRadFromQuaternion(q_est);
      const Eigen::Quaterniond q_level(Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()));
      const Eigen::Vector3d var_rpy_rad2(var_flat_ground_rp_, var_flat_ground_rp_, 1.0e6);
      const StateSnapshot state_before = captureState();
      const auto obs_status = ekf_.observationUpdateOrientationWithStatus(q_level, var_rpy_rad2);
      if (obs_status == core::EKFEstimator::ObservationUpdateStatus::kInvalidMeasurement) {
        publishUpdateDeltaDebug(imu_msg.header.stamp, 2, -1, state_before, state_before);
        RCLCPP_WARN_THROTTLE(
          node_.get_logger(), clock_, 5000,
          "skip EKF flat-ground update due to invalid quaternion measurement");
      } else if (obs_status == core::EKFEstimator::ObservationUpdateStatus::kInvalidVariance) {
        publishUpdateDeltaDebug(imu_msg.header.stamp, 2, -2, state_before, state_before);
        RCLCPP_WARN_THROTTLE(
          node_.get_logger(), clock_, 5000,
          "skip EKF flat-ground update due to invalid variance (need finite positive)");
      } else if (obs_status == core::EKFEstimator::ObservationUpdateStatus::kUpdated) {
        publishUpdateDeltaDebug(imu_msg.header.stamp, 2, 1, state_before, captureState());
      } else {
        publishUpdateDeltaDebug(imu_msg.header.stamp, 2, -3, state_before, state_before);
        RCLCPP_ERROR_THROTTLE(
          node_.get_logger(), clock_, 5000,
          "skip EKF flat-ground update due to numerical invariant failure");
      }
    }
    if (use_nonholonomic_constraint_) {
      updateNonholonomicConstraint(imu_msg);
    }
    if (use_zupt_ || use_zihr_) {
      updateZeroVelocity(imu_msg, current_time_imu);
    }
    return true;
  }

  void updateZeroVelocity(const sensor_msgs::msg::Imu & imu_msg, const double time_sec)
  {
    const Eigen::Vector3d angular_velocity(
      imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z);
    const Eigen::Vector3d linear_acceleration(
      imu_msg.linear_acceleration.x,
      imu_msg.linear_acceleration.y,
      imu_msg.linear_acceleration.z);
    (void)time_sec;
    (void)linear_acceleration;
    const auto stationary_state = stationary_detector_.state();
    if (stationary_state != core::StationaryDetector::State::kStationary) {
      return;
    }

    if (use_zupt_) {
      const StateSnapshot state_before = captureState();
      const Eigen::Vector3d variance = Eigen::Vector3d::Constant(var_zupt_velocity_);
      const auto status = ekf_.observationUpdateVelocityWithStatus(
        Eigen::Vector3d::Zero(), variance, true);
      int status_code = 1;
      if (status == core::EKFEstimator::ObservationUpdateStatus::kInvalidMeasurement) {
        status_code = -1;
      } else if (status == core::EKFEstimator::ObservationUpdateStatus::kInvalidVariance) {
        status_code = -2;
      } else if (status == core::EKFEstimator::ObservationUpdateStatus::kNumericalFailure) {
        status_code = -3;
      }
      publishUpdateDeltaDebug(
        imu_msg.header.stamp, 6, status_code, state_before, captureState());
    }
    if (use_zihr_) {
      const StateSnapshot state_before = captureState();
      const Eigen::Vector3d variance = Eigen::Vector3d::Constant(var_zihr_gyro_);
      const auto status = ekf_.observationUpdateGyroBiasWithStatus(
        angular_velocity, variance);
      int status_code = 1;
      if (status == core::EKFEstimator::ObservationUpdateStatus::kInvalidMeasurement) {
        status_code = -1;
      } else if (status == core::EKFEstimator::ObservationUpdateStatus::kInvalidVariance) {
        status_code = -2;
      } else if (status == core::EKFEstimator::ObservationUpdateStatus::kNumericalFailure) {
        status_code = -3;
      }
      publishUpdateDeltaDebug(
        imu_msg.header.stamp, 7, status_code, state_before, captureState());
    }
  }

  void updateNonholonomicConstraint(const sensor_msgs::msg::Imu & imu_msg)
  {
    const Eigen::Quaterniond orientation = ekf_.getOrientation().normalized();
    const Eigen::Vector3d body_velocity =
      orientation.toRotationMatrix().transpose() * ekf_.getVelocity();
    if (!body_velocity.allFinite() ||
      std::fabs(body_velocity.x()) < min_nhc_forward_speed_mps_)
    {
      return;
    }

    double wheel_innovation = 0.0;
    if (has_latest_wheel_speed_) {
      wheel_innovation = latest_raw_wheel_speed_mps_ * wheel_speed_scale_factor_ -
        body_velocity.x();
    }
    core::VehicleModelInput vehicle_model_input;
    vehicle_model_input.time_sec = stampToSec(imu_msg.header.stamp);
    vehicle_model_input.body_velocity = body_velocity;
    vehicle_model_input.yaw_rate_radps = imu_msg.angular_velocity.z;
    vehicle_model_input.lateral_acceleration_mps2 = imu_msg.linear_acceleration.y;
    vehicle_model_input.wheel_innovation_mps = wheel_innovation;
    vehicle_model_input.has_wheel_innovation = has_latest_wheel_speed_;
    auto vehicle_model_output = vehicle_model_->evaluate(vehicle_model_input);
    if (!vehicle_model_output.valid || !vehicle_model_output.apply_nonholonomic_constraint ||
      !std::isfinite(vehicle_model_output.nhc_variance_scale) ||
      vehicle_model_output.nhc_variance_scale < 1.0)
    {
      return;
    }
    vehicle_model_output.nhc_variance_scale = std::min(
      max_nhc_variance_scale_, vehicle_model_output.nhc_variance_scale);
    const double variance_scale = vehicle_model_output.nhc_variance_scale;
    publishTypedObservability(
      imu_msg.header.stamp, vehicle_model_output, wheel_innovation);
    if (debug_observability_pub_) {
      std_msgs::msg::Float64MultiArray message;
      message.data = {
        stampToSec(imu_msg.header.stamp),
        static_cast<double>(stationary_detector_.state()),
        static_cast<double>(vehicle_model_output.slip_state),
        variance_scale,
        vehicle_model_output.slip_severity,
        wheel_innovation};
      debug_observability_pub_->publish(message);
    }
    const Eigen::Vector2d variance(
      var_nhc_lateral_velocity_ * variance_scale,
      vehicle_model_output.constrain_vertical_velocity ?
      var_nhc_vertical_velocity_ * variance_scale : 1.0e6);
    const StateSnapshot state_before = captureState();
    const auto status = ekf_.observationUpdateBodyVelocityConstraintWithStatus(
      Eigen::Vector2d::Zero(), variance);
    int status_code = 1;
    if (status == core::EKFEstimator::ObservationUpdateStatus::kInvalidMeasurement) {
      status_code = -1;
    } else if (status == core::EKFEstimator::ObservationUpdateStatus::kInvalidVariance) {
      status_code = -2;
    } else if (status == core::EKFEstimator::ObservationUpdateStatus::kNumericalFailure) {
      status_code = -3;
    }
    publishUpdateDeltaDebug(
      imu_msg.header.stamp, 5, status_code, state_before, captureState());
  }

  void setGnssNavSatFixOrigin(double latitude_deg, double longitude_deg, double altitude_m)
  {
    gnss_navsatfix_origin_latitude_ = latitude_deg;
    gnss_navsatfix_origin_longitude_ = longitude_deg;
    gnss_navsatfix_origin_altitude_ = altitude_m;
    gnss_navsatfix_origin_ecef_ = geodeticToEcef(latitude_deg, longitude_deg, altitude_m);
    has_gnss_navsatfix_origin_ = true;
  }

  bool convertNavSatFixToPose(
    const sensor_msgs::msg::NavSatFix & fix_msg,
    geometry_msgs::msg::PoseStamped & pose_msg)
  {
    if (fix_msg.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
      return false;
    }
    if (!isValidLatitudeLongitude(fix_msg.latitude, fix_msg.longitude)) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), clock_, 5000,
        "skip NavSatFix due to invalid latitude/longitude");
      return false;
    }
    const double altitude = std::isfinite(fix_msg.altitude) ? fix_msg.altitude : 0.0;

    if (!has_gnss_navsatfix_origin_) {
      if (!gnss_navsatfix_use_first_fix_as_origin_) {
        return false;
      }
      setGnssNavSatFixOrigin(fix_msg.latitude, fix_msg.longitude, altitude);
      RCLCPP_INFO(
        node_.get_logger(),
        "NavSatFix ENU origin locked to first fix: %.8f, %.8f, %.3f",
        gnss_navsatfix_origin_latitude_,
        gnss_navsatfix_origin_longitude_,
        gnss_navsatfix_origin_altitude_);
    }

    const Eigen::Vector3d ecef = geodeticToEcef(fix_msg.latitude, fix_msg.longitude, altitude);
    const Eigen::Vector3d enu = ecefToEnu(
      ecef,
      gnss_navsatfix_origin_ecef_,
      gnss_navsatfix_origin_latitude_,
      gnss_navsatfix_origin_longitude_);

    pose_msg.header.stamp = fix_msg.header.stamp;
    pose_msg.header.frame_id = reference_frame_id_;
    pose_msg.pose.position.x = enu.x();
    pose_msg.pose.position.y = enu.y();
    pose_msg.pose.position.z = enu.z();
    pose_msg.pose.orientation.x = 0.0;
    pose_msg.pose.orientation.y = 0.0;
    pose_msg.pose.orientation.z = 0.0;
    pose_msg.pose.orientation.w = 1.0;
    return true;
  }

  Eigen::Matrix3d getNavSatFixCovariance(const sensor_msgs::msg::NavSatFix & fix_msg) const
  {
    const Eigen::Matrix3d fallback = var_gnss_.asDiagonal();
    if (!gnss_navsatfix_use_position_covariance_ ||
      fix_msg.position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN)
    {
      return fallback;
    }
    Eigen::Matrix3d candidate;
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        candidate(row, column) = fix_msg.position_covariance[3 * row + column];
      }
    }
    candidate = 0.5 * (candidate + candidate.transpose());
    if (!candidate.allFinite()) {
      return fallback;
    }
    const Eigen::Vector3d minimum(
      gnss_navsatfix_min_variance_xy_,
      gnss_navsatfix_min_variance_xy_,
      gnss_navsatfix_min_variance_z_);
    const Eigen::Vector3d maximum(
      gnss_navsatfix_max_variance_xy_,
      gnss_navsatfix_max_variance_xy_,
      gnss_navsatfix_max_variance_z_);
    const Eigen::Vector3d diagonal = core::sanitizeMeasurementVariance(
      candidate.diagonal(), var_gnss_, minimum, maximum);
    Eigen::Matrix3d result = diagonal.asDiagonal();
    for (int row = 0; row < 3; ++row) {
      for (int column = row + 1; column < 3; ++column) {
        if (candidate(row, row) > 0.0 && candidate(column, column) > 0.0) {
          const double correlation = std::max(-0.999, std::min(
              0.999, candidate(row, column) /
              std::sqrt(candidate(row, row) * candidate(column, column))));
          result(row, column) = correlation * std::sqrt(diagonal(row) * diagonal(column));
          result(column, row) = result(row, column);
        }
      }
    }
    const Eigen::LDLT<Eigen::Matrix3d> decomposition(result);
    return decomposition.info() == Eigen::Success && decomposition.isPositive() ?
           result : diagonal.asDiagonal();
  }

  void handleGnssPose(
    const geometry_msgs::msg::PoseStamped & pose_msg,
    const Eigen::Matrix3d & position_covariance)
  {
    if (initial_pose_received_ && use_gnss_) {
      const double sensor_time = stampToSec(pose_msg.header.stamp);
      if (!sensorInputAllowed(core::SensorFaultMonitor::Sensor::kGnss, sensor_time)) {
        return;
      }
      latest_gnss_time_ = stampToSec(pose_msg.header.stamp) + gnss_time_offset_sec_;
      has_latest_gnss_ = std::isfinite(latest_gnss_time_);
      geometry_msgs::msg::PoseStamped body_pose_msg = pose_msg;
      const Eigen::Vector3d antenna_position(
        pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);
      const Eigen::Vector3d body_position = core::removeLeverArmFromPosition(
        antenna_position, ekf_.getOrientation(), gnss_lever_arm_body_);
      body_pose_msg.pose.position.x = body_position.x();
      body_pose_msg.pose.position.y = body_position.y();
      body_pose_msg.pose.position.z = body_position.z();
      if (enable_measurement_replay_) {
        const Eigen::Vector3d lever_arm = gnss_lever_arm_body_;
        const auto status = applyReplayMeasurement(
          pose_msg.header.stamp, 1U,
          [antenna_position, position_covariance, lever_arm](core::EKFEstimator & estimator) {
            return estimator.observationUpdateLeverArmPositionWithCovariance(
              antenna_position, lever_arm, position_covariance).status;
          },
          gnss_time_offset_sec_);
        if (status == core::EskfReplay::Status::kApplied ||
          status == core::EskfReplay::Status::kQueuedFuture)
        {
          const double measurement_time = stampToSec(pose_msg.header.stamp) +
            gnss_time_offset_sec_;
          previous_gnss_position_time_ = measurement_time;
          has_previous_gnss_position_time_ = true;
        }
        return;
      }
      geometry_msgs::msg::PoseStamped measurement_pose_msg = pose_msg;
      if (compensate_gnss_delay_ && has_latest_imu_stamp_) {
        const double imu_time =
          latest_imu_stamp_.seconds();
        const double measurement_time =
          body_pose_msg.header.stamp.sec + body_pose_msg.header.stamp.nanosec * 1.0e-9 +
          gnss_time_offset_sec_;
        const double delay = imu_time - measurement_time;
        if (delay > max_gnss_delay_compensation_sec_) {
          RCLCPP_WARN_THROTTLE(
            node_.get_logger(), clock_, 5000,
            "skip GNSS measurement with excessive delay: %f > %f [sec]",
            delay, max_gnss_delay_compensation_sec_);
          return;
        }
        if (delay > 0.0) {
          const Eigen::Vector3d compensated_position =
            core::extrapolatePositionConstantVelocity(
            antenna_position, ekf_.getVelocity(), delay);
          measurement_pose_msg.pose.position.x = compensated_position.x();
          measurement_pose_msg.pose.position.y = compensated_position.y();
          measurement_pose_msg.pose.position.z = compensated_position.z();
        }
      }
      measurementUpdate(
        measurement_pose_msg, position_covariance, true, gnss_lever_arm_body_);
      if (use_gnss_velocity_ || estimate_wheel_speed_scale_factor_) {
        updateVelocityFromGnss(body_pose_msg, position_covariance);
      }
      if (use_gnss_course_yaw_) {
        updateYawFromGnssCourse(body_pose_msg);
      }
    }
  }

  static double computePositionNis(
    const Eigen::Vector3d & innovation,
    const Eigen::Matrix3d & measurement_covariance,
    const Eigen::MatrixXd & covariance,
    const core::EKFEstimator::ObservationJacobian3 & jacobian)
  {
    if (covariance.rows() < core::EKFEstimator::kErrorStateSize ||
      covariance.cols() < core::EKFEstimator::kErrorStateSize)
    {
      return std::numeric_limits<double>::quiet_NaN();
    }
    Eigen::Matrix3d s = jacobian * covariance * jacobian.transpose() +
      measurement_covariance;
    if (!s.allFinite()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const Eigen::LDLT<Eigen::Matrix3d> decomposition(0.5 * (s + s.transpose()));
    if (decomposition.info() != Eigen::Success || !decomposition.isPositive()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const Eigen::Vector3d solved_innovation = decomposition.solve(innovation);
    return decomposition.info() == Eigen::Success && solved_innovation.allFinite() ?
           innovation.dot(solved_innovation) : std::numeric_limits<double>::quiet_NaN();
  }

  double computeYawNis(
    const double innovation,
    const double variance,
    const Eigen::MatrixXd & covariance) const
  {
    if (covariance.rows() < core::EKFEstimator::kErrorStateSize ||
      covariance.cols() < core::EKFEstimator::kErrorStateSize)
    {
      return std::numeric_limits<double>::quiet_NaN();
    }
    Eigen::Matrix<double, 1, core::EKFEstimator::kErrorStateSize> jacobian;
    (void)core::EKFEstimator::yawObservation(ekf_.getState(), &jacobian);
    const double s = (jacobian * covariance * jacobian.transpose())(0, 0) + variance;
    if (!(s > 0.0) || !std::isfinite(s)) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return innovation * innovation / s;
  }

  static double computeAdaptiveVarianceScale(
    const double nis,
    const double threshold,
    const double max_scale)
  {
    if (!(threshold > 0.0) || !(max_scale > 1.0) || !std::isfinite(nis)) {
      return 1.0;
    }
    if (nis <= threshold) {
      return 1.0;
    }
    const double scale = std::sqrt(nis / threshold);
    return std::min(max_scale, std::max(1.0, scale));
  }

  static double computeAdaptiveMagnitudeScale(
    const double magnitude,
    const double threshold,
    const double max_scale)
  {
    if (!(threshold > 0.0) || !(max_scale > 1.0) || !std::isfinite(magnitude)) {
      return 1.0;
    }
    if (magnitude <= threshold) {
      return 1.0;
    }
    const double scale = magnitude / threshold;
    return std::min(max_scale, std::max(1.0, scale));
  }

  bool updateGnssReacquisitionState(
    const geometry_msgs::msg::PoseStamped & pose_msg)
  {
    if (!(gnss_position_reacquisition_dt_sec_ > 0.0) ||
      (!(gnss_position_reacquisition_variance_scale_ > 1.0) &&
      !hasGnssReacquisitionCovarianceCap() &&
      !gnss_position_reacquisition_reset_position_))
    {
      return false;
    }

    const double t = stampToSec(pose_msg.header.stamp);
    if (
      has_previous_gnss_position_time_ && std::isfinite(t) &&
      std::isfinite(previous_gnss_position_time_))
    {
      const double dt = t - previous_gnss_position_time_;
      if (dt > gnss_position_reacquisition_dt_sec_) {
        gnss_position_reacquisition_updates_remaining_ =
          std::max(1, gnss_position_reacquisition_update_count_);
      }
    }
    if (std::isfinite(t)) {
      previous_gnss_position_time_ = t;
      has_previous_gnss_position_time_ = true;
    }
    if (gnss_position_reacquisition_updates_remaining_ > 0) {
      --gnss_position_reacquisition_updates_remaining_;
      return true;
    }
    return false;
  }

  bool hasGnssReacquisitionCovarianceCap() const
  {
    return gnss_position_reacquisition_covariance_cap_xy_ > 0.0 ||
           gnss_position_reacquisition_covariance_cap_z_ > 0.0 ||
           gnss_position_reacquisition_velocity_covariance_cap_xy_ > 0.0 ||
           gnss_position_reacquisition_velocity_covariance_cap_z_ > 0.0 ||
           gnss_position_reacquisition_attitude_covariance_cap_rp_ > 0.0 ||
           gnss_position_reacquisition_attitude_covariance_cap_yaw_ > 0.0;
  }

  static double covarianceDiagAt(const Eigen::MatrixXd & covariance, const int index)
  {
    if (covariance.rows() <= index || covariance.cols() <= index) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return covariance(index, index);
  }

  static UpdateDeltaExtra computePositionUpdateExtra(
    const Eigen::MatrixXd & covariance,
    const Eigen::Vector3d & raw_innovation,
    const Eigen::Vector3d & used_innovation,
    const Eigen::Vector3d & used_variance,
    const double raw_nis,
    const double used_nis,
    const double variance_scale)
  {
    UpdateDeltaExtra extra;
    extra.raw_innovation_norm = raw_innovation.norm();
    extra.used_innovation_norm = used_innovation.norm();
    extra.raw_nis = raw_nis;
    extra.used_nis = used_nis;
    extra.variance_scale = variance_scale;
    extra.used_variance = used_variance;
    extra.covariance_position_diag = Eigen::Vector3d(
      covarianceDiagAt(covariance, 0), covarianceDiagAt(covariance, 1),
      covarianceDiagAt(covariance, 2));
    extra.covariance_velocity_diag = Eigen::Vector3d(
      covarianceDiagAt(covariance, 3), covarianceDiagAt(covariance, 4),
      covarianceDiagAt(covariance, 5));
    extra.covariance_attitude_diag = Eigen::Vector3d(
      covarianceDiagAt(covariance, 6), covarianceDiagAt(covariance, 7),
      covarianceDiagAt(covariance, 8));

    if (covariance.rows() < 15 || covariance.cols() < 15 || !used_variance.allFinite()) {
      return extra;
    }

    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    R.diagonal() = used_variance;
    const Eigen::Matrix3d S = covariance.block<3, 3>(0, 0) + R;
    const Eigen::LDLT<Eigen::Matrix3d> decomposition(0.5 * (S + S.transpose()));
    if (decomposition.info() != Eigen::Success || !decomposition.isPositive()) {
      return extra;
    }

    const Eigen::Matrix<double, 15, 3> K =
      decomposition.solve(covariance.block<15, 3>(0, 0).transpose()).transpose();
    if (decomposition.info() != Eigen::Success || !K.allFinite()) {
      return extra;
    }
    const Eigen::Matrix<double, 15, 1> dx = K * used_innovation;
    extra.kalman_gain_norm = K.norm();
    extra.kalman_gain_position_norm = K.block<3, 3>(0, 0).norm();
    extra.kalman_gain_velocity_norm = K.block<3, 3>(3, 0).norm();
    extra.kalman_gain_attitude_norm = K.block<3, 3>(6, 0).norm();
    extra.predicted_dx_norm = dx.norm();
    extra.predicted_dx_xy_norm = dx.segment<2>(0).norm();
    extra.predicted_dtheta_norm = dx.segment<3>(6).norm();
    return extra;
  }

  void publishGnssPositionDebug(
    const geometry_msgs::msg::PoseStamped & pose_msg,
    const Eigen::Vector3d & innovation,
    const double nis,
    const int status_code,
    const Eigen::Vector3d & variance,
    const Eigen::MatrixXd & covariance,
    const double raw_nis,
    const double variance_scale)
  {
    if (!debug_gnss_position_pub_) {
      return;
    }
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {
      stampToSec(pose_msg.header.stamp),
      innovation.x(),
      innovation.y(),
      innovation.z(),
      nis,
      static_cast<double>(status_code),
      variance.x(),
      variance.y(),
      variance.z(),
      covariance.rows() > 0 && covariance.cols() > 0 ? covariance(0, 0) :
      std::numeric_limits<double>::quiet_NaN(),
      covariance.rows() > 1 && covariance.cols() > 1 ? covariance(1, 1) :
      std::numeric_limits<double>::quiet_NaN(),
      covariance.rows() > 2 && covariance.cols() > 2 ? covariance(2, 2) :
      std::numeric_limits<double>::quiet_NaN(),
      raw_nis,
      variance_scale};
    debug_gnss_position_pub_->publish(msg);
  }

  void publishMeasurementQuality(
    const builtin_interfaces::msg::Time & stamp, const int source,
    const bool accepted, const core::MeasurementRejectReason reason,
    const double innovation_magnitude, const double raw_nis, const double used_nis,
    const double variance_scale = 1.0)
  {
    if (accepted) {
      ++accepted_measurement_count_;
    } else {
      ++rejected_measurement_count_;
    }
    if (reason == core::MeasurementRejectReason::kNumericalFailure) {
      ++numerical_failure_count_;
    }
    if (source >= 1 && source <= 3) {
      recordSensorOutcome(
        core::SensorFaultMonitor::Sensor::kGnss, accepted, stampToSec(stamp));
    }
    if (typed_measurement_quality_pub_) {
      kalman_filter_localization_msgs::msg::MeasurementQuality typed_message;
      typed_message.header.stamp = stamp;
      typed_message.header.frame_id = reference_frame_id_;
      typed_message.sequence = ++measurement_quality_sequence_;
      typed_message.source_id = static_cast<std::uint8_t>(std::max(0, source));
      typed_message.source = measurementSourceText(source);
      typed_message.accepted = accepted;
      typed_message.reject_reason = static_cast<std::uint8_t>(reason);
      typed_message.reject_reason_text = measurementRejectReasonText(reason);
      typed_message.innovation_magnitude = innovation_magnitude;
      typed_message.raw_nis = raw_nis;
      typed_message.used_nis = used_nis;
      typed_message.variance_scale = variance_scale;
      typed_measurement_quality_pub_->publish(typed_message);
    }
    if (!debug_measurement_quality_pub_) {
      return;
    }
    std_msgs::msg::Float64MultiArray message;
    message.data = {
      stampToSec(stamp), static_cast<double>(source), accepted ? 1.0 : 0.0,
      static_cast<double>(reason), innovation_magnitude, raw_nis, used_nis};
    debug_measurement_quality_pub_->publish(message);
  }

  void publishTypedObservability(
    const builtin_interfaces::msg::Time & stamp,
    const core::VehicleModelOutput & vehicle_model_output,
    const double wheel_innovation)
  {
    if (!typed_observability_pub_) {
      return;
    }
    kalman_filter_localization_msgs::msg::ObservabilityStatus message;
    message.header.stamp = stamp;
    message.header.frame_id = robot_frame_id_;
    message.stationary_state = static_cast<std::uint8_t>(stationary_detector_.state());
    message.stationary_state_text = stationaryStateText(stationary_detector_.state());
    message.vehicle_model = vehicle_model_->name();
    message.nhc_vertical_constrained = vehicle_model_output.constrain_vertical_velocity;
    message.slip_state = vehicle_model_output.slip_state;
    message.slip_state_text = vehicle_model_output.slip_state_text;
    message.nhc_variance_scale = vehicle_model_output.nhc_variance_scale;
    message.slip_severity = vehicle_model_output.slip_severity;
    message.wheel_innovation_mps = wheel_innovation;
    message.gyro_bias_learning_enabled = ekf_.gyroBiasLearningEnabled();
    message.accel_bias_learning_enabled = ekf_.accelBiasLearningEnabled();
    typed_observability_pub_->publish(message);
  }

  void publishGnssCourseYawDebug(
    const geometry_msgs::msg::PoseStamped & pose_msg,
    const double yaw_meas,
    const double yaw_est,
    const double innovation,
    const double nis,
    const int status_code,
    const int reason_code,
    const double dt,
    const double distance,
    const double speed,
    const Eigen::MatrixXd & covariance,
    const double variance,
    const double raw_nis = std::numeric_limits<double>::quiet_NaN(),
    const double variance_scale = 1.0)
  {
    if (!debug_gnss_course_yaw_pub_) {
      return;
    }
    constexpr int kErrorStateYawIndex = 8;
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {
      stampToSec(pose_msg.header.stamp),
      yaw_meas,
      yaw_est,
      innovation,
      nis,
      static_cast<double>(status_code),
      static_cast<double>(reason_code),
      dt,
      distance,
      speed,
      variance,
      covariance.rows() > kErrorStateYawIndex && covariance.cols() > kErrorStateYawIndex ?
      covariance(kErrorStateYawIndex, kErrorStateYawIndex) :
      std::numeric_limits<double>::quiet_NaN(),
      raw_nis,
      variance_scale};
    debug_gnss_course_yaw_pub_->publish(msg);
  }

  StateSnapshot captureState() const
  {
    return StateSnapshot{
      ekf_.getPosition(),
      getRpyRadFromQuaternion(ekf_.getOrientation().normalized())};
  }

  void publishUpdateDeltaDebug(
    const builtin_interfaces::msg::Time & stamp,
    const int update_type,
    const int status_code,
    const StateSnapshot & before,
    const StateSnapshot & after)
  {
    publishUpdateDeltaDebug(stamp, update_type, status_code, before, after, UpdateDeltaExtra{});
  }

  void publishUpdateDeltaDebug(
    const builtin_interfaces::msg::Time & stamp,
    const int update_type,
    const int status_code,
    const StateSnapshot & before,
    const StateSnapshot & after,
    const UpdateDeltaExtra & extra)
  {
    if (!debug_update_delta_pub_) {
      return;
    }
    const Eigen::Vector3d dp = after.position - before.position;
    const double droll = wrapToPi(after.rpy.x() - before.rpy.x());
    const double dpitch = wrapToPi(after.rpy.y() - before.rpy.y());
    const double dyaw = wrapToPi(after.rpy.z() - before.rpy.z());

    std_msgs::msg::Float64MultiArray msg;
    msg.data = {
      stampToSec(stamp),
      static_cast<double>(update_type),
      static_cast<double>(status_code),
      dp.x(),
      dp.y(),
      dp.z(),
      dp.head<2>().norm(),
      droll,
      dpitch,
      dyaw,
      before.position.x(),
      before.position.y(),
      before.position.z(),
      after.position.x(),
      after.position.y(),
      after.position.z(),
      before.rpy.x(),
      before.rpy.y(),
      before.rpy.z(),
      after.rpy.x(),
      after.rpy.y(),
      after.rpy.z(),
      extra.raw_innovation_norm,
      extra.used_innovation_norm,
      extra.raw_nis,
      extra.used_nis,
      extra.variance_scale,
      extra.kalman_gain_norm,
      extra.kalman_gain_position_norm,
      extra.kalman_gain_velocity_norm,
      extra.kalman_gain_attitude_norm,
      extra.predicted_dx_norm,
      extra.predicted_dx_xy_norm,
      extra.predicted_dtheta_norm,
      extra.covariance_position_diag.x(),
      extra.covariance_position_diag.y(),
      extra.covariance_position_diag.z(),
      extra.covariance_velocity_diag.x(),
      extra.covariance_velocity_diag.y(),
      extra.covariance_velocity_diag.z(),
      extra.covariance_attitude_diag.x(),
      extra.covariance_attitude_diag.y(),
      extra.covariance_attitude_diag.z(),
      extra.used_variance.x(),
      extra.used_variance.y(),
      extra.used_variance.z()};
    debug_update_delta_pub_->publish(msg);
  }

  void measurementUpdate(
    const geometry_msgs::msg::PoseStamped & pose_msg,
    const Eigen::Vector3d & variance,
    const bool publish_gnss_debug = false,
    const Eigen::Vector3d & lever_arm_body = Eigen::Vector3d::Zero())
  {
    const Eigen::Matrix3d covariance = variance.asDiagonal();
    measurementUpdate(pose_msg, covariance, publish_gnss_debug, lever_arm_body);
  }

  void measurementUpdate(
    const geometry_msgs::msg::PoseStamped & pose_msg,
    const Eigen::Matrix3d & measurement_covariance,
    const bool publish_gnss_debug,
    const Eigen::Vector3d & lever_arm_body)
  {
    has_received_input_ = true;
    current_stamp_ = pose_msg.header.stamp;
    const Eigen::Vector3d y = Eigen::Vector3d(
      pose_msg.pose.position.x,
      pose_msg.pose.position.y,
      pose_msg.pose.position.z);
    const Eigen::Vector3d position_before_update = ekf_.getPosition();
    const Eigen::Vector3d predicted_measurement =
      position_before_update + ekf_.getOrientation().normalized() * lever_arm_body;
    const Eigen::Vector3d innovation = y - predicted_measurement;
    const Eigen::MatrixXd covariance = ekf_.getCovariance();
    core::EKFEstimator::ObservationJacobian3 observation_jacobian;
    (void)core::EKFEstimator::leverArmPositionObservation(
      ekf_.getState(), lever_arm_body, &observation_jacobian);
    const double raw_nis = computePositionNis(
      innovation, measurement_covariance, covariance, observation_jacobian);
    const Eigen::Vector3d variance = measurement_covariance.diagonal();
    const StateSnapshot state_before = captureState();
    const double innovation_norm = innovation.norm();
    double reacquisition_variance_scale = 1.0;
    if (publish_gnss_debug && gnss_position_reacquisition_dt_sec_ > 0.0) {
      const double measurement_time = stampToSec(pose_msg.header.stamp);
      double velocity_innovation = 0.0;
      if (has_reacquisition_previous_position_) {
        const double dt = measurement_time - reacquisition_previous_time_;
        if (dt > 0.0) {
          velocity_innovation = ((y - reacquisition_previous_position_) / dt -
            ekf_.getVelocity()).norm();
        }
      }
      reacquisition_previous_position_ = y;
      reacquisition_previous_time_ = measurement_time;
      has_reacquisition_previous_position_ = true;
      const auto reacquisition = gnss_reacquisition_gate_.update(
        measurement_time, innovation_norm, velocity_innovation);
      reacquisition_variance_scale = reacquisition.variance_scale;
      if (!reacquisition.accepted) {
        publishGnssPositionDebug(
          pose_msg, innovation, raw_nis,
          -10 - static_cast<int>(reacquisition.reason), variance, covariance, raw_nis,
          reacquisition_variance_scale);
        publishUpdateDeltaDebug(
          pose_msg.header.stamp, 1, -4, state_before, state_before);
        publishMeasurementQuality(
          pose_msg.header.stamp, 1, false,
          core::MeasurementRejectReason::kReacquisitionConsistency,
          innovation_norm, raw_nis, raw_nis);
        return;
      }
    }

    const auto quality = core::evaluateMeasurementQuality(
      true, innovation_norm, raw_nis,
      max_gnss_position_innovation_m_, max_gnss_position_nis_);
    if (!quality.accepted) {
      if (publish_gnss_debug) {
        publishGnssPositionDebug(
          pose_msg, innovation, raw_nis, 0, variance, covariance, raw_nis, 1.0);
      }
      publishMeasurementQuality(
        pose_msg.header.stamp, 1, false, quality.reason,
        innovation_norm, raw_nis, raw_nis);
      return;
    }

    if (
      max_gnss_position_innovation_m_ > 0.0 && std::isfinite(innovation_norm) &&
      innovation_norm > max_gnss_position_innovation_m_)
    {
      if (publish_gnss_debug) {
        publishGnssPositionDebug(
          pose_msg, innovation, raw_nis, 0, variance, covariance, raw_nis, 1.0);
        publishUpdateDeltaDebug(pose_msg.header.stamp, 1, 0, state_before, state_before);
      }
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), clock_, 5000,
        "skip GNSS position update due to large innovation: |innov|=%f [m] > max=%f [m]",
        innovation_norm, max_gnss_position_innovation_m_);
      return;
    }

    if (
      max_gnss_position_nis_ > 0.0 && std::isfinite(raw_nis) &&
      raw_nis > max_gnss_position_nis_)
    {
      if (publish_gnss_debug) {
        publishGnssPositionDebug(
          pose_msg, innovation, raw_nis, 0, variance, covariance, raw_nis, 1.0);
        publishUpdateDeltaDebug(pose_msg.header.stamp, 1, 0, state_before, state_before);
      }
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), clock_, 5000,
        "skip GNSS position update due to large NIS: nis=%f > max=%f",
        raw_nis, max_gnss_position_nis_);
      return;
    }

    const double nis_variance_scale = computeAdaptiveVarianceScale(
      raw_nis,
      gnss_position_nis_adaptive_threshold_,
      max_gnss_position_variance_scale_);
    const double innovation_variance_scale = computeAdaptiveMagnitudeScale(
      innovation.norm(),
      gnss_position_innovation_adaptive_threshold_m_,
      max_gnss_position_innovation_variance_scale_);
    const double robust_variance_scale = core::EKFEstimator::computeRobustVarianceScale(
      std::sqrt(std::max(0.0, raw_nis)),
      gnss_position_robust_loss_,
      gnss_position_robust_tuning_,
      max_gnss_position_robust_variance_scale_);
    const double variance_scale =
      std::max({
        nis_variance_scale, innovation_variance_scale,
        reacquisition_variance_scale, robust_variance_scale});
    Eigen::Vector3d update_innovation = innovation;
    if (
      max_gnss_position_innovation_clip_m_ > 0.0 &&
      std::isfinite(innovation_norm) &&
      innovation_norm > max_gnss_position_innovation_clip_m_)
    {
      update_innovation *= max_gnss_position_innovation_clip_m_ / innovation_norm;
    }
    const Eigen::Vector3d y_update = predicted_measurement + update_innovation;
    const Eigen::Matrix3d used_covariance = measurement_covariance * variance_scale;
    const Eigen::Vector3d used_variance = used_covariance.diagonal();
    const double used_nis = computePositionNis(
      update_innovation, used_covariance, covariance, observation_jacobian);
    const UpdateDeltaExtra update_extra = computePositionUpdateExtra(
      covariance, innovation, update_innovation, used_variance, raw_nis, used_nis, variance_scale);

    const auto diagnostics = lever_arm_body.squaredNorm() > 0.0 ?
      ekf_.observationUpdateLeverArmPositionWithCovariance(
      y_update, lever_arm_body, used_covariance) :
      ekf_.observationUpdatePositionWithCovariance(y_update, used_covariance);
    const auto status = diagnostics.status;
    if (status == core::EKFEstimator::ObservationUpdateStatus::kInvalidMeasurement) {
      if (publish_gnss_debug) {
        publishGnssPositionDebug(
          pose_msg, innovation, used_nis, -1, used_variance, covariance, raw_nis, variance_scale);
        publishUpdateDeltaDebug(
          pose_msg.header.stamp, 1, -1, state_before, state_before, update_extra);
      }
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), clock_, 5000,
        "skip EKF observation update due to invalid measurement (NaN/Inf)");
      return;
    }
    if (status == core::EKFEstimator::ObservationUpdateStatus::kInvalidVariance) {
      if (publish_gnss_debug) {
        publishGnssPositionDebug(
          pose_msg, innovation, used_nis, -2, used_variance, covariance, raw_nis, variance_scale);
        publishUpdateDeltaDebug(
          pose_msg.header.stamp, 1, -2, state_before, state_before, update_extra);
      }
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), clock_, 5000,
        "skip EKF observation update due to invalid variance (need finite positive)");
      return;
    }
    if (status == core::EKFEstimator::ObservationUpdateStatus::kNumericalFailure) {
      if (publish_gnss_debug) {
        publishGnssPositionDebug(
          pose_msg, innovation, used_nis, -3, used_variance, covariance, raw_nis, variance_scale);
        publishUpdateDeltaDebug(
          pose_msg.header.stamp, 1, -3, state_before, state_before, update_extra);
      }
      RCLCPP_ERROR_THROTTLE(
        node_.get_logger(), clock_, 5000,
        "skip EKF observation update due to innovation or covariance numerical failure");
      return;
    }
    if (publish_gnss_debug) {
      publishGnssPositionDebug(
        pose_msg, innovation, used_nis, 1, used_variance, covariance, raw_nis, variance_scale);
      publishUpdateDeltaDebug(pose_msg.header.stamp, 1, 1, state_before, captureState(),
          update_extra);
      publishMeasurementQuality(
        pose_msg.header.stamp, 1, diagnostics.accepted,
        diagnostics.accepted ? core::MeasurementRejectReason::kNone :
        core::MeasurementRejectReason::kNumericalFailure,
        innovation_norm, raw_nis, diagnostics.nis);
    }
  }

  void updateYawFromGnssCourse(const geometry_msgs::msg::PoseStamped & pose_msg)
  {
    const double t = pose_msg.header.stamp.sec + pose_msg.header.stamp.nanosec * 1e-9;
    if (has_latest_gnss_doppler_ &&
      std::fabs(t - latest_gnss_doppler_time_) <= gnss_doppler_fallback_timeout_sec_)
    {
      has_course_base_gnss_ = false;
      return;
    }
    const double x = pose_msg.pose.position.x;
    const double y = pose_msg.pose.position.y;

    if (!has_course_base_gnss_) {
      course_base_gnss_time_ = t;
      course_base_gnss_x_ = x;
      course_base_gnss_y_ = y;
      has_course_base_gnss_ = true;
      publishGnssCourseYawDebug(
        pose_msg,
        std::numeric_limits<double>::quiet_NaN(),
        getYawRadFromQuaternion(ekf_.getOrientation().normalized()),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        0,
        1,
        std::numeric_limits<double>::quiet_NaN(),
        0.0,
        std::numeric_limits<double>::quiet_NaN(),
        ekf_.getCovariance(),
        var_gnss_course_yaw_);
      return;
    }

    const double dt = t - course_base_gnss_time_;
    if (!(dt > 0.0) || dt > max_gnss_course_dt_sec_) {
      // Reset if time is invalid or too old.
      course_base_gnss_time_ = t;
      course_base_gnss_x_ = x;
      course_base_gnss_y_ = y;
      publishGnssCourseYawDebug(
        pose_msg,
        std::numeric_limits<double>::quiet_NaN(),
        getYawRadFromQuaternion(ekf_.getOrientation().normalized()),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        0,
        2,
        dt,
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        ekf_.getCovariance(),
        var_gnss_course_yaw_);
      return;
    }

    const double dx = x - course_base_gnss_x_;
    const double dy = y - course_base_gnss_y_;
    const double dist = std::hypot(dx, dy);
    if (dist < min_gnss_course_distance_m_) {
      // Accumulate until we have enough displacement.
      publishGnssCourseYawDebug(
        pose_msg,
        std::numeric_limits<double>::quiet_NaN(),
        getYawRadFromQuaternion(ekf_.getOrientation().normalized()),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        0,
        3,
        dt,
        dist,
        dist / dt,
        ekf_.getCovariance(),
        var_gnss_course_yaw_);
      return;
    }

    const double speed = dist / dt;
    if (speed < min_gnss_course_speed_mps_) {
      publishGnssCourseYawDebug(
        pose_msg,
        std::numeric_limits<double>::quiet_NaN(),
        getYawRadFromQuaternion(ekf_.getOrientation().normalized()),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        0,
        4,
        dt,
        dist,
        speed,
        ekf_.getCovariance(),
        var_gnss_course_yaw_);
      return;
    }

    const double yaw_meas = std::atan2(dy, dx);
    const Eigen::Quaterniond q_est = ekf_.getOrientation().normalized();
    const double yaw_est = getYawRadFromQuaternion(q_est);
    const double dyaw = wrapToPi(yaw_meas - yaw_est);
    const Eigen::MatrixXd covariance = ekf_.getCovariance();
    const double raw_nis = computeYawNis(dyaw, var_gnss_course_yaw_, covariance);
    const StateSnapshot state_before = captureState();
    const auto course_quality = core::evaluateMeasurementQuality(
      true, std::fabs(dyaw), raw_nis,
      max_gnss_course_dyaw_rad_, max_gnss_course_yaw_nis_);
    if (!course_quality.accepted) {
      publishMeasurementQuality(
        pose_msg.header.stamp, 3, false, course_quality.reason,
        std::fabs(dyaw), raw_nis, raw_nis);
      course_base_gnss_time_ = t;
      course_base_gnss_x_ = x;
      course_base_gnss_y_ = y;
      return;
    }
    if (
      max_gnss_course_yaw_nis_ > 0.0 && std::isfinite(raw_nis) &&
      raw_nis > max_gnss_course_yaw_nis_)
    {
      publishGnssCourseYawDebug(
        pose_msg,
        yaw_meas,
        yaw_est,
        dyaw,
        raw_nis,
        0,
        6,
        dt,
        dist,
        speed,
        covariance,
        var_gnss_course_yaw_,
        raw_nis,
        1.0);
      publishUpdateDeltaDebug(pose_msg.header.stamp, 3, 0, state_before, state_before);
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(),
        clock_,
        5000,
        "skip GNSS course yaw update due to large NIS: nis=%f > max=%f",
        raw_nis,
        max_gnss_course_yaw_nis_);
      course_base_gnss_time_ = t;
      course_base_gnss_x_ = x;
      course_base_gnss_y_ = y;
      return;
    }
    const double variance_scale = computeAdaptiveVarianceScale(
      raw_nis,
      gnss_course_yaw_nis_adaptive_threshold_,
      max_gnss_course_yaw_variance_scale_);
    const double used_yaw_variance = var_gnss_course_yaw_ * variance_scale;
    const double used_nis = computeYawNis(dyaw, used_yaw_variance, covariance);
    if (std::fabs(dyaw) > max_gnss_course_dyaw_rad_) {
      publishMeasurementQuality(
        pose_msg.header.stamp, 3, false,
        core::MeasurementRejectReason::kInnovationMagnitude,
        std::fabs(dyaw), std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN());
      publishGnssCourseYawDebug(
        pose_msg,
        yaw_meas,
        yaw_est,
        dyaw,
        used_nis,
        0,
        5,
        dt,
        dist,
        speed,
        covariance,
        used_yaw_variance,
        raw_nis,
        variance_scale);
      publishUpdateDeltaDebug(pose_msg.header.stamp, 3, 0, state_before, state_before);
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(),
        clock_,
        5000,
        "skip GNSS course yaw update due to large innovation: |dyaw|=%f [rad] > max=%f [rad]",
        dyaw,
        max_gnss_course_dyaw_rad_);
      course_base_gnss_time_ = t;
      course_base_gnss_x_ = x;
      course_base_gnss_y_ = y;
      return;
    }

    const auto yaw_diagnostics =
      ekf_.observationUpdateYawWithVariance(yaw_meas, used_yaw_variance);
    const auto status = yaw_diagnostics.status;
    int status_code = 1;
    if (status == core::EKFEstimator::ObservationUpdateStatus::kInvalidMeasurement) {
      status_code = -1;
    } else if (status == core::EKFEstimator::ObservationUpdateStatus::kInvalidVariance) {
      status_code = -2;
    } else if (status == core::EKFEstimator::ObservationUpdateStatus::kNumericalFailure) {
      status_code = -3;
    }
    publishGnssCourseYawDebug(
      pose_msg,
      yaw_meas,
      yaw_est,
      dyaw,
      used_nis,
      status_code,
      0,
      dt,
      dist,
      speed,
      covariance,
      used_yaw_variance,
      raw_nis,
      variance_scale);
    publishUpdateDeltaDebug(pose_msg.header.stamp, 3, status_code, state_before, captureState());
    publishMeasurementQuality(
      pose_msg.header.stamp, 3, status_code == 1,
      status_code == 1 ? core::MeasurementRejectReason::kNone :
      core::MeasurementRejectReason::kNumericalFailure,
      std::fabs(dyaw), raw_nis, used_nis);

    // Update the base point after applying the measurement.
    course_base_gnss_time_ = t;
    course_base_gnss_x_ = x;
    course_base_gnss_y_ = y;
  }

  void updateVelocityFromGnss(
    const geometry_msgs::msg::PoseStamped & pose_msg,
    const Eigen::Matrix3d & position_covariance)
  {
    const double t = pose_msg.header.stamp.sec + pose_msg.header.stamp.nanosec * 1e-9;
    const Eigen::Vector3d position(
      pose_msg.pose.position.x,
      pose_msg.pose.position.y,
      pose_msg.pose.position.z);
    if (!std::isfinite(t) || !position.allFinite()) {
      return;
    }
    if (has_latest_gnss_doppler_ &&
      std::fabs(t - latest_gnss_doppler_time_) <= gnss_doppler_fallback_timeout_sec_)
    {
      has_previous_velocity_gnss_ = false;
      return;
    }

    if (!has_previous_velocity_gnss_) {
      previous_velocity_gnss_time_ = t;
      previous_velocity_gnss_position_ = position;
      previous_velocity_gnss_covariance_ = position_covariance;
      has_previous_velocity_gnss_ = true;
      return;
    }

    const double dt = t - previous_velocity_gnss_time_;
    if (!(dt > 0.0) || dt > max_gnss_velocity_dt_sec_) {
      previous_velocity_gnss_time_ = t;
      previous_velocity_gnss_position_ = position;
      previous_velocity_gnss_covariance_ = position_covariance;
      return;
    }

    const Eigen::Vector3d delta = position - previous_velocity_gnss_position_;
    if (delta.head<2>().norm() < min_gnss_velocity_distance_m_) {
      return;
    }

    const Eigen::Vector3d velocity_meas = delta / dt;
    if (!velocity_meas.allFinite()) {
      previous_velocity_gnss_time_ = t;
      previous_velocity_gnss_position_ = position;
      previous_velocity_gnss_covariance_ = position_covariance;
      return;
    }

    if (wheel_scale_reference_ != "odom") {
      updateWheelSpeedScaleFactor(velocity_meas.head<2>().norm(), t);
    }

    if (use_gnss_velocity_) {
      const Eigen::Matrix3d base_covariance =
        gnss_position_velocity_propagate_covariance_ ?
        (position_covariance + previous_velocity_gnss_covariance_) / (dt * dt) :
        Eigen::Matrix3d(var_gnss_velocity_.asDiagonal());
      const Eigen::Matrix3d derived_covariance =
        base_covariance * gnss_position_velocity_correlation_scale_;
      updateVelocityMeasurement(
        velocity_meas, derived_covariance, "GNSS position-derived fallback",
        pose_msg.header.stamp);
    }

    previous_velocity_gnss_time_ = t;
    previous_velocity_gnss_position_ = position;
    previous_velocity_gnss_covariance_ = position_covariance;
  }

  void updateWheelSpeedScaleFactor(const double gnss_speed_mps, const double gnss_time)
  {
    if (!estimate_wheel_speed_scale_factor_ || !has_latest_wheel_speed_ ||
      !std::isfinite(gnss_speed_mps) || gnss_speed_mps < wheel_scale_min_speed_mps_ ||
      std::fabs(latest_raw_wheel_speed_mps_) < wheel_scale_min_speed_mps_ ||
      latest_abs_yaw_rate_radps_ > wheel_scale_max_yaw_rate_radps_ ||
      std::fabs(gnss_time - latest_wheel_speed_time_) > wheel_scale_max_sample_age_sec_)
    {
      return;
    }
    const double sample = gnss_speed_mps / std::fabs(latest_raw_wheel_speed_mps_);
    if (!std::isfinite(sample) || sample < wheel_scale_min_factor_ ||
      sample > wheel_scale_max_factor_)
    {
      return;
    }
    wheel_scale_samples_.push_back(sample);
    if (wheel_scale_samples_.size() > static_cast<std::size_t>(wheel_scale_window_size_)) {
      wheel_scale_samples_.erase(wheel_scale_samples_.begin());
    }
    if (wheel_scale_samples_.size() >= static_cast<std::size_t>(wheel_scale_min_samples_)) {
      wheel_speed_scale_factor_ = core::medianWheelSpeedScaleFactor(
        wheel_scale_samples_, wheel_speed_scale_factor_, wheel_scale_min_factor_,
        wheel_scale_max_factor_);
      RCLCPP_INFO_THROTTLE(
        node_.get_logger(), clock_, 5000, "estimated wheel-speed scale factor: %.6f (%zu samples)",
        wheel_speed_scale_factor_, wheel_scale_samples_.size());
    }
  }

  void updateVelocityMeasurement(
    const Eigen::Vector3d & velocity_meas,
    const Eigen::Vector3d & variance,
    const char * source,
    const builtin_interfaces::msg::Time & stamp)
  {
    const Eigen::Matrix3d covariance = variance.asDiagonal();
    updateVelocityMeasurement(velocity_meas, covariance, source, stamp);
  }

  void updateVelocityMeasurement(
    const Eigen::Vector3d & velocity_meas,
    const Eigen::Matrix3d & measurement_covariance,
    const char * source,
    const builtin_interfaces::msg::Time & stamp)
  {
    if (!velocity_meas.allFinite()) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), clock_, 5000, "skip invalid %s velocity measurement", source);
      return;
    }
    const Eigen::Vector3d innovation = velocity_meas - ekf_.getVelocity();
    const double innovation_norm = innovation.norm();
    core::EKFEstimator::ObservationJacobian3 jacobian;
    (void)core::EKFEstimator::worldVelocityObservation(ekf_.getState(), &jacobian);
    const double raw_nis = computePositionNis(
      innovation, measurement_covariance, ekf_.getCovariance(), jacobian);
    const auto quality = core::evaluateMeasurementQuality(
      true, innovation_norm, raw_nis,
      max_gnss_velocity_innovation_mps_, max_gnss_velocity_nis_);
    if (!quality.accepted) {
      publishMeasurementQuality(
        stamp, 2, false, quality.reason, innovation_norm, raw_nis, raw_nis);
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(),
        clock_,
        5000,
        "skip %s velocity update due to large innovation: |dv|=%f [m/s] > max=%f [m/s]",
        source,
        innovation_norm,
        max_gnss_velocity_innovation_mps_);
      return;
    }

    core::EKFEstimator::ObservationUpdateStatus status;
    double used_nis = raw_nis;
    bool accepted = true;
    if (propagate_gnss_velocity_cross_state_) {
      const auto diagnostics = ekf_.observationUpdateWorldVelocityWithCovariance(
        velocity_meas, measurement_covariance);
      status = diagnostics.status;
      used_nis = diagnostics.nis;
      accepted = diagnostics.accepted;
    } else {
      status = ekf_.observationUpdateVelocityWithStatus(
        velocity_meas, measurement_covariance.diagonal(), false);
    }
    if (status == core::EKFEstimator::ObservationUpdateStatus::kInvalidMeasurement) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), clock_, 5000,
        "skip EKF velocity update due to invalid %s velocity measurement", source);
    } else if (status == core::EKFEstimator::ObservationUpdateStatus::kInvalidVariance) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), clock_, 5000,
        "skip EKF velocity update due to invalid %s velocity variance", source);
    } else if (status == core::EKFEstimator::ObservationUpdateStatus::kNumericalFailure) {
      RCLCPP_ERROR_THROTTLE(
        node_.get_logger(), clock_, 5000,
        "skip EKF velocity update due to %s innovation or covariance numerical failure", source);
    }
    const bool update_succeeded =
      status == core::EKFEstimator::ObservationUpdateStatus::kUpdated && accepted;
    publishMeasurementQuality(
      stamp, 2, update_succeeded,
      update_succeeded ? core::MeasurementRejectReason::kNone :
      core::MeasurementRejectReason::kNumericalFailure,
      innovation_norm, raw_nis, used_nis);
  }

  void updateYawFromDopplerVelocity(
    const Eigen::Vector3d & velocity,
    const Eigen::Matrix3d & velocity_covariance,
    const builtin_interfaces::msg::Time & stamp)
  {
    const double speed_squared = velocity.x() * velocity.x() + velocity.y() * velocity.y();
    const double speed = std::sqrt(speed_squared);
    if (!std::isfinite(speed) || speed < min_gnss_doppler_course_speed_mps_) {
      return;
    }

    const double yaw_meas = std::atan2(velocity.y(), velocity.x());
    const Eigen::Quaterniond q_est = ekf_.getOrientation().normalized();
    const double yaw_est = getYawRadFromQuaternion(q_est);
    const double dyaw = wrapToPi(yaw_meas - yaw_est);
    if (std::fabs(dyaw) > max_gnss_course_dyaw_rad_) {
      publishMeasurementQuality(
        stamp, 3, false, core::MeasurementRejectReason::kInnovationMagnitude,
        std::fabs(dyaw), std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN());
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), clock_, 5000,
        "skip GNSS Doppler course yaw due to large innovation: |dyaw|=%f > max=%f [rad]",
        std::fabs(dyaw), max_gnss_course_dyaw_rad_);
      return;
    }

    const Eigen::Vector2d yaw_jacobian(
      -velocity.y() / speed_squared, velocity.x() / speed_squared);
    double yaw_variance = (yaw_jacobian.transpose() *
      velocity_covariance.topLeftCorner<2, 2>() * yaw_jacobian)(0, 0);
    if (!(yaw_variance > 0.0) || !std::isfinite(yaw_variance)) {
      yaw_variance = var_gnss_course_yaw_;
    }
    yaw_variance = std::max(yaw_variance, 1.0e-6);
    const Eigen::MatrixXd covariance = ekf_.getCovariance();
    const double raw_nis = computeYawNis(dyaw, yaw_variance, covariance);
    const auto quality = core::evaluateMeasurementQuality(
      true, std::fabs(dyaw), raw_nis,
      max_gnss_course_dyaw_rad_, max_gnss_course_yaw_nis_);
    if (!quality.accepted) {
      publishMeasurementQuality(
        stamp, 3, false, quality.reason, std::fabs(dyaw), raw_nis, raw_nis);
      return;
    }
    if (
      max_gnss_course_yaw_nis_ > 0.0 && std::isfinite(raw_nis) &&
      raw_nis > max_gnss_course_yaw_nis_)
    {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), clock_, 5000,
        "skip GNSS Doppler course yaw due to large NIS: nis=%f > max=%f",
        raw_nis, max_gnss_course_yaw_nis_);
      return;
    }
    const double variance_scale = computeAdaptiveVarianceScale(
      raw_nis,
      gnss_course_yaw_nis_adaptive_threshold_,
      max_gnss_course_yaw_variance_scale_);
    yaw_variance *= variance_scale;

    const StateSnapshot state_before = captureState();
    const auto yaw_diagnostics =
      ekf_.observationUpdateYawWithVariance(yaw_meas, yaw_variance);
    const auto status = yaw_diagnostics.status;
    int status_code = 1;
    if (status == core::EKFEstimator::ObservationUpdateStatus::kInvalidMeasurement) {
      status_code = -1;
    } else if (status == core::EKFEstimator::ObservationUpdateStatus::kInvalidVariance) {
      status_code = -2;
    } else if (status == core::EKFEstimator::ObservationUpdateStatus::kNumericalFailure) {
      status_code = -3;
    }
    publishUpdateDeltaDebug(stamp, 4, status_code, state_before, captureState());
    publishMeasurementQuality(
      stamp, 3, status_code == 1,
      status_code == 1 ? core::MeasurementRejectReason::kNone :
      core::MeasurementRejectReason::kNumericalFailure,
      std::fabs(dyaw), raw_nis, computeYawNis(dyaw, yaw_variance, covariance));
  }

  void publishSmoothedPose()
  {
    core::FixedLagSmoother::SmoothedNode node;
    std::size_t published = 0;
    while (smoother_->popSmoothed(node)) {
      published++;
      smoothed_pose_.header.stamp = stampFromSec(node.time);
      smoothed_pose_.header.frame_id = reference_frame_id_;
      smoothed_pose_.pose.position.x = node.state.position.x();
      smoothed_pose_.pose.position.y = node.state.position.y();
      smoothed_pose_.pose.position.z = node.state.position.z();
      smoothed_pose_.pose.orientation.x = node.state.orientation.x();
      smoothed_pose_.pose.orientation.y = node.state.orientation.y();
      smoothed_pose_.pose.orientation.z = node.state.orientation.z();
      smoothed_pose_.pose.orientation.w = node.state.orientation.w();
      smoothed_pose_pub_->publish(smoothed_pose_);
    }
    if (published > 0U) {
      RCLCPP_INFO_THROTTLE(
        node_.get_logger(), clock_, 5000,
        "published %zu smoothed poses; smoother nodes=%zu counters={imu=%llu "
        "applied=%llu emitted=%llu}",
        published, smoother_->nodeCount(),
        static_cast<std::uint64_t>(smoother_->counters().imu_samples),
        static_cast<std::uint64_t>(smoother_->counters().measurements_applied),
        static_cast<std::uint64_t>(smoother_->counters().smoothed_emitted));
    }
  }

  void broadcastPose()
  {
    if (!initial_pose_received_ || !has_received_input_) {
      publishEstimatorStatus();
      return;
    }
    const auto pose = ekf_.getPose();
    if (output_stamp_source_ == "ros_time") {
      current_pose_.header.stamp = node_.now();
    } else if (output_stamp_source_ == "imu") {
      if (!has_latest_imu_stamp_) {
        return;
      }
      current_pose_.header.stamp = latest_imu_stamp_;
    } else {
      current_pose_.header.stamp = current_stamp_;
    }
    current_pose_.header.frame_id = reference_frame_id_;
    current_pose_.pose.position.x = pose.position.x();
    current_pose_.pose.position.y = pose.position.y();
    current_pose_.pose.position.z = pose.position.z();
    current_pose_.pose.orientation.x = pose.orientation.x();
    current_pose_.pose.orientation.y = pose.orientation.y();
    current_pose_.pose.orientation.z = pose.orientation.z();
    current_pose_.pose.orientation.w = pose.orientation.w();
    current_pose_pub_->publish(current_pose_);
    ++published_pose_count_;

    if (smoother_ && smoothed_pose_pub_) {
      publishSmoothedPose();
    }
    const auto state = ekf_.getState();
    const Eigen::MatrixXd covariance = ekf_.getCovariance();
    nav_msgs::msg::Odometry odometry_msg;
    odometry_msg.header = current_pose_.header;
    odometry_msg.child_frame_id = robot_frame_id_;
    odometry_msg.pose.pose = current_pose_.pose;
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        odometry_msg.pose.covariance[row * 6 + col] = covariance(row, col);
        odometry_msg.pose.covariance[(row + 3) * 6 + col + 3] =
          covariance(row + 6, col + 6);
      }
    }
    const Eigen::Matrix3d world_from_body = state.orientation.normalized().toRotationMatrix();
    const Eigen::Vector3d body_velocity = world_from_body.transpose() * state.velocity;
    const Eigen::Matrix3d body_velocity_covariance =
      world_from_body.transpose() * covariance.block<3, 3>(3, 3) * world_from_body;
    odometry_msg.twist.twist.linear.x = body_velocity.x();
    odometry_msg.twist.twist.linear.y = body_velocity.y();
    odometry_msg.twist.twist.linear.z = body_velocity.z();
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        odometry_msg.twist.covariance[row * 6 + col] = body_velocity_covariance(row, col);
      }
      odometry_msg.twist.covariance[(row + 3) * 6 + row + 3] = 1.0e6;
    }
    current_odometry_pub_->publish(odometry_msg);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = current_pose_.header;
      transform.child_frame_id = robot_frame_id_;
      transform.transform.translation.x = current_pose_.pose.position.x;
      transform.transform.translation.y = current_pose_.pose.position.y;
      transform.transform.translation.z = current_pose_.pose.position.z;
      transform.transform.rotation = current_pose_.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }

    geometry_msgs::msg::Vector3Stamped gyro_bias_msg;
    gyro_bias_msg.header = current_pose_.header;
    gyro_bias_msg.header.frame_id = robot_frame_id_;
    gyro_bias_msg.vector.x = state.gyro_bias.x();
    gyro_bias_msg.vector.y = state.gyro_bias.y();
    gyro_bias_msg.vector.z = state.gyro_bias.z();
    current_gyro_bias_pub_->publish(gyro_bias_msg);

    geometry_msgs::msg::Vector3Stamped accel_bias_msg;
    accel_bias_msg.header = current_pose_.header;
    accel_bias_msg.header.frame_id = robot_frame_id_;
    accel_bias_msg.vector.x = state.accel_bias.x();
    accel_bias_msg.vector.y = state.accel_bias.y();
    accel_bias_msg.vector.z = state.accel_bias.z();
    current_accel_bias_pub_->publish(accel_bias_msg);
    publishEstimatorStatus();
  }

  EkfLocalizationComponent & node_;

  std::string reference_frame_id_;
  std::string robot_frame_id_;
  std::string initial_pose_topic_;
  std::string initial_pose_covariance_topic_;
  std::string imu_topic_;
  std::string odom_topic_;
  std::string odom_input_mode_;
  std::string gnss_pose_topic_;
  std::string gnss_input_type_;
  std::string gnss_navsatfix_topic_;
  std::string gnss_doppler_velocity_topic_;
  bool gnss_navsatfix_use_first_fix_as_origin_{true};
  bool gnss_navsatfix_use_position_covariance_{false};
  double gnss_navsatfix_min_variance_xy_{1.0e-4};
  double gnss_navsatfix_min_variance_z_{1.0e-4};
  double gnss_navsatfix_max_variance_xy_{100.0};
  double gnss_navsatfix_max_variance_z_{100.0};
  double gnss_navsatfix_origin_latitude_{std::numeric_limits<double>::quiet_NaN()};
  double gnss_navsatfix_origin_longitude_{std::numeric_limits<double>::quiet_NaN()};
  double gnss_navsatfix_origin_altitude_{std::numeric_limits<double>::quiet_NaN()};
  Eigen::Vector3d gnss_navsatfix_origin_ecef_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gnss_lever_arm_body_{Eigen::Vector3d::Zero()};
  bool compensate_gnss_delay_{false};
  double gnss_time_offset_sec_{0.0};
  double max_gnss_delay_compensation_sec_{0.5};
  bool has_gnss_navsatfix_origin_{false};
  int pub_period_{0};
  int input_qos_depth_{1};
  double input_reorder_window_sec_{0.0};
  int input_reorder_drain_sleep_usec_{0};
  int output_qos_depth_{10};
  bool enable_measurement_replay_{false};
  double measurement_history_duration_sec_{1.0};
  double max_future_measurement_wait_sec_{0.5};
  bool enable_fixed_lag_smoothing_{false};
  double fixed_lag_duration_sec_{5.0};
  int fixed_lag_node_subsample_{1};

  double var_imu_w_{0.0};
  double var_imu_acc_{0.0};
  bool use_continuous_process_noise_density_{false};
  bool use_second_order_state_transition_{false};
  bool use_second_order_process_noise_{false};
  std::string propagation_model_name_;
  double var_imu_gyro_bias_{0.0};
  double initial_imu_gyro_bias_covariance_{0.0};
  double tau_gyro_bias_sec_{0.0};
  double var_imu_acc_bias_{0.0};
  double initial_imu_acc_bias_covariance_{0.0};
  double initial_position_variance_xy_{100.0};
  double initial_position_variance_z_{100.0};
  double initial_velocity_variance_xy_{100.0};
  double initial_velocity_variance_z_{100.0};
  double initial_attitude_variance_rp_{100.0};
  double initial_attitude_variance_yaw_{100.0};
  bool enable_stationary_initialization_{false};
  double stationary_initialization_window_sec_{1.0};
  int stationary_initialization_min_samples_{50};
  double stationary_initialization_max_gyro_std_radps_{0.01};
  double stationary_initialization_max_accel_std_mps2_{0.1};
  double stationary_initialization_max_accel_norm_error_mps2_{0.3};
  double initial_dual_antenna_yaw_rad_{std::numeric_limits<double>::quiet_NaN()};
  double initial_dual_antenna_yaw_variance_{0.01};
  double tau_acc_bias_sec_{0.0};
  bool use_imu_orientation_{false};
  bool use_imu_orientation_covariance_{true};
  double var_imu_orientation_rpy_{0.0};
  bool use_flat_ground_{false};
  double var_flat_ground_rp_{0.0};
  bool use_nonholonomic_constraint_{false};
  double var_nhc_lateral_velocity_{0.05};
  double var_nhc_vertical_velocity_{0.02};
  double min_nhc_forward_speed_mps_{0.5};
  double nhc_adaptive_yaw_rate_radps_{0.5};
  double nhc_adaptive_lateral_accel_mps2_{1.5};
  double max_nhc_variance_scale_{100.0};
  std::string vehicle_model_plugin_{"kalman_filter_localization/GroundVehicleModel"};
  double nhc_slip_wheel_innovation_mps_{1.0};
  int nhc_recovery_samples_{5};
  bool use_wheel_speed_{false};
  std::string wheel_speed_topic_{"/wheel_speed"};
  double wheel_speed_scale_factor_{1.0};
  bool wheel_speed_use_nonholonomic_constraints_{false};
  bool wheel_speed_propagate_cross_state_{false};
  bool wheel_speed_nhc_only_during_gnss_outage_{false};
  double wheel_vertical_nhc_gnss_available_variance_scale_{1.0};
  double wheel_speed_gnss_outage_threshold_sec_{1.0};
  bool estimate_wheel_speed_scale_factor_{false};
  int wheel_scale_window_size_{100};
  int wheel_scale_min_samples_{20};
  double wheel_scale_min_speed_mps_{2.78};
  double wheel_scale_max_yaw_rate_radps_{0.0873};
  double wheel_scale_min_factor_{0.8};
  double wheel_scale_max_factor_{1.2};
  double wheel_scale_max_sample_age_sec_{0.2};
  std::string wheel_scale_reference_{"gnss"};
  std::vector<double> wheel_scale_samples_;
  bool has_latest_wheel_speed_{false};
  double latest_raw_wheel_speed_mps_{0.0};
  double latest_wheel_speed_time_{0.0};
  double latest_abs_yaw_rate_radps_{0.0};
  double var_wheel_speed_{0.04};
  double var_wheel_lateral_velocity_{0.05};
  double var_wheel_vertical_velocity_{0.02};
  double max_wheel_speed_innovation_mps_{5.0};
  bool use_zupt_{false};
  double zupt_max_angular_velocity_radps_{0.02};
  double zupt_max_acceleration_error_mps2_{0.2};
  double zupt_max_speed_mps_{0.3};
  double zupt_min_stationary_duration_sec_{0.5};
  double var_zupt_velocity_{0.01};
  bool use_zihr_{false};
  double var_zihr_gyro_{1.0e-5};
  bool enable_bias_observability_gate_{false};
  double bias_min_angular_excitation_radps_{0.1};
  double bias_min_acceleration_excitation_mps2_{0.5};
  bool has_stationary_start_time_{false};
  double stationary_start_time_{0.0};
  bool use_gnss_course_yaw_{false};
  double var_gnss_course_yaw_{0.0};
  double min_gnss_course_distance_m_{0.0};
  double min_gnss_course_speed_mps_{0.0};
  double max_gnss_course_dt_sec_{0.0};
  double max_gnss_course_dyaw_rad_{kPi};
  bool use_gnss_velocity_{false};
  bool use_gnss_doppler_velocity_{false};
  bool use_gnss_doppler_velocity_covariance_{true};
  bool use_gnss_doppler_course_yaw_{false};
  double min_gnss_doppler_course_speed_mps_{1.0};
  bool propagate_gnss_velocity_cross_state_{true};
  double var_gnss_velocity_xy_{0.0};
  double var_gnss_velocity_z_{0.0};
  double min_gnss_velocity_distance_m_{0.0};
  double max_gnss_velocity_dt_sec_{0.0};
  double gnss_doppler_fallback_timeout_sec_{1.0};
  double gnss_position_velocity_correlation_scale_{2.0};
  bool gnss_position_velocity_propagate_covariance_{true};
  double max_gnss_velocity_innovation_mps_{0.0};
  double max_gnss_velocity_nis_{0.0};
  double max_imu_dt_sec_{0.0};
  double gravity_mps2_{0.0};
  double var_gnss_xy_{0.0};
  double var_gnss_z_{0.0};
  double max_gnss_position_innovation_m_{0.0};
  double max_gnss_position_innovation_clip_m_{0.0};
  double max_gnss_position_nis_{0.0};
  double max_gnss_course_yaw_nis_{0.0};
  double gnss_position_nis_adaptive_threshold_{0.0};
  double max_gnss_position_variance_scale_{1.0};
  std::string gnss_position_robust_loss_name_{"none"};
  core::EKFEstimator::RobustLoss gnss_position_robust_loss_{
    core::EKFEstimator::RobustLoss::kNone};
  double gnss_position_robust_tuning_{2.5};
  double max_gnss_position_robust_variance_scale_{100.0};
  double gnss_position_reacquisition_dt_sec_{0.0};
  double gnss_position_reacquisition_variance_scale_{1.0};
  int gnss_position_reacquisition_update_count_{1};
  double gnss_position_reacquisition_covariance_cap_xy_{0.0};
  double gnss_position_reacquisition_covariance_cap_z_{0.0};
  double gnss_position_reacquisition_velocity_covariance_cap_xy_{0.0};
  double gnss_position_reacquisition_velocity_covariance_cap_z_{0.0};
  double gnss_position_reacquisition_attitude_covariance_cap_rp_{0.0};
  double gnss_position_reacquisition_attitude_covariance_cap_yaw_{0.0};
  bool gnss_position_reacquisition_reset_position_{false};
  bool gnss_position_reacquisition_reset_velocity_{false};
  double gnss_position_innovation_adaptive_threshold_m_{0.0};
  double max_gnss_position_innovation_variance_scale_{1.0};
  double gnss_course_yaw_nis_adaptive_threshold_{0.0};
  double max_gnss_course_yaw_variance_scale_{1.0};
  Eigen::Vector3d var_gnss_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d var_gnss_velocity_{Eigen::Vector3d::Zero()};
  double var_odom_xyz_{0.0};
  Eigen::Vector3d var_odom_{Eigen::Vector3d::Zero()};
  bool use_gnss_{false};
  bool use_odom_{false};
  bool enable_sensor_fault_isolation_{false};
  int sensor_fault_trip_count_{5};
  double sensor_fault_hold_sec_{5.0};
  bool publish_debug_topics_{false};
  std::string output_stamp_source_{"latest_input"};
  std::string output_publish_mode_{"timer"};
  std::string output_odometry_topic_;
  bool publish_tf_{false};

  bool initial_pose_received_{false};
  bool has_received_input_{false};
  bool has_latest_imu_stamp_{false};
  bool has_latest_gnss_{false};
  double latest_gnss_time_{std::numeric_limits<double>::quiet_NaN()};

  std::uint64_t received_initial_pose_count_{0};
  std::uint64_t received_imu_count_{0};
  std::uint64_t received_odom_count_{0};
  std::uint64_t received_gnss_pose_count_{0};
  std::uint64_t received_gnss_navsatfix_count_{0};
  std::uint64_t received_gnss_doppler_count_{0};
  std::uint64_t received_wheel_count_{0};
  std::uint64_t published_pose_count_{0};
  std::uint64_t measurement_quality_sequence_{0};
  std::uint64_t accepted_measurement_count_{0};
  std::uint64_t rejected_measurement_count_{0};
  std::uint64_t numerical_failure_count_{0};
  std::uint64_t isolated_measurement_count_{0};
  std::uint64_t replay_timing_sequence_{0};
  std::uint64_t imu_callback_count_{0};
  double imu_callback_total_us_{0.0};
  double imu_callback_max_us_{0.0};
  bool has_imu_wall_start_{false};
  bool has_imu_sensor_start_{false};
  std::chrono::steady_clock::time_point imu_wall_start_{};
  double first_imu_sensor_time_{0.0};
  double last_imu_sensor_time_{0.0};
  std::uint64_t reorder_late_input_count_{0};
  std::uint64_t reorder_processed_input_count_{0};
  std::uint64_t next_buffered_input_sequence_{0};
  std::int64_t latest_buffered_stamp_nanoseconds_{std::numeric_limits<std::int64_t>::min()};
  std::int64_t last_processed_stamp_nanoseconds_{0};
  int last_processed_priority_{0};
  bool has_processed_buffered_input_{false};
  std::priority_queue<BufferedInput, std::vector<BufferedInput>, BufferedInputLater>
  input_reorder_queue_;

  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::PoseStamped smoothed_pose_;
  rclcpp::Time current_stamp_;
  rclcpp::Time latest_imu_stamp_;

  // IMU time base (kept in ROS2 layer so the core EKF can operate on dt only).
  double previous_time_imu_{0.0};
  bool has_previous_time_imu_{false};

  // GNSS base point for course/heading estimation.
  bool has_course_base_gnss_{false};
  double course_base_gnss_time_{0.0};
  double course_base_gnss_x_{0.0};
  double course_base_gnss_y_{0.0};
  bool has_previous_velocity_gnss_{false};
  double previous_velocity_gnss_time_{0.0};
  Eigen::Vector3d previous_velocity_gnss_position_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d previous_velocity_gnss_covariance_{Eigen::Matrix3d::Identity()};
  bool has_latest_gnss_doppler_{false};
  double latest_gnss_doppler_time_{0.0};
  bool has_previous_gnss_position_time_{false};
  double previous_gnss_position_time_{0.0};
  int gnss_position_reacquisition_updates_remaining_{0};
  int gnss_position_reacquisition_consistent_count_{3};
  bool has_reacquisition_previous_position_{false};
  double reacquisition_previous_time_{0.0};
  Eigen::Vector3d reacquisition_previous_position_{Eigen::Vector3d::Zero()};

  core::EKFEstimator ekf_;
  core::ImuStationaryInitializer stationary_initializer_;
  core::YawInitializer yaw_initializer_;
  core::StationaryDetector stationary_detector_;
  core::GnssReacquisitionGate gnss_reacquisition_gate_;
  core::SensorFaultMonitor sensor_fault_monitor_;
  pluginlib::ClassLoader<core::VehicleModel> vehicle_model_loader_;
  std::shared_ptr<core::VehicleModel> vehicle_model_;
  bool stationary_initialization_complete_{false};
  std::unique_ptr<core::EskfReplay> replay_;
  core::EKFEstimator smoother_ekf_;
  std::unique_ptr<core::FixedLagSmoother> smoother_;
  core::StationaryDetector smoother_stationary_detector_;
  std::shared_ptr<core::VehicleModel> smoother_vehicle_model_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_initial_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    sub_initial_pose_covariance_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_gnss_pose_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gnss_navsatfix_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    sub_gnss_doppler_velocity_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    sub_wheel_speed_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr drain_input_buffer_service_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr smoothed_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr current_odometry_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr current_gyro_bias_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr current_accel_bias_pub_;
  rclcpp::Publisher<kalman_filter_localization_msgs::msg::EstimatorStatus>::SharedPtr
    estimator_status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<kalman_filter_localization_msgs::msg::MeasurementQuality>::SharedPtr
    typed_measurement_quality_pub_;
  rclcpp::Publisher<kalman_filter_localization_msgs::msg::ObservabilityStatus>::SharedPtr
    typed_observability_pub_;
  rclcpp::Publisher<kalman_filter_localization_msgs::msg::ReplayTiming>::SharedPtr
    typed_replay_timing_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_gnss_position_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_gnss_course_yaw_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_update_delta_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_replay_timing_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_initialization_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_observability_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    debug_measurement_quality_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Clock clock_;
  tf2_ros::Buffer tfbuffer_;
  tf2_ros::TransformListener listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  geometry_msgs::msg::PoseStamped current_pose_odom_;
  Eigen::Matrix4d previous_odom_mat_{Eigen::Matrix4d::Identity()};
  bool has_previous_odom_{false};
};

EkfLocalizationComponent::EkfLocalizationComponent(const rclcpp::NodeOptions & options)
: Node("ekf_localization", options),
  impl_(std::make_unique<Impl>(*this))
{
  impl_->init();
}

EkfLocalizationComponent::~EkfLocalizationComponent() = default;
}  // namespace kalman_filter_localization

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_filter_localization::EkfLocalizationComponent)

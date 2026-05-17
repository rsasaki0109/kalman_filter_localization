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
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <rclcpp/qos.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <kalman_filter_localization/core/ekf_estimator.hpp>
#include <kalman_filter_localization/core/odometry.hpp>

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
}  // namespace

struct EkfLocalizationComponent::Impl
{
  struct StateSnapshot
  {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpy{Eigen::Vector3d::Zero()};
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
    listener_(tfbuffer_)
  {
  }

  void init()
  {
    node_.declare_parameter("reference_frame_id", "map");
    node_.get_parameter("reference_frame_id", reference_frame_id_);
    node_.declare_parameter("robot_frame_id", "base_link");
    node_.get_parameter("robot_frame_id", robot_frame_id_);
    node_.declare_parameter("initial_pose_topic", node_.get_name() + std::string("/initial_pose"));
    node_.get_parameter("initial_pose_topic", initial_pose_topic_);
    node_.declare_parameter("imu_topic", node_.get_name() + std::string("/imu"));
    node_.get_parameter("imu_topic", imu_topic_);
    node_.declare_parameter("odom_topic", node_.get_name() + std::string("/odom"));
    node_.get_parameter("odom_topic", odom_topic_);
    node_.declare_parameter("gnss_pose_topic", node_.get_name() + std::string("/gnss_pose"));
    node_.get_parameter("gnss_pose_topic", gnss_pose_topic_);
    node_.declare_parameter("gnss_input_type", "pose");
    node_.get_parameter("gnss_input_type", gnss_input_type_);
    node_.declare_parameter("gnss_navsatfix_topic", node_.get_name() + std::string("/gnss/fix"));
    node_.get_parameter("gnss_navsatfix_topic", gnss_navsatfix_topic_);
    node_.declare_parameter("gnss_navsatfix_use_first_fix_as_origin", true);
    node_.get_parameter(
      "gnss_navsatfix_use_first_fix_as_origin", gnss_navsatfix_use_first_fix_as_origin_);
    node_.declare_parameter(
      "gnss_navsatfix_origin_latitude", std::numeric_limits<double>::quiet_NaN());
    node_.get_parameter("gnss_navsatfix_origin_latitude", gnss_navsatfix_origin_latitude_);
    node_.declare_parameter(
      "gnss_navsatfix_origin_longitude", std::numeric_limits<double>::quiet_NaN());
    node_.get_parameter("gnss_navsatfix_origin_longitude", gnss_navsatfix_origin_longitude_);
    node_.declare_parameter(
      "gnss_navsatfix_origin_altitude", std::numeric_limits<double>::quiet_NaN());
    node_.get_parameter("gnss_navsatfix_origin_altitude", gnss_navsatfix_origin_altitude_);

    node_.declare_parameter("pub_period", 10);
    node_.get_parameter("pub_period", pub_period_);
    node_.declare_parameter("var_imu_w", 0.01);
    node_.get_parameter("var_imu_w", var_imu_w_);
    node_.declare_parameter("var_imu_acc", 0.01);
    node_.get_parameter("var_imu_acc", var_imu_acc_);
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
    node_.declare_parameter("max_gnss_velocity_innovation_mps", 5.0);
    node_.get_parameter("max_gnss_velocity_innovation_mps", max_gnss_velocity_innovation_mps_);
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
    node_.declare_parameter("gnss_position_reacquisition_dt_sec", 0.0);
    node_.get_parameter("gnss_position_reacquisition_dt_sec", gnss_position_reacquisition_dt_sec_);
    node_.declare_parameter("gnss_position_reacquisition_variance_scale", 1.0);
    node_.get_parameter(
      "gnss_position_reacquisition_variance_scale",
      gnss_position_reacquisition_variance_scale_);
    node_.declare_parameter("gnss_position_reacquisition_update_count", 1);
    node_.get_parameter(
      "gnss_position_reacquisition_update_count", gnss_position_reacquisition_update_count_);
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
    node_.declare_parameter("publish_debug_topics", false);
    node_.get_parameter("publish_debug_topics", publish_debug_topics_);
    node_.declare_parameter("output_stamp_source", "latest_input");
    node_.get_parameter("output_stamp_source", output_stamp_source_);
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
    if (gnss_input_type_ != "pose" && gnss_input_type_ != "navsatfix") {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter gnss_input_type='%s'. fallback to default='pose'",
        gnss_input_type_.c_str());
      gnss_input_type_ = "pose";
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
    if (use_gnss_velocity_) {
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
        "invalid parameter max_gnss_position_innovation_m=%f. disabling GNSS position innovation gate",
        max_gnss_position_innovation_m_);
      max_gnss_position_innovation_m_ = 0.0;
    }
    if (
      !(max_gnss_position_innovation_clip_m_ >= 0.0) ||
      !std::isfinite(max_gnss_position_innovation_clip_m_))
    {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter max_gnss_position_innovation_clip_m=%f. disabling GNSS position innovation clipping",
        max_gnss_position_innovation_clip_m_);
      max_gnss_position_innovation_clip_m_ = 0.0;
    }
    if (
      !(gnss_position_nis_adaptive_threshold_ >= 0.0) ||
      !std::isfinite(gnss_position_nis_adaptive_threshold_))
    {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter gnss_position_nis_adaptive_threshold=%f. disabling adaptive GNSS position covariance",
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
        "invalid parameter gnss_position_reacquisition_dt_sec=%f. disabling GNSS reacquisition scaling",
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
    if (
      !(gnss_position_innovation_adaptive_threshold_m_ >= 0.0) ||
      !std::isfinite(gnss_position_innovation_adaptive_threshold_m_))
    {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter gnss_position_innovation_adaptive_threshold_m=%f. disabling innovation-norm GNSS position covariance",
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
        "invalid parameter gnss_course_yaw_nis_adaptive_threshold=%f. disabling adaptive GNSS course-yaw covariance",
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

    ekf_.setVarImuGyro(var_imu_w_);
    ekf_.setVarImuAcc(var_imu_acc_);
    ekf_.setVarImuGyroBias(var_imu_gyro_bias_);
    if (!ekf_.setInitialGyroBiasCovariance(initial_imu_gyro_bias_covariance_)) {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter initial_imu_gyro_bias_covariance=%f. fallback to default=0.0",
        initial_imu_gyro_bias_covariance_);
    }
    ekf_.setTauGyroBias(tau_gyro_bias_sec_);
    ekf_.setVarImuAccBias(var_imu_acc_bias_);
    if (!ekf_.setInitialAccelBiasCovariance(initial_imu_acc_bias_covariance_)) {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter initial_imu_acc_bias_covariance=%f. fallback to default=0.0",
        initial_imu_acc_bias_covariance_);
    }
    ekf_.setTauAccBias(tau_acc_bias_sec_);
    if (!ekf_.setMaxPredictionDtSec(max_imu_dt_sec_)) {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter max_imu_dt_sec=%f. fallback to default=%f",
        max_imu_dt_sec_, ekf_.getMaxPredictionDtSec());
    }
    if (!ekf_.setGravityZ(gravity_mps2_)) {
      RCLCPP_WARN(
        node_.get_logger(),
        "invalid parameter gravity_mps2=%f. fallback to default=%f",
        gravity_mps2_, ekf_.getGravityZ());
    }
    var_gnss_ << var_gnss_xy_, var_gnss_xy_, var_gnss_z_;
    var_gnss_velocity_ << var_gnss_velocity_xy_, var_gnss_velocity_xy_, var_gnss_velocity_z_;
    var_odom_ << var_odom_xyz_, var_odom_xyz_, var_odom_xyz_;

    // Setup Publisher
    const std::string output_pose_name = node_.get_name() + std::string("/current_pose");
    current_pose_pub_ =
      node_.create_publisher<geometry_msgs::msg::PoseStamped>(output_pose_name, 10);
    const std::string output_gyro_bias_name =
      node_.get_name() + std::string("/current_gyro_bias");
    current_gyro_bias_pub_ =
      node_.create_publisher<geometry_msgs::msg::Vector3Stamped>(output_gyro_bias_name, 10);
    const std::string output_accel_bias_name =
      node_.get_name() + std::string("/current_accel_bias");
    current_accel_bias_pub_ =
      node_.create_publisher<geometry_msgs::msg::Vector3Stamped>(output_accel_bias_name, 10);
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
    }

    // Setup Subscriber
    auto initial_pose_callback =
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void
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

        // Reset IMU dt integration base on re-initialization.
        has_previous_time_imu_ = false;
        previous_time_imu_ = 0.0;

        // Reset odom baseline too.
        current_pose_odom_ = current_pose_;
        has_previous_odom_ = false;
        previous_odom_mat_ = Eigen::Matrix4d::Identity();

        // Reset GNSS-derived baselines on re-initialization.
        has_course_base_gnss_ = false;
        has_previous_velocity_gnss_ = false;
      };

    auto imu_callback =
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) -> void
      {
        if (!initial_pose_received_) {
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
          predictUpdate(transformed_msg);
        } catch (tf2::TransformException & e) {
          RCLCPP_ERROR(node_.get_logger(), "%s", e.what());
          return;
        } catch (std::runtime_error & e) {
          RCLCPP_ERROR(node_.get_logger(), "%s", e.what());
          return;
        }
      };

    auto odom_callback =
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) -> void
      {
        if (!initial_pose_received_ || !use_odom_) {
          return;
        }
        Eigen::Affine3d affine;
        tf2::fromMsg(msg->pose.pose, affine);
        const Eigen::Matrix4d odom_mat = affine.matrix();
        if (!has_previous_odom_) {
          current_pose_odom_ = current_pose_;
          previous_odom_mat_ = odom_mat;
          has_previous_odom_ = true;
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
        measurementUpdate(pose, var_odom_);

        current_pose_odom_ = current_pose_;
        previous_odom_mat_ = odom_mat;
      };

    auto gnss_pose_callback =
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void
      {
        handleGnssPose(*msg);
      };

    auto gnss_navsatfix_callback =
      [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) -> void
      {
        if (!initial_pose_received_ || !use_gnss_) {
          return;
        }
        geometry_msgs::msg::PoseStamped pose_msg;
        if (convertNavSatFixToPose(*msg, pose_msg)) {
          handleGnssPose(pose_msg);
        }
      };

    sub_initial_pose_ =
      node_.create_subscription<geometry_msgs::msg::PoseStamped>(
      initial_pose_topic_, 1,
      initial_pose_callback);
    rclcpp::SensorDataQoS imu_qos;
    imu_qos.keep_last(1);
    sub_imu_ =
      node_.create_subscription<sensor_msgs::msg::Imu>(imu_topic_, imu_qos, imu_callback);
    sub_odom_ =
      node_.create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 1,
      odom_callback);
    if (gnss_input_type_ == "navsatfix") {
      rclcpp::SensorDataQoS gnss_qos;
      gnss_qos.keep_last(1);
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
        gnss_pose_topic_, 1,
        gnss_pose_callback);
    }
    const std::chrono::milliseconds period(pub_period_);
    timer_ = node_.create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {broadcastPose();});
  }

  void predictUpdate(const sensor_msgs::msg::Imu & imu_msg)
  {
    has_received_input_ = true;
    current_stamp_ = imu_msg.header.stamp;
    latest_imu_stamp_ = imu_msg.header.stamp;
    has_latest_imu_stamp_ = true;

    const double current_time_imu = imu_msg.header.stamp.sec +
      imu_msg.header.stamp.nanosec * 1e-9;

    if (!has_previous_time_imu_) {
      previous_time_imu_ = current_time_imu;
      has_previous_time_imu_ = true;
      return;
    }
    const double dt_imu = current_time_imu - previous_time_imu_;
    // Always advance the time base to allow recovery after large/invalid dt.
    previous_time_imu_ = current_time_imu;

    const Eigen::Vector3d gyro = Eigen::Vector3d(
      imu_msg.angular_velocity.x,
      imu_msg.angular_velocity.y,
      imu_msg.angular_velocity.z);
    const Eigen::Vector3d linear_acceleration = Eigen::Vector3d(
      imu_msg.linear_acceleration.x,
      imu_msg.linear_acceleration.y,
      imu_msg.linear_acceleration.z);

    const auto status = ekf_.predictionUpdateDt(dt_imu, gyro, linear_acceleration);
    if (status == core::EKFEstimator::PredictionUpdateStatus::kNonPositiveDt) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), clock_, 5000,
        "skip EKF prediction update due to non-positive IMU dt: %f [sec]", dt_imu);
      return;
    }
    if (status == core::EKFEstimator::PredictionUpdateStatus::kDtTooLarge) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), clock_, 5000,
        "skip EKF prediction update due to too large IMU dt: %f [sec]", dt_imu);
      return;
    }

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
      } else {
        publishUpdateDeltaDebug(imu_msg.header.stamp, 2, 1, state_before, captureState());
      }
    }
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

  void handleGnssPose(const geometry_msgs::msg::PoseStamped & pose_msg)
  {
    if (initial_pose_received_ && use_gnss_) {
      measurementUpdate(pose_msg, var_gnss_, true);
      if (use_gnss_velocity_) {
        updateVelocityFromGnss(pose_msg);
      }
      if (use_gnss_course_yaw_) {
        updateYawFromGnssCourse(pose_msg);
      }
    }
  }

  static double computePositionNis(
    const Eigen::Vector3d & innovation,
    const Eigen::Vector3d & variance,
    const Eigen::MatrixXd & covariance)
  {
    if (covariance.rows() < 3 || covariance.cols() < 3) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    Eigen::Matrix3d s = covariance.block<3, 3>(0, 0);
    s(0, 0) += variance.x();
    s(1, 1) += variance.y();
    s(2, 2) += variance.z();
    if (!s.allFinite()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const Eigen::FullPivLU<Eigen::Matrix3d> lu(s);
    if (!lu.isInvertible()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return innovation.transpose() * lu.inverse() * innovation;
  }

  static double computeYawNis(
    const double innovation,
    const double variance,
    const Eigen::MatrixXd & covariance)
  {
    constexpr int kErrorStateYawIndex = 8;
    if (covariance.rows() <= kErrorStateYawIndex || covariance.cols() <= kErrorStateYawIndex) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const double s = covariance(kErrorStateYawIndex, kErrorStateYawIndex) + variance;
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

  double computeGnssReacquisitionVarianceScale(
    const geometry_msgs::msg::PoseStamped & pose_msg)
  {
    if (!(gnss_position_reacquisition_dt_sec_ > 0.0) ||
      !(gnss_position_reacquisition_variance_scale_ > 1.0))
    {
      return 1.0;
    }

    const double t = stampToSec(pose_msg.header.stamp);
    double scale = 1.0;
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
      scale = gnss_position_reacquisition_variance_scale_;
      --gnss_position_reacquisition_updates_remaining_;
    }
    return scale;
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
    const Eigen::FullPivLU<Eigen::Matrix3d> lu(S);
    if (!lu.isInvertible()) {
      return extra;
    }

    const Eigen::Matrix<double, 15, 3> K = covariance.block<15, 3>(0, 0) * lu.inverse();
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
    const bool publish_gnss_debug = false)
  {
    has_received_input_ = true;
    current_stamp_ = pose_msg.header.stamp;
    const Eigen::Vector3d y = Eigen::Vector3d(
      pose_msg.pose.position.x,
      pose_msg.pose.position.y,
      pose_msg.pose.position.z);
    const Eigen::Vector3d position_before_update = ekf_.getPosition();
    const Eigen::Vector3d innovation = y - position_before_update;
    const Eigen::MatrixXd covariance = ekf_.getCovariance();
    const double raw_nis = computePositionNis(innovation, variance, covariance);
    const StateSnapshot state_before = captureState();
    const double innovation_norm = innovation.norm();
    const double reacquisition_variance_scale = publish_gnss_debug ?
      computeGnssReacquisitionVarianceScale(pose_msg) : 1.0;

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
    const double variance_scale =
      std::max({nis_variance_scale, innovation_variance_scale, reacquisition_variance_scale});
    Eigen::Vector3d update_innovation = innovation;
    if (
      max_gnss_position_innovation_clip_m_ > 0.0 &&
      std::isfinite(innovation_norm) &&
      innovation_norm > max_gnss_position_innovation_clip_m_)
    {
      update_innovation *= max_gnss_position_innovation_clip_m_ / innovation_norm;
    }
    const Eigen::Vector3d y_update = position_before_update + update_innovation;
    const Eigen::Vector3d used_variance = variance * variance_scale;
    const double used_nis = computePositionNis(update_innovation, used_variance, covariance);
    const UpdateDeltaExtra update_extra = computePositionUpdateExtra(
      covariance, innovation, update_innovation, used_variance, raw_nis, used_nis, variance_scale);

    const auto status = ekf_.observationUpdateWithStatus(y_update, used_variance);
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
    if (publish_gnss_debug) {
      publishGnssPositionDebug(
        pose_msg, innovation, used_nis, 1, used_variance, covariance, raw_nis, variance_scale);
      publishUpdateDeltaDebug(pose_msg.header.stamp, 1, 1, state_before, captureState(), update_extra);
    }
  }

  void updateYawFromGnssCourse(const geometry_msgs::msg::PoseStamped & pose_msg)
  {
    const double t = pose_msg.header.stamp.sec + pose_msg.header.stamp.nanosec * 1e-9;
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

    // Apply yaw rotation in the world frame (left multiplication). This yields the correct
    // body-frame innovation when the EKF uses right-multiplicative error state updates.
    const Eigen::Quaterniond dq_world_yaw(Eigen::AngleAxisd(dyaw, Eigen::Vector3d::UnitZ()));
    const Eigen::Quaterniond q_meas = (dq_world_yaw * q_est).normalized();
    const Eigen::Vector3d var_rpy_rad2(1.0e6, 1.0e6, used_yaw_variance);
    const auto status = ekf_.observationUpdateOrientationWithStatus(q_meas, var_rpy_rad2);
    int status_code = 1;
    if (status == core::EKFEstimator::ObservationUpdateStatus::kInvalidMeasurement) {
      status_code = -1;
    } else if (status == core::EKFEstimator::ObservationUpdateStatus::kInvalidVariance) {
      status_code = -2;
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

    // Update the base point after applying the measurement.
    course_base_gnss_time_ = t;
    course_base_gnss_x_ = x;
    course_base_gnss_y_ = y;
  }

  void updateVelocityFromGnss(const geometry_msgs::msg::PoseStamped & pose_msg)
  {
    const double t = pose_msg.header.stamp.sec + pose_msg.header.stamp.nanosec * 1e-9;
    const Eigen::Vector3d position(
      pose_msg.pose.position.x,
      pose_msg.pose.position.y,
      pose_msg.pose.position.z);
    if (!std::isfinite(t) || !position.allFinite()) {
      return;
    }

    if (!has_previous_velocity_gnss_) {
      previous_velocity_gnss_time_ = t;
      previous_velocity_gnss_position_ = position;
      has_previous_velocity_gnss_ = true;
      return;
    }

    const double dt = t - previous_velocity_gnss_time_;
    if (!(dt > 0.0) || dt > max_gnss_velocity_dt_sec_) {
      previous_velocity_gnss_time_ = t;
      previous_velocity_gnss_position_ = position;
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
      return;
    }

    const double innovation_norm = (velocity_meas - ekf_.getVelocity()).norm();
    if (innovation_norm > max_gnss_velocity_innovation_mps_) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(),
        clock_,
        5000,
        "skip GNSS velocity update due to large innovation: |dv|=%f [m/s] > max=%f [m/s]",
        innovation_norm,
        max_gnss_velocity_innovation_mps_);
      previous_velocity_gnss_time_ = t;
      previous_velocity_gnss_position_ = position;
      return;
    }

    const auto status = ekf_.observationUpdateVelocityWithStatus(
      velocity_meas, var_gnss_velocity_, propagate_gnss_velocity_cross_state_);
    if (status == core::EKFEstimator::ObservationUpdateStatus::kInvalidMeasurement) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), clock_, 5000,
        "skip EKF velocity update due to invalid GNSS velocity measurement");
    } else if (status == core::EKFEstimator::ObservationUpdateStatus::kInvalidVariance) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), clock_, 5000,
        "skip EKF velocity update due to invalid GNSS velocity variance");
    }

    previous_velocity_gnss_time_ = t;
    previous_velocity_gnss_position_ = position;
  }

  void broadcastPose()
  {
    if (!initial_pose_received_ || !has_received_input_) {
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

    const auto state = ekf_.getState();
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
  }

  EkfLocalizationComponent & node_;

  std::string reference_frame_id_;
  std::string robot_frame_id_;
  std::string initial_pose_topic_;
  std::string imu_topic_;
  std::string odom_topic_;
  std::string gnss_pose_topic_;
  std::string gnss_input_type_;
  std::string gnss_navsatfix_topic_;
  bool gnss_navsatfix_use_first_fix_as_origin_{true};
  double gnss_navsatfix_origin_latitude_{std::numeric_limits<double>::quiet_NaN()};
  double gnss_navsatfix_origin_longitude_{std::numeric_limits<double>::quiet_NaN()};
  double gnss_navsatfix_origin_altitude_{std::numeric_limits<double>::quiet_NaN()};
  Eigen::Vector3d gnss_navsatfix_origin_ecef_{Eigen::Vector3d::Zero()};
  bool has_gnss_navsatfix_origin_{false};
  int pub_period_{0};

  double var_imu_w_{0.0};
  double var_imu_acc_{0.0};
  double var_imu_gyro_bias_{0.0};
  double initial_imu_gyro_bias_covariance_{0.0};
  double tau_gyro_bias_sec_{0.0};
  double var_imu_acc_bias_{0.0};
  double initial_imu_acc_bias_covariance_{0.0};
  double tau_acc_bias_sec_{0.0};
  bool use_imu_orientation_{false};
  bool use_imu_orientation_covariance_{true};
  double var_imu_orientation_rpy_{0.0};
  bool use_flat_ground_{false};
  double var_flat_ground_rp_{0.0};
  bool use_gnss_course_yaw_{false};
  double var_gnss_course_yaw_{0.0};
  double min_gnss_course_distance_m_{0.0};
  double min_gnss_course_speed_mps_{0.0};
  double max_gnss_course_dt_sec_{0.0};
  double max_gnss_course_dyaw_rad_{kPi};
  bool use_gnss_velocity_{false};
  bool propagate_gnss_velocity_cross_state_{true};
  double var_gnss_velocity_xy_{0.0};
  double var_gnss_velocity_z_{0.0};
  double min_gnss_velocity_distance_m_{0.0};
  double max_gnss_velocity_dt_sec_{0.0};
  double max_gnss_velocity_innovation_mps_{0.0};
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
  double gnss_position_reacquisition_dt_sec_{0.0};
  double gnss_position_reacquisition_variance_scale_{1.0};
  int gnss_position_reacquisition_update_count_{1};
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
  bool publish_debug_topics_{false};
  std::string output_stamp_source_{"latest_input"};

  bool initial_pose_received_{false};
  bool has_received_input_{false};
  bool has_latest_imu_stamp_{false};

  geometry_msgs::msg::PoseStamped current_pose_;
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
  bool has_previous_gnss_position_time_{false};
  double previous_gnss_position_time_{0.0};
  int gnss_position_reacquisition_updates_remaining_{0};

  core::EKFEstimator ekf_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_initial_pose_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_gnss_pose_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gnss_navsatfix_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr current_gyro_bias_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr current_accel_bias_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_gnss_position_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_gnss_course_yaw_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_update_delta_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Clock clock_;
  tf2_ros::Buffer tfbuffer_;
  tf2_ros::TransformListener listener_;

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

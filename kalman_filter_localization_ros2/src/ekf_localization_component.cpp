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

#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <rclcpp/qos.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <kalman_filter_localization/core/ekf.hpp>

namespace kalman_filter_localization
{

struct EkfLocalizationComponent::Impl
{
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

    node_.declare_parameter("pub_period", 10);
    node_.get_parameter("pub_period", pub_period_);
    node_.declare_parameter("var_imu_w", 0.01);
    node_.get_parameter("var_imu_w", var_imu_w_);
    node_.declare_parameter("var_imu_acc", 0.01);
    node_.get_parameter("var_imu_acc", var_imu_acc_);
    node_.declare_parameter("var_gnss_xy", 0.1);
    node_.get_parameter("var_gnss_xy", var_gnss_xy_);
    node_.declare_parameter("var_gnss_z", 0.15);
    node_.get_parameter("var_gnss_z", var_gnss_z_);
    node_.declare_parameter("var_odom_xyz", 0.2);
    node_.get_parameter("var_odom_xyz", var_odom_xyz_);
    node_.declare_parameter("use_gnss", true);
    node_.get_parameter("use_gnss", use_gnss_);
    node_.declare_parameter("use_odom", false);
    node_.get_parameter("use_odom", use_odom_);

    ekf_.setVarImuGyro(var_imu_w_);
    ekf_.setVarImuAcc(var_imu_acc_);
    var_gnss_ << var_gnss_xy_, var_gnss_xy_, var_gnss_z_;
    var_odom_ << var_odom_xyz_, var_odom_xyz_, var_odom_xyz_;

    // Setup Publisher
    const std::string output_pose_name = node_.get_name() + std::string("/current_pose");
    current_pose_pub_ =
      node_.create_publisher<geometry_msgs::msg::PoseStamped>(output_pose_name, 10);

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
          transformed_msg.angular_velocity.x = w_out.vector.x;
          transformed_msg.angular_velocity.y = w_out.vector.y;
          transformed_msg.angular_velocity.z = w_out.vector.z;
          transformed_msg.linear_acceleration.x = acc_out.vector.x;
          transformed_msg.linear_acceleration.y = acc_out.vector.y;
          transformed_msg.linear_acceleration.z = acc_out.vector.z;
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
        Eigen::Matrix4d current_trans = current_affine.matrix();
        current_trans = current_trans * previous_odom_mat_.inverse() * odom_mat;

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
        if (initial_pose_received_ && use_gnss_) {
          measurementUpdate(*msg, var_gnss_);
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
    sub_gnss_pose_ =
      node_.create_subscription<geometry_msgs::msg::PoseStamped>(
      gnss_pose_topic_, 1,
      gnss_pose_callback);
    const std::chrono::milliseconds period(pub_period_);
    timer_ = node_.create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {broadcastPose();});
  }

  void predictUpdate(const sensor_msgs::msg::Imu & imu_msg)
  {
    current_stamp_ = imu_msg.header.stamp;

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
  }

  void measurementUpdate(
    const geometry_msgs::msg::PoseStamped & pose_msg,
    const Eigen::Vector3d & variance)
  {
    current_stamp_ = pose_msg.header.stamp;
    const Eigen::Vector3d y = Eigen::Vector3d(
      pose_msg.pose.position.x,
      pose_msg.pose.position.y,
      pose_msg.pose.position.z);

    const auto status = ekf_.observationUpdateWithStatus(y, variance);
    if (status == core::EKFEstimator::ObservationUpdateStatus::kInvalidMeasurement) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), clock_, 5000,
        "skip EKF observation update due to invalid measurement (NaN/Inf)");
      return;
    }
    if (status == core::EKFEstimator::ObservationUpdateStatus::kInvalidVariance) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), clock_, 5000,
        "skip EKF observation update due to invalid variance (need finite positive)");
      return;
    }
  }

  void broadcastPose()
  {
    if (!initial_pose_received_) {
      return;
    }
    const auto pose = ekf_.getPose();
    current_pose_.header.stamp = current_stamp_;
    current_pose_.header.frame_id = reference_frame_id_;
    current_pose_.pose.position.x = pose.position.x();
    current_pose_.pose.position.y = pose.position.y();
    current_pose_.pose.position.z = pose.position.z();
    current_pose_.pose.orientation.x = pose.orientation.x();
    current_pose_.pose.orientation.y = pose.orientation.y();
    current_pose_.pose.orientation.z = pose.orientation.z();
    current_pose_.pose.orientation.w = pose.orientation.w();
    current_pose_pub_->publish(current_pose_);
  }

  EkfLocalizationComponent & node_;

  std::string reference_frame_id_;
  std::string robot_frame_id_;
  std::string initial_pose_topic_;
  std::string imu_topic_;
  std::string odom_topic_;
  std::string gnss_pose_topic_;
  int pub_period_{0};

  double var_imu_w_{0.0};
  double var_imu_acc_{0.0};
  double var_gnss_xy_{0.0};
  double var_gnss_z_{0.0};
  Eigen::Vector3d var_gnss_{Eigen::Vector3d::Zero()};
  double var_odom_xyz_{0.0};
  Eigen::Vector3d var_odom_{Eigen::Vector3d::Zero()};
  bool use_gnss_{false};
  bool use_odom_{false};

  bool initial_pose_received_{false};

  geometry_msgs::msg::PoseStamped current_pose_;
  rclcpp::Time current_stamp_;

  // IMU time base (kept in ROS2 layer so the core EKF can operate on dt only).
  double previous_time_imu_{0.0};
  bool has_previous_time_imu_{false};

  core::EKFEstimator ekf_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_initial_pose_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_gnss_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
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

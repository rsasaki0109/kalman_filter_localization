#ifndef KFL_EKFL_COMPONENT_H_INCLUDED
#define KFL_EKFL_COMPONENT_H_INCLUDED

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define KFL_EKFL_EXPORT __attribute__ ((dllexport))
    #define KFL_EKFL_IMPORT __attribute__ ((dllimport))
  #else
    #define KFL_EKFL_EXPORT __declspec(dllexport)
    #define KFL_EKFL_IMPORT __declspec(dllimport)
  #endif
  #ifdef KFL_EKFL_BUILDING_DLL
    #define KFL_EKFL_PUBLIC KFL_EKFL_EXPORT
  #else
    #define KFL_EKFL_PUBLIC KFL_EKFL_IMPORT
  #endif
  #define KFL_EKFL_PUBLIC_TYPE KFL_EKFL_PUBLIC
  #define KFL_EKFL_LOCAL
#else
  #define KFL_EKFL_EXPORT __attribute__ ((visibility("default")))
  #define KFL_EKFL_IMPORT
  #if __GNUC__ >= 4
    #define KFL_EKFL_PUBLIC __attribute__ ((visibility("default")))
    #define KFL_EKFL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define KFL_EKFL_PUBLIC
    #define KFL_EKFL_LOCAL
  #endif
  #define KFL_EKFL_PUBLIC_TYPE
#endif

#if __cplusplus
} // extern "C"
#endif

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>

#include <rclcpp_components/register_node_macro.hpp>

#include <Eigen/Core>
//#include <Eigen/Dense>


namespace kalman_filter_localization
{
    class EkfLocalizationComponent: public rclcpp::Node
    {
    public:
        KFL_EKFL_PUBLIC
        explicit EkfLocalizationComponent(const rclcpp::NodeOptions & options);
    private:
        std::string reference_frame_id_;
        std::string robot_frame_id_;
        std::string initial_pose_topic_;
        std::string imu_topic_;
        std::string odom_topic_;
        std::string gnss_pose_topic_;
        int pub_period_;
        int num_state_;
        int num_error_state_;
        int num_obs_;
        double var_imu_w_;
        double var_imu_acc_;
        double var_gnss_xy_;
        double var_gnss_z_;
        double var_gnss_[3];
        double var_odom_xyz_;
        double var_odom_[3];
        bool use_gnss_;
        bool use_odom_;
        bool initial_pose_recieved_;
        double previous_time_imu_;
        rclcpp::Time current_stamp_;
        Eigen::VectorXd x_;
        Eigen::MatrixXd P_;
        Eigen::Vector3d gravity_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_initial_pose_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_gnss_pose_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        void predictUpdate(const sensor_msgs::msg::Imu input_imu_msg);
        void measurementUpdate(const geometry_msgs::msg::PoseStamped input_pose_msg, const double variance[]);
        void broadcastPose();
        geometry_msgs::msg::PoseStamped current_pose_;
        enum STATE{
          X  = 0,  Y = 1,  Z = 2,
          VX = 3, VY = 4, VZ = 5,
          QX = 6, QY = 7, QZ = 8, QW = 9,
        };
        enum ERROR_STATE{
          DX   = 0,   DY = 1,   DZ = 2,
          DVX  = 3,  DVY = 4,  DVZ = 5,
          DTHX = 6, DTHY = 7, DTHZ = 8,
        };
    };
}

#endif  //KFL_EKFL_COMPONENT_H_INCLUDED
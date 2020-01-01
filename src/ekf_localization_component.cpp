#include <kalman_filter_localization/ekf_localization_component.h>

namespace kalman_filter_localization
{
    EkfLocalizationComponent::EkfLocalizationComponent(const rclcpp::NodeOptions & options)
    : Node("ekf_localization", options)
    {
        /* Static Parameters */
        declare_parameter("initial_pose_topic",get_name() + std::string("/initial_pose"));
        get_parameter("initial_pose_topic",initial_pose_topic_);
        declare_parameter("gnss_pose_topic",get_name() + std::string("/input"));
        get_parameter("gnss_pose_topic",gnss_pose_topic_);

        /* Dynamic Parameters */
        declare_parameter("num_state",10);
        get_parameter("num_state",num_state_);
        declare_parameter("num_error_state",9);
        get_parameter("num_error_state",num_error_state_);
        declare_parameter("num_obs",3);
        get_parameter("num_obs",num_obs_);

        set_on_parameters_set_callback(
        [this](const std::vector<rclcpp::Parameter> params) -> rcl_interfaces::msg::SetParametersResult 
        {
            auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
            for(auto param : params)
            {
                if(param.get_name() == "num_state")
                {
                    if(num_state_ >0)
                    {
                        num_state_ = param.as_int();
                        results->successful = true;
                        results->reason = "";
                    }
                    else
                    {
                        results->successful = false;
                        results->reason = "number of states must over 0";
                    }
                }
                if(param.get_name() == "num_error_state")
                {
                    if(num_error_state_ >0)
                    {
                        num_error_state_ = param.as_int();
                        results->successful = true;
                        results->reason = "";
                    }
                    else
                    {
                        results->successful = false;
                        results->reason = "number of error states must over 0";
                    }
                }
                if(param.get_name() == "num_obs")
                {
                    if(num_obs_ >0)
                    {
                        num_obs_ = param.as_int();
                        results->successful = true;
                        results->reason = "";
                    }
                    else
                    {
                        results->successful = false;
                        results->reason = "number of observation must over 0";
                    }
                }
            }
            if(!results->successful)
            {
                results->successful = false;
                results->reason = "";
            }
            return *results;
        }
        );

        /* Setup Publisher */
        std::string output_pose_name = get_name() + std::string("/current_pose");
        current_pose_pub_ = 
            create_publisher<geometry_msgs::msg::PoseStamped>(output_pose_name,10);

        /* Setup Subscriber */
        initial_pose_recieved_ = false;

        auto initial_pose_callback =
        [this](const typename geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void
        {
            std::cout << "initial pose callback" << std::endl;
            initial_pose_recieved_ = true;
            current_pose_ = *msg;
        };

        auto imu_callback =
        [this](const typename sensor_msgs::msg::Imu::SharedPtr msg) -> void
        {
            if(initial_pose_recieved_){
                predictUpdate(*msg);
            }    
        };

        auto odom_callback =
        [this](const typename geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void
        {
            std::cout << msg->header.stamp.sec << std::endl;
        };

        auto gnss_pose_callback =
        [this](const typename geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void
        {
            if(initial_pose_recieved_){
                measurementUpdate(*msg);
                current_pose_pub_->publish(current_pose_);   
            }    
        };

        sub_initial_pose_ = 
            create_subscription<geometry_msgs::msg::PoseStamped>(initial_pose_topic_, 1,
                initial_pose_callback);
        sub_imu_ = 
            create_subscription<sensor_msgs::msg::Imu>(imu_topic_, 1,
                imu_callback);
        sub_odom_ = 
            create_subscription<geometry_msgs::msg::PoseStamped>(odom_topic_, 1,
                odom_callback);
        sub_gnss_pose_ = 
            create_subscription<geometry_msgs::msg::PoseStamped>(gnss_pose_topic_, 1,
                gnss_pose_callback);
    }   

    /* states
     * x  = [p v q] = [x y z vx vy vz qx qy qz qw]
     * dx = [dp dv dth] = [dx dy dz dvx dvy dvz dthx dthy dthz]
     * 
     * pos_k = pos_{k-1} + vel_k * dt + (1/2) * (Rot(q_{k-1}) acc_{k-1}^{imu} - g) *dt^2
     * vel_k = vel_{k-1} + (Rot(quat_{k-1})) acc_{k-1}^{imu} - g) *dt
     * quat_k = Rot(w_{k-1}^{imu}*dt)*quat_{k-1}
     * 
     * covariance
     * P_{k} = F_k P_{k-1} F_k^T + L Q_k L^T
     */
    void EkfLocalizationComponent::predictUpdate(const sensor_msgs::msg::Imu input_imu_msg)
    {
        std::cout << input_imu_msg.header.stamp.sec << std::endl;
        //MatrixXd::Identity(num_error_state_,num_error_state_) F;
        //std::cout << F << std::endl;
        return;
    }

    /* 
     * y = pobs = [xobs yobs zobs]
     * 
     * K = P_k H^T (H P_k H^T + R)^{-1}
     * 
     * dx = K (y_k - p_k )
     * 
     * p_x = p_{k-1} + dp_k
     * v_k = v_{k-1} + dv_k
     * q_k = Rot(dth) q_{k-1}
     * 
     * P_k = (I - KH)*P_{k-1} 
     */
    void EkfLocalizationComponent::measurementUpdate(const geometry_msgs::msg::PoseStamped input_pose_msg)
    {
        std::cout << input_pose_msg.header.stamp.sec << std::endl;

        return;
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_filter_localization::EkfLocalizationComponent)
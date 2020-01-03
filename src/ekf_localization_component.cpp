#include <kalman_filter_localization/ekf_localization_component.h>
#include <chrono>
using namespace std::chrono_literals;

namespace kalman_filter_localization
{
    EkfLocalizationComponent::EkfLocalizationComponent(const rclcpp::NodeOptions & options)
    : Node("ekf_localization", options)
    {
        /* Static Parameters */
        declare_parameter("reference_frame_id","map");
        get_parameter("reference_frame_id",reference_frame_id_);
        declare_parameter("initial_pose_topic",get_name() + std::string("/initial_pose"));
        get_parameter("initial_pose_topic",initial_pose_topic_);
        declare_parameter("imu_topic",get_name() + std::string("/imu"));
        get_parameter("imu_topic",imu_topic_);
        declare_parameter("odom_topic",get_name() + std::string("/odom"));
        get_parameter("odom_topic",odom_topic_);
        declare_parameter("gnss_pose_topic",get_name() + std::string("/gnss_pose"));
        get_parameter("gnss_pose_topic",gnss_pose_topic_);

        /* Dynamic Parameters */
        declare_parameter("frequency_pub",0.01);
        get_parameter("frequency_pub",frequency_pub_);
        declare_parameter("num_state",10);
        get_parameter("num_state",num_state_);
        declare_parameter("num_error_state",9);
        get_parameter("num_error_state",num_error_state_);
        declare_parameter("var_imu_w",0.01);
        get_parameter("var_imu_w",var_imu_w_);
        declare_parameter("var_imu_acc",0.01);
        get_parameter("var_imu_acc",var_imu_acc_);
        declare_parameter("var_gnss",0.1);
        get_parameter("var_gnss",var_gnss_);

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
        
        /* Init */
        previous_time_imu_ = -1;
        x_ = Eigen::VectorXd::Zero(num_state_);
        P_ = Eigen::MatrixXd::Identity(num_error_state_,num_error_state_) * 100;//todo:cross var
        gravity_ << 0,0,-9.81;

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
            x_(STATE::X) = 0;//current_pose_.pose.position.x;
            x_(STATE::Y) = 1;//current_pose_.pose.position.y;
            x_(STATE::Z) = 2;//current_pose_.pose.position.z;

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
                //current_pose_pub_->publish(current_pose_);   
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
        timer_ = create_wall_timer(0.01s, std::bind(&EkfLocalizationComponent::broadcastPose, this));
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
        std::cout << "predictUpdate" << std::endl;
        current_stamp_ = input_imu_msg.header.stamp;

        // dt_imu
        double current_time_imu = input_imu_msg.header.stamp.sec 
                                    + input_imu_msg.header.stamp.nanosec * 1e-9;
        if(previous_time_imu_ == -1){
            previous_time_imu_ = current_time_imu;
            return;
        }
        double dt_imu = current_time_imu - previous_time_imu_;
        previous_time_imu_ = current_time_imu;
        
        // state
        Eigen::Quaterniond previous_quat = Eigen::Quaterniond(x_(STATE::QW), x_(STATE::QX), x_(STATE::QY), x_(STATE::QZ));
        Eigen::MatrixXd rot_mat = previous_quat.toRotationMatrix();
        Eigen::Vector3d acc = Eigen::Vector3d(input_imu_msg.linear_acceleration.x, input_imu_msg.linear_acceleration.y, input_imu_msg.linear_acceleration.z);
        // pos
        x_.segment(STATE::X, 3) = x_.segment(STATE::X, 3) + dt_imu * x_.segment(STATE::VX, 3) 
                                            + 1/2 * dt_imu * dt_imu * (rot_mat * acc - gravity_); 
        // vel
        x_.segment(STATE::VX, 3) = x_.segment(STATE::VX, 3) + dt_imu * (rot_mat * acc - gravity_);
        // quat
        double thx = input_imu_msg.angular_velocity.x * dt_imu;
        double thy = input_imu_msg.angular_velocity.y * dt_imu;
        double thz = input_imu_msg.angular_velocity.z * dt_imu;
        double norm = sqrt(thx*thx + thy*thy + thz*thz);
        Eigen::Quaterniond quat_wdt;
        if (norm < 1e-5) quat_wdt = Eigen::Quaterniond(1, 0, 0, 0);
        else quat_wdt = Eigen::Quaterniond(cos(norm/2), sin(norm/2)*thx/norm, sin(norm/2)*thy/norm, sin(norm/2)*thz/norm);
        Eigen::Quaterniond predicted_quat = quat_wdt * previous_quat;
        x_.segment(STATE::QX, 4) = Eigen::Vector4d(predicted_quat.w(), predicted_quat.x(), predicted_quat.y(), predicted_quat.z());//!!!

        // F
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(num_error_state_,num_error_state_);
        F.block<3,3>(0,3) = dt_imu *  Eigen::MatrixXd::Identity(3,3);
        Eigen::Matrix3d acc_skew ;
        acc_skew << 0      ,-acc(2), acc(1),
                    acc(2) ,0      ,-acc(0),
                    -acc(1),acc(0) ,0;
        F.block<3,3>(3,6) = rot_mat *(-acc_skew) * dt_imu;
        
        // Q
        Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(6,6);
        Q.block<3,3>(0, 0) = var_imu_acc_ * Q.block<3,3>(0, 0);
        Q.block<3,3>(3, 3) = var_imu_w_ * Q.block<3,3>(3, 3);
        Q = Q * (dt_imu * dt_imu);

        // L
        Eigen::MatrixXd L = Eigen::MatrixXd::Zero(6,9);
        L.block<3,3>(3, 0) = Eigen::MatrixXd::Identity(3,3);
        L.block<3,3>(6, 3) = Eigen::MatrixXd::Identity(3,3);

        P_ = F * P_ * F.transpose() + L * Q * L.transpose();

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
        // error state
        current_stamp_ = input_pose_msg.header.stamp;
        Eigen::MatrixXd R = var_gnss_ * Eigen::MatrixXd::Identity(3,3);
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3,9);
        H.block<3,3>(0, 0) =  Eigen::MatrixXd::Identity(3,3);
        Eigen::MatrixXd K = P_ * H.transpose() * (H * P_ * H.transpose() + R).inverse(); 
        Eigen::Vector3d y = Eigen::Vector3d(input_pose_msg.pose.position.x, input_pose_msg.pose.position.y, input_pose_msg.pose.position.z);
        Eigen::VectorXd dx = K *(y - x_.segment(STATE::X, 3));

        // state
        x_.segment(STATE::X, 3) = x_.segment(STATE::X, 3) + dx.segment(STATE::X, 3);
        x_.segment(STATE::VX, 3) = x_.segment(STATE::VX, 3) + dx.segment(STATE::VX, 3);
        double norm_quat = sqrt(dx(6)*dx(6) + dx(7)*dx(7) + dx(8)*dx(8));
        if (norm_quat < 1e-10) x_.segment(STATE::QX, 4) = Eigen::Vector4d(cos(norm_quat/2), 0, 0, 0);
        else x_.segment(STATE::QX, 4) = Eigen::Vector4d(cos(norm_quat/2), sin(norm_quat/2) * dx(STATE::QX)/norm_quat,
                                                 sin(norm_quat/2) * dx(STATE::QY)/norm_quat, sin(norm_quat/2) * dx(STATE::QZ)/norm_quat);
        P_ = (Eigen::MatrixXd::Identity(9,9) - K*H) * P_;

        return;
    }

    void EkfLocalizationComponent::broadcastPose()
    {
        if(initial_pose_recieved_){
            current_pose_.header.stamp = current_stamp_;
            current_pose_.header.frame_id = reference_frame_id_;
            current_pose_.pose.position.x = x_(STATE::X);
            current_pose_.pose.position.y = x_(STATE::Y);
            current_pose_.pose.position.z = x_(STATE::Z);
            current_pose_.pose.orientation.x = x_(STATE::QX);
            current_pose_.pose.orientation.y = x_(STATE::QY);
            current_pose_.pose.orientation.z = x_(STATE::QZ);
            current_pose_.pose.orientation.w = x_(STATE::QW);
            current_pose_pub_->publish(current_pose_);   
        }
        return;
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_filter_localization::EkfLocalizationComponent)
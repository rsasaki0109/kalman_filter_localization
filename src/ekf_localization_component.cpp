#include <kalman_filter_localization/ekf_localization_component.h>
#include <chrono>
using namespace std::chrono_literals;

namespace kalman_filter_localization
{
    EkfLocalizationComponent::EkfLocalizationComponent(const rclcpp::NodeOptions & options)
    : Node("ekf_localization", options),
        clock_(RCL_ROS_TIME), 
        tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
        listener_(tfbuffer_)
    {
        declare_parameter("reference_frame_id","map");
        get_parameter("reference_frame_id",reference_frame_id_);
        declare_parameter("robot_frame_id","base_link");
        get_parameter("robot_frame_id",robot_frame_id_);
        declare_parameter("initial_pose_topic",get_name() + std::string("/initial_pose"));
        get_parameter("initial_pose_topic",initial_pose_topic_);
        declare_parameter("imu_topic",get_name() + std::string("/imu"));
        get_parameter("imu_topic",imu_topic_);
        declare_parameter("odom_topic",get_name() + std::string("/odom"));
        get_parameter("odom_topic",odom_topic_);
        declare_parameter("gnss_pose_topic",get_name() + std::string("/gnss_pose"));
        get_parameter("gnss_pose_topic",gnss_pose_topic_);

        declare_parameter("pub_period",10);
        get_parameter("pub_period",pub_period_);
        declare_parameter("num_state",10);
        get_parameter("num_state",num_state_);
        declare_parameter("num_error_state",9);
        get_parameter("num_error_state",num_error_state_);
        declare_parameter("var_imu_w",0.01);
        get_parameter("var_imu_w",var_imu_w_);
        declare_parameter("var_imu_acc",0.01);
        get_parameter("var_imu_acc",var_imu_acc_);
        declare_parameter("var_gnss_xy",0.1);
        get_parameter("var_gnss_xy",var_gnss_xy_);
        declare_parameter("var_gnss_z",0.15);
        get_parameter("var_gnss_z",var_gnss_z_);
        declare_parameter("var_odom_xyz",0.2);
        get_parameter("var_odom_xyz",var_odom_xyz_);
        declare_parameter("use_gnss",true);
        get_parameter("use_gnss",use_gnss_);
        declare_parameter("use_odom",false);
        get_parameter("use_odom",use_odom_);

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
        
        // Init 
        previous_time_imu_ = -1;
        x_ = Eigen::VectorXd::Zero(num_state_);
        x_(STATE::QW) = 1;
        P_ = Eigen::MatrixXd::Identity(num_error_state_,num_error_state_) * 100;//todo:set initial value properly
        gravity_ << 0,0,-9.80665;

        var_gnss_[0] = var_gnss_xy_;
        var_gnss_[1] = var_gnss_xy_;
        var_gnss_[2] = var_gnss_z_;
        var_odom_[0] = var_odom_xyz_;
        var_odom_[1] = var_odom_xyz_;
        var_odom_[2] = var_odom_xyz_;

        initial_pose_recieved_ = false;
        
        // Setup Publisher 
        std::string output_pose_name = get_name() + std::string("/current_pose");
        current_pose_pub_ = 
            create_publisher<geometry_msgs::msg::PoseStamped>(output_pose_name,10);

        // Setup Subscriber 
        auto initial_pose_callback =
        [this](const typename geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void
        {
            std::cout << "initial pose callback" << std::endl;
            initial_pose_recieved_ = true;
            current_pose_ = *msg;
            x_(STATE::X) = current_pose_.pose.position.x;
            x_(STATE::Y) = current_pose_.pose.position.y;
            x_(STATE::Z) = current_pose_.pose.position.z;
            x_(STATE::QX) = current_pose_.pose.orientation.x;
            x_(STATE::QY) = current_pose_.pose.orientation.y;
            x_(STATE::QZ) = current_pose_.pose.orientation.z;
            x_(STATE::QW) = current_pose_.pose.orientation.w;
            std::cout << "initial_x" << std::endl;
            std::cout << x_ << std::endl;
            std::cout << "----------------------" << std::endl;
        };

        auto imu_callback =
        [this](const typename sensor_msgs::msg::Imu::SharedPtr msg) -> void
        {
            if(initial_pose_recieved_){
                sensor_msgs::msg::Imu transformed_msg;
                try
                {
                    geometry_msgs::msg::Vector3Stamped acc_in, acc_out, w_in, w_out;
                    acc_in.vector.x = msg->linear_acceleration.x;
                    acc_in.vector.y = msg->linear_acceleration.y;
                    acc_in.vector.z = msg->linear_acceleration.z;
                    w_in.vector.x = msg->angular_velocity.x;
                    w_in.vector.y = msg->angular_velocity.y;
                    w_in.vector.z = msg->angular_velocity.z;
                    tf2::TimePoint time_point = tf2::TimePoint(
                        std::chrono::seconds(msg->header.stamp.sec) +
                        std::chrono::nanoseconds(msg->header.stamp.nanosec));
                    const geometry_msgs::msg::TransformStamped transform = tfbuffer_.lookupTransform(
                        robot_frame_id_, msg->header.frame_id, time_point);
                    tf2::doTransform(acc_in, acc_out, transform);
                    tf2::doTransform(w_in, w_out, transform);
                    transformed_msg.header.stamp = msg->header.stamp;
                    transformed_msg.angular_velocity.x = w_out.vector.x;
                    transformed_msg.angular_velocity.y = w_out.vector.y;
                    transformed_msg.angular_velocity.z = w_out.vector.z;
                    transformed_msg.linear_acceleration.x = acc_out.vector.x;
                    transformed_msg.linear_acceleration.y = acc_out.vector.y;
                    transformed_msg.linear_acceleration.z = acc_out.vector.z;       
                }
                catch (tf2::TransformException& e){
                    RCLCPP_ERROR(this->get_logger(),"%s",e.what());
                }
                predictUpdate(transformed_msg);
            }       
        };

        auto odom_callback =
        [this](const typename nav_msgs::msg::Odometry::SharedPtr msg) -> void
        {
            if(initial_pose_recieved_ && use_odom_){
                geometry_msgs::msg::PoseStamped pose;
                pose.header = msg->header;
                pose.pose.position.x = msg->pose.pose.position.x;
                pose.pose.position.y = msg->pose.pose.position.y;
                pose.pose.position.z = msg->pose.pose.position.z;
                measurementUpdate(pose, var_odom_); 
            }    
        };

        auto gnss_pose_callback =
        [this](const typename geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void
        {
            if(initial_pose_recieved_ && use_gnss_){
                measurementUpdate(*msg, var_gnss_); 
            }    
        };

        sub_initial_pose_ = 
            create_subscription<geometry_msgs::msg::PoseStamped>(initial_pose_topic_, 1,
                initial_pose_callback);
        sub_imu_ = 
            create_subscription<sensor_msgs::msg::Imu>(imu_topic_, 1,
                imu_callback);
        sub_odom_ = 
            create_subscription<nav_msgs::msg::Odometry>(odom_topic_, 1,
                odom_callback);
        sub_gnss_pose_ = 
            create_subscription<geometry_msgs::msg::PoseStamped>(gnss_pose_topic_, 1,
                gnss_pose_callback);
        std::chrono::milliseconds period(pub_period_);
        timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period), std::bind(&EkfLocalizationComponent::broadcastPose, this));
    }   

    /* state
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
                                            + 0.5 * dt_imu * dt_imu * (rot_mat * acc - gravity_); 
        // vel
        x_.segment(STATE::VX, 3) = x_.segment(STATE::VX, 3) + dt_imu * (rot_mat * acc - gravity_);
        // quat 
        Eigen::Quaterniond quat_wdt =  Eigen::Quaterniond(Eigen::AngleAxisd(input_imu_msg.angular_velocity.x * dt_imu, Eigen::Vector3d::UnitX()) 
                                        * Eigen::AngleAxisd(input_imu_msg.angular_velocity.y * dt_imu, Eigen::Vector3d::UnitY())    
                                        * Eigen::AngleAxisd(input_imu_msg.angular_velocity.z * dt_imu, Eigen::Vector3d::UnitZ()));  
        Eigen::Quaterniond predicted_quat = quat_wdt * previous_quat;
        x_.segment(STATE::QX, 4) = Eigen::Vector4d(predicted_quat.x(), predicted_quat.y(), predicted_quat.z(), predicted_quat.w());

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
        Eigen::MatrixXd L = Eigen::MatrixXd::Zero(9,6);
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
    void EkfLocalizationComponent::measurementUpdate(const geometry_msgs::msg::PoseStamped input_pose_msg, const double variance[])
    {
        // error state
        current_stamp_ = input_pose_msg.header.stamp;
        Eigen::Matrix3d R;
        R << variance[0], 0, 0,
             0, variance[1], 0 ,
             0, 0, variance[2];
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, num_error_state_);
        H.block<3,3>(0, 0) =  Eigen::MatrixXd::Identity(3,3);
        Eigen::MatrixXd K = P_ * H.transpose() * (H * P_ * H.transpose() + R).inverse(); 
        Eigen::Vector3d y = Eigen::Vector3d(input_pose_msg.pose.position.x, input_pose_msg.pose.position.y, input_pose_msg.pose.position.z);
        Eigen::VectorXd dx = K *(y - x_.segment(STATE::X, 3));

        // state
        x_.segment(STATE::X, 3) = x_.segment(STATE::X, 3) + dx.segment(ERROR_STATE::DX, 3);
        x_.segment(STATE::VX, 3) = x_.segment(STATE::VX, 3) + dx.segment(ERROR_STATE::DVX, 3);
        double norm_quat = sqrt(dx(ERROR_STATE::DTHX)*dx(ERROR_STATE::DTHX) + dx(ERROR_STATE::DTHY)*dx(ERROR_STATE::DTHY) + dx(ERROR_STATE::DTHZ)*dx(ERROR_STATE::DTHZ));
        if (norm_quat < 1e-10) x_.segment(STATE::QX, 4) = Eigen::Vector4d(0, 0, 0, cos(norm_quat/2));
        else x_.segment(STATE::QX, 4) = Eigen::Vector4d(sin(norm_quat/2) * dx(ERROR_STATE::DTHX)/norm_quat, sin(norm_quat/2) * dx(ERROR_STATE::DTHY)/norm_quat,
                                                 sin(norm_quat/2) * dx(ERROR_STATE::DTHZ)/norm_quat, cos(norm_quat/2));
                                                 
        P_ = (Eigen::MatrixXd::Identity(num_error_state_, num_error_state_) - K*H) * P_;

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
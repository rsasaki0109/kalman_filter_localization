import os

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ekf_param_dir = launch.substitutions.LaunchConfiguration(
        'ekf_param_dir',
        default=os.path.join(
            get_package_share_directory('kalman_filter_localization'),
            'param',
            'ekf.yaml'))

    ekf = launch_ros.actions.Node(
        package='kalman_filter_localization',
        node_executable='ekf_localization_node',
        parameters=[ekf_param_dir],
        output='screen'
        )
    
    tf = launch_ros.actions.Node(
        package='tf2_ros',
        node_executable='static_transform_publisher',
        remappings=[('/ekf_localization/gnss_pose','/gnss_pose'),('/ekf_localization/imu','/imu')],
        arguments=['0','0','0','0','0','0','1','base_link','imu_link']
        )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'ekf_param_dir',
            default_value=ekf_param_dir,
            description='Full path to ekf parameter file to load'),
        ekf,
        tf,
            ])

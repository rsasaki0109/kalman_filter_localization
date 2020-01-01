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
        node_executable='ekf_node',
        parameters=[ekf_param_dir],
        output='screen'
        )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'ekf_param_dir',
            default_value=ekf_param_dir,
            description='Full path to ekf parameter file to load'),
        ekf,
            ])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('vision60_bringup'),
        'config',
        'ekf.yaml',
    )
    return LaunchDescription([
        Node(
            package='vision60_mock',
            executable='nav2_mock_robot',
            output='screen',
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[params],
            remappings=[('odometry/filtered', '/state/odometry')],
        ),
    ])

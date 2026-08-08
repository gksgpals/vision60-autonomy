from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('vision60_bringup'),
        'config',
        'nav2_controller.yaml',
    )
    return LaunchDescription([
        Node(
            package='vision60_mock',
            executable='nav2_mock_robot',
            output='screen',
        ),
        Node(
            package='comm_recovery_manager',
            executable='recovery_path_follower',
            output='screen',
            parameters=[{'require_returning_state': False}],
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[params],
            remappings=[('cmd_vel', '/cmd_vel_nav')],
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            output='screen',
            parameters=[params],
            remappings=[
                ('cmd_vel', '/cmd_vel_nav'),
                ('cmd_vel_smoothed', '/cmd_vel_safe'),
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[params],
        ),
    ])

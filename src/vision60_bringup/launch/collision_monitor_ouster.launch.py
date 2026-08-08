import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('vision60_bringup'),
        'config',
        'collision_monitor_ouster.yaml',
    )
    return LaunchDescription([
        LifecycleNode(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            namespace='',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='comm_recovery_manager',
            executable='safety_velocity_gate',
            name='safety_velocity_gate',
            output='screen',
        ),
    ])

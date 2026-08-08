from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('vision60_bringup'))
    config_path = str(
        package_share / 'config' / 'vision60_bridge.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'allow_motion_output',
            default_value='false',
            description='Permit non-zero output to the selected transport',
        ),
        Node(
            package='vision60_bridge',
            executable='vision60_bridge',
            name='vision60_bridge',
            output='screen',
            parameters=[
                config_path,
                {
                    'allow_motion_output': LaunchConfiguration(
                        'allow_motion_output'
                    )
                },
            ],
        ),
    ])

"""Start a read-only Foxglove connection for the operator workstation."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('vision60_bringup')
    defaults = os.path.join(share, 'config', 'foxglove_bridge.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'address',
            default_value='127.0.0.1',
            description=(
                'WebSocket bind address. Keep loopback and use an SSH '
                'tunnel unless the robot network is already isolated.'
            ),
        ),
        DeclareLaunchArgument(
            'port',
            default_value='8765',
            description='Foxglove WebSocket port',
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[
                defaults,
                {
                    'address': LaunchConfiguration('address'),
                    'port': ParameterValue(
                        LaunchConfiguration('port'), value_type=int
                    ),
                },
            ],
        ),
    ])

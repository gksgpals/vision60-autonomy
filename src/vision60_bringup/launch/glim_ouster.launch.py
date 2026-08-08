from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('vision60_bringup'))
    default_glim_config = str(package_share / 'config' / 'glim_cpu_sample')
    ekf_config = str(package_share / 'config' / 'ekf_glim.yaml')

    glim_config = LaunchConfiguration('glim_config')
    use_ekf = LaunchConfiguration('use_ekf')

    return LaunchDescription([
        DeclareLaunchArgument(
            'glim_config',
            default_value=default_glim_config,
            description='Absolute GLIM configuration directory',
        ),
        DeclareLaunchArgument(
            'use_ekf',
            default_value='true',
            description='Start robot_localization after GLIM',
        ),
        Node(
            package='glim_ros',
            executable='glim_rosnode',
            name='glim_ros',
            output='screen',
            parameters=[{'config_path': glim_config}],
            remappings=[
                ('/os_cloud_node/points', '/ouster/points'),
                ('/os_cloud_node/imu', '/ouster/imu'),
                ('/glim_ros/odom', '/slam/odom'),
            ],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
            remappings=[('odometry/filtered', '/state/odometry')],
            condition=IfCondition(use_ekf),
        ),
    ])

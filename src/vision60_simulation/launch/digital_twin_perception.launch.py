"""Run camera-LiDAR mission perception in the Gazebo digital twin."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the simulated sensors, replaceable detector, and test harness."""
    share = get_package_share_directory('vision60_simulation')
    twin_launch = os.path.join(share, 'launch', 'digital_twin.launch.py')
    world = os.path.join(share, 'worlds', 'perception_test.sdf')
    output_dir = LaunchConfiguration('output_dir')
    return LaunchDescription([
        DeclareLaunchArgument('output_dir', default_value='/artifacts'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(twin_launch),
            launch_arguments={'headless': 'true', 'world_file': world}.items(),
        ),
        Node(
            package='mission_perception', executable='mission_perception',
            name='mission_perception', output='screen',
            parameters=[{
                'use_sim_time': True,
                'detector_backend': 'simulation_color',
                'mission_id': 'vision60_perception_test',
            }],
        ),
        Node(
            package='vision60_simulation',
            executable='digital_twin_perception_harness',
            name='digital_twin_perception_harness', output='screen',
            parameters=[{'use_sim_time': True, 'output_dir': output_dir}],
        ),
    ])

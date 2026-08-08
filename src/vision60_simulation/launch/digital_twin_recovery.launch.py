"""Run the production communication-recovery chain in the Gazebo twin."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    simulation_share = get_package_share_directory('vision60_simulation')
    digital_twin_launch = os.path.join(
        simulation_share, 'launch', 'digital_twin.launch.py'
    )
    output_dir = LaunchConfiguration('output_dir')

    return LaunchDescription([
        DeclareLaunchArgument(
            'output_dir', default_value='/artifacts',
            description='Directory for recovery video and report artifacts',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(digital_twin_launch),
            launch_arguments={'headless': 'true'}.items(),
        ),
        Node(
            package='vision60_mock',
            executable='vision60_mock',
            name='communication_fault_scenario',
            output='screen',
            parameters=[{
                'publish_robot_state': False,
                'degraded_after_s': 9.0,
                'disconnected_after_s': 12.0,
                'reconnected_after_s': -1.0,
                'available_channels': ['mock_backup_wifi'],
            }],
        ),
        Node(
            package='route_recorder',
            executable='route_recorder',
            name='digital_twin_route_recorder',
            output='screen',
            parameters=[{'distance_threshold_m': 0.12}],
            remappings=[('/slam/odom', '/vision60/odom')],
        ),
        Node(
            package='mission_logger',
            executable='mission_logger',
            name='digital_twin_mission_logger',
            output='screen',
            parameters=[{
                'database_path': '/tmp/vision60_digital_twin_mission.sqlite3',
                'transport': 'mock',
                'record_all_data': True,
            }],
            remappings=[('/state/odometry', '/vision60/odom')],
        ),
        Node(
            package='comm_recovery_manager',
            executable='comm_recovery_manager',
            name='digital_twin_comm_recovery_manager',
            output='screen',
            parameters=[{
                'lost_timeout_s': 1.0,
                'max_channel_switch_attempts': 2,
            }],
            remappings=[('/state/odometry', '/vision60/odom')],
        ),
        Node(
            package='comm_recovery_manager',
            executable='communication_channel_manager',
            name='digital_twin_channel_manager',
            output='screen',
            parameters=[{
                'candidate_channels': [
                    'mock_unavailable', 'mock_backup_wifi'
                ],
            }],
        ),
        Node(
            package='comm_recovery_manager',
            executable='recovery_path_follower',
            name='digital_twin_recovery_path_follower',
            output='screen',
            parameters=[{
                'require_returning_state': True,
                'odometry_topic': '/vision60/odom',
                'controller_id': 'RecoveryPath',
                'goal_checker_id': 'recovery_goal_checker',
            }],
        ),
        Node(
            package='comm_recovery_manager',
            executable='reentry_path_follower',
            name='digital_twin_reentry_path_follower',
            output='screen',
            parameters=[{
                'reentry_speed_mps': 0.10,
                'odometry_topic': '/vision60/odom',
                'controller_id': 'RecoveryPath',
                'goal_checker_id': 'goal_checker',
            }],
        ),
        Node(
            package='vision60_simulation',
            executable='digital_twin_recovery_harness',
            name='digital_twin_recovery_harness',
            output='screen',
            parameters=[{'output_dir': output_dir}],
        ),
    ])

"""Run dynamic avoidance and communication recovery in one Gazebo test."""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('vision60_simulation')
    runtime_dir = tempfile.mkdtemp(prefix='vision60_integrated_')
    obstacle_launch = os.path.join(
        share, 'launch', 'digital_twin_obstacle_avoidance.launch.py'
    )
    integrated_output_dir = LaunchConfiguration('integrated_output_dir')

    return LaunchDescription([
        DeclareLaunchArgument(
            'integrated_output_dir', default_value='/artifacts'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(obstacle_launch),
            launch_arguments={
                'dynamic_mode': 'true',
                'require_motion_permission': 'true',
                'target_distance_m': '4.80',
                'output_dir': os.path.join(runtime_dir, 'obstacle_probe'),
            }.items(),
        ),
        Node(
            package='vision60_mock', executable='vision60_mock',
            name='integrated_communication_fault', output='screen',
            parameters=[{
                'publish_robot_state': False,
                'degraded_after_s': 999.0,
                'disconnected_after_s': 999.0,
                'reconnected_after_s': -1.0,
                'external_odom_topic': '/vision60/odom',
                'degraded_after_x_m': 3.00,
                'disconnected_after_x_m': 4.00,
                'available_channels': ['mock_backup_wifi'],
            }],
        ),
        Node(
            package='route_recorder', executable='route_recorder',
            name='integrated_route_recorder', output='screen',
            parameters=[{'distance_threshold_m': 0.10}],
            remappings=[('/slam/odom', '/vision60/odom')],
        ),
        Node(
            package='mission_logger', executable='mission_logger',
            name='integrated_mission_logger', output='screen',
            parameters=[{
                'database_path': os.path.join(
                    runtime_dir, 'mission.sqlite3'
                ),
                'transport': 'mock',
                'record_all_data': True,
            }],
            remappings=[('/state/odometry', '/vision60/odom')],
        ),
        Node(
            package='comm_recovery_manager',
            executable='comm_recovery_manager',
            name='integrated_comm_recovery_manager', output='screen',
            parameters=[{
                'lost_timeout_s': 1.0,
                'max_channel_switch_attempts': 2,
            }],
            remappings=[('/state/odometry', '/vision60/odom')],
        ),
        Node(
            package='comm_recovery_manager',
            executable='communication_channel_manager',
            name='integrated_channel_manager', output='screen',
            parameters=[{
                'candidate_channels': [
                    'mock_unavailable', 'mock_backup_wifi'
                ],
            }],
        ),
        Node(
            package='comm_recovery_manager',
            executable='recovery_path_follower',
            name='integrated_recovery_path_follower', output='screen',
            parameters=[{
                'require_returning_state': True,
                'odometry_topic': '/vision60/odom',
                'waypoint_tolerance_m': 0.65,
                'controller_id': 'RecoveryPath',
                'goal_checker_id': 'recovery_goal_checker',
            }],
        ),
        Node(
            package='comm_recovery_manager',
            executable='reentry_path_follower',
            name='integrated_reentry_path_follower', output='screen',
            parameters=[{
                'reentry_speed_mps': 0.10,
                'odometry_topic': '/vision60/odom',
                'waypoint_tolerance_m': 0.18,
                'controller_id': 'RecoveryPath',
                'goal_checker_id': 'goal_checker',
            }],
        ),
        Node(
            package='vision60_simulation',
            executable='digital_twin_integrated_recovery_harness',
            name='digital_twin_integrated_recovery_harness',
            output='screen',
            parameters=[{
                'output_dir': integrated_output_dir,
                'timeout_s': 105.0,
            }],
        ),
    ])

"""Run frontier exploration with communication-loss recovery in Gazebo."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the complete exploration, safety, and recovery graph."""
    share = get_package_share_directory('vision60_simulation')
    obstacle_launch = os.path.join(
        share, 'launch', 'digital_twin_obstacle_avoidance.launch.py'
    )
    params = os.path.join(
        share, 'config', 'frontier_exploration.yaml'
    )
    frontier_bt = os.path.join(
        share, 'config', 'frontier_navigation.xml'
    )
    output_dir = LaunchConfiguration('output_dir')

    return LaunchDescription([
        DeclareLaunchArgument('output_dir', default_value='/artifacts'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(obstacle_launch),
            launch_arguments={
                'params_file': params,
                'run_probe': 'false',
                'dynamic_mode': 'false',
                'require_motion_permission': 'true',
            }.items(),
        ),
        Node(
            package='nav2_bt_navigator', executable='bt_navigator',
            name='bt_navigator', output='screen', parameters=[
                params,
                {
                    'default_nav_to_pose_bt_xml': frontier_bt,
                    'default_nav_through_poses_bt_xml': frontier_bt,
                },
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_frontier', output='screen',
            parameters=[params],
        ),
        Node(
            package='explore_lite', executable='explore',
            name='explore_node', output='screen', parameters=[params],
        ),
        Node(
            package='comm_recovery_manager',
            executable='exploration_safety_gate',
            name='exploration_safety_gate', output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='vision60_mock', executable='vision60_mock',
            name='frontier_communication_fault', output='screen',
            parameters=[{
                'publish_robot_state': False,
                'degraded_after_s': 999.0,
                'disconnected_after_s': 999.0,
                'reconnected_after_s': -1.0,
                'external_odom_topic': '/vision60/odom',
                'degraded_after_x_m': 1.10,
                'disconnected_after_x_m': 1.70,
                'available_channels': ['mock_backup_wifi'],
            }],
        ),
        Node(
            package='route_recorder', executable='route_recorder',
            name='frontier_route_recorder', output='screen',
            parameters=[{'distance_threshold_m': 0.10}],
            remappings=[('/slam/odom', '/vision60/odom')],
        ),
        Node(
            package='mission_logger', executable='mission_logger',
            name='frontier_mission_logger', output='screen',
            parameters=[{
                'database_path': '/tmp/vision60_frontier_mission.sqlite3',
                'transport': 'mock',
                'record_all_data': True,
            }],
            remappings=[('/state/odometry', '/vision60/odom')],
        ),
        Node(
            package='comm_recovery_manager',
            executable='comm_recovery_manager',
            name='frontier_comm_recovery_manager', output='screen',
            parameters=[{
                'lost_timeout_s': 1.0,
                'max_channel_switch_attempts': 2,
            }],
            remappings=[('/state/odometry', '/vision60/odom')],
        ),
        Node(
            package='comm_recovery_manager',
            executable='communication_channel_manager',
            name='frontier_channel_manager', output='screen',
            parameters=[{
                'candidate_channels': [
                    'mock_unavailable', 'mock_backup_wifi'
                ],
            }],
        ),
        Node(
            package='comm_recovery_manager',
            executable='recovery_path_follower',
            name='frontier_recovery_path_follower', output='screen',
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
            name='frontier_reentry_path_follower', output='screen',
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
            executable='digital_twin_frontier_harness',
            name='digital_twin_frontier_harness', output='screen',
            parameters=[{
                'use_sim_time': True,
                'output_dir': output_dir,
                'timeout_s': 135.0,
            }],
        ),
    ])

"""Run Nav2 LiDAR obstacle avoidance in the Gazebo digital twin."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    share = get_package_share_directory('vision60_simulation')
    default_params = os.path.join(
        share, 'config', 'nav2_obstacle_avoidance.yaml'
    )
    params = LaunchConfiguration('params_file')
    digital_twin = os.path.join(share, 'launch', 'digital_twin.launch.py')
    dynamic_world = os.path.join(
        share, 'worlds', 'dynamic_obstacle_test.sdf'
    )
    output_dir = LaunchConfiguration('output_dir')
    dynamic_mode = LaunchConfiguration('dynamic_mode')
    require_motion_permission = LaunchConfiguration(
        'require_motion_permission'
    )
    target_distance_m = LaunchConfiguration('target_distance_m')
    run_probe = LaunchConfiguration('run_probe')
    dynamic_model = os.path.join(
        share, 'models', 'dynamic_crate', 'model.sdf'
    )

    return LaunchDescription([
        DeclareLaunchArgument('output_dir', default_value='/artifacts'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('run_probe', default_value='true'),
        DeclareLaunchArgument('dynamic_mode', default_value='false'),
        DeclareLaunchArgument(
            'require_motion_permission', default_value='false'
        ),
        DeclareLaunchArgument('target_distance_m', default_value='4.10'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(digital_twin),
            launch_arguments={'headless': 'true'}.items(),
            condition=UnlessCondition(dynamic_mode),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(digital_twin),
            launch_arguments={
                'headless': 'true',
                'world_file': dynamic_world,
            }.items(),
            condition=IfCondition(dynamic_mode),
        ),
        Node(
            package='ros_gz_sim', executable='create',
            name='dynamic_crate_spawner', output='screen',
            arguments=[
                '-world', 'vision60_disaster_test',
                '-file', dynamic_model,
                '-name', 'dynamic_crate',
                '-x', '2.00', '-y', '0.95', '-z', '0.30',
            ],
            condition=IfCondition(dynamic_mode),
        ),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='dynamic_crate_command_bridge', output='screen',
            arguments=[
                '/model/dynamic_crate/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            ],
            remappings=[
                ('/model/dynamic_crate/cmd_vel', '/dynamic_crate/cmd_vel_world'),
            ],
            condition=IfCondition(dynamic_mode),
        ),
        Node(
            package='nav2_planner', executable='planner_server',
            name='planner_server', output='screen', parameters=[params],
        ),
        Node(
            package='nav2_controller', executable='controller_server',
            name='controller_server', output='screen', parameters=[params],
            remappings=[('cmd_vel', '/cmd_vel_nav')],
        ),
        Node(
            package='nav2_velocity_smoother', executable='velocity_smoother',
            name='velocity_smoother', output='screen', parameters=[params],
            remappings=[
                ('cmd_vel', '/cmd_vel_nav'),
                ('cmd_vel_smoothed', '/cmd_vel_smoothed'),
            ],
        ),
        LifecycleNode(
            package='nav2_collision_monitor', executable='collision_monitor',
            name='collision_monitor', namespace='', output='screen',
            parameters=[params],
        ),
        Node(
            package='comm_recovery_manager', executable='safety_velocity_gate',
            name='digital_twin_safety_velocity_gate', output='screen',
            parameters=[{
                'use_sim_time': True,
                'lidar_timeout_s': 0.6,
                'command_timeout_s': 0.35,
                'require_motion_permission': require_motion_permission,
            }],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[params],
        ),
        Node(
            package='vision60_simulation',
            executable='digital_twin_obstacle_probe',
            name='digital_twin_obstacle_probe', output='screen',
            parameters=[{
                'use_sim_time': True,
                'output_dir': output_dir,
                'dynamic_mode': dynamic_mode,
                'target_distance_m': target_distance_m,
            }],
            condition=IfCondition(run_probe),
        ),
    ])

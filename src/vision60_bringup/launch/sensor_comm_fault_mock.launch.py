"""Inject LiDAR delay or loss while the communication link disconnects."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def mode_is(expected: str):
    return IfCondition(
        PythonExpression([
            "'", LaunchConfiguration('fault_mode'), "' == '", expected, "'",
        ])
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'fault_mode',
            default_value='delay',
            choices=['delay', 'drop'],
            description='LiDAR fault injected by topic_tools',
        ),
        Node(
            package='vision60_mock',
            executable='vision60_mock',
            name='communication_fault_mock',
            output='screen',
            parameters=[{
                'publish_robot_state': False,
                'degraded_after_s': 1.0,
                'disconnected_after_s': 2.0,
                'reconnected_after_s': -1.0,
            }],
        ),
        Node(
            package='vision60_mock',
            executable='mock_lidar_heartbeat',
            name='raw_lidar_mock',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'output_topic': '/ouster/points_raw',
            }],
        ),
        Node(
            package='topic_tools',
            executable='delay',
            name='lidar_delay_fault',
            output='screen',
            condition=mode_is('delay'),
            parameters=[{
                'input_topic': '/ouster/points_raw',
                'output_topic': '/ouster/points',
                'delay': 0.8,
                'use_wall_clock': True,
            }],
        ),
        Node(
            package='topic_tools',
            executable='drop',
            name='lidar_drop_fault',
            output='screen',
            condition=mode_is('drop'),
            parameters=[{
                'input_topic': '/ouster/points_raw',
                'output_topic': '/ouster/points',
                'X': 1,
                'Y': 1,
            }],
        ),
        Node(
            package='comm_recovery_manager',
            executable='safety_velocity_gate',
            name='safety_velocity_gate',
            output='screen',
            parameters=[{
                'lidar_timeout_s': 0.5,
                'command_timeout_s': 0.3,
            }],
        ),
    ])

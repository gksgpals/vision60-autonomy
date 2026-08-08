"""Full mock chain from Nav2 safe velocity to recorded-route recovery.

    Nav2 controller / velocity smoother
        -> Collision Monitor
        -> LiDAR freshness safety gate
        -> /cmd_vel_safe
        -> vision60_bridge (mock transport)
        -> /vision60/odom
        -> robot_localization -> /state/odometry + odom -> base_link
        -> route_recorder (real travelled route)
        -> link loss -> confirmed stop -> RETURNING
        -> /mission/recovery_path -> Nav2 FollowPath -> reverse drive

The mock scenario node only supplies /communication/state here. The bridge is
the single owner of /vision60/safety_state and /vision60/request_safe_stop.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node

ODOMETRY_REMAP = ('/slam/odom', '/state/odometry')


def generate_launch_description():
    share = get_package_share_directory('vision60_bringup')
    nav2_params = os.path.join(share, 'config', 'nav2_controller.yaml')
    bridge_params = os.path.join(share, 'config', 'vision60_bridge.yaml')
    ekf_params = os.path.join(share, 'config', 'ekf_bridge_mock.yaml')
    collision_params = os.path.join(
        share,
        'config',
        'collision_monitor_ouster.yaml',
    )
    twist_mux_params = os.path.join(
        share,
        'config',
        'twist_mux.yaml',
    )

    allow_motion_output = LaunchConfiguration('allow_motion_output')
    degraded_after_s = LaunchConfiguration('degraded_after_s')
    disconnected_after_s = LaunchConfiguration('disconnected_after_s')
    reconnected_after_s = LaunchConfiguration('reconnected_after_s')

    return LaunchDescription([
        DeclareLaunchArgument(
            'allow_motion_output',
            default_value='false',
            description='Permit non-zero output to the mock transport',
        ),
        # The windows must outlast Nav2 lifecycle activation so that the
        # outbound leg records safe route points while the link is NORMAL
        # and keeps driving through DEGRADED before the link drops.
        DeclareLaunchArgument(
            'degraded_after_s',
            default_value='20.0',
            description='Mock scenario: seconds until the link degrades',
        ),
        DeclareLaunchArgument(
            'disconnected_after_s',
            default_value='32.0',
            description='Mock scenario: seconds until the link drops',
        ),
        DeclareLaunchArgument(
            'reconnected_after_s',
            default_value='-1.0',
            description=(
                'Primary-link recovery time; negative forces alternate '
                'channel testing'
            ),
        ),

        # Communication scenario only. The bridge owns robot safety state.
        Node(
            package='vision60_mock',
            executable='vision60_mock',
            name='vision60_mock',
            output='screen',
            parameters=[{
                'publish_robot_state': False,
                'degraded_after_s': degraded_after_s,
                'disconnected_after_s': disconnected_after_s,
                'reconnected_after_s': reconnected_after_s,
            }],
        ),
        Node(
            package='vision60_mock',
            executable='mock_lidar_heartbeat',
            name='mock_lidar_heartbeat',
            output='screen',
            parameters=[{
                'frame_id': 'os_sensor',
                'slowdown_after_s': 10.0,
                'slowdown_duration_s': 3.0,
                'stop_after_s': 15.0,
                'stop_duration_s': 3.0,
            }],
        ),

        # SDK boundary. Applies every safety gate before the transport.
        Node(
            package='vision60_bridge',
            executable='vision60_bridge',
            name='vision60_bridge',
            output='screen',
            parameters=[
                bridge_params,
                {'allow_motion_output': allow_motion_output},
            ],
        ),

        # Stands in for GLIM + body odometry fusion on the real robot.
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params],
            remappings=[('odometry/filtered', '/state/odometry')],
        ),

        Node(
            package='route_recorder',
            executable='route_recorder',
            name='route_recorder',
            output='screen',
            remappings=[ODOMETRY_REMAP],
        ),
        Node(
            package='mission_logger',
            executable='mission_logger',
            name='mission_logger',
            output='screen',
            parameters=[{
                'database_path': '/tmp/vision60_mock_mission.sqlite3',
                'transport': 'mock',
            }],
        ),
        Node(
            package='comm_recovery_manager',
            executable='comm_recovery_manager',
            name='comm_recovery_manager',
            output='screen',
            parameters=[{'max_channel_switch_attempts': 2}],
        ),
        Node(
            package='comm_recovery_manager',
            executable='communication_channel_manager',
            name='communication_channel_manager',
            output='screen',
            parameters=[{
                'candidate_channels': [
                    'mock_unavailable',
                    'mock_backup_wifi',
                ],
            }],
        ),
        Node(
            package='comm_recovery_manager',
            executable='recovery_path_follower',
            name='recovery_path_follower',
            output='screen',
            parameters=[{'require_returning_state': True}],
        ),
        Node(
            package='comm_recovery_manager',
            executable='reentry_path_follower',
            name='reentry_path_follower',
            output='screen',
            parameters=[{'reentry_speed_mps': 0.1}],
        ),
        Node(
            package='comm_recovery_manager',
            executable='motion_lock_adapter',
            name='motion_lock_adapter',
            output='screen',
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[nav2_params],
            remappings=[('cmd_vel', '/cmd_vel_recovery'), ODOMETRY_REMAP],
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[twist_mux_params],
            remappings=[('cmd_vel_out', '/cmd_vel_muxed')],
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            output='screen',
            parameters=[nav2_params],
            remappings=[
                ('cmd_vel', '/cmd_vel_muxed'),
                ('cmd_vel_smoothed', '/cmd_vel_smoothed'),
                ODOMETRY_REMAP,
            ],
        ),
        LifecycleNode(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            namespace='',
            output='screen',
            parameters=[collision_params],
        ),
        Node(
            package='comm_recovery_manager',
            executable='safety_velocity_gate',
            name='safety_velocity_gate',
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[nav2_params],
        ),
    ])

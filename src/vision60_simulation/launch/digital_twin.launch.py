# Copyright 2026 Kookmin AI Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch the Vision 60 disaster-world digital twin and ROS bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Return headless or graphical Gazebo plus sensor bridges."""
    share = get_package_share_directory('vision60_simulation')
    gz_share = get_package_share_directory('ros_gz_sim')
    default_world = os.path.join(share, 'worlds', 'disaster_test.sdf')
    world = LaunchConfiguration('world_file')
    models = os.path.join(share, 'models')
    headless = LaunchConfiguration('headless')
    gz_launch = os.path.join(gz_share, 'launch', 'gz_sim.launch.py')

    server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': ['-r -s ', world]}.items(),
        condition=IfCondition(headless),
    )
    graphical = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': ['-r ', world]}.items(),
        condition=UnlessCondition(headless),
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='vision60_sim_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/vision60/sim/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/ouster/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/world/vision60_disaster_test/dynamic_pose/info@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
        ],
        remappings=[
            (
                '/world/vision60_disaster_test/dynamic_pose/info',
                '/vision60/sim/poses',
            ),
        ],
        output='screen',
    )
    command_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='vision60_command_bridge',
        arguments=[
            '/model/vision60/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        remappings=[
            ('/model/vision60/cmd_vel', '/vision60/sim/cmd_vel_world'),
        ],
        output='screen',
    )
    odometry = Node(
        package='vision60_simulation',
        executable='pose_to_odometry',
        name='vision60_pose_to_odometry',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    body_twist_adapter = Node(
        package='vision60_simulation',
        executable='body_twist_to_world',
        name='vision60_body_twist_to_world',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'headless', default_value='true',
            description='Run only the Gazebo server for CI and mock tests',
        ),
        DeclareLaunchArgument(
            'world_file', default_value=default_world,
            description='SDF world used by the digital twin',
        ),
        AppendEnvironmentVariable(
            'IGN_GAZEBO_RESOURCE_PATH', models
        ),
        server,
        graphical,
        bridge,
        command_bridge,
        odometry,
        body_twist_adapter,
    ])

"""Fuse an Isaac ROS RT-DETR Detection2DArray with Ouster point clouds."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('vision60_bringup')
    parameters = os.path.join(share, 'config', 'mission_perception_rtdetr.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'detections_topic',
            default_value='/detections_output',
            description='vision_msgs/Detection2DArray from Isaac ROS RT-DETR',
        ),
        Node(
            package='mission_perception',
            executable='mission_perception',
            name='mission_perception',
            output='screen',
            parameters=[parameters, {
                'external_detection_topic': LaunchConfiguration(
                    'detections_topic'
                ),
            }],
        ),
    ])

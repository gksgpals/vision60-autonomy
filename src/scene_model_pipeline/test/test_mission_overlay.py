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

import json

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rosbag2_py
from rclpy.serialization import serialize_message
from vision60_msgs.msg import MissionEvent, RecoveryStatus, RecoveryWaypoint

from scene_model_pipeline.mission_overlay import build_mission_overlay
from scene_model_pipeline.synthetic_bag import ros_time, topic_metadata


def test_overlay_uses_last_recorded_route_without_fabricated_events(tmp_path):
    bag_path = tmp_path / 'mission_bag'
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    writer.create_topic(topic_metadata(
        '/mission/recorded_path', 'nav_msgs/msg/Path'
    ))
    path = Path()
    path.header.stamp = ros_time(3_000_000_000)
    path.header.frame_id = 'odom'
    for x in (0.0, 0.5, 1.0):
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    writer.write(
        '/mission/recorded_path',
        serialize_message(path),
        3_000_000_000,
    )
    del writer

    scene_path = tmp_path / 'cumulative_manifest.json'
    scene_path.write_text(json.dumps({
        'mission_id': 'mission_001',
        'scene_id': 'scene_001',
        'coordinate_frame': 'odom',
        'frames': [{'lidar_timestamp_ns': 2_000_000_000}],
    }), encoding='utf-8')
    output_path = tmp_path / 'overlay.json'
    overlay = build_mission_overlay(
        scene_path, bag_path, output_path
    )

    assert len(overlay['route']['points']) == 3
    assert overlay['route']['points'][-1][0] == 1.0
    assert overlay['communication_zones'] == []
    assert overlay['mission_events'] == []
    assert overlay['recovery_route']['points'] == []
    assert overlay['recovery_waypoints'] == []
    assert overlay['sources']['route_topic'] == '/mission/recorded_path'


def test_overlay_connects_recovery_path_waypoint_and_final_channel(tmp_path):
    bag_path = tmp_path / 'mission_bag'
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    topics = (
        ('/mission/recorded_path', 'nav_msgs/msg/Path'),
        ('/mission/recovery_path', 'nav_msgs/msg/Path'),
        (
            '/mission/recovery_waypoint',
            'vision60_msgs/msg/RecoveryWaypoint',
        ),
        (
            '/communication/recovery_status',
            'vision60_msgs/msg/RecoveryStatus',
        ),
    )
    for name, type_name in topics:
        writer.create_topic(topic_metadata(name, type_name))

    route = Path()
    route.header.stamp = ros_time(3_000_000_000)
    route.header.frame_id = 'odom'
    for x_value in (0.0, 0.5, 1.0):
        pose = PoseStamped()
        pose.pose.position.x = x_value
        pose.pose.orientation.w = 1.0
        route.poses.append(pose)
    writer.write(
        '/mission/recorded_path', serialize_message(route), 3_000_000_000
    )

    recovery = Path()
    recovery.header = route.header
    recovery.poses = list(reversed(route.poses[1:]))
    writer.write(
        '/mission/recovery_path',
        serialize_message(recovery),
        3_100_000_000,
    )

    waypoint = RecoveryWaypoint()
    waypoint.header.stamp = ros_time(3_200_000_000)
    waypoint.mission_id = 'mission_001'
    waypoint.waypoint_id = 'mission_001_wp_00002'
    waypoint.pose.position.x = 0.5
    waypoint.pose.orientation.w = 1.0
    waypoint.channel = 'mock_ethernet'
    waypoint.signal_strength_dbm = -45.0
    waypoint.snr_db = 22.0
    waypoint.safe_to_return = True
    waypoint.route_edge_id = 'mission_001_route'
    writer.write(
        '/mission/recovery_waypoint',
        serialize_message(waypoint),
        3_200_000_000,
    )

    for index, state in enumerate((
        RecoveryStatus.RETURNING,
        RecoveryStatus.CHANNEL_SWITCH,
        RecoveryStatus.NORMAL,
    )):
        status = RecoveryStatus()
        status.header.stamp = ros_time(3_300_000_000 + index)
        status.mission_id = 'mission_001'
        status.state = state
        status.target_waypoint_id = waypoint.waypoint_id
        status.active_channel = (
            'mock_backup_wifi'
            if state == RecoveryStatus.NORMAL else 'mock_ethernet'
        )
        status.channel_switch_attempts = 2
        writer.write(
            '/communication/recovery_status',
            serialize_message(status),
            3_300_000_000 + index,
        )
    del writer

    scene_path = tmp_path / 'cumulative_manifest.json'
    scene_path.write_text(json.dumps({
        'mission_id': 'mission_001',
        'scene_id': 'scene_001',
        'coordinate_frame': 'odom',
        'frames': [{'lidar_timestamp_ns': 2_000_000_000}],
    }), encoding='utf-8')
    overlay = build_mission_overlay(
        scene_path, bag_path, tmp_path / 'overlay.json'
    )

    assert len(overlay['recovery_route']['points']) == 2
    assert overlay['recovery_waypoints'][0]['selected_for_recovery'] is True
    assert overlay['communication_summary']['final_channel'] \
        == 'mock_backup_wifi'
    assert overlay['communication_summary']['state_sequence'] == [
        'RETURNING', 'CHANNEL_SWITCH', 'NORMAL',
    ]


def test_perception_events_do_not_become_communication_zones(tmp_path):
    bag_path = tmp_path / 'mission_bag'
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    writer.create_topic(topic_metadata(
        '/mission/recorded_path', 'nav_msgs/msg/Path'
    ))
    writer.create_topic(topic_metadata(
        '/mission/event', 'vision60_msgs/msg/MissionEvent'
    ))
    path = Path()
    path.header.stamp = ros_time(3_000_000_000)
    for x_value in (0.0, 0.5):
        pose = PoseStamped()
        pose.pose.position.x = x_value
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    writer.write(
        '/mission/recorded_path', serialize_message(path), 3_000_000_000
    )
    for index, event_type in enumerate((
        'victim_candidate', 'hazard_candidate', 'channel_anomaly_candidate',
    )):
        event = MissionEvent()
        event.header.stamp = ros_time(3_100_000_000 + index)
        event.mission_id = 'mission_001'
        event.event_id = f'event_{index}'
        event.event_type = event_type
        event.pose.position.x = 1.0 + index
        event.pose.orientation.w = 1.0
        event.confidence = 0.9
        writer.write(
            '/mission/event', serialize_message(event), 3_100_000_000 + index
        )
    del writer

    scene_path = tmp_path / 'cumulative_manifest.json'
    scene_path.write_text(json.dumps({
        'mission_id': 'mission_001',
        'scene_id': 'scene_001',
        'coordinate_frame': 'odom',
        'frames': [{'lidar_timestamp_ns': 2_000_000_000}],
    }), encoding='utf-8')
    overlay = build_mission_overlay(
        scene_path, bag_path, tmp_path / 'overlay.json'
    )

    assert len(overlay['mission_events']) == 3
    assert len(overlay['communication_zones']) == 1
    assert overlay['communication_zones'][0]['classification'] \
        == 'channel_anomaly_candidate'

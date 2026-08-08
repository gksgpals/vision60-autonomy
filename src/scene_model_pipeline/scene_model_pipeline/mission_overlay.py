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

"""Build scene overlays from the actual route and mission event topics."""

import argparse
import json
import math
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from scene_model_pipeline.bag_sequence import message_stamp_ns
from scene_model_pipeline.core import load_json
from scene_model_pipeline.sequence import path_sha256


RECOVERY_STATE_NAMES = {
    1: 'NORMAL',
    2: 'DEGRADED',
    3: 'LINK_LOST',
    4: 'STOPPING',
    5: 'RETURNING',
    6: 'LINK_RECOVERED',
    7: 'SYNCING',
    8: 'REENTRY_TEST',
    9: 'SAFE_STOP',
    10: 'CHANNEL_SWITCH',
    11: 'CLASSIFYING',
}


def _classification(event_type: str) -> str:
    if 'location_radio_shadow' in event_type:
        return 'radio_shadow_candidate'
    if 'channel_anomaly' in event_type:
        return 'channel_anomaly_candidate'
    if 'total_link_failure' in event_type:
        return 'total_link_failure_candidate'
    return 'transient_network_instability'


def _is_communication_event(event_type: str) -> bool:
    """Return whether an event belongs on the radio-quality map layer."""
    return any(token in event_type for token in (
        'radio_shadow', 'channel_anomaly', 'total_link_failure',
        'network_instability', 'communication',
    ))


def build_mission_overlay(
    scene_manifest_path: Path,
    bag_path: Path,
    output_path: Path,
    path_topic: str = '/mission/recorded_path',
    event_topic: str = '/mission/event',
    recovery_path_topic: str = '/mission/recovery_path',
    waypoint_topic: str = '/mission/recovery_waypoint',
    recovery_status_topic: str = '/communication/recovery_status',
) -> dict:
    """Use only messages recorded in the source mission bag."""
    scene = load_json(Path(scene_manifest_path))
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    if path_topic not in topic_types:
        raise ValueError(f'bag is missing route topic: {path_topic}')
    optional_topics = {
        event_topic,
        recovery_path_topic,
        waypoint_topic,
        recovery_status_topic,
    }
    selected = {path_topic} | {
        topic for topic in optional_topics if topic in topic_types
    }
    message_types = {
        topic: get_message(topic_types[topic]) for topic in selected
    }
    latest_path = None
    latest_recovery_path = None
    latest_timestamp_ns = 0
    events = []
    waypoints = {}
    recovery_state_sequence = []
    selected_waypoint_ids = set()
    latest_recovery_status = None
    maximum_channel_switch_attempts = 0
    while reader.has_next():
        topic, serialized, receive_ns = reader.read_next()
        if topic not in selected:
            continue
        message = deserialize_message(serialized, message_types[topic])
        timestamp_ns = message_stamp_ns(message, receive_ns)
        latest_timestamp_ns = max(latest_timestamp_ns, timestamp_ns)
        if topic == path_topic:
            latest_path = message
        elif topic == recovery_path_topic:
            latest_recovery_path = message
        elif topic == waypoint_topic:
            if (
                message.safe_to_return
                and (
                    not message.mission_id
                    or message.mission_id == scene['mission_id']
                )
            ):
                waypoints[message.waypoint_id] = (message, timestamp_ns)
        elif topic == recovery_status_topic:
            if (
                message.mission_id
                and message.mission_id != scene['mission_id']
            ):
                continue
            state = int(message.state)
            if (
                not recovery_state_sequence
                or recovery_state_sequence[-1] != state
            ):
                recovery_state_sequence.append(state)
            if message.target_waypoint_id:
                selected_waypoint_ids.add(message.target_waypoint_id)
            maximum_channel_switch_attempts = max(
                maximum_channel_switch_attempts,
                int(message.channel_switch_attempts),
            )
            latest_recovery_status = message
        elif topic == event_topic and (
            not message.mission_id
            or message.mission_id == scene['mission_id']
        ):
            events.append(message)
    if latest_path is None or len(latest_path.poses) < 2:
        raise ValueError('recorded route contains fewer than two poses')

    route = [
        [
            float(pose.pose.position.x),
            float(pose.pose.position.y),
            float(pose.pose.position.z),
        ]
        for pose in latest_path.poses
    ]
    recovery_route = []
    if latest_recovery_path is not None:
        recovery_route = [
            [
                float(pose.pose.position.x),
                float(pose.pose.position.y),
                float(pose.pose.position.z),
            ]
            for pose in latest_recovery_path.poses
        ]
    if recovery_route and waypoints and not selected_waypoint_ids:
        target = recovery_route[-1]
        nearest_id = min(
            waypoints,
            key=lambda waypoint_id: math.dist(
                target,
                [
                    float(waypoints[waypoint_id][0].pose.position.x),
                    float(waypoints[waypoint_id][0].pose.position.y),
                    float(waypoints[waypoint_id][0].pose.position.z),
                ],
            ),
        )
        selected_waypoint_ids.add(nearest_id)

    recovery_waypoints = []
    for waypoint_id, (waypoint, timestamp_ns) in sorted(
        waypoints.items(), key=lambda item: item[1][1]
    ):
        recovery_waypoints.append({
            'id': waypoint_id,
            'position': [
                float(waypoint.pose.position.x),
                float(waypoint.pose.position.y),
                float(waypoint.pose.position.z),
            ],
            'channel': waypoint.channel,
            'signal_strength_dbm': float(waypoint.signal_strength_dbm),
            'snr_db': float(waypoint.snr_db),
            'packet_loss_ratio': float(waypoint.packet_loss_ratio),
            'latency_ms': float(waypoint.latency_ms),
            'safe_to_return': bool(waypoint.safe_to_return),
            'route_edge_id': waypoint.route_edge_id,
            'selected_for_recovery': waypoint_id in selected_waypoint_ids,
            'timestamp_ns': timestamp_ns,
        })
    zones = []
    mission_events = []
    obstacles = []
    for index, event in enumerate(events):
        position = [
            float(event.pose.position.x),
            float(event.pose.position.y),
            float(event.pose.position.z),
        ]
        confidence = min(max(float(event.confidence), 0.0), 1.0)
        if _is_communication_event(event.event_type):
            zones.append({
                'id': f'comm_zone_{index:03d}',
                'center': position,
                'radius_m': 0.35,
                'classification': _classification(event.event_type),
                'confidence': confidence,
                'source_log_id': event.event_id,
            })
        mission_events.append({
            'id': event.event_id or f'mission_event_{index:03d}',
            'type': event.event_type,
            'position': position,
            'label': event.event_type,
            'confidence': confidence,
            'verified': False,
            'source_id': event.source_id,
        })
        if 'obstacle' in event.event_type:
            obstacles.append({
                'id': event.event_id or f'obstacle_{index:03d}',
                'position': position,
                'dimensions_m': [0.5, 0.5, 0.5],
                'classification': event.event_type,
                'label': event.event_type,
                'confidence': confidence,
                'source_id': event.source_id,
            })
    frame_timestamp_ns = max(
        int(frame['lidar_timestamp_ns']) for frame in scene['frames']
    )
    overlay = {
        'schema_version': 1,
        'mission_id': scene['mission_id'],
        'scene_id': scene['scene_id'],
        'coordinate_frame': scene['coordinate_frame'],
        'timestamp_ns': max(latest_timestamp_ns, frame_timestamp_ns),
        'route': {
            'id': f"{scene['mission_id']}_recorded_route",
            'points': route,
        },
        'recovery_route': {
            'id': f"{scene['mission_id']}_recovery_route",
            'points': recovery_route,
        },
        'recovery_waypoints': recovery_waypoints,
        'communication_zones': zones,
        'communication_summary': {
            'final_channel': (
                latest_recovery_status.active_channel
                if latest_recovery_status is not None else ''
            ),
            'final_state': (
                RECOVERY_STATE_NAMES.get(
                    int(latest_recovery_status.state), 'UNKNOWN'
                )
                if latest_recovery_status is not None else 'UNKNOWN'
            ),
            'state_sequence': [
                RECOVERY_STATE_NAMES.get(state, str(state))
                for state in recovery_state_sequence
            ],
            'channel_switch_attempts': maximum_channel_switch_attempts,
            'failure_cause': (
                int(latest_recovery_status.failure_cause)
                if latest_recovery_status is not None else 0
            ),
            'failure_confidence': (
                float(latest_recovery_status.failure_confidence)
                if latest_recovery_status is not None else 0.0
            ),
        },
        'obstacles': obstacles,
        'mission_events': mission_events,
        'sources': {
            'bag_sha256': path_sha256(Path(bag_path)),
            'route_topic': path_topic,
            'event_topic': event_topic,
            'recovery_path_topic': recovery_path_topic,
            'waypoint_topic': waypoint_topic,
            'recovery_status_topic': recovery_status_topic,
        },
    }
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(overlay, indent=2) + '\n', encoding='utf-8'
    )
    return overlay


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--scene-manifest', required=True, type=Path)
    parser.add_argument('--bag', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    overlay = build_mission_overlay(
        args.scene_manifest, args.bag, args.output
    )
    print(
        'MISSION_OVERLAY=PASS '
        f"route_points={len(overlay['route']['points'])} "
        f"events={len(overlay['mission_events'])}"
    )

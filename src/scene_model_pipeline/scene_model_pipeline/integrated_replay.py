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

"""Build one time-aligned operator MCAP from mission and scene bags."""

import argparse
import gc
import hashlib
import json
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

from geometry_msgs.msg import TransformStamped
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from tf2_msgs.msg import TFMessage
import yaml

from scene_model_pipeline.synthetic_bag import ros_time, topic_metadata


SCENE_TOPICS = {
    '/mission/scene_cloud',
    '/mission/scene_markers',
    '/mission/scene_mesh',
    '/mission/voxel_markers',
}
REQUIRED_CUSTOM_SCHEMA_FIELDS = (
    b'signal_strength_dbm',
    b'operator_attention_required',
    b'event_type',
)


def _reader(bag_path: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    return reader


def _metadata(bag_path: Path) -> Dict:
    metadata_path = Path(bag_path) / 'metadata.yaml'
    if not metadata_path.is_file():
        raise ValueError(f'missing rosbag metadata: {metadata_path}')
    value = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))
    return value['rosbag2_bagfile_information']


def _topic_counts(info: Dict) -> Dict[str, int]:
    return {
        item['topic_metadata']['name']: int(item['message_count'])
        for item in info['topics_with_message_count']
    }


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _set_message_stamp(message, timestamp_ns: int) -> None:
    stamp = ros_time(timestamp_ns)
    if hasattr(message, 'header'):
        message.header.stamp = stamp
        return
    if hasattr(message, 'markers'):
        for marker in message.markers:
            marker.header.stamp = stamp
        return
    raise ValueError(
        f'cannot retime message without header: {type(message).__name__}'
    )


def retime_serialized_message(
    serialized: bytes,
    type_name: str,
    timestamp_ns: int,
) -> bytes:
    """Retimestamp a standard scene message, including nested markers."""
    message = deserialize_message(serialized, get_message(type_name))
    _set_message_stamp(message, timestamp_ns)
    return serialize_message(message)


def _copy_topic_metadata(value):
    try:
        return rosbag2_py.TopicMetadata(
            name=value.name,
            type=value.type,
            serialization_format=value.serialization_format,
            offered_qos_profiles=value.offered_qos_profiles,
        )
    except TypeError:
        return rosbag2_py.TopicMetadata(
            name=value.name,
            type=value.type,
            serialization_format=value.serialization_format,
        )


def _identity_map_to_odom(timestamp_ns: int) -> TFMessage:
    transform = TransformStamped()
    transform.header.stamp = ros_time(timestamp_ns)
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'odom'
    transform.transform.rotation.w = 1.0
    return TFMessage(transforms=[transform])


def _write_integrated_bag(
    mission_bag: Path,
    scene_bag: Path,
    output_bag: Path,
    scene_target_ns: int,
    scene_source_ns: int,
) -> None:
    mission_reader = _reader(mission_bag)
    scene_reader = _reader(scene_bag)
    mission_topics = mission_reader.get_all_topics_and_types()
    scene_topics = scene_reader.get_all_topics_and_types()
    mission_names = {value.name for value in mission_topics}
    scene_names = {value.name for value in scene_topics}
    overlap = sorted(mission_names & scene_names)
    if overlap:
        raise ValueError(f'input bags contain duplicate topics: {overlap}')
    if not SCENE_TOPICS <= scene_names:
        raise ValueError(
            'scene bag is missing topics: '
            f'{sorted(SCENE_TOPICS - scene_names)}'
        )
    if '/tf_static' in scene_names:
        raise ValueError('scene bag must not contain /tf_static')
    add_static_transform = '/tf_static' not in mission_names

    scene_types = {value.name: value.type for value in scene_topics}
    scene_messages: List[Tuple[int, str, bytes]] = []
    while scene_reader.has_next():
        topic, serialized, source_timestamp = scene_reader.read_next()
        target_timestamp = (
            scene_target_ns + int(source_timestamp) - scene_source_ns
        )
        scene_messages.append((
            target_timestamp,
            topic,
            retime_serialized_message(
                serialized, scene_types[topic], target_timestamp
            ),
        ))
    if add_static_transform:
        scene_messages.append((
            scene_target_ns,
            '/tf_static',
            serialize_message(_identity_map_to_odom(scene_target_ns)),
        ))
    scene_messages.sort(key=lambda value: (value[0], value[1]))

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(output_bag), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    for value in mission_topics:
        writer.create_topic(_copy_topic_metadata(value))
    for value in scene_topics:
        writer.create_topic(_copy_topic_metadata(value))
    if add_static_transform:
        writer.create_topic(
            topic_metadata('/tf_static', 'tf2_msgs/msg/TFMessage')
        )

    scene_index = 0
    while mission_reader.has_next():
        topic, serialized, timestamp = mission_reader.read_next()
        while (
            scene_index < len(scene_messages)
            and scene_messages[scene_index][0] <= timestamp
        ):
            target, scene_topic, scene_serialized = scene_messages[scene_index]
            writer.write(scene_topic, scene_serialized, target)
            scene_index += 1
        writer.write(topic, serialized, timestamp)
    for target, scene_topic, scene_serialized in scene_messages[scene_index:]:
        writer.write(scene_topic, scene_serialized, target)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _bag_files(bag_path: Path, info: Dict) -> Iterable[Path]:
    return [Path(bag_path) / value for value in info['relative_file_paths']]


def validate_integrated_replay(
    output_bag: Path,
    mission_bag: Path,
    scene_bag: Path,
    scene_target_ns: int,
) -> Dict:
    """Verify topic preservation, scene timestamps, TF, and MCAP schemas."""
    output_info = _metadata(output_bag)
    mission_info = _metadata(mission_bag)
    scene_info = _metadata(scene_bag)
    output_counts = _topic_counts(output_info)
    expected_counts = _topic_counts(mission_info)
    for topic, count in _topic_counts(scene_info).items():
        expected_counts[topic] = expected_counts.get(topic, 0) + count
    if '/tf_static' not in expected_counts:
        expected_counts['/tf_static'] = 1
    if output_counts != expected_counts:
        raise ValueError('integrated topic counts do not match both sources')

    target_scene_types = {}
    reader = _reader(output_bag)
    topic_types = {
        value.name: value.type for value in reader.get_all_topics_and_types()
    }
    for topic in SCENE_TOPICS:
        target_scene_types[topic] = topic_types[topic]
    seen_scene = set()
    static_pairs = set()
    recovery_states = set()
    while reader.has_next():
        topic, serialized, timestamp = reader.read_next()
        if topic in SCENE_TOPICS:
            message = deserialize_message(
                serialized, get_message(target_scene_types[topic])
            )
            stamps = []
            if hasattr(message, 'header'):
                stamps.append(_stamp_ns(message.header.stamp))
            else:
                stamps.extend(
                    _stamp_ns(marker.header.stamp)
                    for marker in message.markers
                )
            if not stamps or any(value != timestamp for value in stamps):
                raise ValueError(f'scene header timestamp mismatch on {topic}')
            seen_scene.add(topic)
        elif topic == '/tf_static':
            message = deserialize_message(serialized, TFMessage)
            for transform in message.transforms:
                static_pairs.add((
                    transform.header.frame_id,
                    transform.child_frame_id,
                ))
        elif topic == '/communication/recovery_status':
            message = deserialize_message(
                serialized, get_message(topic_types[topic])
            )
            recovery_states.add(int(message.state))
    if seen_scene != SCENE_TOPICS or ('map', 'odom') not in static_pairs:
        raise ValueError('integrated visualization messages are incomplete')
    if recovery_states and not {4, 5, 7, 8, 10} <= recovery_states:
        raise ValueError('communication recovery sequence is incomplete')

    output_files = list(_bag_files(output_bag, output_info))
    schema_blob = b''.join(path.read_bytes() for path in output_files)
    absent_fields = [
        value.decode('ascii') for value in REQUIRED_CUSTOM_SCHEMA_FIELDS
        if value not in schema_blob
    ]
    if _topic_counts(mission_info).get('/communication/state'):
        if absent_fields:
            raise ValueError(
                f'custom schemas are not embedded: {absent_fields}'
            )

    output_start_ns = int(
        output_info['starting_time']['nanoseconds_since_epoch']
    )
    mission_start_ns = int(
        mission_info['starting_time']['nanoseconds_since_epoch']
    )
    invalid_timeline = (
        output_start_ns != mission_start_ns
        or scene_target_ns < output_start_ns
    )
    if invalid_timeline:
        raise ValueError('integrated timeline start is invalid')
    return {
        'duration_s': int(output_info['duration']['nanoseconds']) / 1e9,
        'message_count': int(output_info['message_count']),
        'topic_counts': dict(sorted(output_counts.items())),
        'verification': {
            'source_topic_counts_preserved': True,
            'scene_headers_retimed': True,
            'map_to_odom_static_transform': True,
            'recorded_sensor_static_transforms_preserved': (
                ('base_link', 'os_sensor') in static_pairs
                and ('base_link', 'camera_optical') in static_pairs
            ),
            'custom_message_definitions_embedded': not absent_fields,
            'required_recovery_sequence_present': bool(recovery_states),
        },
        'files': [
            {
                'file': path.name,
                'size_bytes': path.stat().st_size,
                'sha256': _sha256(path),
            }
            for path in [Path(output_bag) / 'metadata.yaml', *output_files]
        ],
    }


def build_integrated_replay(
    mission_bag: Path,
    scene_bag: Path,
    output_bag: Path,
    mission_manifest: Path = None,
    scene_manifest: Path = None,
    scene_offset_s: float = 0.0,
    preserve_scene_time: bool = False,
) -> Dict:
    """Combine bags, align scene time, validate output, and write lineage."""
    mission_bag = Path(mission_bag)
    scene_bag = Path(scene_bag)
    output_bag = Path(output_bag)
    if output_bag.exists():
        raise FileExistsError(f'output bag already exists: {output_bag}')
    mission_info = _metadata(mission_bag)
    scene_info = _metadata(scene_bag)
    mission_start_ns = int(
        mission_info['starting_time']['nanoseconds_since_epoch']
    )
    scene_source_ns = int(
        scene_info['starting_time']['nanoseconds_since_epoch']
    )
    scene_target_ns = (
        scene_source_ns
        if preserve_scene_time
        else mission_start_ns + int(scene_offset_s * 1e9)
    )
    _write_integrated_bag(
        mission_bag,
        scene_bag,
        output_bag,
        scene_target_ns,
        scene_source_ns,
    )
    gc.collect()
    result = validate_integrated_replay(
        output_bag, mission_bag, scene_bag, scene_target_ns
    )
    result.update({
        'schema_version': 1,
        'scenario': 'integrated_operator_mock_replay',
        'synthetic_scenario': True,
        'storage_identifier': 'mcap',
        'time_alignment': {
            'mode': (
                'preserved_source_time'
                if preserve_scene_time
                else 'shifted_to_mission_time'
            ),
            'mission_start_ns': mission_start_ns,
            'scene_source_start_ns': scene_source_ns,
            'scene_target_start_ns': scene_target_ns,
            'scene_shift_ns': scene_target_ns - scene_source_ns,
        },
        'frame_alignment': {
            'parent_frame': 'map',
            'child_frame': 'odom',
            'transform': 'identity',
            'basis': 'full-system mock map and odom share one origin',
        },
        'sources': {
            'mission_bag': {
                'directory': mission_bag.name,
                'metadata_sha256': _sha256(mission_bag / 'metadata.yaml'),
            },
            'scene_bag': {
                'directory': scene_bag.name,
                'metadata_sha256': _sha256(scene_bag / 'metadata.yaml'),
            },
        },
    })
    for key, path in (
        ('mission_manifest', mission_manifest),
        ('scene_manifest', scene_manifest),
    ):
        if path is not None:
            path = Path(path)
            result['sources'][key] = {
                'file': path.name,
                'sha256': _sha256(path),
            }
    manifest_path = output_bag / 'replay_manifest.json'
    manifest_path.write_text(
        json.dumps(result, indent=2) + '\n', encoding='utf-8'
    )
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--mission-bag', required=True, type=Path)
    parser.add_argument('--scene-bag', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--mission-manifest', type=Path)
    parser.add_argument('--scene-manifest', type=Path)
    parser.add_argument('--scene-offset-sec', type=float, default=0.0)
    parser.add_argument('--preserve-scene-time', action='store_true')
    args = parser.parse_args()
    manifest = build_integrated_replay(
        args.mission_bag,
        args.scene_bag,
        args.output,
        args.mission_manifest,
        args.scene_manifest,
        args.scene_offset_sec,
        args.preserve_scene_time,
    )
    print(
        'INTEGRATED_OPERATOR_REPLAY=PASS '
        f"messages={manifest['message_count']} "
        f"duration={manifest['duration_s']:.3f}s"
    )


def validation_main() -> None:
    """Validate an existing integrated bag against both source bags."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--mission-bag', required=True, type=Path)
    parser.add_argument('--scene-bag', required=True, type=Path)
    parser.add_argument('--integrated-bag', required=True, type=Path)
    args = parser.parse_args()
    manifest_path = args.integrated_bag / 'replay_manifest.json'
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    target_ns = int(manifest['time_alignment']['scene_target_start_ns'])
    result = validate_integrated_replay(
        args.integrated_bag,
        args.mission_bag,
        args.scene_bag,
        target_ns,
    )
    print(
        'INTEGRATED_OPERATOR_REPLAY_VALIDATION=PASS '
        f"messages={result['message_count']} "
        f"duration={result['duration_s']:.3f}s"
    )


if __name__ == '__main__':
    main()

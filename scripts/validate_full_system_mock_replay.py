#!/usr/bin/env python3
"""Validate a recorded recovery MCAP and write its lineage manifest."""
import argparse
from bisect import bisect_left
import hashlib
import json
from pathlib import Path

import yaml


REQUIRED_TOPICS = {
    '/communication/state',
    '/communication/recovery_status',
    '/communication/recovery_event',
    '/mission/event',
    '/mission/sync_status',
    '/mission/recorded_path',
    '/mission/recovery_path',
    '/vision60/odom',
    '/vision60/safety_state',
    '/vision60/mock_obstacle_phase',
    '/cmd_vel_safe',
    '/ouster/points',
    '/camera/image_raw',
    '/camera/camera_info',
    '/tf_static',
}

REQUIRED_SCHEMA_FIELDS = (
    b'signal_strength_dbm',
    b'operator_attention_required',
    b'event_type',
)


def deserialize_sensor_samples(bag_dir: Path) -> dict:
    """Verify dense LiDAR and synchronized raw camera samples."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2
    from tf2_msgs.msg import TFMessage

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    lidar_stamps = []
    image_stamps = []
    maximum_points = 0
    image_shape = None
    camera_info_count = 0
    camera_info_stamps = []
    camera_intrinsic = None
    camera_info_frame = None
    static_transforms = {}
    while reader.has_next():
        topic, data, receive_ns = reader.read_next()
        if topic == '/ouster/points':
            message = deserialize_message(data, PointCloud2)
            maximum_points = max(
                maximum_points, int(message.width) * int(message.height)
            )
            stamp = message.header.stamp
            lidar_stamps.append(
                int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
                or int(receive_ns)
            )
        elif topic == '/camera/image_raw':
            message = deserialize_message(data, Image)
            expected_size = int(message.height) * int(message.step)
            if len(message.data) != expected_size:
                raise ValueError('camera image payload size is invalid')
            image_shape = [int(message.height), int(message.width), 3]
            stamp = message.header.stamp
            image_stamps.append(
                int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
                or int(receive_ns)
            )
        elif topic == '/camera/camera_info':
            message = deserialize_message(data, CameraInfo)
            camera_info_count += 1
            camera_intrinsic = list(message.k)
            camera_info_frame = message.header.frame_id
            stamp = message.header.stamp
            camera_info_stamps.append(
                int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
                or int(receive_ns)
            )
            if (
                int(message.width) != 160
                or int(message.height) != 120
                or message.distortion_model != 'plumb_bob'
            ):
                raise ValueError('camera calibration metadata is invalid')
        elif topic == '/tf_static':
            message = deserialize_message(data, TFMessage)
            for transform in message.transforms:
                pair = (
                    transform.header.frame_id,
                    transform.child_frame_id,
                )
                static_transforms[pair] = transform
    if maximum_points < 1000:
        raise ValueError(
            f'LiDAR scene is too sparse: maximum_points={maximum_points}'
        )
    if not lidar_stamps or not image_stamps or image_shape != [120, 160, 3]:
        raise ValueError('camera/LiDAR sensor recording is incomplete')
    if camera_info_count < len(image_stamps) * 0.99:
        raise ValueError('fewer than 99% of images have CameraInfo')
    if camera_intrinsic != [
        110.0, 0.0, 80.0,
        0.0, 110.0, 60.0,
        0.0, 0.0, 1.0,
    ] or camera_info_frame != 'camera_optical':
        raise ValueError('embedded CameraInfo values are invalid')
    required_pairs = {
        ('map', 'odom'),
        ('base_link', 'os_sensor'),
        ('base_link', 'camera_optical'),
    }
    if not required_pairs <= static_transforms.keys():
        raise ValueError('static sensor transform tree is incomplete')
    lidar_tf = static_transforms[('base_link', 'os_sensor')]
    camera_tf = static_transforms[('base_link', 'camera_optical')]
    if (
        abs(float(lidar_tf.transform.translation.x) - 0.20) > 1e-6
        or abs(float(lidar_tf.transform.translation.z) - 0.45) > 1e-6
        or abs(float(camera_tf.transform.translation.x) - 0.30) > 1e-6
        or abs(float(camera_tf.transform.translation.z) - 0.40) > 1e-6
    ):
        raise ValueError('static sensor mount translation is invalid')
    image_stamps.sort()
    camera_info_stamps.sort()
    calibrated_images = 0
    for stamp in image_stamps:
        insertion = bisect_left(camera_info_stamps, stamp)
        candidates = camera_info_stamps[
            max(0, insertion - 1):insertion + 1
        ]
        if candidates and min(abs(value - stamp) for value in candidates) \
                <= 1_000_000:
            calibrated_images += 1
    if calibrated_images < len(image_stamps) * 0.99:
        raise ValueError('fewer than 99% of images match CameraInfo timestamps')
    synchronized = 0
    for stamp in lidar_stamps:
        insertion = bisect_left(image_stamps, stamp)
        candidates = image_stamps[max(0, insertion - 1):insertion + 1]
        if candidates and min(abs(value - stamp) for value in candidates) \
                <= 120_000_000:
            synchronized += 1
    if synchronized < len(lidar_stamps) * 0.9:
        raise ValueError('fewer than 90% of LiDAR frames match a camera frame')
    return {
        'lidar_message_count': len(lidar_stamps),
        'camera_message_count': len(image_stamps),
        'camera_info_message_count': camera_info_count,
        'calibrated_camera_ratio': calibrated_images / len(image_stamps),
        'maximum_points_per_lidar_frame': maximum_points,
        'camera_shape': image_shape,
        'synchronized_lidar_ratio': synchronized / len(lidar_stamps),
        'static_transform_pairs': [
            list(value) for value in sorted(static_transforms)
        ],
    }


def deserialize_recovery_sequence(bag_dir: Path) -> dict:
    """Read custom ROS messages back from MCAP and verify state coverage."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from vision60_msgs.msg import (
        CommunicationState,
        RecoveryEvent,
        RecoveryStatus,
        SyncStatus,
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    target_topics = {
        '/communication/state',
        '/communication/recovery_status',
        '/communication/recovery_event',
        '/mission/sync_status',
        '/mission/event',
    }
    values = {topic: [] for topic in target_topics}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in target_topics:
            continue
        message = deserialize_message(data, get_message(topic_types[topic]))
        if topic == '/mission/event':
            values[topic].append(message.event_type)
        elif topic == '/communication/recovery_event':
            values[topic].append(int(message.event))
        else:
            values[topic].append(int(message.state))

    expected = {
        '/communication/state': {
            CommunicationState.NORMAL,
            CommunicationState.DEGRADED,
            CommunicationState.LOST,
        },
        '/communication/recovery_status': {
            RecoveryStatus.NORMAL,
            RecoveryStatus.STOPPING,
            RecoveryStatus.RETURNING,
            RecoveryStatus.SYNCING,
            RecoveryStatus.REENTRY_TEST,
            RecoveryStatus.CHANNEL_SWITCH,
        },
        '/communication/recovery_event': {
            RecoveryEvent.RETURN_SUCCEEDED,
            RecoveryEvent.REENTRY_SUCCEEDED,
            RecoveryEvent.CHANNEL_SWITCH_SUCCEEDED,
            RecoveryEvent.CHANNEL_SWITCH_FAILED,
        },
        '/mission/sync_status': {SyncStatus.COMPLETE},
    }
    for topic, expected_values in expected.items():
        missing = sorted(expected_values - set(values[topic]))
        if missing:
            raise ValueError(
                f'recovery sequence is incomplete on {topic}: {missing}'
            )
    if not values['/mission/event']:
        raise ValueError('no mission event could be deserialized')

    state_names = {
        '/communication/state': {
            1: 'NORMAL', 2: 'DEGRADED', 3: 'LOST',
        },
        '/communication/recovery_status': {
            1: 'NORMAL', 2: 'DEGRADED', 3: 'LINK_LOST',
            4: 'STOPPING', 5: 'RETURNING', 6: 'LINK_RECOVERED',
            7: 'SYNCING', 8: 'REENTRY_TEST', 9: 'SAFE_STOP',
            10: 'CHANNEL_SWITCH', 11: 'CLASSIFYING',
        },
        '/communication/recovery_event': {
            1: 'RETURN_SUCCEEDED', 2: 'RETURN_FAILED',
            3: 'SYNC_STARTED', 4: 'SYNC_SUCCEEDED', 5: 'SYNC_FAILED',
            6: 'REENTRY_SUCCEEDED', 7: 'REENTRY_LINK_LOST',
            8: 'CHANNEL_SWITCH_SUCCEEDED', 9: 'CHANNEL_SWITCH_FAILED',
            10: 'CLASSIFICATION_RECORDED', 11: 'REENTRY_FAILED',
        },
        '/mission/sync_status': {
            0: 'IDLE', 1: 'PENDING', 2: 'IN_PROGRESS',
            3: 'COMPLETE', 4: 'FAILED',
        },
    }
    result = {}
    for topic, names in state_names.items():
        result[topic] = [
            names[value] for value in sorted(set(values[topic]))
        ]
    result['/mission/event'] = sorted(set(values['/mission/event']))
    return result


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def validate(bag_dir: Path) -> dict:
    metadata_path = bag_dir / 'metadata.yaml'
    if not metadata_path.is_file():
        raise ValueError(f'missing rosbag metadata: {metadata_path}')
    metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))
    info = metadata['rosbag2_bagfile_information']
    if info['storage_identifier'] != 'mcap':
        raise ValueError('recording is not MCAP')

    topic_counts = {
        item['topic_metadata']['name']: int(item['message_count'])
        for item in info['topics_with_message_count']
    }
    missing = sorted(REQUIRED_TOPICS - topic_counts.keys())
    empty = sorted(
        topic for topic in REQUIRED_TOPICS
        if topic_counts.get(topic, 0) <= 0
    )
    if missing or empty:
        raise ValueError(
            f'incomplete recovery recording: missing={missing}, empty={empty}'
        )

    mcap_paths = [
        bag_dir / relative for relative in info['relative_file_paths']
    ]
    if not mcap_paths or any(not path.is_file() for path in mcap_paths):
        raise ValueError('metadata references a missing MCAP file')
    schema_blob = b''.join(path.read_bytes() for path in mcap_paths)
    absent_fields = [
        field.decode('ascii') for field in REQUIRED_SCHEMA_FIELDS
        if field not in schema_blob
    ]
    if absent_fields:
        raise ValueError(
            f'custom message definitions are not embedded: {absent_fields}'
        )

    duration_s = int(info['duration']['nanoseconds']) / 1e9
    if duration_s < 20.0:
        raise ValueError(f'recovery recording is too short: {duration_s:.3f}s')
    recovered_sequence = deserialize_recovery_sequence(bag_dir)
    sensor_samples = deserialize_sensor_samples(bag_dir)
    manifest = {
        'schema_version': 1,
        'scenario': 'full_system_communication_recovery_mock',
        'synthetic_scenario': True,
        'storage_identifier': 'mcap',
        'duration_s': duration_s,
        'message_count': int(info['message_count']),
        'topic_counts': dict(sorted(topic_counts.items())),
        'verification': {
            'full_system_probe': 'PASS',
            'required_topics_nonempty': True,
            'custom_message_definitions_embedded': True,
            'custom_messages_deserialized': True,
            'required_recovery_sequence_present': True,
            'dense_lidar_camera_synchronized': True,
            'embedded_calibration_and_sensor_tf': True,
        },
        'observed_sequence_values': recovered_sequence,
        'sensor_samples': sensor_samples,
        'files': [
            {
                'file': path.name,
                'size_bytes': path.stat().st_size,
                'sha256': sha256(path),
            }
            for path in [metadata_path, *mcap_paths]
        ],
    }
    manifest_path = bag_dir / 'replay_manifest.json'
    manifest_path.write_text(
        json.dumps(manifest, indent=2) + '\n', encoding='utf-8'
    )
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('bag_dir', type=Path)
    args = parser.parse_args()
    manifest = validate(args.bag_dir)
    print(
        'FULL_SYSTEM_MCAP_VALIDATION=PASS '
        f"duration={manifest['duration_s']:.3f}s "
        f"messages={manifest['message_count']}"
    )


if __name__ == '__main__':
    main()

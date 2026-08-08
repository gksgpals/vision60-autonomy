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

"""Read synchronized LiDAR, image, and pose frames from rosbag2."""

from bisect import bisect_left
from dataclasses import dataclass
from pathlib import Path
from typing import List, Sequence

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2


@dataclass
class SynchronizedFrame:
    """One approximately synchronized LiDAR, image, and pose sample."""

    lidar_stamp_ns: int
    image_stamp_ns: int
    pose_stamp_ns: int
    cloud: object
    image: object
    pose: object


def message_stamp_ns(message, fallback_ns: int) -> int:
    """Read a ROS header timestamp or use the bag receive timestamp."""
    if not hasattr(message, 'header'):
        return int(fallback_ns)
    stamp = message.header.stamp
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value if value > 0 else int(fallback_ns)


def closest_timed(
    items: Sequence,
    stamps: Sequence[int],
    target_ns: int,
    tolerance_ns: int,
):
    """Return the nearest timestamped item inside a tolerance."""
    if not items:
        return None
    insertion = bisect_left(stamps, target_ns)
    candidates = []
    if insertion < len(items):
        candidates.append(insertion)
    if insertion > 0:
        candidates.append(insertion - 1)
    index = min(candidates, key=lambda value: abs(stamps[value] - target_ns))
    if abs(stamps[index] - target_ns) > tolerance_ns:
        return None
    return items[index]


def pointcloud_xyz(message) -> np.ndarray:
    """Convert a PointCloud2 message to finite XYZ points."""
    values = list(
        point_cloud2.read_points(
            message,
            field_names=('x', 'y', 'z'),
            skip_nans=True,
        )
    )
    array = np.asarray(values)
    if array.dtype.names:
        points = np.column_stack([
            array['x'],
            array['y'],
            array['z'],
        ]).astype(np.float64)
    else:
        points = array.astype(np.float64).reshape(-1, 3)
    if len(points) == 0:
        raise ValueError('PointCloud2 contains no finite XYZ points')
    return points


def image_to_rgb(message) -> np.ndarray:
    """Convert common uncompressed sensor_msgs/Image encodings to RGB."""
    encoding = message.encoding.lower()
    channels = {
        'rgb8': 3,
        'bgr8': 3,
        'rgba8': 4,
        'bgra8': 4,
        'mono8': 1,
    }.get(encoding)
    if channels is None:
        raise ValueError(f'unsupported image encoding: {message.encoding}')
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(
        int(message.height), int(message.step)
    )
    pixels = rows[:, :int(message.width) * channels].reshape(
        int(message.height), int(message.width), channels
    )
    if encoding == 'mono8':
        return np.repeat(pixels, 3, axis=2)
    if encoding in ('bgr8', 'bgra8'):
        pixels = pixels[:, :, [2, 1, 0, 3]] if channels == 4 else pixels[:, :, ::-1]
    return np.ascontiguousarray(pixels[:, :, :3])


def quaternion_rotation(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Convert a normalized xyzw quaternion to a rotation matrix."""
    norm = np.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise ValueError('pose quaternion has zero length')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def odometry_matrix(message) -> np.ndarray:
    """Convert nav_msgs/Odometry pose to a homogeneous matrix."""
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = quaternion_rotation(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    )
    matrix[:3, 3] = [position.x, position.y, position.z]
    return matrix


def read_synchronized_frames(
    bag_path: Path,
    lidar_topic: str = '/ouster/points',
    image_topic: str = '/camera/image_raw',
    pose_topic: str = '/slam/odom',
    max_frames: int = 100,
    image_tolerance_ms: float = 50.0,
    pose_tolerance_ms: float = 100.0,
    storage_id: str = 'mcap',
    min_lidar_interval_ms: float = 0.0,
) -> List[SynchronizedFrame]:
    """Read and approximately synchronize selected topics from a bag."""
    selected_topics = {lidar_topic, image_topic, pose_topic}
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id),
        rosbag2_py.ConverterOptions('', ''),
    )
    type_names = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    missing = selected_topics.difference(type_names)
    if missing:
        raise ValueError(f'bag is missing required topics: {sorted(missing)}')
    message_types = {
        topic: get_message(type_names[topic]) for topic in selected_topics
    }
    collected = {topic: [] for topic in selected_topics}
    read_cutoff_ns = None
    lookahead_ns = int(max(image_tolerance_ms, pose_tolerance_ms) * 1e6)
    lidar_interval_ns = int(min_lidar_interval_ms * 1e6)
    while reader.has_next():
        topic, serialized, receive_ns = reader.read_next()
        if topic not in selected_topics:
            continue
        message = deserialize_message(serialized, message_types[topic])
        stamp_ns = message_stamp_ns(message, receive_ns)
        if read_cutoff_ns is not None and stamp_ns > read_cutoff_ns:
            break
        if (
            topic == lidar_topic
            and len(collected[lidar_topic]) >= max_frames
        ):
            continue
        if (
            topic == lidar_topic
            and collected[lidar_topic]
            and stamp_ns - collected[lidar_topic][-1][0]
            < lidar_interval_ns
        ):
            continue
        collected[topic].append((stamp_ns, message))
        if (
            topic == lidar_topic
            and len(collected[lidar_topic]) == max_frames
        ):
            read_cutoff_ns = stamp_ns + lookahead_ns

    for values in collected.values():
        values.sort(key=lambda item: item[0])
    images = collected[image_topic]
    poses = collected[pose_topic]
    image_stamps = [item[0] for item in images]
    pose_stamps = [item[0] for item in poses]
    image_tolerance_ns = int(image_tolerance_ms * 1e6)
    pose_tolerance_ns = int(pose_tolerance_ms * 1e6)
    frames = []
    for lidar_stamp, cloud in collected[lidar_topic]:
        image_item = closest_timed(
            images, image_stamps, lidar_stamp, image_tolerance_ns
        )
        pose_item = closest_timed(
            poses, pose_stamps, lidar_stamp, pose_tolerance_ns
        )
        if image_item is None or pose_item is None:
            continue
        frames.append(SynchronizedFrame(
            lidar_stamp_ns=lidar_stamp,
            image_stamp_ns=image_item[0],
            pose_stamp_ns=pose_item[0],
            cloud=cloud,
            image=image_item[1],
            pose=pose_item[1],
        ))
        if len(frames) >= max_frames:
            break
    if not frames:
        raise ValueError('no synchronized LiDAR/image/pose frames found')
    return frames

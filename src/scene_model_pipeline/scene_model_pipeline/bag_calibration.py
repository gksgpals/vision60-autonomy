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

"""Extract camera calibration and sensor extrinsics from a rosbag2."""

from collections import deque
from pathlib import Path
from typing import Dict

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from scene_model_pipeline.bag_sequence import quaternion_rotation


def transform_matrix(transform) -> np.ndarray:
    """Convert geometry_msgs/Transform to a homogeneous matrix."""
    translation = transform.translation
    rotation = transform.rotation
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = quaternion_rotation(
        rotation.x, rotation.y, rotation.z, rotation.w
    )
    matrix[:3, 3] = [translation.x, translation.y, translation.z]
    return matrix


def resolve_transform(graph: Dict, source: str, target: str) -> np.ndarray:
    """Return target_from_source by walking a static transform graph."""
    source = source.lstrip('/')
    target = target.lstrip('/')
    if source == target:
        return np.eye(4, dtype=np.float64)
    queue = deque([(source, np.eye(4, dtype=np.float64))])
    visited = {source}
    while queue:
        current, current_from_source = queue.popleft()
        for neighbor, neighbor_from_current in graph.get(current, []):
            if neighbor in visited:
                continue
            neighbor_from_source = (
                neighbor_from_current @ current_from_source
            )
            if neighbor == target:
                return neighbor_from_source
            visited.add(neighbor)
            queue.append((neighbor, neighbor_from_source))
    raise ValueError(
        f'no static transform path from {source} to {target}'
    )


def read_embedded_calibration(
    bag_path: Path,
    lidar_topic: str = '/ouster/points',
    image_topic: str = '/camera/image_raw',
    pose_topic: str = '/slam/odom',
    camera_info_topic: str = '/camera/camera_info',
    tf_static_topic: str = '/tf_static',
    storage_id: str = 'mcap',
) -> Dict:
    """Read CameraInfo and static TF and return pipeline calibration."""
    required = {
        lidar_topic,
        image_topic,
        pose_topic,
        camera_info_topic,
        tf_static_topic,
    }
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id),
        rosbag2_py.ConverterOptions('', ''),
    )
    type_names = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    missing = required.difference(type_names)
    if missing:
        raise ValueError(
            f'bag is missing embedded calibration topics: {sorted(missing)}'
        )
    message_types = {
        topic: get_message(type_names[topic]) for topic in required
    }
    info = None
    lidar_frame = None
    camera_frame = None
    pose_child_frame = None
    graph = {}
    static_pairs = set()
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic not in required:
            continue
        message = deserialize_message(serialized, message_types[topic])
        if topic == camera_info_topic and info is None:
            info = message
            camera_frame = message.header.frame_id.lstrip('/')
        elif topic == lidar_topic and lidar_frame is None:
            lidar_frame = message.header.frame_id.lstrip('/')
        elif topic == image_topic and camera_frame is None:
            camera_frame = message.header.frame_id.lstrip('/')
        elif topic == pose_topic and pose_child_frame is None:
            pose_child_frame = message.child_frame_id.lstrip('/')
        elif topic == tf_static_topic:
            for stamped in message.transforms:
                parent = stamped.header.frame_id.lstrip('/')
                child = stamped.child_frame_id.lstrip('/')
                parent_from_child = transform_matrix(stamped.transform)
                graph.setdefault(child, []).append(
                    (parent, parent_from_child)
                )
                graph.setdefault(parent, []).append(
                    (child, np.linalg.inv(parent_from_child))
                )
                static_pairs.add((parent, child))
        if (
            info is not None
            and lidar_frame
            and camera_frame
            and pose_child_frame
            and graph
        ):
            try:
                camera_from_lidar = resolve_transform(
                    graph, lidar_frame, camera_frame
                )
                pose_child_from_lidar = resolve_transform(
                    graph, lidar_frame, pose_child_frame
                )
                break
            except ValueError:
                pass
    else:
        camera_from_lidar = None
        pose_child_from_lidar = None

    if info is None or not lidar_frame or not camera_frame or not pose_child_frame:
        raise ValueError('embedded calibration messages are incomplete')
    if camera_from_lidar is None or pose_child_from_lidar is None:
        camera_from_lidar = resolve_transform(
            graph, lidar_frame, camera_frame
        )
        pose_child_from_lidar = resolve_transform(
            graph, lidar_frame, pose_child_frame
        )
    if len(info.k) != 9 or float(info.k[0]) <= 0 or float(info.k[4]) <= 0:
        raise ValueError('CameraInfo intrinsic matrix is invalid')
    camera_model = (
        'OPENCV' if info.distortion_model == 'plumb_bob' else 'PINHOLE'
    )
    return {
        'camera_intrinsic': np.asarray(info.k, dtype=np.float64).reshape(
            3, 3
        ).tolist(),
        'camera_from_lidar': camera_from_lidar.tolist(),
        'pose_child_from_lidar': pose_child_from_lidar.tolist(),
        'camera_model': camera_model,
        'distortion_coefficients': list(info.d),
        'embedded_source': {
            'camera_info_topic': camera_info_topic,
            'tf_static_topic': tf_static_topic,
            'lidar_frame': lidar_frame,
            'camera_frame': camera_frame,
            'pose_child_frame': pose_child_frame,
            'static_transform_pairs': [
                list(value) for value in sorted(static_pairs)
            ],
        },
    }

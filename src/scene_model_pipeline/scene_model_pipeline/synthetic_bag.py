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

"""Create a deterministic multi-frame MCAP for sequence testing."""

import argparse
import json
from pathlib import Path

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
import numpy as np
from nav_msgs.msg import Odometry
import rosbag2_py
from rclpy.serialization import serialize_message
from sensor_msgs.msg import CameraInfo, Image
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage


def ros_time(timestamp_ns: int) -> Time:
    """Convert nanoseconds to builtin_interfaces/Time."""
    value = Time()
    value.sec = timestamp_ns // 1_000_000_000
    value.nanosec = timestamp_ns % 1_000_000_000
    return value


def topic_metadata(name: str, type_name: str):
    """Construct TopicMetadata across Humble patch releases."""
    try:
        return rosbag2_py.TopicMetadata(
            name=name,
            type=type_name,
            serialization_format='cdr',
            offered_qos_profiles='',
        )
    except TypeError:
        return rosbag2_py.TopicMetadata(
            name=name,
            type=type_name,
            serialization_format='cdr',
        )


def synthetic_image(width: int, height: int, frame_index: int) -> np.ndarray:
    """Create a repeatable RGB test image."""
    red = np.broadcast_to(
        np.linspace(20, 240, width, dtype=np.uint8),
        (height, width),
    )
    green = np.broadcast_to(
        np.linspace(30, 220, height, dtype=np.uint8)[:, None],
        (height, width),
    )
    blue = (
        (np.indices((height, width)).sum(axis=0) // 24 + frame_index)
        % 2 * 160 + 60
    ).astype(np.uint8)
    return np.ascontiguousarray(np.dstack([red, green, blue]))


def local_surface_points(
    width: int,
    height: int,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    robot_x: float,
) -> np.ndarray:
    """Return a fixed world surface expressed in a moving LiDAR frame."""
    world_x, world_y = np.meshgrid(
        np.linspace(-1.6, 1.6, 65),
        np.linspace(-1.1, 1.1, 45),
    )
    world_z = (
        3.2
        + 0.15 * np.sin(world_x * 3.0)
        + 0.10 * np.cos(world_y * 4.0)
    )
    points = np.column_stack([
        (world_x - robot_x).ravel(),
        world_y.ravel(),
        world_z.ravel(),
    ])
    projected_u = fx * points[:, 0] / points[:, 2] + cx
    projected_v = fy * points[:, 1] / points[:, 2] + cy
    visible = (
        (projected_u >= 0)
        & (projected_u < width)
        & (projected_v >= 0)
        & (projected_v < height)
    )
    return points[visible]


def write_synthetic_mcap(
    bag_path: Path,
    frame_count: int = 4,
    image_offset_ms: float = -10.0,
    pose_offset_ms: float = -5.0,
) -> None:
    """Write synchronized PointCloud2, Image, and Odometry messages."""
    bag_path = Path(bag_path)
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    topics = [
        ('/ouster/points', 'sensor_msgs/msg/PointCloud2'),
        ('/camera/image_raw', 'sensor_msgs/msg/Image'),
        ('/camera/camera_info', 'sensor_msgs/msg/CameraInfo'),
        ('/slam/odom', 'nav_msgs/msg/Odometry'),
        ('/tf_static', 'tf2_msgs/msg/TFMessage'),
    ]
    for name, type_name in topics:
        writer.create_topic(topic_metadata(name, type_name))

    width, height = 320, 240
    fx = fy = 220.0
    cx, cy = width / 2.0, height / 2.0
    transform = TransformStamped()
    transform.header.stamp = ros_time(500_000_000)
    transform.header.frame_id = 'lidar'
    transform.child_frame_id = 'camera'
    transform.transform.rotation.w = 1.0
    writer.write(
        '/tf_static',
        serialize_message(TFMessage(transforms=[transform])),
        500_000_000,
    )
    for index in range(frame_count):
        lidar_ns = (index + 1) * 1_000_000_000
        image_ns = lidar_ns + int(image_offset_ms * 1e6)
        pose_ns = lidar_ns + int(pose_offset_ms * 1e6)
        robot_x = index * 0.20

        image_array = synthetic_image(width, height, index)
        image = Image()
        image.header.stamp = ros_time(image_ns)
        image.header.frame_id = 'camera'
        image.height = height
        image.width = width
        image.encoding = 'rgb8'
        image.is_bigendian = False
        image.step = width * 3
        image.data = image_array.tobytes()

        camera_info = CameraInfo()
        camera_info.header = image.header
        camera_info.height = height
        camera_info.width = width
        camera_info.distortion_model = 'plumb_bob'
        camera_info.d = [0.0] * 5
        camera_info.k = [
            fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0
        ]
        camera_info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        camera_info.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]

        odometry = Odometry()
        odometry.header.stamp = ros_time(pose_ns)
        odometry.header.frame_id = 'map'
        odometry.child_frame_id = 'lidar'
        odometry.pose.pose.position.x = robot_x
        odometry.pose.pose.orientation.w = 1.0

        header = Header()
        header.stamp = ros_time(lidar_ns)
        header.frame_id = 'lidar'
        points = local_surface_points(
            width, height, fx, fy, cx, cy, robot_x
        )
        cloud = point_cloud2.create_cloud_xyz32(header, points)

        writer.write(
            '/camera/image_raw', serialize_message(image), image_ns
        )
        writer.write(
            '/camera/camera_info',
            serialize_message(camera_info),
            image_ns,
        )
        writer.write('/slam/odom', serialize_message(odometry), pose_ns)
        writer.write('/ouster/points', serialize_message(cloud), lidar_ns)


def write_sequence_inputs(output_dir: Path, **bag_options) -> Path:
    """Write MCAP plus calibration and mission metadata files."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    bag_path = output_dir / 'mission_sequence'
    write_synthetic_mcap(bag_path, **bag_options)
    calibration = {
        'camera_intrinsic': [
            [220.0, 0.0, 160.0],
            [0.0, 220.0, 120.0],
            [0.0, 0.0, 1.0],
        ],
        'camera_from_lidar': np.eye(4).tolist(),
    }
    metadata = {
        'mission_id': 'mock_sequence_mission_001',
        'scene_id': 'mock_sequence_scene_001',
        'source_id': 'synthetic_mcap_lidar_camera_glim_pose',
        'coordinate_frame': 'map',
    }
    for name, value in (
        ('calibration.json', calibration),
        ('metadata.json', metadata),
    ):
        with (output_dir / name).open('w', encoding='utf-8') as stream:
            json.dump(value, stream, indent=2)
            stream.write('\n')
    return bag_path


def main(args=None) -> None:
    """Generate a test MCAP and companion files."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--frames', type=int, default=4)
    parsed = parser.parse_args(args)
    bag_path = write_sequence_inputs(
        parsed.output,
        frame_count=parsed.frames,
    )
    print('SYNTHETIC_MCAP=PASS')
    print(str(bag_path))

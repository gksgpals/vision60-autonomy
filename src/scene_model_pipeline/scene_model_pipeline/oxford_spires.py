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

"""Convert a minimal Oxford Spires sample to the project MCAP schema."""

from bisect import bisect_left
import csv
import json
from pathlib import Path
from typing import Dict, List

import cv2
from nav_msgs.msg import Odometry
import numpy as np
import open3d as o3d
import rosbag2_py
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import yaml

from scene_model_pipeline.bag_sequence import quaternion_rotation
from scene_model_pipeline.core import colorize_visible_points, file_sha256
from scene_model_pipeline.sequence import path_sha256
from scene_model_pipeline.synthetic_bag import ros_time, topic_metadata


DATASET_REVISION = '03f4382308333aa70c3253f12acd3fbf0c7c4a15'
TOOLS_COMMIT = 'b456e1e2f263a79c19b6ed4052390eba609011d4'
DATASET_URL = (
    'https://huggingface.co/datasets/ori-drs/'
    'oxford_spires_dataset'
)
TOOLS_URL = 'https://github.com/ori-drs/oxford_spires_dataset'


def timestamp_from_name(path: Path) -> int:
    """Parse a seconds.nanoseconds filename without float rounding."""
    seconds, nanoseconds = Path(path).stem.split('.', maxsplit=1)
    return int(seconds) * 1_000_000_000 + int(nanoseconds.ljust(9, '0'))


def transform_from_xyz_quaternion(values) -> np.ndarray:
    """Convert xyz+xyzw into a homogeneous transformation matrix."""
    if len(values) != 7:
        raise ValueError('transform must contain xyz and xyzw values')
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = quaternion_rotation(*map(float, values[3:]))
    transform[:3, 3] = np.asarray(values[:3], dtype=np.float64)
    return transform


def _load_poses(path: Path) -> List[Dict]:
    poses = []
    with Path(path).open(encoding='utf-8') as stream:
        reader = csv.reader(
            line for line in stream if not line.lstrip().startswith('#')
        )
        for row in reader:
            values = [value.strip() for value in row]
            timestamp_ns = (
                int(values[1]) * 1_000_000_000 + int(values[2])
            )
            poses.append({
                'timestamp_ns': timestamp_ns,
                'position': list(map(float, values[3:6])),
                'orientation_xyzw': list(map(float, values[6:10])),
            })
    if not poses:
        raise ValueError('Oxford pose CSV contains no poses')
    return sorted(poses, key=lambda value: value['timestamp_ns'])


def _closest(values: List[Dict], timestamp_ns: int) -> Dict:
    stamps = [value['timestamp_ns'] for value in values]
    index = bisect_left(stamps, timestamp_ns)
    candidates = []
    if index < len(values):
        candidates.append(values[index])
    if index > 0:
        candidates.append(values[index - 1])
    return min(
        candidates,
        key=lambda value: abs(value['timestamp_ns'] - timestamp_ns),
    )


def _image_message(path: Path, timestamp_ns: int) -> Image:
    bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError(f'failed to read image: {path}')
    rgb = np.ascontiguousarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
    message = Image()
    message.header.stamp = ros_time(timestamp_ns)
    message.header.frame_id = 'cam_front'
    message.height, message.width = rgb.shape[:2]
    message.encoding = 'rgb8'
    message.is_bigendian = False
    message.step = message.width * 3
    message.data = rgb.tobytes()
    return message


def _cloud_message(path: Path, timestamp_ns: int):
    cloud = o3d.io.read_point_cloud(str(path))
    points = np.asarray(cloud.points)
    if len(points) == 0:
        raise ValueError(f'failed to read cloud: {path}')
    header = Header()
    header.stamp = ros_time(timestamp_ns)
    header.frame_id = 'lidar'
    return point_cloud2.create_cloud_xyz32(header, points)


def _pose_message(pose: Dict) -> Odometry:
    message = Odometry()
    message.header.stamp = ros_time(pose['timestamp_ns'])
    message.header.frame_id = 'map'
    message.child_frame_id = 'base'
    position = pose['position']
    orientation = pose['orientation_xyzw']
    message.pose.pose.position.x = position[0]
    message.pose.pose.position.y = position[1]
    message.pose.pose.position.z = position[2]
    message.pose.pose.orientation.x = orientation[0]
    message.pose.pose.orientation.y = orientation[1]
    message.pose.pose.orientation.z = orientation[2]
    message.pose.pose.orientation.w = orientation[3]
    return message


def _write_json(path: Path, value: Dict) -> None:
    with Path(path).open('w', encoding='utf-8') as stream:
        json.dump(value, stream, indent=2)
        stream.write('\n')


def _write_projection_overlay(
    cloud_path: Path,
    image_path: Path,
    calibration: Dict,
    output_path: Path,
) -> int:
    """Render a depth-colored LiDAR overlay for calibration inspection."""
    cloud = o3d.io.read_point_cloud(str(cloud_path))
    points = np.asarray(cloud.points)
    bgr = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    intrinsic = np.asarray(calibration['camera_intrinsic'])
    camera_from_lidar = np.asarray(calibration['camera_from_lidar'])
    visible, _ = colorize_visible_points(
        points,
        rgb,
        intrinsic,
        camera_from_lidar,
        calibration['camera_model'],
        calibration['distortion_coefficients'],
    )
    homogeneous = np.column_stack([visible, np.ones(len(visible))])
    camera_points = (camera_from_lidar @ homogeneous.T).T[:, :3]
    pixels, _ = cv2.fisheye.projectPoints(
        camera_points.reshape(1, -1, 3),
        np.zeros(3),
        np.zeros(3),
        intrinsic,
        np.asarray(calibration['distortion_coefficients']),
    )
    pixels = np.rint(pixels.reshape(-1, 2)).astype(np.int32)
    depth = camera_points[:, 2]
    lower, upper = np.percentile(depth, [2.0, 98.0])
    normalized = np.clip((depth - lower) / max(upper - lower, 1e-6), 0, 1)
    indices = np.rint((1.0 - normalized) * 255).astype(np.uint8)
    colors = cv2.applyColorMap(indices[:, None], cv2.COLORMAP_TURBO)
    for pixel, color in zip(pixels, colors[:, 0]):
        cv2.circle(
            bgr,
            (int(pixel[0]), int(pixel[1])),
            1,
            tuple(int(value) for value in color),
            -1,
            cv2.LINE_AA,
        )
    cv2.putText(
        bgr,
        f'Oxford Spires LiDAR projection: {len(visible)} points',
        (24, 42),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    if not cv2.imwrite(str(output_path), bgr):
        raise RuntimeError('failed to write Oxford projection overlay')
    return len(visible)


def convert_oxford_spires_sample(
    input_dir: Path,
    output_dir: Path,
    image_tolerance_ms: float = 25.0,
    pose_tolerance_ms: float = 100.0,
) -> Dict:
    """Convert selected real sensor files into synchronized ROS2 MCAP."""
    input_dir = Path(input_dir)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    image_paths = sorted((input_dir / 'images' / 'cam0').glob('*.jpg'))
    cloud_paths = sorted(
        (input_dir / 'clouds' / 'lidar-clouds').glob('*.pcd')
    )
    if not image_paths or not cloud_paths:
        raise ValueError('Oxford sample is missing images or LiDAR clouds')
    images = [
        {'timestamp_ns': timestamp_from_name(path), 'path': path}
        for path in image_paths
    ]
    poses = _load_poses(input_dir / 'slam-poses.csv')

    with (input_dir / 'sensor.yaml').open(encoding='utf-8') as stream:
        sensor = yaml.safe_load(stream)['sensor']
    camera = next(
        value for value in sensor['cameras']
        if value['label'] == 'cam_front'
    )
    fx, fy, cx, cy = map(float, camera['intrinsics'])
    calibration = {
        'camera_intrinsic': [
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0],
        ],
        'camera_from_lidar': transform_from_xyz_quaternion(
            camera['T_cam_lidar_t_xyz_q_xyzw']
        ).tolist(),
        'camera_model': sensor['camera_model'],
        'distortion_coefficients': list(map(
            float, camera['extra_params']
        )),
        'pose_child_from_lidar': transform_from_xyz_quaternion(
            sensor['T_base_lidar_t_xyz_q_xyzw']
        ).tolist(),
        'source': {
            'repository': TOOLS_URL,
            'commit': TOOLS_COMMIT,
        },
    }
    calibration_path = output_dir / 'calibration.json'
    _write_json(calibration_path, calibration)
    metadata = {
        'mission_id': 'oxford_spires_public_regression_001',
        'scene_id': 'keble_college_01_sample',
        'source_id': 'oxford_spires_2024-03-12-keble-college-01',
        'coordinate_frame': 'map',
        'dataset_url': DATASET_URL,
        'dataset_revision': DATASET_REVISION,
        'license': 'CC-BY-NC-SA-4.0',
    }
    metadata_path = output_dir / 'metadata.json'
    _write_json(metadata_path, metadata)

    bag_path = output_dir / 'oxford_spires_sequence'
    if bag_path.exists():
        raise FileExistsError(f'output bag already exists: {bag_path}')
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    for name, type_name in (
        ('/ouster/points', 'sensor_msgs/msg/PointCloud2'),
        ('/camera/image_raw', 'sensor_msgs/msg/Image'),
        ('/slam/odom', 'nav_msgs/msg/Odometry'),
    ):
        writer.create_topic(topic_metadata(name, type_name))

    selected = []
    for cloud_path in cloud_paths:
        lidar_ns = timestamp_from_name(cloud_path)
        image = _closest(images, lidar_ns)
        pose = _closest(poses, lidar_ns)
        image_delta_ms = abs(image['timestamp_ns'] - lidar_ns) / 1e6
        pose_delta_ms = abs(pose['timestamp_ns'] - lidar_ns) / 1e6
        if image_delta_ms > image_tolerance_ms:
            raise ValueError('Oxford image/LiDAR pair exceeds tolerance')
        if pose_delta_ms > pose_tolerance_ms:
            raise ValueError('Oxford pose/LiDAR pair exceeds tolerance')
        cloud_message = _cloud_message(cloud_path, lidar_ns)
        image_message = _image_message(
            image['path'], image['timestamp_ns']
        )
        pose_message = _pose_message(pose)
        writer.write(
            '/ouster/points', serialize_message(cloud_message), lidar_ns
        )
        writer.write(
            '/camera/image_raw',
            serialize_message(image_message),
            image['timestamp_ns'],
        )
        writer.write(
            '/slam/odom',
            serialize_message(pose_message),
            pose['timestamp_ns'],
        )
        selected.append({
            'lidar_file': cloud_path.name,
            'lidar_timestamp_ns': lidar_ns,
            'image_file': image['path'].name,
            'image_timestamp_ns': image['timestamp_ns'],
            'image_delta_ms': image_delta_ms,
            'pose_timestamp_ns': pose['timestamp_ns'],
            'pose_delta_ms': pose_delta_ms,
        })
    del writer

    route_overlay = {
        'schema_version': 1,
        'mission_id': metadata['mission_id'],
        'scene_id': metadata['scene_id'],
        'coordinate_frame': metadata['coordinate_frame'],
        'timestamp_ns': selected[-1]['lidar_timestamp_ns'],
        'route': {
            'id': 'oxford_ground_truth_route_sample',
            'points': [pose['position'] for pose in (
                _closest(poses, value['lidar_timestamp_ns'])
                for value in selected
            )],
        },
        'communication_zones': [],
        'mission_events': [],
    }
    route_path = output_dir / 'route_overlay.json'
    _write_json(route_path, route_overlay)
    cloud_by_name = {path.name: path for path in cloud_paths}
    image_by_name = {path.name: path for path in image_paths}
    projection_path = output_dir / 'projection_overlay.png'
    projected_count = _write_projection_overlay(
        cloud_by_name[selected[0]['lidar_file']],
        image_by_name[selected[0]['image_file']],
        calibration,
        projection_path,
    )
    source_files = [
        input_dir / 'sensor.yaml',
        input_dir / 'slam-poses.csv',
        *image_paths,
        *cloud_paths,
    ]
    license_path = input_dir / 'LICENSE.md'
    if license_path.exists():
        source_files.append(license_path)
    manifest = {
        'schema_version': 1,
        'dataset': 'Oxford Spires Dataset',
        'dataset_url': DATASET_URL,
        'dataset_revision': DATASET_REVISION,
        'tools_repository': TOOLS_URL,
        'tools_commit': TOOLS_COMMIT,
        'license': 'CC-BY-NC-SA-4.0',
        'selected_frames': selected,
        'source_files': [
            {
                'file': str(path.relative_to(input_dir)),
                'sha256': file_sha256(path),
            }
            for path in source_files
        ],
        'products': {
            'bag': {
                'directory': bag_path.name,
                'sha256': path_sha256(bag_path),
            },
            'calibration': {
                'file': calibration_path.name,
                'sha256': file_sha256(calibration_path),
            },
            'metadata': {
                'file': metadata_path.name,
                'sha256': file_sha256(metadata_path),
            },
            'route_overlay': {
                'file': route_path.name,
                'sha256': file_sha256(route_path),
            },
            'projection_overlay': {
                'file': projection_path.name,
                'sha256': file_sha256(projection_path),
                'projected_point_count': projected_count,
            },
        },
    }
    _write_json(output_dir / 'source_manifest.json', manifest)
    return manifest

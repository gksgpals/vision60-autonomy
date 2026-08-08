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

"""Accumulate synchronized rosbag2 frames into one traceable scene."""

import hashlib
import json
from pathlib import Path
from typing import Dict

import numpy as np
import open3d as o3d

from scene_model_pipeline import ALGORITHM_VERSION
from scene_model_pipeline.bag_sequence import (
    image_to_rgb,
    odometry_matrix,
    pointcloud_xyz,
    read_synchronized_frames,
)
from scene_model_pipeline.bag_calibration import read_embedded_calibration
from scene_model_pipeline.core import (
    build_sparse_voxel_map,
    colorize_visible_points,
    file_sha256,
    load_json,
    make_point_cloud,
    reconstruct_mesh,
)


def path_sha256(path: Path) -> str:
    """Hash one file or every file in a rosbag2 directory."""
    path = Path(path)
    if path.is_file():
        return file_sha256(path)
    digest = hashlib.sha256()
    for item in sorted(value for value in path.rglob('*') if value.is_file()):
        digest.update(str(item.relative_to(path)).encode('utf-8'))
        digest.update(bytes.fromhex(file_sha256(item)))
    return digest.hexdigest()


def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    """Apply a homogeneous transform to XYZ points."""
    homogeneous = np.column_stack(
        [points, np.ones(len(points), dtype=np.float64)]
    )
    return (transform @ homogeneous.T).T[:, :3]


def _write_product(path: Path, **details) -> Dict:
    return {'file': path.name, 'sha256': file_sha256(path), **details}


def build_cumulative_scene(
    bag_path: Path,
    calibration_path: Path = None,
    metadata_path: Path = None,
    output_dir: Path = None,
    voxel_size: float = 0.10,
    mesh_depth: int = 7,
    max_frames: int = 100,
    image_tolerance_ms: float = 50.0,
    pose_tolerance_ms: float = 100.0,
    lidar_topic: str = '/ouster/points',
    image_topic: str = '/camera/image_raw',
    pose_topic: str = '/slam/odom',
    storage_id: str = 'mcap',
    frame_interval_ms: float = 0.0,
) -> Dict:
    """Read synchronized bag frames and generate one map-frame model."""
    bag_path = Path(bag_path)
    metadata_path = Path(metadata_path)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    if calibration_path is None:
        calibration = read_embedded_calibration(
            bag_path,
            lidar_topic,
            image_topic,
            pose_topic,
            storage_id=storage_id,
        )
        calibration_blob = json.dumps(
            calibration, sort_keys=True, separators=(',', ':')
        ).encode('utf-8')
        calibration_record = {
            'source': 'bag_topics',
            'camera_info_topic': calibration['embedded_source'][
                'camera_info_topic'
            ],
            'tf_static_topic': calibration['embedded_source'][
                'tf_static_topic'
            ],
            'values_sha256': hashlib.sha256(calibration_blob).hexdigest(),
            'frames': calibration['embedded_source'],
        }
    else:
        calibration_path = Path(calibration_path)
        calibration = load_json(calibration_path)
        calibration_record = {
            'source': 'file',
            'file': calibration_path.name,
            'sha256': file_sha256(calibration_path),
        }
    metadata = load_json(metadata_path)
    intrinsic = np.asarray(
        calibration['camera_intrinsic'], dtype=np.float64
    )
    camera_from_lidar = np.asarray(
        calibration['camera_from_lidar'], dtype=np.float64
    )
    pose_child_from_lidar = np.asarray(
        calibration.get('pose_child_from_lidar', np.eye(4)),
        dtype=np.float64,
    )
    if pose_child_from_lidar.shape != (4, 4):
        raise ValueError('pose_child_from_lidar must have shape (4, 4)')

    frames = read_synchronized_frames(
        bag_path,
        lidar_topic,
        image_topic,
        pose_topic,
        max_frames,
        image_tolerance_ms,
        pose_tolerance_ms,
        storage_id,
        frame_interval_ms,
    )
    colored_points_map = []
    colors = []
    voxel_endpoints_map = []
    voxel_origins_map = []
    frame_records = []
    rejected_frames = []
    expected_map_frame = str(metadata.get('coordinate_frame', 'map'))

    for index, frame in enumerate(frames):
        cloud_frame = frame.cloud.header.frame_id.lstrip('/')
        pose_child_frame = frame.pose.child_frame_id.lstrip('/')
        pose_map_frame = frame.pose.header.frame_id.lstrip('/')
        if pose_map_frame != expected_map_frame:
            raise ValueError(
                f'pose frame {pose_map_frame} != {expected_map_frame}'
            )
        if cloud_frame != pose_child_frame and (
            'pose_child_from_lidar' not in calibration
        ):
            raise ValueError(
                'cloud and pose child frames differ without an extrinsic'
            )

        try:
            points_lidar = pointcloud_xyz(frame.cloud)
            image_rgb = image_to_rgb(frame.image)
            colored_local, frame_colors = colorize_visible_points(
                points_lidar,
                image_rgb,
                intrinsic,
                camera_from_lidar,
                calibration.get('camera_model', 'PINHOLE'),
                calibration.get('distortion_coefficients', []),
            )
        except ValueError as error:
            rejected_frames.append({
                'source_index': index,
                'lidar_timestamp_ns': frame.lidar_stamp_ns,
                'reason': str(error),
            })
            continue
        map_from_lidar = (
            odometry_matrix(frame.pose) @ pose_child_from_lidar
        )
        colored_map = transform_points(colored_local, map_from_lidar)
        colored_points_map.append(colored_map)
        colors.append(frame_colors)

        frame_cloud = make_point_cloud(colored_local, frame_colors)
        frame_voxels = frame_cloud.voxel_down_sample(voxel_size)
        endpoints_map = transform_points(
            np.asarray(frame_voxels.points), map_from_lidar
        )
        origin_map = map_from_lidar[:3, 3]
        voxel_endpoints_map.append(endpoints_map)
        voxel_origins_map.append(
            np.repeat(origin_map[None, :], len(endpoints_map), axis=0)
        )
        frame_records.append({
            'index': len(frame_records),
            'source_index': index,
            'lidar_timestamp_ns': frame.lidar_stamp_ns,
            'image_timestamp_ns': frame.image_stamp_ns,
            'pose_timestamp_ns': frame.pose_stamp_ns,
            'image_delta_ms': abs(
                frame.lidar_stamp_ns - frame.image_stamp_ns
            ) / 1e6,
            'pose_delta_ms': abs(
                frame.lidar_stamp_ns - frame.pose_stamp_ns
            ) / 1e6,
            'colored_point_count': len(colored_map),
            'map_from_lidar': map_from_lidar.tolist(),
        })

    if not colored_points_map:
        raise ValueError('no camera-visible LiDAR frames were usable')
    points_map = np.vstack(colored_points_map)
    all_colors = np.vstack(colors)
    cumulative_cloud = make_point_cloud(points_map, all_colors)
    voxel_cloud = cumulative_cloud.voxel_down_sample(voxel_size)
    endpoints = np.vstack(voxel_endpoints_map)
    origins = np.vstack(voxel_origins_map)
    voxel_map = build_sparse_voxel_map(
        endpoints,
        voxel_size,
        origins,
    )
    mesh = reconstruct_mesh(
        cumulative_cloud,
        origins.mean(axis=0),
        voxel_size,
        mesh_depth,
    )

    cloud_path = output_dir / 'cumulative_colored_cloud.ply'
    voxel_cloud_path = output_dir / 'cumulative_voxel_cloud.ply'
    voxel_map_path = output_dir / 'cumulative_voxel_map.npz'
    mesh_path = output_dir / 'cumulative_scene_mesh.ply'
    if not o3d.io.write_point_cloud(str(cloud_path), cumulative_cloud):
        raise RuntimeError('failed to write cumulative colored cloud')
    if not o3d.io.write_point_cloud(str(voxel_cloud_path), voxel_cloud):
        raise RuntimeError('failed to write cumulative voxel cloud')
    np.savez_compressed(voxel_map_path, **voxel_map)
    if not o3d.io.write_triangle_mesh(str(mesh_path), mesh):
        raise RuntimeError('failed to write cumulative mesh')

    manifest = {
        'schema_version': 1,
        'algorithm_version': ALGORITHM_VERSION,
        'mission_id': metadata['mission_id'],
        'scene_id': metadata['scene_id'],
        'source_id': metadata['source_id'],
        'coordinate_frame': expected_map_frame,
        'bag': {
            'path': bag_path.name,
            'sha256': path_sha256(bag_path),
            'storage_id': storage_id,
            'topics': {
                'lidar': lidar_topic,
                'image': image_topic,
                'pose': pose_topic,
            },
            'frame_interval_ms': frame_interval_ms,
        },
        'calibration': calibration_record,
        'metadata': {
            'file': metadata_path.name,
            'sha256': file_sha256(metadata_path),
        },
        'frame_count': len(frames),
        'usable_frame_count': len(frame_records),
        'rejected_frame_count': len(rejected_frames),
        'frames': frame_records,
        'rejected_frames': rejected_frames,
        'products': {
            'colored_cloud': _write_product(
                cloud_path,
                point_count=len(cumulative_cloud.points),
            ),
            'voxel_cloud': _write_product(
                voxel_cloud_path,
                point_count=len(voxel_cloud.points),
                voxel_size_m=voxel_size,
            ),
            'voxel_map': _write_product(
                voxel_map_path,
                occupied_count=len(voxel_map['occupied_indices']),
                free_count=len(voxel_map['free_indices']),
                unknown_count=int(voxel_map['unknown_count'][0]),
            ),
            'mesh': _write_product(
                mesh_path,
                vertex_count=len(mesh.vertices),
                triangle_count=len(mesh.triangles),
            ),
        },
    }
    manifest_path = output_dir / 'cumulative_manifest.json'
    with manifest_path.open('w', encoding='utf-8') as stream:
        json.dump(manifest, stream, indent=2)
        stream.write('\n')
    return manifest

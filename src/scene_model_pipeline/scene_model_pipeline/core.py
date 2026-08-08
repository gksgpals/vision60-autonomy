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

"""Core processing for traceable offline 3D scene products."""

import hashlib
import json
from pathlib import Path
from typing import Dict, Tuple

import cv2
import numpy as np
import open3d as o3d

from scene_model_pipeline import ALGORITHM_VERSION


def file_sha256(path: Path) -> str:
    """Return the SHA-256 digest of a file without modifying it."""
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path) -> Dict:
    """Load a JSON object from disk."""
    with path.open('r', encoding='utf-8') as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise ValueError(f'expected a JSON object: {path}')
    return value


def validate_timestamps(metadata: Dict, max_delta_ms: float) -> float:
    """Reject camera and LiDAR data that are too far apart in time."""
    lidar_ns = int(metadata['lidar_timestamp_ns'])
    image_ns = int(metadata['image_timestamp_ns'])
    delta_ms = abs(lidar_ns - image_ns) / 1e6
    if delta_ms > max_delta_ms:
        raise ValueError(
            f'camera/LiDAR time delta {delta_ms:.3f} ms exceeds '
            f'{max_delta_ms:.3f} ms'
        )
    return delta_ms


def colorize_visible_points(
    points_lidar: np.ndarray,
    image_rgb: np.ndarray,
    intrinsic: np.ndarray,
    camera_from_lidar: np.ndarray,
    camera_model: str = 'PINHOLE',
    distortion_coefficients=None,
) -> Tuple[np.ndarray, np.ndarray]:
    """Project LiDAR into pinhole or OpenCV images with z-buffering."""
    if points_lidar.ndim != 2 or points_lidar.shape[1] != 3:
        raise ValueError('points_lidar must have shape (N, 3)')
    if image_rgb.ndim != 3 or image_rgb.shape[2] < 3:
        raise ValueError('image must contain at least three color channels')
    if intrinsic.shape != (3, 3):
        raise ValueError('camera intrinsic must have shape (3, 3)')
    if camera_from_lidar.shape != (4, 4):
        raise ValueError('camera_from_lidar must have shape (4, 4)')

    homogeneous = np.column_stack(
        [points_lidar, np.ones(len(points_lidar), dtype=np.float64)]
    )
    points_camera = (camera_from_lidar @ homogeneous.T).T[:, :3]
    depth = points_camera[:, 2]
    positive = depth > 1e-6
    source_indices = np.flatnonzero(positive)
    points_camera = points_camera[positive]
    depth = depth[positive]

    camera_model = str(camera_model).upper()
    coefficients = np.asarray(
        [] if distortion_coefficients is None
        else distortion_coefficients,
        dtype=np.float64,
    )
    if camera_model == 'PINHOLE':
        projected = (intrinsic @ points_camera.T).T
        pixels = projected[:, :2] / projected[:, 2:3]
    elif camera_model == 'OPENCV':
        if coefficients.size not in (4, 5, 8, 12, 14):
            raise ValueError('OPENCV distortion coefficients are invalid')
        pixels, _ = cv2.projectPoints(
            points_camera.reshape(-1, 1, 3),
            np.zeros(3),
            np.zeros(3),
            intrinsic,
            coefficients,
        )
        pixels = pixels.reshape(-1, 2)
    elif camera_model == 'OPENCV_FISHEYE':
        if coefficients.size != 4:
            raise ValueError('OPENCV_FISHEYE requires 4 coefficients')
        pixels, _ = cv2.fisheye.projectPoints(
            points_camera.reshape(1, -1, 3),
            np.zeros(3),
            np.zeros(3),
            intrinsic,
            coefficients,
        )
        pixels = pixels.reshape(-1, 2)
    else:
        raise ValueError(f'unsupported camera model: {camera_model}')
    u = np.rint(pixels[:, 0]).astype(np.int64)
    v = np.rint(pixels[:, 1]).astype(np.int64)
    height, width = image_rgb.shape[:2]
    inside = (u >= 0) & (u < width) & (v >= 0) & (v < height)
    source_indices = source_indices[inside]
    depth = depth[inside]
    u = u[inside]
    v = v[inside]
    if len(source_indices) == 0:
        raise ValueError('no LiDAR points project inside the camera image')

    pixel_id = v * width + u
    order = np.lexsort((depth, pixel_id))
    ordered_pixels = pixel_id[order]
    nearest = np.empty(len(order), dtype=bool)
    nearest[0] = True
    nearest[1:] = ordered_pixels[1:] != ordered_pixels[:-1]
    selected = order[nearest]
    selected_sources = source_indices[selected]
    colors = image_rgb[v[selected], u[selected], :3].astype(np.float64)
    if np.issubdtype(image_rgb.dtype, np.integer):
        colors /= np.iinfo(image_rgb.dtype).max
    else:
        colors = np.clip(colors, 0.0, 1.0)
    return points_lidar[selected_sources], colors


def make_point_cloud(
    points: np.ndarray,
    colors: np.ndarray,
) -> o3d.geometry.PointCloud:
    """Create an Open3D colored point cloud."""
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    cloud.colors = o3d.utility.Vector3dVector(colors.astype(np.float64))
    return cloud


def build_sparse_voxel_map(
    endpoints: np.ndarray,
    voxel_size: float,
    sensor_origin: np.ndarray,
) -> Dict[str, np.ndarray]:
    """Raytrace occupied/free voxels; cells absent from output are unknown."""
    if voxel_size <= 0.0:
        raise ValueError('voxel_size must be positive')
    occupied = set()
    free = set()
    sample_step = voxel_size / 3.0
    sensor_origin = np.asarray(sensor_origin, dtype=np.float64)
    if sensor_origin.shape == (3,):
        sensor_origins = np.repeat(
            sensor_origin[None, :], len(endpoints), axis=0
        )
    elif sensor_origin.shape == endpoints.shape:
        sensor_origins = sensor_origin
    else:
        raise ValueError('sensor_origin must have shape (3,) or (N, 3)')

    for endpoint, ray_origin in zip(endpoints, sensor_origins):
        distance = float(np.linalg.norm(endpoint - ray_origin))
        sample_count = max(2, int(np.ceil(distance / sample_step)) + 1)
        samples = np.linspace(ray_origin, endpoint, sample_count)
        indices = np.floor(samples / voxel_size).astype(np.int32)
        unique_indices = np.unique(indices, axis=0)
        endpoint_index = tuple(
            np.floor(endpoint / voxel_size).astype(np.int32)
        )
        occupied.add(endpoint_index)
        free.update(
            tuple(index) for index in unique_indices
            if tuple(index) != endpoint_index
        )

    free.difference_update(occupied)
    occupied_array = np.asarray(sorted(occupied), dtype=np.int32).reshape(-1, 3)
    free_array = np.asarray(sorted(free), dtype=np.int32).reshape(-1, 3)
    observed = np.vstack([occupied_array, free_array])
    lower = observed.min(axis=0) - 1
    upper = observed.max(axis=0) + 1
    dimensions = upper - lower + 1
    bounded_count = int(np.prod(dimensions, dtype=np.int64))
    unknown_count = bounded_count - len(occupied_array) - len(free_array)
    return {
        'occupied_indices': occupied_array,
        'free_indices': free_array,
        'grid_min_index': lower.astype(np.int32),
        'grid_max_index': upper.astype(np.int32),
        'grid_dimensions': dimensions.astype(np.int32),
        'voxel_size_m': np.asarray([voxel_size], dtype=np.float64),
        'unknown_count': np.asarray([unknown_count], dtype=np.int64),
    }


def reconstruct_mesh(
    cloud: o3d.geometry.PointCloud,
    camera_origin: np.ndarray,
    voxel_size: float,
    depth: int,
) -> o3d.geometry.TriangleMesh:
    """Create a cropped Poisson mesh and transfer nearest point colors."""
    if len(cloud.points) < 50:
        raise ValueError('at least 50 colored points are required for a mesh')
    mesh_cloud = cloud.voxel_down_sample(max(voxel_size / 2.0, 0.01))
    search_radius = max(voxel_size * 4.0, 0.1)
    mesh_cloud.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=search_radius,
            max_nn=40,
        )
    )
    mesh_cloud.orient_normals_towards_camera_location(camera_origin)
    mesh, _densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        mesh_cloud,
        depth=depth,
        scale=1.05,
    )
    bounds = mesh_cloud.get_axis_aligned_bounding_box()
    mesh = mesh.crop(bounds)
    if len(mesh.vertices) == 0 or len(mesh.triangles) == 0:
        raise RuntimeError('mesh reconstruction produced no triangles')

    tree = o3d.geometry.KDTreeFlann(mesh_cloud)
    source_colors = np.asarray(mesh_cloud.colors)
    vertex_colors = []
    for vertex in np.asarray(mesh.vertices):
        _count, indices, _distance = tree.search_knn_vector_3d(vertex, 1)
        vertex_colors.append(source_colors[indices[0]])
    mesh.vertex_colors = o3d.utility.Vector3dVector(
        np.asarray(vertex_colors)
    )
    mesh.compute_vertex_normals()
    return mesh


def _product_record(path: Path, **details) -> Dict:
    return {
        'file': path.name,
        'sha256': file_sha256(path),
        **details,
    }


def run_pipeline(
    points_path: Path,
    image_path: Path,
    calibration_path: Path,
    metadata_path: Path,
    output_dir: Path,
    voxel_size: float = 0.10,
    mesh_depth: int = 7,
    max_time_delta_ms: float = 50.0,
) -> Dict:
    """Generate linked colored-cloud, voxel, mesh, and manifest products."""
    points_path = Path(points_path)
    image_path = Path(image_path)
    calibration_path = Path(calibration_path)
    metadata_path = Path(metadata_path)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    metadata = load_json(metadata_path)
    calibration = load_json(calibration_path)
    time_delta_ms = validate_timestamps(metadata, max_time_delta_ms)
    intrinsic = np.asarray(
        calibration['camera_intrinsic'],
        dtype=np.float64,
    )
    camera_from_lidar = np.asarray(
        calibration['camera_from_lidar'],
        dtype=np.float64,
    )
    source_cloud = o3d.io.read_point_cloud(str(points_path))
    source_points = np.asarray(source_cloud.points)
    if len(source_points) == 0:
        raise ValueError(f'input point cloud is empty: {points_path}')
    image_rgb = np.asarray(o3d.io.read_image(str(image_path)))

    colored_points, colors = colorize_visible_points(
        source_points,
        image_rgb,
        intrinsic,
        camera_from_lidar,
        calibration.get('camera_model', 'PINHOLE'),
        calibration.get('distortion_coefficients', []),
    )
    colored_cloud = make_point_cloud(colored_points, colors)
    voxel_cloud = colored_cloud.voxel_down_sample(voxel_size)
    sensor_origin = np.asarray(
        metadata.get('sensor_origin_xyz', [0.0, 0.0, 0.0]),
        dtype=np.float64,
    )
    voxel_map = build_sparse_voxel_map(
        np.asarray(voxel_cloud.points),
        voxel_size,
        sensor_origin,
    )
    mesh = reconstruct_mesh(
        colored_cloud,
        sensor_origin,
        voxel_size,
        mesh_depth,
    )

    colored_path = output_dir / 'colored_cloud.ply'
    voxel_cloud_path = output_dir / 'voxel_cloud.ply'
    voxel_map_path = output_dir / 'voxel_map.npz'
    mesh_path = output_dir / 'scene_mesh.ply'
    if not o3d.io.write_point_cloud(str(colored_path), colored_cloud):
        raise RuntimeError('failed to write colored point cloud')
    if not o3d.io.write_point_cloud(str(voxel_cloud_path), voxel_cloud):
        raise RuntimeError('failed to write voxel point cloud')
    np.savez_compressed(voxel_map_path, **voxel_map)
    if not o3d.io.write_triangle_mesh(str(mesh_path), mesh):
        raise RuntimeError('failed to write scene mesh')

    manifest = {
        'schema_version': 1,
        'algorithm_version': ALGORITHM_VERSION,
        'mission_id': metadata['mission_id'],
        'scene_id': metadata['scene_id'],
        'source_id': metadata['source_id'],
        'lidar_timestamp_ns': int(metadata['lidar_timestamp_ns']),
        'image_timestamp_ns': int(metadata['image_timestamp_ns']),
        'time_delta_ms': time_delta_ms,
        'pose': metadata['pose'],
        'coordinate_frame': metadata['coordinate_frame'],
        'inputs': {
            'raw_point_cloud': _product_record(points_path),
            'camera_image': _product_record(image_path),
            'calibration': _product_record(calibration_path),
            'metadata': _product_record(metadata_path),
        },
        'products': {
            'colored_cloud': _product_record(
                colored_path,
                point_count=len(colored_cloud.points),
            ),
            'voxel_cloud': _product_record(
                voxel_cloud_path,
                point_count=len(voxel_cloud.points),
                voxel_size_m=voxel_size,
            ),
            'voxel_map': _product_record(
                voxel_map_path,
                occupied_count=len(voxel_map['occupied_indices']),
                free_count=len(voxel_map['free_indices']),
                unknown_count=int(voxel_map['unknown_count'][0]),
                unknown_representation='implicit within bounded grid',
            ),
            'mesh': _product_record(
                mesh_path,
                vertex_count=len(mesh.vertices),
                triangle_count=len(mesh.triangles),
            ),
        },
    }
    manifest_path = output_dir / 'manifest.json'
    with manifest_path.open('w', encoding='utf-8') as stream:
        json.dump(manifest, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    return manifest

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

"""Generate deterministic camera and LiDAR inputs for offline testing."""

import argparse
import json
from pathlib import Path
from typing import Dict

import numpy as np
import open3d as o3d

from scene_model_pipeline.core import make_point_cloud


def write_json(path: Path, value) -> None:
    """Write stable, human-readable JSON."""
    with path.open('w', encoding='utf-8') as stream:
        json.dump(value, stream, indent=2)
        stream.write('\n')


def generate_synthetic_scene(output_dir: Path) -> Dict[str, Path]:
    """Create a curved colored surface plus hidden LiDAR returns."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    width, height = 320, 240
    fx = fy = 220.0
    cx, cy = width / 2.0, height / 2.0

    u_grid, v_grid = np.meshgrid(
        np.arange(12, width - 12, 4),
        np.arange(12, height - 12, 4),
    )
    depth = (
        3.0
        + 0.18 * np.sin(u_grid / 30.0)
        + 0.12 * np.cos(v_grid / 24.0)
    )
    x = (u_grid - cx) * depth / fx
    y = (v_grid - cy) * depth / fy
    front = np.column_stack([x.ravel(), y.ravel(), depth.ravel()])
    hidden_depth = depth + 0.8
    hidden = np.column_stack([
        ((u_grid - cx) * hidden_depth / fx).ravel(),
        ((v_grid - cy) * hidden_depth / fy).ravel(),
        hidden_depth.ravel(),
    ])
    raw_points = np.vstack([front, hidden])
    raw_colors = np.zeros_like(raw_points)

    red = np.broadcast_to(
        np.linspace(20, 240, width, dtype=np.uint8),
        (height, width),
    )
    green = np.broadcast_to(
        np.linspace(30, 220, height, dtype=np.uint8)[:, None],
        (height, width),
    )
    checker = (
        (np.indices((height, width)).sum(axis=0) // 24) % 2 * 160 + 60
    ).astype(np.uint8)
    image = np.dstack([red, green, checker])

    points_path = output_dir / 'raw_points.ply'
    image_path = output_dir / 'camera.png'
    calibration_path = output_dir / 'calibration.json'
    metadata_path = output_dir / 'metadata.json'
    o3d.io.write_point_cloud(
        str(points_path),
        make_point_cloud(raw_points, raw_colors),
    )
    o3d.io.write_image(str(image_path), o3d.geometry.Image(image))
    write_json(
        calibration_path,
        {
            'camera_intrinsic': [
                [fx, 0.0, cx],
                [0.0, fy, cy],
                [0.0, 0.0, 1.0],
            ],
            'camera_from_lidar': np.eye(4).tolist(),
        },
    )
    write_json(
        metadata_path,
        {
            'mission_id': 'mock_mission_001',
            'scene_id': 'mock_scene_001',
            'source_id': 'synthetic_lidar_camera',
            'lidar_timestamp_ns': 1_000_000_000,
            'image_timestamp_ns': 1_010_000_000,
            'coordinate_frame': 'lidar',
            'sensor_origin_xyz': [0.0, 0.0, 0.0],
            'pose': {
                'frame_id': 'map',
                'child_frame_id': 'lidar',
                'translation_xyz': [0.0, 0.0, 0.0],
                'quaternion_xyzw': [0.0, 0.0, 0.0, 1.0],
            },
        },
    )
    return {
        'points': points_path,
        'image': image_path,
        'calibration': calibration_path,
        'metadata': metadata_path,
    }


def main(args=None) -> None:
    """Generate a reusable synthetic input directory."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parsed = parser.parse_args(args)
    paths = generate_synthetic_scene(parsed.output)
    print('SYNTHETIC_SCENE=PASS')
    print(json.dumps({key: str(value) for key, value in paths.items()}))

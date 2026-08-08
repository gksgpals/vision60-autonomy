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

import json

import numpy as np
import open3d as o3d
import pytest

from scene_model_pipeline.core import (
    colorize_visible_points,
    file_sha256,
    run_pipeline,
    validate_timestamps,
)
from scene_model_pipeline.synthetic import generate_synthetic_scene


def test_pipeline_builds_linked_colored_voxel_and_mesh_products(tmp_path):
    inputs = generate_synthetic_scene(tmp_path / 'inputs')
    original_hash = file_sha256(inputs['points'])
    output_dir = tmp_path / 'products'

    manifest = run_pipeline(
        inputs['points'],
        inputs['image'],
        inputs['calibration'],
        inputs['metadata'],
        output_dir,
        voxel_size=0.10,
        mesh_depth=6,
    )

    assert file_sha256(inputs['points']) == original_hash
    assert manifest['mission_id'] == 'mock_mission_001'
    assert manifest['inputs']['raw_point_cloud']['sha256'] == original_hash
    assert manifest['products']['colored_cloud']['point_count'] > 1000
    assert manifest['products']['voxel_map']['occupied_count'] > 0
    assert manifest['products']['voxel_map']['free_count'] > 0
    assert manifest['products']['voxel_map']['unknown_count'] > 0
    assert manifest['products']['mesh']['triangle_count'] > 0

    colored = o3d.io.read_point_cloud(
        str(output_dir / 'colored_cloud.ply')
    )
    colors = np.asarray(colored.colors)
    assert colors.ptp(axis=0).max() > 0.25
    voxel_data = np.load(output_dir / 'voxel_map.npz')
    assert len(voxel_data['occupied_indices']) > 0
    assert len(voxel_data['free_indices']) > 0
    with (output_dir / 'manifest.json').open(encoding='utf-8') as stream:
        saved_manifest = json.load(stream)
    assert saved_manifest == manifest


def test_timestamp_mismatch_is_rejected():
    with pytest.raises(ValueError, match='time delta'):
        validate_timestamps(
            {
                'lidar_timestamp_ns': 0,
                'image_timestamp_ns': 100_000_000,
            },
            max_delta_ms=50.0,
        )


def test_fisheye_projection_colors_visible_points():
    points = np.asarray([
        [0.0, 0.0, 2.0],
        [0.4, 0.0, 2.0],
        [-0.4, 0.2, 2.0],
    ])
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    image[:, :, 0] = np.arange(160, dtype=np.uint8)[None, :]
    image[:, :, 1] = np.arange(120, dtype=np.uint8)[:, None]
    colored, colors = colorize_visible_points(
        points,
        image,
        np.asarray([
            [90.0, 0.0, 80.0],
            [0.0, 90.0, 60.0],
            [0.0, 0.0, 1.0],
        ]),
        np.eye(4),
        camera_model='OPENCV_FISHEYE',
        distortion_coefficients=[-0.01, 0.001, 0.0, 0.0],
    )
    assert colored.shape == (3, 3)
    assert colors.shape == (3, 3)
    assert colors.ptp(axis=0).max() > 0.05

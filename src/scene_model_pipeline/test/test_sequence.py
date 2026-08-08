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

import numpy as np
import open3d as o3d
import pytest

from scene_model_pipeline.bag_sequence import (
    quaternion_rotation,
    read_synchronized_frames,
)
from scene_model_pipeline.sequence import build_cumulative_scene
from scene_model_pipeline.synthetic_bag import write_sequence_inputs


def test_mcap_roundtrip_builds_pose_aligned_cumulative_scene(tmp_path):
    input_dir = tmp_path / 'input'
    bag_path = write_sequence_inputs(input_dir, frame_count=4)
    output_dir = tmp_path / 'products'

    manifest = build_cumulative_scene(
        bag_path,
        input_dir / 'calibration.json',
        input_dir / 'metadata.json',
        output_dir,
        voxel_size=0.12,
        mesh_depth=6,
        max_frames=4,
    )

    assert manifest['frame_count'] == 4
    assert manifest['usable_frame_count'] == 4
    assert manifest['rejected_frame_count'] == 0
    assert manifest['bag']['storage_id'] == 'mcap'
    assert manifest['bag']['sha256']
    assert manifest['products']['colored_cloud']['point_count'] > 8000
    assert manifest['products']['voxel_map']['occupied_count'] > 0
    assert manifest['products']['voxel_map']['free_count'] > 0
    assert manifest['products']['mesh']['triangle_count'] > 0
    x_positions = [
        frame['map_from_lidar'][0][3] for frame in manifest['frames']
    ]
    assert x_positions == pytest.approx([0.0, 0.2, 0.4, 0.6])

    cloud = o3d.io.read_point_cloud(
        str(output_dir / 'cumulative_colored_cloud.ply')
    )
    extent = cloud.get_axis_aligned_bounding_box().get_extent()
    assert 3.0 < extent[0] < 3.3


def test_mcap_embedded_calibration_builds_same_scene(tmp_path):
    input_dir = tmp_path / 'embedded_input'
    bag_path = write_sequence_inputs(input_dir, frame_count=3)
    output_dir = tmp_path / 'embedded_products'

    manifest = build_cumulative_scene(
        bag_path,
        None,
        input_dir / 'metadata.json',
        output_dir,
        voxel_size=0.12,
        mesh_depth=6,
        max_frames=3,
    )

    assert manifest['usable_frame_count'] == 3
    assert manifest['calibration']['source'] == 'bag_topics'
    assert manifest['calibration']['camera_info_topic'] \
        == '/camera/camera_info'
    assert manifest['calibration']['tf_static_topic'] == '/tf_static'
    assert manifest['calibration']['frames']['lidar_frame'] == 'lidar'
    assert manifest['calibration']['frames']['camera_frame'] == 'camera'


def test_sequence_rejects_frames_outside_sync_tolerance(tmp_path):
    input_dir = tmp_path / 'bad_input'
    bag_path = write_sequence_inputs(
        input_dir,
        frame_count=2,
        image_offset_ms=200.0,
    )
    with pytest.raises(ValueError, match='no synchronized'):
        build_cumulative_scene(
            bag_path,
            input_dir / 'calibration.json',
            input_dir / 'metadata.json',
            tmp_path / 'bad_products',
            max_frames=2,
            image_tolerance_ms=50.0,
        )


def test_sequence_can_sample_frames_across_a_longer_timeline(tmp_path):
    input_dir = tmp_path / 'sampled_input'
    bag_path = write_sequence_inputs(input_dir, frame_count=4)
    frames = read_synchronized_frames(
        bag_path,
        max_frames=4,
        min_lidar_interval_ms=1500.0,
    )
    assert [frame.lidar_stamp_ns for frame in frames] == [
        1_000_000_000,
        3_000_000_000,
    ]


def test_quaternion_rotation_preserves_vector_length():
    rotation = quaternion_rotation(0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5))
    rotated = rotation @ np.asarray([1.0, 0.0, 0.0])
    assert rotated == pytest.approx([0.0, 1.0, 0.0])
    assert np.linalg.det(rotation) == pytest.approx(1.0)

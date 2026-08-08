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

import cv2
import numpy as np
import open3d as o3d
import pytest
import rosbag2_py
from rclpy.serialization import deserialize_message
from visualization_msgs.msg import Marker, MarkerArray

from scene_model_pipeline.overlay import (
    build_command_view,
    build_markers,
    validate_overlay,
)


def _write_json(path, value):
    with path.open('w', encoding='utf-8') as stream:
        json.dump(value, stream)


def _scene_and_overlay(tmp_path):
    scene_dir = tmp_path / 'scene'
    scene_dir.mkdir()
    mesh = o3d.geometry.TriangleMesh.create_box(1.2, 0.8, 0.5)
    mesh.paint_uniform_color([0.25, 0.55, 0.80])
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(np.asarray(mesh.vertices))
    cloud.colors = o3d.utility.Vector3dVector(
        np.tile([0.25, 0.55, 0.80], (len(mesh.vertices), 1))
    )
    o3d.io.write_triangle_mesh(str(scene_dir / 'mesh.ply'), mesh)
    o3d.io.write_point_cloud(str(scene_dir / 'cloud.ply'), cloud)
    np.savez_compressed(
        scene_dir / 'voxel_map.npz',
        occupied_indices=np.asarray([[0, 0, 0], [1, 0, 0]]),
        free_indices=np.asarray([[0, 1, 0], [1, 1, 0], [2, 1, 0]]),
        grid_min_index=np.asarray([-1, -1, -1]),
        grid_max_index=np.asarray([3, 2, 2]),
        grid_dimensions=np.asarray([5, 4, 4]),
        voxel_size_m=np.asarray([0.2]),
        unknown_count=np.asarray([75]),
    )
    scene = {
        'mission_id': 'mission-test',
        'scene_id': 'scene-test',
        'coordinate_frame': 'map',
        'products': {
            'colored_cloud': {'file': 'cloud.ply'},
            'mesh': {'file': 'mesh.ply'},
            'voxel_map': {'file': 'voxel_map.npz'},
        },
    }
    _write_json(scene_dir / 'cumulative_manifest.json', scene)
    overlay = {
        'mission_id': 'mission-test',
        'scene_id': 'scene-test',
        'coordinate_frame': 'map',
        'timestamp_ns': 4_000_000_000,
        'route': {
            'points': [[0.0, 0.0, 0.05], [0.8, 0.0, 0.05]],
        },
        'communication_zones': [{
            'id': 'zone-1',
            'center': [0.6, 0.2, 0.05],
            'radius_m': 0.2,
            'classification': 'radio_shadow_candidate',
            'confidence': 0.8,
        }],
        'mission_events': [{
            'id': 'event-1',
            'type': 'victim_candidate',
            'position': [0.2, 0.4, 0.5],
            'label': 'Victim candidate',
            'confidence': 0.9,
        }],
    }
    overlay_path = tmp_path / 'overlay.json'
    _write_json(overlay_path, overlay)
    return scene_dir, overlay_path, scene, overlay


def test_build_command_view_writes_png_and_standard_ros_topics(tmp_path):
    scene_dir, overlay_path, _, _ = _scene_and_overlay(tmp_path)
    output_dir = tmp_path / 'output'
    manifest = build_command_view(
        scene_dir, overlay_path, output_dir, width=640, height=360
    )

    image = cv2.imread(str(output_dir / 'command_view.png'))
    assert image.shape == (360, 640, 3)
    assert image.std() > 10.0
    assert manifest['counts'] == {
        'route_points': 2,
        'recovery_route_points': 0,
        'recovery_waypoints': 0,
        'selected_recovery_waypoints': 0,
        'communication_zones': 1,
        'obstacles': 0,
        'mission_events': 1,
        'ros_markers': 5,
        'overlay_markers': 5,
        'mesh_triangles': 12,
        'voxel_marker_groups': 3,
        'displayed_occupied_voxels': 2,
        'displayed_free_voxels': 3,
    }
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(output_dir / 'command_view_replay'),
            storage_id='mcap',
        ),
        rosbag2_py.ConverterOptions('', ''),
    )
    topics = {
        value.name: value.type for value in reader.get_all_topics_and_types()
    }
    assert topics == {
        '/mission/scene_cloud': 'sensor_msgs/msg/PointCloud2',
        '/mission/scene_markers': 'visualization_msgs/msg/MarkerArray',
        '/mission/scene_mesh': 'visualization_msgs/msg/Marker',
        '/mission/voxel_markers': 'visualization_msgs/msg/MarkerArray',
    }
    decoded = {}
    while reader.has_next():
        topic, serialized, _timestamp = reader.read_next()
        if topic == '/mission/scene_mesh':
            decoded[topic] = deserialize_message(serialized, Marker)
        elif topic in (
            '/mission/scene_markers',
            '/mission/voxel_markers',
        ):
            decoded[topic] = deserialize_message(serialized, MarkerArray)
    assert decoded['/mission/scene_mesh'].type == Marker.TRIANGLE_LIST
    assert len(decoded['/mission/scene_mesh'].points) == 36
    assert len(decoded['/mission/voxel_markers'].markers) == 3
    assert len(decoded['/mission/voxel_markers'].markers[0].points) == 2
    assert len(decoded['/mission/voxel_markers'].markers[1].points) == 3


def test_recovery_route_waypoint_and_obstacle_create_markers(tmp_path):
    _, _, scene, overlay = _scene_and_overlay(tmp_path)
    overlay['recovery_route'] = {
        'points': [[0.8, 0.0, 0.05], [0.3, 0.1, 0.05]],
    }
    overlay['recovery_waypoints'] = [{
        'id': 'crw-1',
        'position': [0.3, 0.1, 0.05],
        'channel': 'backup_wifi',
        'selected_for_recovery': True,
    }]
    overlay['obstacles'] = [{
        'id': 'obstacle-1',
        'position': [0.55, -0.2, 0.25],
        'dimensions_m': [0.4, 0.3, 0.5],
        'classification': 'dynamic_obstacle_candidate',
        'label': 'Dynamic obstacle',
        'confidence': 0.9,
    }]
    validate_overlay(overlay, scene)
    markers = build_markers(overlay, 4_000_000_000)
    namespaces = {marker.ns for marker in markers.markers}
    assert 'recorded_recovery_route' in namespaces
    assert 'communication_recovery_waypoints' in namespaces
    assert 'selected_recovery_waypoints' in namespaces
    assert 'obstacle_candidates' in namespaces


def test_overlay_rejects_wrong_scene_identity(tmp_path):
    _, _, scene, overlay = _scene_and_overlay(tmp_path)
    overlay['scene_id'] = 'different-scene'
    with pytest.raises(ValueError, match='scene_id'):
        validate_overlay(overlay, scene)


def test_overlay_rejects_invalid_confidence(tmp_path):
    _, _, scene, overlay = _scene_and_overlay(tmp_path)
    overlay['mission_events'][0]['confidence'] = 1.2
    with pytest.raises(ValueError, match='confidence'):
        validate_overlay(overlay, scene)

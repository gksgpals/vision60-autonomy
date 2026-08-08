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

"""Create traceable command-view images and ROS visualization messages."""

import json
from pathlib import Path
from typing import Dict, Iterable, Tuple

import cv2
from geometry_msgs.msg import Point
import numpy as np
import open3d as o3d
import rosbag2_py
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from scene_model_pipeline import ALGORITHM_VERSION
from scene_model_pipeline.core import file_sha256, load_json
from scene_model_pipeline.sequence import path_sha256
from scene_model_pipeline.synthetic_bag import ros_time, topic_metadata


EVENT_COLORS = {
    'victim_candidate': (0.95, 0.15, 0.15),
    'hazard_candidate': (1.0, 0.65, 0.0),
    'hazardous_material_candidate': (1.0, 0.65, 0.0),
    'traversable_route': (0.0, 0.85, 1.0),
}
RECOVERY_ROUTE_COLOR = (1.0, 0.58, 0.05)
RECOVERY_WAYPOINT_COLOR = (0.15, 1.0, 0.35)
OBSTACLE_COLOR = (1.0, 0.15, 0.10)
ZONE_COLORS = {
    'radio_shadow_candidate': (0.85, 0.10, 0.15),
    'channel_anomaly_candidate': (1.0, 0.45, 0.0),
    'total_link_failure_candidate': (0.70, 0.0, 0.85),
    'transient_network_instability': (1.0, 0.85, 0.0),
}


def _points(values: Iterable, field_name: str) -> np.ndarray:
    """Validate and return a nonempty collection of XYZ points."""
    result = np.asarray(list(values), dtype=np.float64)
    if result.ndim != 2 or result.shape[1] != 3 or len(result) == 0:
        raise ValueError(f'{field_name} must contain XYZ points')
    if not np.isfinite(result).all():
        raise ValueError(f'{field_name} contains non-finite coordinates')
    return result


def validate_overlay(overlay: Dict, scene_manifest: Dict) -> Dict:
    """Check mission identity, coordinate frame, and overlay values."""
    for key in ('mission_id', 'scene_id', 'coordinate_frame', 'route'):
        if key not in overlay:
            raise ValueError(f'overlay is missing {key}')
    for key in ('mission_id', 'scene_id', 'coordinate_frame'):
        if overlay[key] != scene_manifest[key]:
            raise ValueError(f'overlay {key} does not match scene manifest')
    _points(overlay['route'].get('points', []), 'route.points')
    recovery_points = overlay.get('recovery_route', {}).get('points', [])
    if recovery_points:
        _points(recovery_points, 'recovery_route.points')
    for waypoint in overlay.get('recovery_waypoints', []):
        _points([waypoint.get('position', [])], 'recovery waypoint')
        if not waypoint.get('id'):
            raise ValueError('recovery waypoint id is missing')
    for obstacle in overlay.get('obstacles', []):
        _points([obstacle.get('position', [])], 'obstacle position')
        dimensions = np.asarray(
            obstacle.get('dimensions_m', []), dtype=np.float64
        )
        if dimensions.shape != (3,) or np.any(dimensions <= 0.0):
            raise ValueError('obstacle dimensions_m must be positive XYZ')
        confidence = float(obstacle.get('confidence', -1.0))
        if not 0.0 <= confidence <= 1.0:
            raise ValueError('obstacle confidence is invalid')
    for zone in overlay.get('communication_zones', []):
        _points([zone.get('center', [])], 'communication zone center')
        if float(zone.get('radius_m', 0.0)) <= 0.0:
            raise ValueError('communication zone radius_m must be positive')
        confidence = float(zone.get('confidence', -1.0))
        if not 0.0 <= confidence <= 1.0:
            raise ValueError('communication zone confidence is invalid')
    for event in overlay.get('mission_events', []):
        _points([event.get('position', [])], 'mission event position')
        confidence = float(event.get('confidence', -1.0))
        if not 0.0 <= confidence <= 1.0:
            raise ValueError('mission event confidence is invalid')
    return overlay


def _color(message, rgb: Tuple[float, float, float], alpha: float) -> None:
    message.r, message.g, message.b = rgb
    message.a = alpha


def _point_message(xyz) -> Point:
    point = Point()
    point.x, point.y, point.z = map(float, xyz)
    return point


def build_markers(overlay: Dict, timestamp_ns: int) -> MarkerArray:
    """Convert overlay data to standard ROS visualization markers."""
    frame_id = overlay['coordinate_frame']
    stamp = ros_time(timestamp_ns)
    markers = MarkerArray()

    route = Marker()
    route.header.frame_id = frame_id
    route.header.stamp = stamp
    route.ns = 'actual_traversed_route'
    route.id = 0
    route.type = Marker.LINE_STRIP
    route.action = Marker.ADD
    route.pose.orientation.w = 1.0
    route.scale.x = 0.07
    _color(route.color, (0.0, 0.85, 1.0), 1.0)
    route.points = [
        _point_message(value) for value in overlay['route']['points']
    ]
    markers.markers.append(route)

    recovery_points = overlay.get('recovery_route', {}).get('points', [])
    if len(recovery_points) >= 2:
        recovery_route = Marker()
        recovery_route.header.frame_id = frame_id
        recovery_route.header.stamp = stamp
        recovery_route.ns = 'recorded_recovery_route'
        recovery_route.id = 10
        recovery_route.type = Marker.LINE_STRIP
        recovery_route.action = Marker.ADD
        recovery_route.pose.orientation.w = 1.0
        recovery_route.scale.x = 0.09
        _color(recovery_route.color, RECOVERY_ROUTE_COLOR, 1.0)
        recovery_route.points = [
            _point_message(value) for value in recovery_points
        ]
        markers.markers.append(recovery_route)

    recovery_waypoints = overlay.get('recovery_waypoints', [])
    if recovery_waypoints:
        waypoint_cloud = Marker()
        waypoint_cloud.header.frame_id = frame_id
        waypoint_cloud.header.stamp = stamp
        waypoint_cloud.ns = 'communication_recovery_waypoints'
        waypoint_cloud.id = 20
        waypoint_cloud.type = Marker.SPHERE_LIST
        waypoint_cloud.action = Marker.ADD
        waypoint_cloud.pose.orientation.w = 1.0
        waypoint_cloud.scale.x = 0.10
        waypoint_cloud.scale.y = 0.10
        waypoint_cloud.scale.z = 0.10
        _color(
            waypoint_cloud.color, RECOVERY_WAYPOINT_COLOR, 0.75
        )
        waypoint_cloud.points = [
            _point_message(value['position'])
            for value in recovery_waypoints
        ]
        markers.markers.append(waypoint_cloud)
        selected_id = 30
        for waypoint in recovery_waypoints:
            if not waypoint.get('selected_for_recovery', False):
                continue
            selected = Marker()
            selected.header = waypoint_cloud.header
            selected.ns = 'selected_recovery_waypoints'
            selected.id = selected_id
            selected.type = Marker.SPHERE
            selected.action = Marker.ADD
            selected.pose.position = _point_message(waypoint['position'])
            selected.pose.orientation.w = 1.0
            selected.scale.x = selected.scale.y = selected.scale.z = 0.20
            _color(selected.color, RECOVERY_WAYPOINT_COLOR, 1.0)
            markers.markers.append(selected)

            label = Marker()
            label.header = waypoint_cloud.header
            label.ns = 'selected_recovery_waypoint_labels'
            label.id = selected_id + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = _point_message(waypoint['position'])
            label.pose.position.z += 0.22
            label.pose.orientation.w = 1.0
            label.scale.z = 0.14
            _color(label.color, RECOVERY_WAYPOINT_COLOR, 1.0)
            label.text = (
                f"CRW {waypoint['id']} / {waypoint['channel']}"
            )
            markers.markers.append(label)
            selected_id += 2

    marker_id = 100
    for zone in overlay.get('communication_zones', []):
        rgb = ZONE_COLORS.get(
            zone['classification'], (1.0, 0.4, 0.0)
        )
        area = Marker()
        area.header.frame_id = frame_id
        area.header.stamp = stamp
        area.ns = 'communication_anomaly_zones'
        area.id = marker_id
        area.type = Marker.CYLINDER
        area.action = Marker.ADD
        area.pose.position = _point_message(zone['center'])
        area.pose.orientation.w = 1.0
        diameter = float(zone['radius_m']) * 2.0
        area.scale.x = diameter
        area.scale.y = diameter
        area.scale.z = 0.05
        _color(area.color, rgb, 0.45)
        markers.markers.append(area)

        label = Marker()
        label.header = area.header
        label.ns = 'communication_anomaly_labels'
        label.id = marker_id + 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position = _point_message(zone['center'])
        label.pose.position.z += 0.18
        label.pose.orientation.w = 1.0
        label.scale.z = 0.14
        _color(label.color, rgb, 1.0)
        label.text = (
            f"{zone['classification']} "
            f"({float(zone['confidence']):.2f})"
        )
        markers.markers.append(label)
        marker_id += 2

    marker_id = 600
    for obstacle in overlay.get('obstacles', []):
        symbol = Marker()
        symbol.header.frame_id = frame_id
        symbol.header.stamp = stamp
        symbol.ns = 'obstacle_candidates'
        symbol.id = marker_id
        symbol.type = Marker.CUBE
        symbol.action = Marker.ADD
        symbol.pose.position = _point_message(obstacle['position'])
        symbol.pose.orientation.w = 1.0
        dimensions = obstacle['dimensions_m']
        symbol.scale.x = float(dimensions[0])
        symbol.scale.y = float(dimensions[1])
        symbol.scale.z = float(dimensions[2])
        _color(symbol.color, OBSTACLE_COLOR, 0.65)
        markers.markers.append(symbol)

        label = Marker()
        label.header = symbol.header
        label.ns = 'obstacle_candidate_labels'
        label.id = marker_id + 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position = _point_message(obstacle['position'])
        label.pose.position.z += float(dimensions[2]) * 0.7
        label.pose.orientation.w = 1.0
        label.scale.z = 0.14
        _color(label.color, OBSTACLE_COLOR, 1.0)
        label.text = str(obstacle['label'])
        markers.markers.append(label)
        marker_id += 2

    marker_id = 1000
    for event in overlay.get('mission_events', []):
        rgb = EVENT_COLORS.get(event['type'], (0.2, 1.0, 0.2))
        symbol = Marker()
        symbol.header.frame_id = frame_id
        symbol.header.stamp = stamp
        symbol.ns = 'mission_events'
        symbol.id = marker_id
        symbol.type = Marker.SPHERE
        symbol.action = Marker.ADD
        symbol.pose.position = _point_message(event['position'])
        symbol.pose.orientation.w = 1.0
        symbol.scale.x = symbol.scale.y = symbol.scale.z = 0.18
        _color(symbol.color, rgb, 1.0)
        markers.markers.append(symbol)

        label = Marker()
        label.header = symbol.header
        label.ns = 'mission_event_labels'
        label.id = marker_id + 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position = _point_message(event['position'])
        label.pose.position.z += 0.18
        label.pose.orientation.w = 1.0
        label.scale.z = 0.14
        _color(label.color, rgb, 1.0)
        label.text = (
            f"{event['label']} ({float(event['confidence']):.2f})"
        )
        markers.markers.append(label)
        marker_id += 2
    return markers


def _colored_cloud_message(cloud, frame_id: str, timestamp_ns: int):
    """Convert an Open3D cloud to one colored PointCloud2 message."""
    header = Header()
    header.frame_id = frame_id
    header.stamp = ros_time(timestamp_ns)
    points = np.asarray(cloud.points)
    colors = np.asarray(cloud.colors)
    if len(colors) != len(points):
        colors = np.full((len(points), 3), 0.75)
    colors = np.clip(np.rint(colors * 255.0), 0, 255).astype(np.uint32)
    packed = (
        (colors[:, 0] << 16) | (colors[:, 1] << 8) | colors[:, 2]
    )
    fields = [
        PointField(
            name='x', offset=0, datatype=PointField.FLOAT32, count=1
        ),
        PointField(
            name='y', offset=4, datatype=PointField.FLOAT32, count=1
        ),
        PointField(
            name='z', offset=8, datatype=PointField.FLOAT32, count=1
        ),
        PointField(
            name='rgb', offset=12, datatype=PointField.UINT32, count=1
        ),
    ]
    rows = [
        (float(x), float(y), float(z), int(rgb))
        for (x, y, z), rgb in zip(points, packed)
    ]
    return point_cloud2.create_cloud(header, fields, rows)


def _mesh_marker(mesh, frame_id: str, timestamp_ns: int) -> Marker:
    """Convert an Open3D triangle mesh to a standard ROS marker."""
    vertices = np.asarray(mesh.vertices)
    triangles = np.asarray(mesh.triangles)
    if len(vertices) == 0 or len(triangles) == 0:
        raise ValueError('scene mesh contains no triangles')
    vertex_colors = np.asarray(mesh.vertex_colors)
    if len(vertex_colors) != len(vertices):
        vertex_colors = np.full((len(vertices), 3), 0.68)

    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = ros_time(timestamp_ns)
    marker.ns = 'scene_mesh'
    marker.id = 0
    marker.type = Marker.TRIANGLE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0
    marker.color.a = 1.0
    for triangle in triangles:
        for vertex_index in triangle:
            marker.points.append(_point_message(vertices[vertex_index]))
            rgb = vertex_colors[vertex_index]
            marker.colors.append(ColorRGBA(
                r=float(rgb[0]),
                g=float(rgb[1]),
                b=float(rgb[2]),
                a=0.82,
            ))
    return marker


def _voxel_cube_marker(
    indices: np.ndarray,
    voxel_size: float,
    frame_id: str,
    timestamp_ns: int,
    namespace: str,
    marker_id: int,
    rgb: Tuple[float, float, float],
    alpha: float,
    scale_ratio: float,
) -> Marker:
    """Create a cube-list marker for a collection of voxel indices."""
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = ros_time(timestamp_ns)
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.CUBE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = marker.scale.y = marker.scale.z = (
        voxel_size * scale_ratio
    )
    _color(marker.color, rgb, alpha)
    centers = (indices.astype(np.float64) + 0.5) * voxel_size
    marker.points = [_point_message(value) for value in centers]
    return marker


def _voxel_bounds_marker(
    lower: np.ndarray,
    upper: np.ndarray,
    voxel_size: float,
    frame_id: str,
    timestamp_ns: int,
) -> Marker:
    """Show the observed-grid bounds without materializing unknown cells."""
    minimum = lower.astype(np.float64) * voxel_size
    maximum = (upper.astype(np.float64) + 1.0) * voxel_size
    corners = np.asarray([
        [minimum[0], minimum[1], minimum[2]],
        [maximum[0], minimum[1], minimum[2]],
        [maximum[0], maximum[1], minimum[2]],
        [minimum[0], maximum[1], minimum[2]],
        [minimum[0], minimum[1], maximum[2]],
        [maximum[0], minimum[1], maximum[2]],
        [maximum[0], maximum[1], maximum[2]],
        [minimum[0], maximum[1], maximum[2]],
    ])
    edge_indices = (
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    )
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = ros_time(timestamp_ns)
    marker.ns = 'voxel_observation_bounds'
    marker.id = 2
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = max(voxel_size * 0.08, 0.01)
    _color(marker.color, (0.72, 0.72, 0.78), 0.8)
    marker.points = [
        _point_message(corners[index])
        for edge in edge_indices
        for index in edge
    ]
    return marker


def build_voxel_markers(
    voxel_map: Dict,
    frame_id: str,
    timestamp_ns: int,
    max_free_voxels: int = 20000,
) -> MarkerArray:
    """Visualize occupied/free voxels and the implicit unknown bounds."""
    occupied = np.asarray(
        voxel_map['occupied_indices'], dtype=np.int32
    ).reshape(-1, 3)
    free = np.asarray(
        voxel_map['free_indices'], dtype=np.int32
    ).reshape(-1, 3)
    voxel_size = float(np.asarray(voxel_map['voxel_size_m']).reshape(-1)[0])
    lower = np.asarray(voxel_map['grid_min_index'], dtype=np.int32)
    upper = np.asarray(voxel_map['grid_max_index'], dtype=np.int32)
    if voxel_size <= 0.0:
        raise ValueError('voxel map has an invalid voxel size')
    if max_free_voxels <= 0:
        raise ValueError('max_free_voxels must be positive')
    if len(free) > max_free_voxels:
        stride = int(np.ceil(len(free) / max_free_voxels))
        free = free[::stride]

    markers = MarkerArray()
    markers.markers.append(_voxel_cube_marker(
        occupied,
        voxel_size,
        frame_id,
        timestamp_ns,
        'voxel_occupied',
        0,
        (1.0, 0.24, 0.08),
        0.9,
        0.88,
    ))
    markers.markers.append(_voxel_cube_marker(
        free,
        voxel_size,
        frame_id,
        timestamp_ns,
        'voxel_free',
        1,
        (0.15, 0.72, 1.0),
        0.16,
        0.28,
    ))
    markers.markers.append(_voxel_bounds_marker(
        lower, upper, voxel_size, frame_id, timestamp_ns
    ))
    return markers


def write_visualization_bag(
    output_path: Path,
    cloud,
    mesh,
    voxel_map: Dict,
    overlay: Dict,
    timestamp_ns: int,
) -> Dict[str, int]:
    """Write point, voxel, mesh, and overlay visualization messages."""
    output_path = Path(output_path)
    if output_path.exists():
        raise FileExistsError(f'output bag already exists: {output_path}')
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(
            uri=str(output_path), storage_id='mcap'
        ),
        rosbag2_py.ConverterOptions('', ''),
    )
    writer.create_topic(topic_metadata(
        '/mission/scene_cloud', 'sensor_msgs/msg/PointCloud2'
    ))
    writer.create_topic(topic_metadata(
        '/mission/scene_markers', 'visualization_msgs/msg/MarkerArray'
    ))
    writer.create_topic(topic_metadata(
        '/mission/scene_mesh', 'visualization_msgs/msg/Marker'
    ))
    writer.create_topic(topic_metadata(
        '/mission/voxel_markers', 'visualization_msgs/msg/MarkerArray'
    ))
    cloud_message = _colored_cloud_message(
        cloud, overlay['coordinate_frame'], timestamp_ns
    )
    markers = build_markers(overlay, timestamp_ns)
    mesh_marker = _mesh_marker(
        mesh, overlay['coordinate_frame'], timestamp_ns
    )
    voxel_markers = build_voxel_markers(
        voxel_map, overlay['coordinate_frame'], timestamp_ns
    )
    writer.write(
        '/mission/scene_cloud',
        serialize_message(cloud_message),
        timestamp_ns,
    )
    writer.write(
        '/mission/scene_markers',
        serialize_message(markers),
        timestamp_ns,
    )
    writer.write(
        '/mission/scene_mesh',
        serialize_message(mesh_marker),
        timestamp_ns,
    )
    writer.write(
        '/mission/voxel_markers',
        serialize_message(voxel_markers),
        timestamp_ns,
    )
    return {
        'overlay_markers': len(markers.markers),
        'mesh_triangles': len(mesh_marker.points) // 3,
        'voxel_marker_groups': len(voxel_markers.markers),
        'displayed_occupied_voxels': len(
            voxel_markers.markers[0].points
        ),
        'displayed_free_voxels': len(voxel_markers.markers[1].points),
    }


def _camera_projection(points: np.ndarray, width: int, height: int):
    """Project world points into a stable isometric operator view."""
    center = (points.min(axis=0) + points.max(axis=0)) * 0.5
    extent = np.maximum(points.max(axis=0) - points.min(axis=0), 0.1)
    radius = float(np.linalg.norm(extent))
    eye_direction = np.asarray([1.35, -1.55, 1.10])
    eye_direction /= np.linalg.norm(eye_direction)
    eye = center + eye_direction * radius * 2.5
    forward = center - eye
    forward /= np.linalg.norm(forward)
    up_hint = np.asarray([0.0, 0.0, 1.0])
    if abs(float(np.dot(forward, up_hint))) > 0.95:
        up_hint = np.asarray([0.0, 1.0, 0.0])
    right = np.cross(forward, up_hint)
    right /= np.linalg.norm(right)
    camera_up = np.cross(right, forward)
    relative = points - eye
    camera = np.column_stack([
        relative @ right,
        relative @ camera_up,
        relative @ forward,
    ])
    span_x = max(float(np.ptp(camera[:, 0])), 0.1)
    span_y = max(float(np.ptp(camera[:, 1])), 0.1)
    scale = min(width * 0.72 / span_x, height * 0.72 / span_y)
    pixels = np.column_stack([
        width * 0.5 + camera[:, 0] * scale,
        height * 0.52 - camera[:, 1] * scale,
    ])
    return pixels, camera[:, 2], (eye, right, camera_up, forward, scale)


def _project_extra(points: np.ndarray, camera, width: int, height: int):
    eye, right, camera_up, forward, scale = camera
    relative = points - eye
    return np.column_stack([
        width * 0.5 + (relative @ right) * scale,
        height * 0.52 - (relative @ camera_up) * scale,
        relative @ forward,
    ])


def _overlay_framing_points(vertices: np.ndarray, overlay: Dict):
    """Return geometry and overlay points used by every operator render."""
    route_points = _points(overlay['route']['points'], 'route.points')
    extras = [route_points]
    recovery_points = overlay.get('recovery_route', {}).get('points', [])
    if recovery_points:
        extras.append(_points(recovery_points, 'recovery_route.points'))
    extras.extend(
        _points([waypoint['position']], 'recovery waypoint')
        for waypoint in overlay.get('recovery_waypoints', [])
    )
    extras.extend(
        _points([obstacle['position']], 'obstacle position')
        for obstacle in overlay.get('obstacles', [])
    )
    extras.extend(
        _points([zone['center']], 'communication zone center')
        for zone in overlay.get('communication_zones', [])
    )
    extras.extend(
        _points([event['position']], 'mission event position')
        for event in overlay.get('mission_events', [])
    )
    return np.vstack([vertices, *extras])


def render_command_view(
    mesh,
    cloud,
    overlay: Dict,
    output_path: Path,
    width: int = 1280,
    height: int = 720,
) -> None:
    """Render mesh and mission overlays with a deterministic CPU path."""
    vertices = np.asarray(mesh.vertices)
    triangles = np.asarray(mesh.triangles)
    if len(vertices) == 0:
        vertices = np.asarray(cloud.points)
        triangles = np.empty((0, 3), dtype=np.int32)
    route_points = _points(overlay['route']['points'], 'route.points')
    recovery_points = overlay.get('recovery_route', {}).get('points', [])
    framing_points = _overlay_framing_points(vertices, overlay)
    _, _, camera = _camera_projection(framing_points, width, height)
    projected = _project_extra(vertices, camera, width, height)
    image = np.full((height, width, 3), (25, 20, 18), dtype=np.uint8)

    vertex_colors = np.asarray(mesh.vertex_colors)
    if len(vertex_colors) != len(vertices):
        vertex_colors = np.full((len(vertices), 3), 0.68)
    if len(triangles):
        means = projected[triangles, 2].mean(axis=1)
        light = np.asarray([0.25, -0.35, 0.90])
        light /= np.linalg.norm(light)
        for index in np.argsort(means)[::-1]:
            triangle = triangles[index]
            xyz = vertices[triangle]
            normal = np.cross(xyz[1] - xyz[0], xyz[2] - xyz[0])
            length = np.linalg.norm(normal)
            shade = 0.75
            if length > 1e-12:
                shade = 0.35 + 0.65 * abs(float(normal @ light) / length)
            rgb = np.clip(
                vertex_colors[triangle].mean(axis=0) * shade * 255.0,
                0,
                255,
            ).astype(np.uint8)
            polygon = np.rint(projected[triangle, :2]).astype(np.int32)
            cv2.fillConvexPoly(
                image,
                polygon,
                tuple(int(value) for value in rgb[::-1]),
                lineType=cv2.LINE_AA,
            )
    else:
        sample = projected[::max(1, len(projected) // 8000)]
        for x, y, depth in sample:
            if depth > 0:
                cv2.circle(image, (round(x), round(y)), 1, (180, 180, 180))

    route_pixels = _project_extra(
        route_points, camera, width, height
    )[:, :2]
    cv2.polylines(
        image,
        [np.rint(route_pixels).astype(np.int32)],
        False,
        (255, 220, 0),
        6,
        cv2.LINE_AA,
    )
    for x, y in route_pixels:
        cv2.circle(image, (round(x), round(y)), 6, (255, 220, 0), -1)

    if recovery_points:
        recovery_pixels = _project_extra(
            np.asarray(recovery_points, dtype=np.float64),
            camera,
            width,
            height,
        )[:, :2]
        recovery_bgr = tuple(
            int(value * 255) for value in RECOVERY_ROUTE_COLOR[::-1]
        )
        cv2.polylines(
            image,
            [np.rint(recovery_pixels).astype(np.int32)],
            False,
            recovery_bgr,
            5,
            cv2.LINE_AA,
        )

    waypoint_bgr = tuple(
        int(value * 255) for value in RECOVERY_WAYPOINT_COLOR[::-1]
    )
    for waypoint in overlay.get('recovery_waypoints', []):
        pixel = _project_extra(
            np.asarray([waypoint['position']]), camera, width, height
        )[0]
        center = (round(pixel[0]), round(pixel[1]))
        selected = bool(waypoint.get('selected_for_recovery', False))
        radius = 10 if selected else 4
        cv2.circle(image, center, radius, waypoint_bgr, -1, cv2.LINE_AA)
        if selected:
            cv2.circle(
                image, center, radius + 3, (255, 255, 255),
                2, cv2.LINE_AA,
            )
            cv2.putText(
                image,
                f"CRW / {waypoint['channel']}",
                (center[0] + 14, center[1] + 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.46,
                (245, 245, 245),
                1,
                cv2.LINE_AA,
            )

    for zone in overlay.get('communication_zones', []):
        center = np.asarray(zone['center'], dtype=np.float64)
        radius = float(zone['radius_m'])
        angles = np.linspace(0.0, 2.0 * np.pi, 64, endpoint=False)
        ring = center + np.column_stack([
            np.cos(angles) * radius,
            np.sin(angles) * radius,
            np.zeros_like(angles),
        ])
        polygon = np.rint(_project_extra(
            ring, camera, width, height
        )[:, :2]).astype(np.int32)
        rgb = ZONE_COLORS.get(zone['classification'], (1.0, 0.4, 0.0))
        bgr = tuple(int(value * 255) for value in rgb[::-1])
        layer = image.copy()
        cv2.fillPoly(layer, [polygon], bgr, lineType=cv2.LINE_AA)
        image = cv2.addWeighted(layer, 0.28, image, 0.72, 0.0)
        cv2.polylines(image, [polygon], True, bgr, 3, cv2.LINE_AA)

    obstacle_bgr = tuple(
        int(value * 255) for value in OBSTACLE_COLOR[::-1]
    )
    for obstacle in overlay.get('obstacles', []):
        pixel = _project_extra(
            np.asarray([obstacle['position']]), camera, width, height
        )[0]
        center = (round(pixel[0]), round(pixel[1]))
        cv2.rectangle(
            image,
            (center[0] - 10, center[1] - 10),
            (center[0] + 10, center[1] + 10),
            obstacle_bgr,
            -1,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            str(obstacle['label']),
            (center[0] + 14, center[1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (245, 245, 245),
            1,
            cv2.LINE_AA,
        )

    for event in overlay.get('mission_events', []):
        pixel = _project_extra(
            np.asarray([event['position']]), camera, width, height
        )[0]
        rgb = EVENT_COLORS.get(event['type'], (0.2, 1.0, 0.2))
        bgr = tuple(int(value * 255) for value in rgb[::-1])
        center = (round(pixel[0]), round(pixel[1]))
        cv2.circle(image, center, 11, (255, 255, 255), -1, cv2.LINE_AA)
        cv2.circle(image, center, 8, bgr, -1, cv2.LINE_AA)
        cv2.putText(
            image,
            str(event['label']),
            (center[0] + 14, center[1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (245, 245, 245),
            1,
            cv2.LINE_AA,
        )

    cv2.rectangle(image, (18, 16), (690, 132), (12, 12, 12), -1)
    cv2.putText(
        image,
        f"Mission: {overlay['mission_id']}",
        (34, 48),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (245, 245, 245),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        image,
        'Cyan: actual | Orange: recovery | Green: CRW | Rings: comm',
        (34, 82),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (220, 220, 220),
        1,
        cv2.LINE_AA,
    )
    summary = overlay.get('communication_summary', {})
    cv2.putText(
        image,
        'Final channel: '
        f"{summary.get('final_channel', 'unknown')}  |  "
        f"State: {summary.get('final_state', 'UNKNOWN')}",
        (34, 112),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (150, 240, 180),
        1,
        cv2.LINE_AA,
    )
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output_path), image):
        raise RuntimeError('failed to write command-view image')


def render_recovery_video(
    mesh,
    cloud,
    overlay: Dict,
    background_path: Path,
    output_path: Path,
    width: int = 1280,
    height: int = 720,
    fps: int = 10,
) -> int:
    """Animate the recorded recovery state sequence over the 3D view."""
    vertices = np.asarray(mesh.vertices)
    if len(vertices) == 0:
        vertices = np.asarray(cloud.points)
    _, _, camera = _camera_projection(
        _overlay_framing_points(vertices, overlay), width, height
    )
    route_pixels = _project_extra(
        _points(overlay['route']['points'], 'route.points'),
        camera,
        width,
        height,
    )[:, :2]
    recovery_values = overlay.get('recovery_route', {}).get('points', [])
    recovery_pixels = (
        _project_extra(
            _points(recovery_values, 'recovery_route.points'),
            camera,
            width,
            height,
        )[:, :2]
        if recovery_values else route_pixels[::-1]
    )
    background = cv2.imread(str(background_path))
    if background is None or background.shape[:2] != (height, width):
        raise ValueError('command-view background is missing or invalid')
    writer = cv2.VideoWriter(
        str(output_path), cv2.VideoWriter_fourcc(*'mp4v'),
        float(fps), (width, height)
    )
    if not writer.isOpened():
        raise RuntimeError('failed to create recovery 3D replay video')

    summary = overlay.get('communication_summary', {})
    sequence = summary.get('state_sequence', []) or ['NORMAL']
    final_channel = summary.get('final_channel', 'unknown')
    primary_channel = next(
        (
            waypoint.get('channel')
            for waypoint in overlay.get('recovery_waypoints', [])
            if waypoint.get('channel')
        ),
        'primary_link',
    )
    state_colors = {
        'NORMAL': (80, 220, 120),
        'DEGRADED': (60, 210, 255),
        'STOPPING': (65, 95, 245),
        'RETURNING': (45, 150, 255),
        'CHANNEL_SWITCH': (220, 150, 70),
        'SYNCING': (210, 190, 80),
        'REENTRY_TEST': (220, 100, 220),
        'SAFE_STOP': (65, 95, 245),
    }
    frames = []
    normal_seen = False
    for state in sequence:
        if state == 'NORMAL' and not normal_seen:
            positions = route_pixels
            normal_seen = True
        elif state == 'RETURNING':
            positions = recovery_pixels
        elif state == 'REENTRY_TEST':
            positions = recovery_pixels[::-1]
        elif state in ('DEGRADED', 'STOPPING'):
            positions = route_pixels[-1:]
        elif state in ('CHANNEL_SWITCH', 'SYNCING'):
            positions = recovery_pixels[-1:]
        else:
            positions = recovery_pixels[:1]
        target_frames = 40 if len(positions) > 1 else 12
        indices = np.linspace(
            0, len(positions) - 1, target_frames
        ).astype(np.int32)
        for index in indices:
            frames.append((state, positions[index]))

    switched = False
    for state, position in frames:
        if state in ('CHANNEL_SWITCH', 'SYNCING', 'REENTRY_TEST'):
            switched = True
        channel = final_channel if switched else primary_channel
        link = (
            'LINK LOST'
            if state in ('STOPPING', 'RETURNING', 'CHANNEL_SWITCH')
            else 'LINK CONNECTED'
        )
        frame = background.copy()
        cv2.rectangle(
            frame, (900, 20), (1255, 148), (12, 12, 12), -1
        )
        color = state_colors.get(state, (235, 235, 235))
        cv2.putText(
            frame, 'RECORDED 3D MOCK REPLAY', (920, 50),
            cv2.FONT_HERSHEY_SIMPLEX, 0.52, (240, 240, 240),
            1, cv2.LINE_AA,
        )
        cv2.putText(
            frame, state, (920, 82), cv2.FONT_HERSHEY_SIMPLEX,
            0.64, color, 2, cv2.LINE_AA,
        )
        cv2.putText(
            frame, link, (920, 111), cv2.FONT_HERSHEY_SIMPLEX,
            0.45, color, 1, cv2.LINE_AA,
        )
        cv2.putText(
            frame, channel[:32], (920, 136),
            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (190, 210, 220),
            1, cv2.LINE_AA,
        )
        center = (round(float(position[0])), round(float(position[1])))
        cv2.circle(frame, center, 13, (255, 255, 255), -1, cv2.LINE_AA)
        cv2.circle(frame, center, 9, color, -1, cv2.LINE_AA)
        writer.write(frame)
    writer.release()
    return len(frames)


def build_command_view(
    scene_dir: Path,
    overlay_path: Path,
    output_dir: Path,
    width: int = 1280,
    height: int = 720,
) -> Dict:
    """Build an offline PNG, ROS replay bag, and lineage manifest."""
    scene_dir = Path(scene_dir)
    overlay_path = Path(overlay_path)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    scene_manifest_path = scene_dir / 'cumulative_manifest.json'
    scene_manifest = load_json(scene_manifest_path)
    overlay = validate_overlay(load_json(overlay_path), scene_manifest)
    cloud_path = scene_dir / scene_manifest['products']['colored_cloud']['file']
    mesh_path = scene_dir / scene_manifest['products']['mesh']['file']
    voxel_map_path = (
        scene_dir / scene_manifest['products']['voxel_map']['file']
    )
    cloud = o3d.io.read_point_cloud(str(cloud_path))
    mesh = o3d.io.read_triangle_mesh(str(mesh_path))
    with np.load(voxel_map_path) as stored_voxels:
        voxel_map = {
            key: stored_voxels[key] for key in stored_voxels.files
        }
    if len(cloud.points) == 0:
        raise ValueError('scene cloud is empty')

    image_path = output_dir / 'command_view.png'
    render_command_view(mesh, cloud, overlay, image_path, width, height)
    video_path = output_dir / 'recovery_3d_replay.mp4'
    video_frames = render_recovery_video(
        mesh, cloud, overlay, image_path, video_path, width, height
    )
    timestamp_ns = int(overlay.get('timestamp_ns', 1))
    bag_path = output_dir / 'command_view_replay'
    replay_counts = write_visualization_bag(
        bag_path, cloud, mesh, voxel_map, overlay, timestamp_ns
    )
    manifest = {
        'schema_version': 1,
        'algorithm_version': ALGORITHM_VERSION,
        'mission_id': overlay['mission_id'],
        'scene_id': overlay['scene_id'],
        'coordinate_frame': overlay['coordinate_frame'],
        'sources': {
            'scene_manifest': {
                'file': scene_manifest_path.name,
                'sha256': file_sha256(scene_manifest_path),
            },
            'colored_cloud': {
                'file': cloud_path.name,
                'sha256': file_sha256(cloud_path),
            },
            'mesh': {
                'file': mesh_path.name,
                'sha256': file_sha256(mesh_path),
            },
            'voxel_map': {
                'file': voxel_map_path.name,
                'sha256': file_sha256(voxel_map_path),
            },
            'overlay': {
                'file': overlay_path.name,
                'sha256': file_sha256(overlay_path),
            },
        },
        'counts': {
            'route_points': len(overlay['route']['points']),
            'recovery_route_points': len(
                overlay.get('recovery_route', {}).get('points', [])
            ),
            'recovery_waypoints': len(
                overlay.get('recovery_waypoints', [])
            ),
            'selected_recovery_waypoints': sum(
                bool(value.get('selected_for_recovery', False))
                for value in overlay.get('recovery_waypoints', [])
            ),
            'communication_zones': len(
                overlay.get('communication_zones', [])
            ),
            'obstacles': len(overlay.get('obstacles', [])),
            'mission_events': len(overlay.get('mission_events', [])),
            'ros_markers': replay_counts['overlay_markers'],
            **replay_counts,
        },
        'communication_summary': overlay.get(
            'communication_summary', {}
        ),
        'products': {
            'command_view': {
                'file': image_path.name,
                'sha256': file_sha256(image_path),
                'width': width,
                'height': height,
            },
            'recovery_video': {
                'file': video_path.name,
                'sha256': file_sha256(video_path),
                'width': width,
                'height': height,
                'fps': 10,
                'frame_count': video_frames,
            },
            'replay_bag': {
                'directory': bag_path.name,
                'sha256': path_sha256(bag_path),
                'topics': [
                    '/mission/scene_cloud',
                    '/mission/scene_markers',
                    '/mission/scene_mesh',
                    '/mission/voxel_markers',
                ],
            },
        },
    }
    manifest_path = output_dir / 'command_view_manifest.json'
    with manifest_path.open('w', encoding='utf-8') as stream:
        json.dump(manifest, stream, indent=2)
        stream.write('\n')
    return manifest

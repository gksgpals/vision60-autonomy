"""ROS node for simulation detection and camera-LiDAR 3D localization."""

import json
import math
from uuid import uuid4

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)
from vision60_msgs.msg import MissionEvent
from visualization_msgs.msg import Marker, MarkerArray

from mission_perception.core import (
    detect_simulation_targets,
    lidar_point_to_map,
    localize_with_lidar,
    normalize_external_detection,
)


def yaw_from_quaternion(quaternion):
    """Return planar yaw from a quaternion."""
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2),
    )


def point_cloud_xyz(message):
    """Read x/y/z float fields without copying padding bytes."""
    offsets = {field.name: field.offset for field in message.fields}
    if not {'x', 'y', 'z'} <= offsets.keys():
        return np.empty((0, 3), dtype=np.float32)
    endian = '>' if message.is_bigendian else '<'
    dtype = np.dtype({
        'names': ['x', 'y', 'z'],
        'formats': [endian + 'f4'] * 3,
        'offsets': [offsets['x'], offsets['y'], offsets['z']],
        'itemsize': message.point_step,
    })
    structured = np.ndarray(
        shape=(message.height, message.width), dtype=dtype,
        buffer=message.data,
        strides=(message.row_step, message.point_step),
    )
    points = np.column_stack((
        structured['x'].reshape(-1),
        structured['y'].reshape(-1),
        structured['z'].reshape(-1),
    ))
    return points[np.isfinite(points).all(axis=1)]


class MissionPerceptionNode(Node):
    """Fuse replaceable 2D detections with LiDAR to publish map events."""

    def __init__(self):
        super().__init__('mission_perception')
        self.declare_parameter('detector_backend', 'simulation_color')
        self.declare_parameter('mission_id', 'vision60_digital_twin')
        self.declare_parameter('external_detection_topic', '/detections_output')
        self.declare_parameter(
            'external_class_map_json',
            '{"0":"fire_candidate","1":"smoke_candidate"}',
        )
        self.declare_parameter('minimum_confidence', 0.35)
        self.declare_parameter('maximum_detection_age_s', 0.25)
        self.backend = self.get_parameter('detector_backend').value
        if self.backend not in {'simulation_color', 'external_vision_msgs'}:
            raise ValueError(
                'detector_backend must be simulation_color or external_vision_msgs'
            )
        self.mission_id = self.get_parameter('mission_id').value
        self.minimum_confidence = float(
            self.get_parameter('minimum_confidence').value
        )
        self.maximum_detection_age_s = float(
            self.get_parameter('maximum_detection_age_s').value
        )
        self.external_class_map = json.loads(
            self.get_parameter('external_class_map_json').value
        )
        self.camera_info = None
        self.cloud = None
        self.odom = None
        self.latest_image = None
        self.confirmations = {}
        self.published_events = set()
        self.det2_pub = self.create_publisher(
            Detection2DArray, '/perception/detections_2d', 10
        )
        self.det3_pub = self.create_publisher(
            Detection3DArray, '/perception/detections_3d', 10
        )
        self.image_pub = self.create_publisher(
            Image, '/perception/annotated_image', 10
        )
        self.event_pub = self.create_publisher(
            MissionEvent, '/mission/event', 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, '/perception/markers', 10
        )
        self.create_subscription(
            CameraInfo, '/camera/camera_info', self.info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2, '/ouster/points', self.cloud_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry, '/vision60/odom', self.odom_callback, 20
        )
        self.create_subscription(
            Image, '/camera/image_raw', self.image_callback,
            qos_profile_sensor_data,
        )
        if self.backend == 'external_vision_msgs':
            self.create_subscription(
                Detection2DArray,
                self.get_parameter('external_detection_topic').value,
                self.external_detection_callback,
                qos_profile_sensor_data,
            )

    def info_callback(self, message):
        self.camera_info = message

    def cloud_callback(self, message):
        self.cloud = point_cloud_xyz(message)

    def odom_callback(self, message):
        self.odom = message

    def image_callback(self, message):
        image = self.decode_image(message)
        if image is None:
            return
        if self.backend == 'external_vision_msgs':
            self.latest_image = (message, image)
            return
        self.process_detections(
            message,
            image,
            detect_simulation_targets(image),
            'simulation_color+ouster_fusion',
        )

    @staticmethod
    def stamp_seconds(header):
        """Convert a ROS timestamp into floating-point seconds."""
        return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9

    def external_detection_callback(self, message):
        """Fuse Isaac ROS-compatible Detection2DArray output with LiDAR."""
        if self.latest_image is None:
            return
        image_message, image = self.latest_image
        age = abs(
            self.stamp_seconds(message.header)
            - self.stamp_seconds(image_message.header)
        )
        if age > self.maximum_detection_age_s:
            self.get_logger().warning(
                f'Ignoring stale detector result ({age:.3f}s)',
                throttle_duration_sec=2.0,
            )
            return
        detections = []
        for item in message.detections:
            if not item.results:
                continue
            result = max(
                item.results, key=lambda entry: entry.hypothesis.score
            )
            converted = normalize_external_detection(
                result.hypothesis.class_id,
                float(result.hypothesis.score),
                float(item.bbox.center.position.x),
                float(item.bbox.center.position.y),
                float(item.bbox.size_x),
                float(item.bbox.size_y),
                int(image_message.width),
                int(image_message.height),
                self.external_class_map,
                self.minimum_confidence,
            )
            if converted is not None:
                detections.append(converted)
        self.process_detections(
            image_message,
            image.copy(),
            detections,
            'isaac_ros_rtdetr+ouster_fusion',
        )

    def process_detections(self, message, image, detections, source_id):
        """Publish normalized 2D, fused 3D, visualization, and events."""
        array_2d = Detection2DArray()
        array_2d.header = message.header
        array_3d = Detection3DArray()
        array_3d.header = message.header
        array_3d.header.frame_id = 'map'
        markers = MarkerArray()
        positions = {}
        for index, detection in enumerate(detections):
            array_2d.detections.append(
                self.make_2d(message, detection, index)
            )
            position = self.localize(detection)
            if position is None:
                self.draw(image, detection, None)
                continue
            positions[detection.class_id] = position
            array_3d.detections.append(
                self.make_3d(message, detection, position, index)
            )
            markers.markers.append(
                self.make_marker(message, detection, position, index)
            )
            self.maybe_publish_event(message, detection, position, source_id)
            self.draw(image, detection, position)
        self.det2_pub.publish(array_2d)
        self.det3_pub.publish(array_3d)
        self.marker_pub.publish(markers)
        self.publish_annotated(message, image)

    def decode_image(self, message):
        raw = np.frombuffer(message.data, dtype=np.uint8)
        expected = int(message.height) * int(message.step)
        if expected <= 0 or raw.size < expected or message.step < message.width * 3:
            return None
        image = raw[:expected].reshape(int(message.height), int(message.step))
        image = image[:, :int(message.width) * 3].reshape(
            int(message.height), int(message.width), 3
        ).copy()
        if message.encoding.lower() == 'rgb8':
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return image

    def localize(self, detection):
        if self.camera_info is None or self.cloud is None or self.odom is None:
            return None
        intrinsic = np.asarray(self.camera_info.k, dtype=np.float64).reshape(3, 3)
        point = localize_with_lidar(detection, self.cloud, intrinsic)
        if point is None:
            return None
        pose = self.odom.pose.pose
        robot = (
            float(pose.position.x), float(pose.position.y),
            float(pose.position.z), yaw_from_quaternion(pose.orientation),
        )
        return lidar_point_to_map(point, robot)

    def make_2d(self, image, detection, index):
        output = Detection2D()
        output.header = image.header
        output.id = f'{detection.class_id}-{index}'
        result = ObjectHypothesisWithPose()
        result.hypothesis.class_id = detection.class_id
        result.hypothesis.score = detection.confidence
        output.results.append(result)
        center_x, center_y = detection.center
        output.bbox.center.position.x = center_x
        output.bbox.center.position.y = center_y
        output.bbox.size_x = float(detection.x_max - detection.x_min + 1)
        output.bbox.size_y = float(detection.y_max - detection.y_min + 1)
        return output

    def make_3d(self, image, detection, position, index):
        output = Detection3D()
        output.header = image.header
        output.header.frame_id = 'map'
        output.id = f'{detection.class_id}-{index}'
        result = ObjectHypothesisWithPose()
        result.hypothesis.class_id = detection.class_id
        result.hypothesis.score = detection.confidence
        result.pose.pose.position.x = float(position[0])
        result.pose.pose.position.y = float(position[1])
        result.pose.pose.position.z = float(position[2])
        result.pose.pose.orientation.w = 1.0
        output.results.append(result)
        output.bbox.center = result.pose.pose
        size = (0.45, 0.45, 1.55) if 'victim' in detection.class_id else (0.55, 0.55, 0.90)
        output.bbox.size.x, output.bbox.size.y, output.bbox.size.z = size
        return output

    def make_marker(self, image, detection, position, index):
        marker = Marker()
        marker.header = image.header
        marker.header.frame_id = 'map'
        marker.ns = 'mission_perception'
        marker.id = index
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.35
        if 'victim' in detection.class_id:
            marker.color.r, marker.color.b = 1.0, 1.0
        else:
            marker.color.r, marker.color.g = 1.0, 0.85
        marker.color.a = 0.95
        marker.lifetime.sec = 1
        return marker

    def maybe_publish_event(self, image, detection, position, source_id):
        count = self.confirmations.get(detection.class_id, 0) + 1
        self.confirmations[detection.class_id] = count
        if count < 3 or detection.class_id in self.published_events:
            return
        event = MissionEvent()
        event.header = image.header
        event.header.frame_id = 'map'
        event.mission_id = self.mission_id
        event.event_id = str(uuid4())
        event.event_type = detection.class_id
        event.severity = (
            MissionEvent.CRITICAL if 'victim' in detection.class_id
            else MissionEvent.WARNING
        )
        event.pose.position.x = float(position[0])
        event.pose.position.y = float(position[1])
        event.pose.position.z = float(position[2])
        event.pose.orientation.w = 1.0
        event.source_id = source_id
        event.description = 'CANDIDATE: requires operator or multi-sensor verification'
        event.confidence = float(detection.confidence)
        self.event_pub.publish(event)
        self.published_events.add(detection.class_id)

    def draw(self, image, detection, position):
        color = (255, 80, 255) if 'victim' in detection.class_id else (0, 220, 255)
        cv2.rectangle(
            image, (detection.x_min, detection.y_min),
            (detection.x_max, detection.y_max), color, 2,
        )
        label = detection.class_id
        if position is not None:
            label += f' map({position[0]:.2f},{position[1]:.2f},{position[2]:.2f})'
        cv2.putText(
            image, label, (max(1, detection.x_min), max(12, detection.y_min - 3)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.30, color, 1, cv2.LINE_AA,
        )

    def publish_annotated(self, source, image):
        output = Image()
        output.header = source.header
        output.height, output.width = image.shape[:2]
        output.encoding = 'bgr8'
        output.is_bigendian = False
        output.step = output.width * 3
        output.data = image.tobytes()
        self.image_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = MissionPerceptionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

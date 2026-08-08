from math import atan2, cos, sin

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from vision60_msgs.msg import CommunicationState, RecoveryWaypoint

from route_recorder.core import RecordedPoint, RouteRecorderCore


class RouteRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__('route_recorder')
        self.declare_parameter('mission_id', 'mission_001')
        self.declare_parameter('distance_threshold_m', 0.25)
        self.declare_parameter('heading_threshold_rad', 0.25)
        self.declare_parameter('communication_change_threshold', 0.15)
        self.declare_parameter('safe_packet_loss_ratio', 0.05)
        self.declare_parameter('safe_latency_ms', 100.0)

        self._core = RouteRecorderCore(
            self.get_parameter('distance_threshold_m').value,
            self.get_parameter('heading_threshold_rad').value,
            self.get_parameter('communication_change_threshold').value,
        )
        self._communication = CommunicationState()
        self._previous_communication_state = CommunicationState.UNKNOWN
        self._recovery_path_pending = False
        self._path_publisher = self.create_publisher(
            Path, '/mission/recorded_path', 10
        )
        self._waypoint_publisher = self.create_publisher(
            RecoveryWaypoint, '/mission/recovery_waypoint', 10
        )
        self._recovery_path_publisher = self.create_publisher(
            Path, '/mission/recovery_path', 10
        )
        self.create_subscription(
            CommunicationState,
            '/communication/state',
            self._communication_callback,
            10,
        )
        self.create_subscription(
            Odometry, '/slam/odom', self._odometry_callback, 20
        )

    def _communication_callback(self, message: CommunicationState) -> None:
        if (
            message.state == CommunicationState.LOST
            and self._previous_communication_state
            != CommunicationState.LOST
        ):
            self._recovery_path_pending = True
        self._previous_communication_state = message.state
        self._communication = message

    def _odometry_callback(self, message: Odometry) -> None:
        pose = message.pose.pose
        yaw = self._yaw_from_quaternion(pose.orientation)
        safe_to_return = (
            self._communication.connected
            and self._communication.state == CommunicationState.NORMAL
            and self._communication.packet_loss_ratio
            <= self.get_parameter('safe_packet_loss_ratio').value
            and self._communication.latency_ms
            <= self.get_parameter('safe_latency_ms').value
        )
        candidate = RecordedPoint(
            timestamp_ns=rclpy.time.Time.from_msg(
                message.header.stamp
            ).nanoseconds,
            x=pose.position.x,
            y=pose.position.y,
            z=pose.position.z,
            yaw=yaw,
            communication_state=self._communication.state,
            packet_loss_ratio=self._communication.packet_loss_ratio,
            latency_ms=self._communication.latency_ms,
            safe_to_return=safe_to_return,
        )
        added = self._core.add(candidate)
        frame_id = message.header.frame_id or 'map'
        if self._recovery_path_pending:
            self._publish_recovery_path(frame_id)
            self._recovery_path_pending = False
        if not added:
            return

        self._publish_path(frame_id)
        if safe_to_return:
            self._publish_waypoint(message)

    def _publish_path(self, frame_id: str) -> None:
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame_id
        for point in self._core.points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.position.z = point.z
            pose.pose.orientation.z = sin(point.yaw / 2.0)
            pose.pose.orientation.w = cos(point.yaw / 2.0)
            path.poses.append(pose)
        self._path_publisher.publish(path)

    def _publish_recovery_path(self, frame_id: str) -> None:
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame_id
        for point in self._core.recovery_segment():
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.position.z = point.z
            pose.pose.orientation.z = sin(point.yaw / 2.0)
            pose.pose.orientation.w = cos(point.yaw / 2.0)
            path.poses.append(pose)
        if len(path.poses) < 2:
            self.get_logger().error(
                'Cannot publish recovery path: fewer than two route points'
            )
            return
        self._recovery_path_publisher.publish(path)
        self.get_logger().warning(
            f'Published recovery path with {len(path.poses)} points'
        )

    def _publish_waypoint(self, odometry: Odometry) -> None:
        mission_id = self.get_parameter('mission_id').value
        waypoint = RecoveryWaypoint()
        waypoint.header = odometry.header
        waypoint.mission_id = mission_id
        waypoint.waypoint_id = f'{mission_id}_wp_{len(self._core.points):05d}'
        waypoint.pose = odometry.pose.pose
        waypoint.channel = self._communication.channel
        waypoint.signal_strength_dbm = (
            self._communication.signal_strength_dbm
        )
        waypoint.snr_db = self._communication.snr_db
        waypoint.packet_loss_ratio = (
            self._communication.packet_loss_ratio
        )
        waypoint.latency_ms = self._communication.latency_ms
        waypoint.last_connected_time = (
            self._communication.last_heartbeat
        )
        waypoint.safe_to_return = True
        waypoint.route_edge_id = f'{mission_id}_route'
        self._waypoint_publisher.publish(waypoint)

    @staticmethod
    def _yaw_from_quaternion(quaternion) -> float:
        siny_cosp = 2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        )
        return atan2(siny_cosp, cosy_cosp)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RouteRecorderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

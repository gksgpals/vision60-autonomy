from math import hypot

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from vision60_msgs.msg import RecoveryStatus, RobotSafetyState


def clamp_planar_speed(message: Twist, limit_mps: float) -> Twist:
    """Copy a Twist and cap its planar linear speed without changing heading."""
    output = Twist()
    output.linear.x = message.linear.x
    output.linear.y = message.linear.y
    output.linear.z = message.linear.z
    output.angular.x = message.angular.x
    output.angular.y = message.angular.y
    output.angular.z = message.angular.z
    speed = hypot(output.linear.x, output.linear.y)
    if speed > limit_mps > 0.0:
        scale = limit_mps / speed
        output.linear.x *= scale
        output.linear.y *= scale
    return output


class SafetyVelocityGate(Node):
    """Allow motion only while LiDAR and velocity inputs are fresh."""

    def __init__(self) -> None:
        super().__init__('safety_velocity_gate')
        self.declare_parameter('lidar_timeout_s', 0.5)
        self.declare_parameter('command_timeout_s', 0.3)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('require_motion_permission', False)
        self.declare_parameter('max_reentry_speed_mps', 0.10)
        self._lidar_timeout = float(
            self.get_parameter('lidar_timeout_s').value
        )
        self._command_timeout = float(
            self.get_parameter('command_timeout_s').value
        )
        rate = float(self.get_parameter('publish_rate_hz').value)
        self._last_lidar_ns = 0
        self._last_command_ns = 0
        self._command = Twist()
        self._require_motion_permission = bool(
            self.get_parameter('require_motion_permission').value
        )
        self._motion_allowed = not self._require_motion_permission
        self._reentry_active = False
        self._max_reentry_speed = float(
            self.get_parameter('max_reentry_speed_mps').value
        )
        self._publisher = self.create_publisher(
            Twist, '/cmd_vel_safe', 10
        )
        self.create_subscription(
            PointCloud2, '/ouster/points', self._lidar_callback, 10
        )
        self.create_subscription(
            Twist,
            '/cmd_vel_collision_checked',
            self._command_callback,
            10,
        )
        self.create_subscription(
            RobotSafetyState,
            '/vision60/safety_state',
            self._safety_callback,
            10,
        )
        self.create_subscription(
            RecoveryStatus,
            '/communication/recovery_status',
            self._recovery_status_callback,
            10,
        )
        self.create_timer(1.0 / rate, self._tick)

    def _lidar_callback(self, _message: PointCloud2) -> None:
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = (
            _message.header.stamp.sec * 1_000_000_000
            + _message.header.stamp.nanosec
        )
        if source_stamp_is_fresh(now_ns, stamp_ns, self._lidar_timeout):
            self._last_lidar_ns = now_ns

    def _command_callback(self, message: Twist) -> None:
        self._command = message
        self._last_command_ns = self.get_clock().now().nanoseconds

    def _safety_callback(self, message: RobotSafetyState) -> None:
        self._motion_allowed = bool(message.motion_allowed)

    def _recovery_status_callback(self, message: RecoveryStatus) -> None:
        self._reentry_active = message.state == RecoveryStatus.REENTRY_TEST

    def _tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        lidar_fresh = is_fresh(
            now_ns, self._last_lidar_ns, self._lidar_timeout
        )
        command_fresh = is_fresh(
            now_ns, self._last_command_ns, self._command_timeout
        )
        if not (lidar_fresh and command_fresh and self._motion_allowed):
            self._publisher.publish(Twist())
            return
        command = (
            clamp_planar_speed(self._command, self._max_reentry_speed)
            if self._reentry_active else self._command
        )
        self._publisher.publish(command)


def is_fresh(now_ns: int, last_ns: int, timeout_s: float) -> bool:
    return (
        last_ns > 0
        and now_ns >= last_ns
        and (now_ns - last_ns) / 1e9 <= timeout_s
    )


def source_stamp_is_fresh(
    now_ns: int, stamp_ns: int, timeout_s: float
) -> bool:
    return is_fresh(now_ns, stamp_ns, timeout_s)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyVelocityGate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class Nav2MockRobot(Node):
    """Integrate Nav2 velocity commands into mock odometry and TF."""

    def __init__(self) -> None:
        super().__init__('nav2_mock_robot')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('initial_x', 2.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', math.pi)
        self.declare_parameter('command_timeout_s', 0.5)

        self._rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._timeout_s = float(
            self.get_parameter('command_timeout_s').value
        )
        self._x = float(self.get_parameter('initial_x').value)
        self._y = float(self.get_parameter('initial_y').value)
        self._yaw = float(self.get_parameter('initial_yaw').value)
        self._command = Twist()
        self._last_command_ns = 0

        self._odom_publisher = self.create_publisher(
            Odometry, '/slam/odom', 20
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Twist, '/cmd_vel_safe', self._command_callback, 20
        )
        self.create_timer(1.0 / self._rate_hz, self._tick)

    def _command_callback(self, message: Twist) -> None:
        self._command = message
        self._last_command_ns = self.get_clock().now().nanoseconds

    def _tick(self) -> None:
        now = self.get_clock().now()
        command_age_s = (
            (now.nanoseconds - self._last_command_ns) / 1e9
            if self._last_command_ns else float('inf')
        )
        linear = (
            self._command.linear.x
            if command_age_s <= self._timeout_s else 0.0
        )
        angular = (
            self._command.angular.z
            if command_age_s <= self._timeout_s else 0.0
        )
        dt = 1.0 / self._rate_hz
        self._yaw += angular * dt
        self._x += linear * math.cos(self._yaw) * dt
        self._y += linear * math.sin(self._yaw) * dt

        qx, qy, qz, qw = yaw_to_quaternion(self._yaw)
        stamp = now.to_msg()
        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = 'odom'
        odometry.child_frame_id = 'base_link'
        odometry.pose.pose.position.x = self._x
        odometry.pose.pose.position.y = self._y
        odometry.pose.pose.orientation.x = qx
        odometry.pose.pose.orientation.y = qy
        odometry.pose.pose.orientation.z = qz
        odometry.pose.pose.orientation.w = qw
        odometry.twist.twist.linear.x = linear
        odometry.twist.twist.angular.z = angular
        self._odom_publisher.publish(odometry)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self._x
        transform.transform.translation.y = self._y
        transform.transform.rotation = odometry.pose.pose.orientation
        self._tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2MockRobot()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

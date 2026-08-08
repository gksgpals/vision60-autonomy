import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from vision60_msgs.msg import RobotSafetyState


class BridgeIntegrationProbe(Node):
    """Verify motion passes and then stops after command timeout."""

    def __init__(self) -> None:
        super().__init__('bridge_integration_probe')
        self._start_ns = self.get_clock().now().nanoseconds
        self._saw_motion = False
        self._saw_odom_motion = False
        self._passed = False
        self.done = False
        self._localization_publisher = self.create_publisher(
            Odometry, '/state/odometry', 10
        )
        self._lidar_publisher = self.create_publisher(
            PointCloud2, '/ouster/points', 10
        )
        self._walk_publisher = self.create_publisher(
            Bool, '/walk_enable', 10
        )
        self._command_publisher = self.create_publisher(
            Twist, '/cmd_vel_safe', 10
        )
        self.create_subscription(
            Twist,
            '/vision60/command_applied',
            self._command_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            '/vision60/odom',
            self._odometry_callback,
            10,
        )
        self.create_subscription(
            RobotSafetyState,
            '/vision60/safety_state',
            self._safety_callback,
            10,
        )
        self.create_timer(0.05, self._tick)

    def _elapsed_s(self) -> float:
        return (
            self.get_clock().now().nanoseconds - self._start_ns
        ) / 1e9

    def _tick(self) -> None:
        now = self.get_clock().now().to_msg()
        localization = Odometry()
        localization.header.stamp = now
        localization.header.frame_id = 'odom'
        localization.child_frame_id = 'base_link'
        localization.pose.pose.orientation.w = 1.0
        self._localization_publisher.publish(localization)

        lidar = PointCloud2()
        lidar.header.stamp = now
        lidar.header.frame_id = 'os_sensor'
        self._lidar_publisher.publish(lidar)

        walk = Bool()
        walk.data = True
        self._walk_publisher.publish(walk)

        if self._elapsed_s() < 1.5:
            command = Twist()
            command.linear.x = 0.2
            self._command_publisher.publish(command)
        elif self._elapsed_s() > 5.0 and not self.done:
            self.get_logger().error('BRIDGE INTEGRATION FAIL: timeout')
            self.done = True

    def _command_callback(self, message: Twist) -> None:
        if message.linear.x > 0.0:
            self._saw_motion = True

    def _odometry_callback(self, message: Odometry) -> None:
        if message.pose.pose.position.x > 0.01:
            self._saw_odom_motion = True

    def _safety_callback(self, message: RobotSafetyState) -> None:
        stopped_after_timeout = (
            self._elapsed_s() > 1.8
            and message.command_timed_out
            and not message.motion_allowed
        )
        if (
            self._saw_motion
            and self._saw_odom_motion
            and stopped_after_timeout
        ):
            self._passed = True
            self.done = True
            self.get_logger().info(
                'BRIDGE INTEGRATION PASS: motion then timeout stop'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BridgeIntegrationProbe()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    passed = node._passed
    node.destroy_node()
    rclpy.shutdown()
    if not passed:
        sys.exit(1)

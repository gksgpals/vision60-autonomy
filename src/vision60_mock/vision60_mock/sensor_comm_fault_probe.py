"""Verify that delayed or lost LiDAR data produces a safe zero command."""
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from vision60_msgs.msg import CommunicationState


def fault_is_confirmed(
    mode: str,
    raw_seen: bool,
    forwarded_seen: bool,
    max_forwarded_age_s: float,
    elapsed_s: float,
) -> bool:
    if mode == 'delay':
        return raw_seen and forwarded_seen and max_forwarded_age_s >= 0.5
    if mode == 'drop':
        return raw_seen and not forwarded_seen and elapsed_s >= 2.0
    return False


class SensorCommunicationFaultProbe(Node):

    def __init__(self) -> None:
        super().__init__('sensor_comm_fault_probe')
        self.declare_parameter('expected_mode', 'delay')
        self.declare_parameter('timeout_s', 15.0)
        self._mode = str(self.get_parameter('expected_mode').value)
        self._timeout_s = float(self.get_parameter('timeout_s').value)
        self._start_ns = self.get_clock().now().nanoseconds
        self._raw_seen = False
        self._forwarded_seen = False
        self._max_forwarded_age_s = 0.0
        self._communication_disconnected = False
        self._safe_zero_during_combined_fault = False
        self.done = False
        self.passed = False

        self._command_publisher = self.create_publisher(
            Twist, '/cmd_vel_collision_checked', 10
        )
        self.create_subscription(
            PointCloud2,
            '/ouster/points_raw',
            self._raw_callback,
            10,
        )
        self.create_subscription(
            PointCloud2,
            '/ouster/points',
            self._forwarded_callback,
            10,
        )
        self.create_subscription(
            CommunicationState,
            '/communication/state',
            self._communication_callback,
            10,
        )
        self.create_subscription(
            Twist,
            '/cmd_vel_safe',
            self._safe_velocity_callback,
            20,
        )
        self.create_timer(0.05, self._tick)

    def _elapsed_s(self) -> float:
        return (
            self.get_clock().now().nanoseconds - self._start_ns
        ) / 1e9

    def _raw_callback(self, _message: PointCloud2) -> None:
        self._raw_seen = True

    def _forwarded_callback(self, message: PointCloud2) -> None:
        self._forwarded_seen = True
        stamp_ns = (
            message.header.stamp.sec * 1_000_000_000
            + message.header.stamp.nanosec
        )
        age_s = (
            self.get_clock().now().nanoseconds - stamp_ns
        ) / 1e9
        self._max_forwarded_age_s = max(
            self._max_forwarded_age_s,
            age_s,
        )

    def _communication_callback(
        self,
        message: CommunicationState,
    ) -> None:
        if not message.connected:
            self._communication_disconnected = True

    def _safe_velocity_callback(self, message: Twist) -> None:
        if (
            self._communication_disconnected
            and self._fault_confirmed()
            and abs(message.linear.x) <= 0.001
            and abs(message.angular.z) <= 0.001
        ):
            self._safe_zero_during_combined_fault = True

    def _fault_confirmed(self) -> bool:
        return fault_is_confirmed(
            self._mode,
            self._raw_seen,
            self._forwarded_seen,
            self._max_forwarded_age_s,
            self._elapsed_s(),
        )

    def _tick(self) -> None:
        command = Twist()
        command.linear.x = 0.2
        self._command_publisher.publish(command)

        if self._safe_zero_during_combined_fault:
            self.passed = True
            self.done = True
            self.get_logger().info(
                'PASS mode=%s raw=%s forwarded=%s max_age=%.3fs '
                'link_disconnected=%s safe_zero=%s'
                % (
                    self._mode,
                    self._raw_seen,
                    self._forwarded_seen,
                    self._max_forwarded_age_s,
                    self._communication_disconnected,
                    self._safe_zero_during_combined_fault,
                )
            )
            return

        if self._elapsed_s() >= self._timeout_s:
            self.done = True
            self.get_logger().error(
                'FAIL mode=%s raw=%s forwarded=%s max_age=%.3fs '
                'link_disconnected=%s safe_zero=%s'
                % (
                    self._mode,
                    self._raw_seen,
                    self._forwarded_seen,
                    self._max_forwarded_age_s,
                    self._communication_disconnected,
                    self._safe_zero_during_combined_fault,
                )
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorCommunicationFaultProbe()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        passed = node.passed
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if passed else 1)

import sys

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Bool
from vision60_msgs.msg import RecoveryStatus


class IntegrationProbe(Node):
    def __init__(self) -> None:
        super().__init__('vision60_integration_probe')
        self._start_ns = self.get_clock().now().nanoseconds
        self._path_points = 0
        self._stopped = False
        self._recovery_state = 0
        self._passed = False
        self.done = False
        self.create_subscription(
            Path, '/mission/recorded_path', self._path_callback, 10
        )
        self.create_subscription(
            Bool, '/vision60/mock_stopped', self._stopped_callback, 10
        )
        self.create_subscription(
            RecoveryStatus,
            '/communication/recovery_status',
            self._status_callback,
            10,
        )
        self.create_timer(0.2, self._check)

    def _path_callback(self, message: Path) -> None:
        self._path_points = max(self._path_points, len(message.poses))

    def _stopped_callback(self, message: Bool) -> None:
        self._stopped = message.data

    def _status_callback(self, message: RecoveryStatus) -> None:
        self._recovery_state = message.state

    def _check(self) -> None:
        elapsed_s = (
            self.get_clock().now().nanoseconds - self._start_ns
        ) / 1e9
        if (
            self._path_points >= 5
            and self._stopped
            and self._recovery_state == RecoveryStatus.RETURNING
        ):
            self._passed = True
            self.get_logger().info(
                'INTEGRATION PASS: path recorded, link loss detected, '
                'safe stop confirmed, recovery authorized'
            )
            self.done = True
        elif elapsed_s > 12.0:
            self.get_logger().error(
                'INTEGRATION FAIL: '
                f'path_points={self._path_points}, '
                f'stopped={self._stopped}, '
                f'recovery_state={self._recovery_state}'
            )
            self.done = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IntegrationProbe()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.2)
    passed = node._passed
    node.destroy_node()
    rclpy.shutdown()
    if not passed:
        sys.exit(1)

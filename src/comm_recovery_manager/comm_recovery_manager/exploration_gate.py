"""Connect communication recovery and robot safety to explore_lite."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from vision60_msgs.msg import RecoveryStatus, RobotSafetyState

from comm_recovery_manager.exploration_core import ExplorationGateCore


class ExplorationSafetyGate(Node):
    """Pause frontier goals whenever recovery or safety owns motion."""

    def __init__(self) -> None:
        super().__init__('exploration_safety_gate')
        self._core = ExplorationGateCore()
        self._last_decision = None
        self._last_subscriber_count = 0

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._resume_publisher = self.create_publisher(
            Bool, '/explore/resume', qos
        )
        self._allowed_publisher = self.create_publisher(
            Bool, '/mission/exploration_allowed', qos
        )
        self.create_subscription(
            RecoveryStatus,
            '/communication/recovery_status',
            self._recovery_callback,
            10,
        )
        self.create_subscription(
            RobotSafetyState,
            '/vision60/safety_state',
            self._safety_callback,
            10,
        )
        self.create_timer(0.25, self._tick)

    def _recovery_callback(self, message: RecoveryStatus) -> None:
        self._core.observe_recovery(message.state)
        self._publish_if_changed()

    def _safety_callback(self, message: RobotSafetyState) -> None:
        self._core.observe_safety(
            message.walk_enabled,
            message.motion_allowed,
            message.emergency_stop,
        )
        self._publish_if_changed()

    def _tick(self) -> None:
        subscriber_count = self._resume_publisher.get_subscription_count()
        subscriber_joined = (
            subscriber_count > 0 and self._last_subscriber_count == 0
        )
        self._last_subscriber_count = subscriber_count
        self._publish_if_changed(force=subscriber_joined)

    def _publish_if_changed(self, force: bool = False) -> None:
        allowed = self._core.exploration_allowed
        if not force and allowed == self._last_decision:
            return
        self._last_decision = allowed
        message = Bool()
        message.data = allowed
        self._resume_publisher.publish(message)
        self._allowed_publisher.publish(message)
        self.get_logger().warning(
            'Frontier exploration '
            + ('RESUMED: ' if allowed else 'PAUSED: ')
            + self._core.reason
        )


def main(args=None) -> None:
    """Run the exploration safety gate."""
    rclpy.init(args=args)
    node = ExplorationSafetyGate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

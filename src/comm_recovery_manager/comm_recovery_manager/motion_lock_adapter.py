import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from vision60_msgs.msg import RobotSafetyState


def motion_lock_required(motion_allowed: bool) -> bool:
    """Return the twist_mux lock value for the robot safety state."""
    return not motion_allowed


class MotionLockAdapter(Node):
    """Convert the Vision60 safety state into a fail-closed twist_mux lock."""

    def __init__(self) -> None:
        super().__init__('motion_lock_adapter')
        self._publisher = self.create_publisher(
            Bool, '/safety/cmd_vel_lock', 10
        )
        self.create_subscription(
            RobotSafetyState,
            '/vision60/safety_state',
            self._safety_callback,
            10,
        )

    def _safety_callback(self, message: RobotSafetyState) -> None:
        lock = Bool()
        lock.data = motion_lock_required(message.motion_allowed)
        self._publisher.publish(lock)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotionLockAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

from math import hypot

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from vision60_msgs.msg import RecoveryEvent, RecoveryStatus


class RecoveryPathFollower(Node):
    """Forward recorded recovery paths to the Nav2 controller."""

    def __init__(self) -> None:
        super().__init__('recovery_path_follower')
        self.declare_parameter('require_returning_state', True)
        self.declare_parameter('mission_id', 'mission_001')
        self.declare_parameter('odometry_topic', '/state/odometry')
        self.declare_parameter('waypoint_tolerance_m', 0.18)
        self.declare_parameter('controller_id', 'FollowPath')
        self.declare_parameter('goal_checker_id', 'goal_checker')
        self._client = ActionClient(self, FollowPath, '/follow_path')
        self._event_publisher = self.create_publisher(
            RecoveryEvent, '/communication/recovery_event', 10
        )
        self._pending_path = None
        self._held_warning_logged = False
        self._goal_handle = None
        self._target_xy = None
        self._result_reported = False
        self._latest_xy = None
        self._returning_authorized = not bool(
            self.get_parameter('require_returning_state').value
        )
        self.create_subscription(
            Path, '/mission/recovery_path', self._path_callback, 10
        )
        self.create_subscription(
            RecoveryStatus,
            '/communication/recovery_status',
            self._status_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter('odometry_topic').value,
            self._odometry_callback,
            20,
        )

    def _path_callback(self, path: Path) -> None:
        if len(path.poses) < 2:
            self.get_logger().error('Ignoring recovery path with < 2 poses')
            return
        self._pending_path = path
        self._held_warning_logged = False
        self._try_follow()

    def _status_callback(self, status: RecoveryStatus) -> None:
        self._returning_authorized = (
            status.state == RecoveryStatus.RETURNING
        )
        self._try_follow()

    def _try_follow(self) -> None:
        if self._pending_path is None:
            return
        if not self._returning_authorized:
            if not self._held_warning_logged:
                self.get_logger().warning(
                    'Recovery path held: waiting for confirmed safe stop'
                )
                self._held_warning_logged = True
            return
        self._held_warning_logged = False
        if not self._client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Nav2 FollowPath server unavailable')
            return
        goal = FollowPath.Goal()
        goal.path = self._pending_path
        goal.controller_id = self.get_parameter('controller_id').value
        goal.goal_checker_id = self.get_parameter('goal_checker_id').value
        target = self._pending_path.poses[-1].pose.position
        self._target_xy = (float(target.x), float(target.y))
        if self._latest_xy is not None:
            self.get_logger().warning(
                'Recovery waypoint start='
                f'({self._latest_xy[0]:.2f},{self._latest_xy[1]:.2f}) '
                'target='
                f'({self._target_xy[0]:.2f},{self._target_xy[1]:.2f})'
            )
        self._result_reported = False
        self._pending_path = None
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._goal_response)

    def _goal_response(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Nav2 rejected the recovery path')
            self._publish_result(
                False, 'Nav2 rejected the recorded recovery path'
            )
            return
        self.get_logger().warning('Nav2 accepted the recovery path')
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._result)

    def _odometry_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        self._latest_xy = (float(position.x), float(position.y))
        if (
            self._goal_handle is None
            or self._target_xy is None
            or self._result_reported
        ):
            return
        distance = hypot(
            self._latest_xy[0] - self._target_xy[0],
            self._latest_xy[1] - self._target_xy[1],
        )
        tolerance = float(
            self.get_parameter('waypoint_tolerance_m').value
        )
        if distance > tolerance:
            return
        self._result_reported = True
        self._publish_result(
            True, 'recorded recovery waypoint reached'
        )
        self._goal_handle.cancel_goal_async()
        self._goal_handle = None
        self._target_xy = None
        self.get_logger().info('Recorded recovery waypoint reached')

    def _result(self, future) -> None:
        if self._result_reported:
            return
        status = future.result().status
        self._result_reported = True
        self._goal_handle = None
        self._target_xy = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Recorded recovery path completed')
            self._publish_result(
                True, 'recorded recovery path completed'
            )
        else:
            self.get_logger().error(
                f'Recorded recovery path failed: status={status}'
            )
            self._publish_result(
                False,
                f'recorded recovery path failed: Nav2 status={status}',
            )

    def _publish_result(self, success: bool, detail: str) -> None:
        event = RecoveryEvent()
        event.header.stamp = self.get_clock().now().to_msg()
        event.header.frame_id = 'map'
        event.mission_id = self.get_parameter('mission_id').value
        event.event = (
            RecoveryEvent.RETURN_SUCCEEDED
            if success
            else RecoveryEvent.RETURN_FAILED
        )
        event.detail = detail
        self._event_publisher.publish(event)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RecoveryPathFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

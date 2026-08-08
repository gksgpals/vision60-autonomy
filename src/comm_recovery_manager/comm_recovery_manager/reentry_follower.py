from math import hypot

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowPath
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from vision60_msgs.msg import (
    CommunicationState,
    RecoveryEvent,
    RecoveryStatus,
)

from comm_recovery_manager.reentry_core import (
    LinkLossMonitor,
    build_reentry_path,
)


class ReentryPathFollower(Node):
    def __init__(self) -> None:
        super().__init__('reentry_path_follower')
        self.declare_parameter('mission_id', 'mission_001')
        self.declare_parameter('reentry_speed_mps', 0.1)
        self.declare_parameter('link_loss_timeout_s', 1.0)
        self.declare_parameter('speed_limit_topic', '/speed_limit')
        self.declare_parameter('odometry_topic', '/state/odometry')
        self.declare_parameter('waypoint_tolerance_m', 0.18)
        self.declare_parameter('controller_id', 'FollowPath')
        self.declare_parameter('goal_checker_id', 'goal_checker')
        self.declare_parameter('stationary_speed_threshold_mps', 0.03)
        self.declare_parameter('stationary_samples_required', 3)

        self._client = ActionClient(self, FollowPath, '/follow_path')
        self._event_publisher = self.create_publisher(
            RecoveryEvent, '/communication/recovery_event', 10
        )
        self._speed_limit_publisher = self.create_publisher(
            SpeedLimit,
            self.get_parameter('speed_limit_topic').value,
            10,
        )
        self._recovery_path = None
        self._active = False
        self._started = False
        self._failure_reported = False
        self._goal_handle = None
        self._target_xy = None
        self._reentry_requested = False
        self._stationary_samples = 0
        self._stationary_wait_logged = False
        self._loss_monitor = LinkLossMonitor(
            float(
                self.get_parameter('link_loss_timeout_s').value
            )
        )
        self.create_subscription(
            Path,
            '/mission/recovery_path',
            self._path_callback,
            10,
        )
        self.create_subscription(
            RecoveryStatus,
            '/communication/recovery_status',
            self._status_callback,
            10,
        )
        self.create_subscription(
            CommunicationState,
            '/communication/state',
            self._communication_callback,
            20,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter('odometry_topic').value,
            self._odometry_callback,
            20,
        )

    def _path_callback(self, message: Path) -> None:
        if len(message.poses) >= 2:
            self._recovery_path = message

    def _status_callback(self, message: RecoveryStatus) -> None:
        if message.state == RecoveryStatus.REENTRY_TEST:
            if (
                not self._reentry_requested
                and not self._active
                and not self._started
            ):
                self._reentry_requested = True
                self._stationary_samples = 0
                self._stationary_wait_logged = False
            self._try_start()
            return
        if not self._active:
            self._started = False
            self._reentry_requested = False
            self._loss_monitor.reset()

    def _try_start(self) -> None:
        if self._started or self._active:
            return
        required = int(
            self.get_parameter('stationary_samples_required').value
        )
        if self._stationary_samples < required:
            if not self._stationary_wait_logged:
                self.get_logger().warning(
                    'Holding reentry until the robot is stationary'
                )
                self._stationary_wait_logged = True
            return
        if self._recovery_path is None:
            self.get_logger().error(
                'Cannot start reentry: recovery path is unavailable'
            )
            return
        if not self._client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warning(
                'Cannot start reentry yet: FollowPath unavailable'
            )
            return

        path = build_reentry_path(self._recovery_path)
        now = self.get_clock().now().to_msg()
        path.header.stamp = now
        for pose in path.poses:
            pose.header.stamp = now

        self._publish_speed_limit(
            float(self.get_parameter('reentry_speed_mps').value)
        )
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self.get_parameter('controller_id').value
        goal.goal_checker_id = self.get_parameter('goal_checker_id').value
        target = path.poses[-1].pose.position
        self._target_xy = (float(target.x), float(target.y))
        self._started = True
        self._reentry_requested = False
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._goal_response)
        self.get_logger().warning(
            f'Low-speed reentry requested with {len(path.poses)} poses'
        )

    def _goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as error:
            self._report_navigation_failure(
                f'reentry goal request failed: {error}'
            )
            return
        if not handle.accepted:
            self._report_navigation_failure(
                'Nav2 rejected the low-speed reentry path'
            )
            return
        self._goal_handle = handle
        self._active = True
        handle.get_result_async().add_done_callback(self._result)

    def _communication_callback(
        self,
        message: CommunicationState,
    ) -> None:
        if not self._active or self._failure_reported:
            return
        now_s = self.get_clock().now().nanoseconds / 1e9
        if not self._loss_monitor.observe(
            bool(message.connected),
            now_s,
        ):
            return

        self._failure_reported = True
        self._publish_event(
            RecoveryEvent.REENTRY_LINK_LOST,
            'communication loss repeated during low-speed reentry',
        )
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._finish()

    def _odometry_callback(self, message: Odometry) -> None:
        twist = message.twist.twist
        planar_speed = hypot(float(twist.linear.x), float(twist.linear.y))
        threshold = float(
            self.get_parameter('stationary_speed_threshold_mps').value
        )
        if self._reentry_requested and planar_speed <= threshold:
            self._stationary_samples += 1
        elif self._reentry_requested:
            self._stationary_samples = 0
        if self._reentry_requested and not self._active:
            self._try_start()
        if (
            not self._active
            or self._target_xy is None
            or self._failure_reported
        ):
            return
        position = message.pose.pose.position
        distance = hypot(
            float(position.x) - self._target_xy[0],
            float(position.y) - self._target_xy[1],
        )
        tolerance = float(
            self.get_parameter('waypoint_tolerance_m').value
        )
        if distance > tolerance:
            return
        self._failure_reported = True
        self._publish_event(
            RecoveryEvent.REENTRY_SUCCEEDED,
            'low-speed reentry waypoint reached with stable communication',
        )
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self.get_logger().info('Low-speed reentry waypoint reached')
        self._finish()

    def _result(self, future) -> None:
        if self._failure_reported:
            return
        try:
            status = future.result().status
        except Exception as error:
            self._report_navigation_failure(
                f'reentry result failed: {error}'
            )
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._publish_event(
                RecoveryEvent.REENTRY_SUCCEEDED,
                'low-speed reentry completed with stable communication',
            )
            self.get_logger().info('Low-speed reentry completed')
            self._finish()
            return
        self._report_navigation_failure(
            f'low-speed reentry failed: Nav2 status={status}'
        )

    def _report_navigation_failure(self, detail: str) -> None:
        if self._failure_reported:
            return
        self._failure_reported = True
        self._publish_event(RecoveryEvent.REENTRY_FAILED, detail)
        self.get_logger().error(detail)
        self._finish()

    def _publish_event(self, event_type: int, detail: str) -> None:
        event = RecoveryEvent()
        event.header.stamp = self.get_clock().now().to_msg()
        event.header.frame_id = 'map'
        event.mission_id = self.get_parameter('mission_id').value
        event.event = event_type
        event.detail = detail
        self._event_publisher.publish(event)

    def _publish_speed_limit(self, speed_mps: float) -> None:
        limit = SpeedLimit()
        limit.header.stamp = self.get_clock().now().to_msg()
        limit.percentage = False
        limit.speed_limit = speed_mps
        self._speed_limit_publisher.publish(limit)

    def _finish(self) -> None:
        self._publish_speed_limit(0.0)
        self._active = False
        self._reentry_requested = False
        self._stationary_samples = 0
        self._goal_handle = None
        self._target_xy = None
        self._loss_monitor.reset()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReentryPathFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

from math import hypot

import rclpy
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from vision60_msgs.action import SynchronizeMission
from vision60_msgs.msg import (
    CommunicationState,
    MissionEvent,
    RecoveryEvent,
    RecoveryStatus,
    RobotSafetyState,
)
from vision60_msgs.srv import RequestSafeStop, SetWalkEnabled

from comm_recovery_manager.core import (
    InvalidTransition,
    RecoveryEventType,
    RecoveryManagerCore,
    RecoveryState,
    apply_recovery_event,
)


class CommunicationRecoveryManagerNode(Node):
    def __init__(self) -> None:
        super().__init__('comm_recovery_manager')
        self.declare_parameter('mission_id', 'mission_001')
        self.declare_parameter('lost_timeout_s', 2.0)
        self.declare_parameter('degraded_packet_loss_ratio', 0.20)
        self.declare_parameter('degraded_latency_ms', 500.0)
        self.declare_parameter('max_channel_switch_attempts', 2)
        self.declare_parameter('reenable_walk_for_recovery', True)
        self.declare_parameter('sync_action_name', '/mission/synchronize')
        self.declare_parameter('reentry_stationary_speed_mps', 0.03)
        self.declare_parameter('reentry_stationary_samples', 3)

        self._core = RecoveryManagerCore(
            self.get_parameter('lost_timeout_s').value,
            self.get_parameter('degraded_packet_loss_ratio').value,
            self.get_parameter('degraded_latency_ms').value,
            int(
                self.get_parameter(
                    'max_channel_switch_attempts'
                ).value
            ),
        )
        self._status_publisher = self.create_publisher(
            RecoveryStatus, '/communication/recovery_status', 10
        )
        self._mission_event_publisher = self.create_publisher(
            MissionEvent, '/mission/event', 10
        )
        self._latest_pose = None
        self._last_published_failure_cause = 0
        self._safe_stop_client = self.create_client(
            RequestSafeStop, '/vision60/request_safe_stop'
        )
        self._walk_enable_client = self.create_client(
            SetWalkEnabled, '/vision60/set_walk_enabled'
        )
        self._sync_action_client = ActionClient(
            self,
            SynchronizeMission,
            self.get_parameter('sync_action_name').value,
        )
        self._sync_goal_pending = False
        self._sync_waiting_for_stop = False
        self._stationary_samples = 0
        self._sync_unavailable_logged = False
        self.create_subscription(
            CommunicationState,
            '/communication/state',
            self._communication_callback,
            10,
        )
        self.create_subscription(
            RobotSafetyState,
            '/vision60/safety_state',
            self._safety_callback,
            10,
        )
        self.create_subscription(
            RecoveryEvent,
            '/communication/recovery_event',
            self._recovery_event_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            '/state/odometry',
            self._odometry_callback,
            10,
        )
        self.create_timer(0.5, self._tick)

    def _communication_callback(self, message: CommunicationState) -> None:
        previous_state = self._core.state
        now_s = self.get_clock().now().nanoseconds / 1e9
        self._core.observe_link(
            message.connected,
            message.packet_loss_ratio,
            message.latency_ms,
            now_s,
            message.channel,
        )
        if (
            self._core.state == RecoveryState.LINK_LOST
            and previous_state != RecoveryState.LINK_LOST
        ):
            self._request_safe_stop()
            self._core.start_stopping()
        self._publish_status()

    def _request_safe_stop(self) -> None:
        if not self._safe_stop_client.service_is_ready():
            self.get_logger().warning(
                'safe-stop service unavailable; state remains STOPPING'
            )
            return
        request = RequestSafeStop.Request()
        request.reason = 'communication link lost'
        request.emergency = False
        self._safe_stop_client.call_async(request)

    def _safety_callback(self, message: RobotSafetyState) -> None:
        if (
            self._core.state == RecoveryState.STOPPING
            and is_confirmed_stopped(message)
        ):
            self._core.confirm_stopped()
            self.get_logger().warning(
                'Robot stop confirmed; recovery motion is now authorized'
            )
            self._authorize_recovery_walk()
            self._publish_status()

    def _authorize_recovery_walk(self) -> None:
        """
        Re-enable walking only after a full stop was confirmed.

        The safe stop that follows link loss clears ``walk_enabled``, so the
        recorded-route return would never move. Walking is re-enabled here,
        strictly after ``STOPPING -> RETURNING``, so the stop-then-return
        order in the recovery policy is preserved.
        """
        if not bool(
            self.get_parameter('reenable_walk_for_recovery').value
        ):
            self.get_logger().warning(
                'reenable_walk_for_recovery=false: recovery drive stays '
                'blocked until an operator enables walking'
            )
            return
        if not self._walk_enable_client.service_is_ready():
            self.get_logger().error(
                'set_walk_enabled unavailable; recovery drive blocked'
            )
            return
        request = SetWalkEnabled.Request()
        request.enabled = True
        self._walk_enable_client.call_async(request)
        self.get_logger().warning(
            'Walking re-enabled for recorded-route recovery only'
        )

    def _recovery_event_callback(self, message: RecoveryEvent) -> None:
        mission_id = self.get_parameter('mission_id').value
        if message.mission_id and message.mission_id != mission_id:
            self.get_logger().warning(
                f'Ignoring recovery event for mission {message.mission_id}'
            )
            return
        try:
            apply_recovery_event(
                self._core,
                RecoveryEventType(message.event),
                message.channel,
                message.detail,
            )
        except (InvalidTransition, ValueError) as error:
            self.get_logger().error(
                f'Rejected recovery event {message.event}: {error}'
            )
            return
        self.get_logger().warning(
            f'Recovery event applied: {message.detail or message.event}'
        )
        self._publish_classification_if_new()
        self._maybe_start_sync()
        self._publish_status()

    def _tick(self) -> None:
        self._maybe_start_sync()
        self._publish_status()

    def _maybe_start_sync(self) -> None:
        if self._core.state != RecoveryState.LINK_RECOVERED:
            return
        if self._sync_goal_pending:
            return
        if not self._sync_action_client.server_is_ready():
            if not self._sync_unavailable_logged:
                self.get_logger().warning(
                    'Mission sync server unavailable; waiting in '
                    'LINK_RECOVERED'
                )
                self._sync_unavailable_logged = True
            return

        self._sync_unavailable_logged = False
        self._sync_goal_pending = True
        self._core.start_sync()
        goal = SynchronizeMission.Goal()
        goal.mission_id = self.get_parameter('mission_id').value
        future = self._sync_action_client.send_goal_async(goal)
        future.add_done_callback(self._sync_goal_response)
        self.get_logger().warning(
            'Mission data synchronization requested'
        )

    def _sync_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as error:
            self._finish_sync(False, f'sync request failed: {error}')
            return
        if not handle.accepted:
            self._finish_sync(False, 'sync request was rejected')
            return
        handle.get_result_async().add_done_callback(
            self._sync_result
        )

    def _sync_result(self, future) -> None:
        try:
            wrapped_result = future.result()
            success = bool(wrapped_result.result.success)
            detail = wrapped_result.result.message
        except Exception as error:
            success = False
            detail = f'sync result failed: {error}'
        self._finish_sync(success, detail)

    def _finish_sync(self, success: bool, detail: str) -> None:
        self._sync_goal_pending = False
        if success:
            self._sync_waiting_for_stop = True
            self._stationary_samples = 0
            self._core.detail = (
                'mission data synchronized; waiting for confirmed stop '
                'before low-speed reentry'
            )
        else:
            self._sync_waiting_for_stop = False
            self._core.finish_sync(False)
            self._core.detail = detail or self._core.detail
        self.get_logger().warning(
            f'Mission synchronization result: {self._core.detail}'
        )
        self._publish_classification_if_new()
        self._publish_status()

    def _odometry_callback(self, message: Odometry) -> None:
        self._latest_pose = message.pose.pose
        if not self._sync_waiting_for_stop:
            return
        twist = message.twist.twist
        speed = hypot(float(twist.linear.x), float(twist.linear.y))
        threshold = float(
            self.get_parameter('reentry_stationary_speed_mps').value
        )
        if speed <= threshold:
            self._stationary_samples += 1
        else:
            self._stationary_samples = 0
        required = int(
            self.get_parameter('reentry_stationary_samples').value
        )
        if self._stationary_samples < required:
            return
        self._sync_waiting_for_stop = False
        self._core.finish_sync(True)
        self._core.detail = 'robot stopped; ready for low-speed reentry test'
        self.get_logger().warning(self._core.detail)
        self._publish_status()

    def _publish_classification_if_new(self) -> None:
        cause = int(self._core.failure_cause)
        if cause == 0 or cause == self._last_published_failure_cause:
            return
        self._last_published_failure_cause = cause

        event_types = {
            1: 'communication_location_radio_shadow_candidate',
            2: 'communication_channel_anomaly_candidate',
            3: 'communication_total_link_failure_candidate',
            4: 'communication_transient_loss',
            5: 'mission_data_sync_failure',
            6: 'reentry_navigation_failure',
        }
        event = MissionEvent()
        now = self.get_clock().now()
        event.header.stamp = now.to_msg()
        event.header.frame_id = 'map'
        event.mission_id = self.get_parameter('mission_id').value
        event.event_id = (
            f'{event.mission_id}_comm_{cause}_{now.nanoseconds}'
        )
        event.event_type = event_types.get(
            cause, 'communication_unknown_candidate'
        )
        event.severity = (
            MissionEvent.CRITICAL
            if cause in (3, 5, 6)
            else MissionEvent.WARNING
        )
        if self._latest_pose is not None:
            event.pose = self._latest_pose
        event.source_id = self.get_name()
        event.description = self._core.detail
        event.confidence = self._core.failure_confidence
        self._mission_event_publisher.publish(event)

    def _publish_status(self) -> None:
        message = RecoveryStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.mission_id = self.get_parameter('mission_id').value
        message.state = int(self._core.state)
        message.detail = self._core.detail
        message.operator_attention_required = (
            self._core.state in (
                RecoveryState.CHANNEL_SWITCH,
                RecoveryState.CLASSIFYING,
                RecoveryState.SAFE_STOP,
            )
        )
        message.active_channel = self._core.active_channel
        message.channel_switch_attempts = (
            self._core.channel_switch_attempts
        )
        message.failure_cause = int(self._core.failure_cause)
        message.failure_confidence = self._core.failure_confidence
        self._status_publisher.publish(message)


def is_confirmed_stopped(message: RobotSafetyState) -> bool:
    return not message.walk_enabled and not message.motion_allowed


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CommunicationRecoveryManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

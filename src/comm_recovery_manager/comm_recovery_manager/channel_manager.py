import rclpy
from rclpy.node import Node
from vision60_msgs.msg import RecoveryEvent, RecoveryStatus
from vision60_msgs.srv import SwitchCommunicationChannel

from comm_recovery_manager.channel_core import ChannelPlan


class CommunicationChannelManager(Node):
    def __init__(self) -> None:
        super().__init__('communication_channel_manager')
        self.declare_parameter('mission_id', 'mission_001')
        self.declare_parameter(
            'candidate_channels',
            ['mock_unavailable', 'mock_backup_wifi'],
        )
        self.declare_parameter(
            'switch_service_name',
            '/communication/switch_channel',
        )

        self._plan = ChannelPlan(
            list(self.get_parameter('candidate_channels').value)
        )
        self._client = self.create_client(
            SwitchCommunicationChannel,
            self.get_parameter('switch_service_name').value,
        )
        self._event_publisher = self.create_publisher(
            RecoveryEvent,
            '/communication/recovery_event',
            10,
        )
        self._pending = False
        self._attempted_index = None
        self._requested_channel = ''
        self.create_subscription(
            RecoveryStatus,
            '/communication/recovery_status',
            self._status_callback,
            10,
        )

    def _status_callback(self, message: RecoveryStatus) -> None:
        if message.state != RecoveryStatus.CHANNEL_SWITCH:
            if not self._pending:
                self._attempted_index = None
            return
        if self._pending:
            return

        attempt = self._plan.attempt_for(
            int(message.channel_switch_attempts)
        )
        if attempt is None:
            if self._attempted_index == -1:
                return
            self._attempted_index = -1
            self._publish_result(
                False,
                '',
                'no untried alternate communication channel remains',
            )
            return
        if self._attempted_index == attempt.attempt_index:
            return
        if not self._client.service_is_ready():
            self.get_logger().warning(
                'Channel switch service unavailable; retrying'
            )
            return

        self._pending = True
        self._attempted_index = attempt.attempt_index
        self._requested_channel = attempt.channel
        request = SwitchCommunicationChannel.Request()
        request.mission_id = self.get_parameter('mission_id').value
        request.requested_channel = attempt.channel
        future = self._client.call_async(request)
        future.add_done_callback(self._switch_result)
        self.get_logger().warning(
            f'Trying alternate channel: {attempt.channel}'
        )

    def _switch_result(self, future) -> None:
        self._pending = False
        try:
            response = future.result()
        except Exception as error:
            self._publish_result(
                False,
                self._requested_channel,
                f'channel switch request failed: {error}',
            )
            return
        self._publish_result(
            bool(response.success),
            response.active_channel or self._requested_channel,
            response.message,
        )

    def _publish_result(
        self,
        success: bool,
        channel: str,
        detail: str,
    ) -> None:
        event = RecoveryEvent()
        event.header.stamp = self.get_clock().now().to_msg()
        event.header.frame_id = 'base_link'
        event.mission_id = self.get_parameter('mission_id').value
        event.event = (
            RecoveryEvent.CHANNEL_SWITCH_SUCCEEDED
            if success
            else RecoveryEvent.CHANNEL_SWITCH_FAILED
        )
        event.channel = channel
        event.detail = detail
        self._event_publisher.publish(event)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CommunicationChannelManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool
from vision60_msgs.msg import CommunicationState, RobotSafetyState
from vision60_msgs.srv import (
    RequestSafeStop,
    SwitchCommunicationChannel,
)

from vision60_mock.core import MockScenarioCore


class Vision60MockNode(Node):
    def __init__(self) -> None:
        super().__init__('vision60_mock')
        self.declare_parameter('mission_id', 'mission_001')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('speed_mps', 0.5)
        self.declare_parameter('degraded_after_s', 3.0)
        self.declare_parameter('disconnected_after_s', 5.0)
        self.declare_parameter('reconnected_after_s', -1.0)
        self.declare_parameter(
            'available_channels',
            ['mock_backup_wifi'],
        )
        self.declare_parameter('publish_robot_state', True)
        self.declare_parameter('external_odom_topic', '')
        self.declare_parameter('degraded_after_x_m', -1.0)
        self.declare_parameter('disconnected_after_x_m', -1.0)

        self._rate_hz = self.get_parameter('publish_rate_hz').value
        self._publish_robot_state = bool(
            self.get_parameter('publish_robot_state').value
        )
        self._scenario = MockScenarioCore(
            self.get_parameter('speed_mps').value,
            self.get_parameter('degraded_after_s').value,
            self.get_parameter('disconnected_after_s').value,
            self.get_parameter('reconnected_after_s').value,
        )
        self._start_ns = self.get_clock().now().nanoseconds
        self._last_heartbeat = self.get_clock().now().to_msg()
        self._forced_channel = ''
        self._external_x = None
        self._external_fault_latched = False
        self._communication_publisher = self.create_publisher(
            CommunicationState, '/communication/state', 10
        )
        self._odom_publisher = None
        self._stopped_publisher = None
        self._safety_publisher = None
        if self._publish_robot_state:
            self._odom_publisher = self.create_publisher(
                Odometry, '/slam/odom', 20
            )
            self._stopped_publisher = self.create_publisher(
                Bool, '/vision60/mock_stopped', 10
            )
            self._safety_publisher = self.create_publisher(
                RobotSafetyState, '/vision60/safety_state', 10
            )
            self.create_service(
                RequestSafeStop,
                '/vision60/request_safe_stop',
                self._safe_stop_callback,
            )
        else:
            self.get_logger().warning(
                'publish_robot_state=false: only /communication/state is '
                'published; vision60_bridge owns the robot safety state'
            )
        self.create_service(
            SwitchCommunicationChannel,
            '/communication/switch_channel',
            self._switch_channel_callback,
        )
        external_odom_topic = str(
            self.get_parameter('external_odom_topic').value
        )
        if external_odom_topic:
            self.create_subscription(
                Odometry,
                external_odom_topic,
                self._external_odom_callback,
                20,
            )
        self.create_timer(1.0 / self._rate_hz, self._tick)

    def _external_odom_callback(self, message: Odometry) -> None:
        self._external_x = float(message.pose.pose.position.x)

    def _tick(self) -> None:
        now = self.get_clock().now()
        elapsed_s = (now.nanoseconds - self._start_ns) / 1e9
        sample = self._scenario.advance(elapsed_s, 1.0 / self._rate_hz)
        external_link = self._external_link_state()
        if external_link is None:
            link_state = sample.communication_state
            link_connected = sample.connected
            packet_loss_ratio = sample.packet_loss_ratio
            latency_ms = sample.latency_ms
        else:
            (
                link_state,
                link_connected,
                packet_loss_ratio,
                latency_ms,
            ) = external_link

        if self._odom_publisher is not None:
            odometry = Odometry()
            odometry.header.stamp = now.to_msg()
            odometry.header.frame_id = 'map'
            odometry.child_frame_id = 'base_link'
            odometry.pose.pose.position.x = sample.x
            odometry.pose.pose.orientation.w = 1.0
            odometry.twist.twist.linear.x = (
                0.0 if self._scenario.stopped else self._scenario.speed_mps
            )
            self._odom_publisher.publish(odometry)

        communication = CommunicationState()
        communication.header.stamp = now.to_msg()
        communication.header.frame_id = 'base_link'
        communication.mission_id = self.get_parameter('mission_id').value
        channel_connected = bool(self._forced_channel)
        communication.state = (
            CommunicationState.NORMAL
            if channel_connected
            else link_state
        )
        communication.channel = (
            self._forced_channel or 'mock_ethernet'
        )
        communication.connected = (
            channel_connected or link_connected
        )
        communication.signal_strength_dbm = (
            -58.0 if communication.connected else -120.0
        )
        communication.snr_db = (
            22.0 if communication.connected else 0.0
        )
        communication.packet_loss_ratio = (
            0.02
            if channel_connected
            else packet_loss_ratio
        )
        communication.latency_ms = (
            35.0 if channel_connected else latency_ms
        )
        if communication.connected:
            self._last_heartbeat = now.to_msg()
        communication.last_heartbeat = self._last_heartbeat
        self._communication_publisher.publish(communication)

        if self._stopped_publisher is not None:
            stopped = Bool()
            stopped.data = self._scenario.stopped
            self._stopped_publisher.publish(stopped)

        if self._safety_publisher is not None:
            safety = RobotSafetyState()
            safety.header.stamp = now.to_msg()
            safety.header.frame_id = 'base_link'
            safety.walk_enabled = not self._scenario.stopped
            safety.emergency_stop = False
            safety.command_timed_out = False
            safety.localization_healthy = True
            safety.lidar_healthy = True
            safety.sdk_connected = True
            safety.motion_allowed = not self._scenario.stopped
            safety.stop_reason = (
                'communication link lost' if self._scenario.stopped else ''
            )
            self._safety_publisher.publish(safety)

    def _external_link_state(self):
        degraded_x = float(
            self.get_parameter('degraded_after_x_m').value
        )
        disconnected_x = float(
            self.get_parameter('disconnected_after_x_m').value
        )
        if (
            self._external_x is None
            or degraded_x < 0.0
            or disconnected_x < 0.0
        ):
            return None
        if self._external_x >= disconnected_x:
            self._external_fault_latched = True
        if self._external_fault_latched:
            return (CommunicationState.LOST, False, 1.0, 2000.0)
        if self._external_x >= degraded_x:
            return (CommunicationState.DEGRADED, True, 0.35, 650.0)
        return (CommunicationState.NORMAL, True, 0.01, 20.0)

    def _safe_stop_callback(self, request, response):
        self._scenario.request_stop()
        response.accepted = True
        response.message = f'mock robot stopped: {request.reason}'
        self.get_logger().warning(response.message)
        return response

    def _switch_channel_callback(self, request, response):
        available = set(
            self.get_parameter('available_channels').value
        )
        if request.requested_channel not in available:
            response.success = False
            response.active_channel = self._forced_channel
            response.message = (
                f'channel unavailable: {request.requested_channel}'
            )
            return response

        self._forced_channel = request.requested_channel
        response.success = True
        response.active_channel = self._forced_channel
        response.message = (
            f'mock channel activated: {self._forced_channel}'
        )
        self.get_logger().warning(response.message)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Vision60MockNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

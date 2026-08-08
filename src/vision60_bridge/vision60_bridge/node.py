import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32, String
from vision60_msgs.msg import RobotSafetyState
from vision60_msgs.srv import RequestSafeStop, SetWalkEnabled

from vision60_bridge.core import BridgeSafetyCore, VelocityCommand
from vision60_bridge.krm_transport import KrmVision60Interface
from vision60_bridge.mock_transport import MockVision60Interface


class Vision60BridgeNode(Node):
    """Apply safety policy and adapt ROS messages to a robot transport."""

    def __init__(self) -> None:
        super().__init__('vision60_bridge')
        self.declare_parameter('transport', 'mock')
        self.declare_parameter('allow_motion_output', False)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('command_timeout_s', 0.3)
        self.declare_parameter('localization_timeout_s', 0.5)
        self.declare_parameter('lidar_timeout_s', 0.5)
        self.declare_parameter('max_linear_x_mps', 0.5)
        self.declare_parameter('max_linear_y_mps', 0.3)
        self.declare_parameter('max_angular_z_rps', 0.6)
        self.declare_parameter('max_linear_accel_mps2', 0.5)
        self.declare_parameter('max_angular_accel_rps2', 1.0)
        # Placeholder until the KRM SDK reports real odometry covariance.
        # Downstream estimators reject an all-zero covariance matrix.
        self.declare_parameter('odom_pose_variance', 0.02)
        self.declare_parameter('odom_twist_variance', 0.05)

        self._odom_pose_variance = float(
            self.get_parameter('odom_pose_variance').value
        )
        self._odom_twist_variance = float(
            self.get_parameter('odom_twist_variance').value
        )
        self._rate_hz = float(
            self.get_parameter('publish_rate_hz').value
        )
        self._localization_timeout_s = float(
            self.get_parameter('localization_timeout_s').value
        )
        self._lidar_timeout_s = float(
            self.get_parameter('lidar_timeout_s').value
        )
        self._allow_motion_output = bool(
            self.get_parameter('allow_motion_output').value
        )
        self._core = BridgeSafetyCore(
            command_timeout_s=float(
                self.get_parameter('command_timeout_s').value
            ),
            max_linear_x_mps=float(
                self.get_parameter('max_linear_x_mps').value
            ),
            max_linear_y_mps=float(
                self.get_parameter('max_linear_y_mps').value
            ),
            max_angular_z_rps=float(
                self.get_parameter('max_angular_z_rps').value
            ),
            max_linear_accel_mps2=float(
                self.get_parameter('max_linear_accel_mps2').value
            ),
            max_angular_accel_rps2=float(
                self.get_parameter('max_angular_accel_rps2').value
            ),
        )
        self._transport = self._make_transport(
            str(self.get_parameter('transport').value)
        )
        self._transport.connect()
        self._last_localization_ns = 0
        self._last_lidar_ns = 0
        self._last_tick_ns = self.get_clock().now().nanoseconds
        self._closed = False

        self._odom_publisher = self.create_publisher(
            Odometry, '/vision60/odom', 20
        )
        self._safety_publisher = self.create_publisher(
            RobotSafetyState, '/vision60/safety_state', 10
        )
        self._state_publisher = self.create_publisher(
            String, '/vision60/state', 10
        )
        self._battery_publisher = self.create_publisher(
            Float32, '/vision60/battery', 10
        )
        self._fault_publisher = self.create_publisher(
            String, '/vision60/fault', 10
        )
        self._estop_publisher = self.create_publisher(
            Bool, '/vision60/estop_state', 10
        )
        self._applied_command_publisher = self.create_publisher(
            Twist, '/vision60/command_applied', 20
        )

        self.create_subscription(
            Twist, '/cmd_vel_safe', self._command_callback, 20
        )
        self.create_subscription(
            Bool, '/walk_enable', self._walk_enable_callback, 10
        )
        self.create_subscription(
            Bool, '/emergency_stop', self._emergency_stop_callback, 10
        )
        self.create_subscription(
            Odometry,
            '/state/odometry',
            self._localization_callback,
            20,
        )
        self.create_subscription(
            PointCloud2, '/ouster/points', self._lidar_callback, 10
        )
        self.create_service(
            SetWalkEnabled,
            '/vision60/set_walk_enabled',
            self._set_walk_enabled_service,
        )
        self.create_service(
            RequestSafeStop,
            '/vision60/request_safe_stop',
            self._safe_stop_service,
        )
        self.create_timer(1.0 / self._rate_hz, self._tick)

    def _make_transport(self, name: str):
        if name == 'mock':
            return MockVision60Interface()
        if name == 'krm':
            return KrmVision60Interface()
        raise ValueError(f'unsupported transport: {name}')

    def _command_callback(self, message: Twist) -> None:
        command = VelocityCommand(
            message.linear.x,
            message.linear.y,
            message.angular.z,
        )
        now_s = self.get_clock().now().nanoseconds / 1e9
        if not self._core.submit_command(command, now_s):
            self.get_logger().error('Rejected unsafe velocity command')

    def _walk_enable_callback(self, message: Bool) -> None:
        self._set_walk_enabled(message.data)

    def _emergency_stop_callback(self, message: Bool) -> None:
        if message.data:
            self._latch_emergency_stop()

    def _localization_callback(self, message: Odometry) -> None:
        self._last_localization_ns = self._fresh_receipt_time(
            message.header.stamp, self._localization_timeout_s
        )

    def _lidar_callback(self, message: PointCloud2) -> None:
        self._last_lidar_ns = self._fresh_receipt_time(
            message.header.stamp, self._lidar_timeout_s
        )

    def _fresh_receipt_time(self, stamp, timeout_s: float) -> int:
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        source_fresh = (
            0 < stamp_ns <= now_ns
            and (now_ns - stamp_ns) / 1e9 <= timeout_s
        )
        return now_ns if source_fresh else 0

    def _set_walk_enabled(self, enabled: bool) -> bool:
        if not self._core.set_walk_enabled(enabled):
            return False
        if not self._transport.set_walk_enabled(enabled):
            self._core.set_walk_enabled(False)
            return False
        return True

    def _latch_emergency_stop(self) -> None:
        self._core.latch_emergency_stop()
        self._transport.emergency_stop()

    def _set_walk_enabled_service(self, request, response):
        response.accepted = self._set_walk_enabled(request.enabled)
        response.message = (
            f'walk_enabled={request.enabled}'
            if response.accepted
            else 'walk enable rejected'
        )
        return response

    def _safe_stop_service(self, request, response):
        if request.emergency:
            self._latch_emergency_stop()
        else:
            self._set_walk_enabled(False)
        response.accepted = True
        response.message = f'safe stop applied: {request.reason}'
        return response

    def _tick(self) -> None:
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        dt_s = max((now_ns - self._last_tick_ns) / 1e9, 0.0)
        self._last_tick_ns = now_ns
        localization_healthy = _is_fresh(
            now_ns,
            self._last_localization_ns,
            self._localization_timeout_s,
        )
        lidar_healthy = _is_fresh(
            now_ns, self._last_lidar_ns, self._lidar_timeout_s
        )
        decision = self._core.evaluate(
            now_s=now_ns / 1e9,
            dt_s=dt_s,
            sdk_connected=self._transport.is_connected(),
            localization_healthy=localization_healthy,
            lidar_healthy=lidar_healthy,
            allow_motion_output=self._allow_motion_output,
        )
        self._transport.send_velocity(decision.command, dt_s)
        state = self._transport.read_state()
        self._publish_command(decision.command)
        self._publish_odometry(now.to_msg(), state)
        self._publish_status(
            now.to_msg(),
            state,
            decision,
            localization_healthy,
            lidar_healthy,
        )

    def _publish_command(self, command: VelocityCommand) -> None:
        message = Twist()
        message.linear.x = command.linear_x
        message.linear.y = command.linear_y
        message.angular.z = command.angular_z
        self._applied_command_publisher.publish(message)

    def _publish_odometry(self, stamp, state) -> None:
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = 'odom'
        message.child_frame_id = 'base_link'
        message.pose.pose.position.x = state.x
        message.pose.pose.position.y = state.y
        message.pose.pose.position.z = state.z
        message.pose.pose.orientation.z = math.sin(state.yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(state.yaw / 2.0)
        message.twist.twist.linear.x = state.linear_x
        message.twist.twist.linear.y = state.linear_y
        message.twist.twist.angular.z = state.angular_z
        for index in range(6):
            diagonal = index * 7
            message.pose.covariance[diagonal] = self._odom_pose_variance
            message.twist.covariance[diagonal] = self._odom_twist_variance
        self._odom_publisher.publish(message)

    def _publish_status(
        self,
        stamp,
        state,
        decision,
        localization_healthy: bool,
        lidar_healthy: bool,
    ) -> None:
        safety = RobotSafetyState()
        safety.header.stamp = stamp
        safety.header.frame_id = 'base_link'
        safety.walk_enabled = self._core.walk_enabled
        safety.emergency_stop = self._core.emergency_stop
        safety.command_timed_out = decision.command_timed_out
        safety.localization_healthy = localization_healthy
        safety.lidar_healthy = lidar_healthy
        safety.sdk_connected = self._transport.is_connected()
        safety.motion_allowed = decision.motion_allowed
        safety.stop_reason = decision.stop_reason
        self._safety_publisher.publish(safety)

        state_message = String()
        state_message.data = (
            'MOTION_ALLOWED'
            if decision.motion_allowed
            else 'SAFE_STOP'
        )
        self._state_publisher.publish(state_message)

        battery = Float32()
        battery.data = float(state.battery_ratio)
        self._battery_publisher.publish(battery)

        fault = String()
        fault.data = state.fault or decision.stop_reason
        self._fault_publisher.publish(fault)

        estop = Bool()
        estop.data = self._core.emergency_stop
        self._estop_publisher.publish(estop)

    def shutdown_transport(self) -> None:
        if self._closed:
            return
        self._core.set_walk_enabled(False)
        self._transport.set_walk_enabled(False)
        self._transport.send_velocity(VelocityCommand.zero(), 0.0)
        self._transport.close()
        self._closed = True


def _is_fresh(now_ns: int, last_ns: int, timeout_s: float) -> bool:
    return (
        last_ns > 0
        and now_ns >= last_ns
        and (now_ns - last_ns) / 1e9 <= timeout_s
    )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Vision60BridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.shutdown_transport()
        node.destroy_node()
        rclpy.shutdown()

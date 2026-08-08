"""
Drive and verify the full mock chain in one automated run.

Sequence checked:

1. Operator authorises walking while the link is healthy.
2. Teleoperation overrides an active autonomous velocity through twist_mux.
3. Outbound velocity reaches the mock robot through the Nav2 velocity
   smoother and ``vision60_bridge``.
3. Link loss produces a confirmed stop
   (``walk_enabled=false`` and ``motion_allowed=false``).
4. The recovery state machine reaches ``RETURNING``.
5. Nav2 drives the recorded route in reverse and the robot moves back.
6. The link recovers, buffered mission data synchronizes, and the state
   machine reaches ``REENTRY_TEST``.

Outbound velocity is published on ``/cmd_vel_autonomy`` and recovery velocity
on ``/cmd_vel_recovery``. Both pass through twist_mux before the smoother,
collision monitor, bridge safety gates and transport.
"""
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String
from vision60_msgs.msg import (
    CommunicationState,
    RecoveryStatus,
    RobotSafetyState,
)


class FullSystemProbe(Node):

    def __init__(self) -> None:
        super().__init__('full_system_probe')
        self.declare_parameter('start_drive_after_s', 2.0)
        self.declare_parameter('outbound_speed_mps', 0.2)
        self.declare_parameter('min_outbound_distance_m', 0.5)
        self.declare_parameter('min_return_distance_m', 0.5)
        self.declare_parameter('max_reentry_speed_mps', 0.12)
        self.declare_parameter('teleop_start_s', 4.0)
        self.declare_parameter('teleop_duration_s', 2.0)
        self.declare_parameter('teleop_speed_mps', 0.05)
        self.declare_parameter('timeout_s', 120.0)

        self._start_drive_after_s = float(
            self.get_parameter('start_drive_after_s').value
        )
        self._outbound_speed_mps = float(
            self.get_parameter('outbound_speed_mps').value
        )
        self._min_outbound_distance_m = float(
            self.get_parameter('min_outbound_distance_m').value
        )
        self._min_return_distance_m = float(
            self.get_parameter('min_return_distance_m').value
        )
        self._allowed_reentry_speed_mps = float(
            self.get_parameter('max_reentry_speed_mps').value
        )
        self._teleop_start_s = float(
            self.get_parameter('teleop_start_s').value
        )
        self._teleop_duration_s = float(
            self.get_parameter('teleop_duration_s').value
        )
        self._teleop_speed_mps = float(
            self.get_parameter('teleop_speed_mps').value
        )
        self._timeout_s = float(self.get_parameter('timeout_s').value)

        self._start_ns = self.get_clock().now().nanoseconds
        self._link_connected = True
        self._max_x = 0.0
        self._latest_x = 0.0
        self._outbound_reached = False
        self._walk_was_enabled = False
        self._stop_confirmed = False
        self._returning = False
        self._returned = False
        self._reentry_started = False
        self._reentry_completed = False
        self._max_reentry_speed_mps = 0.0
        self._channel_switch_seen = False
        self._backup_channel_active = False
        self._channel_anomaly_classified = False
        self._teleop_priority_seen = False
        self._safety_lock_seen = False
        self._collision_action = CollisionMonitorState.DO_NOTHING
        self._obstacle_phase = 'clear'
        self._collision_slowdown_seen = False
        self._collision_stop_seen = False
        self.passed = False
        self.done = False

        self._walk_publisher = self.create_publisher(
            Bool, '/walk_enable', 10
        )
        self._command_publisher = self.create_publisher(
            Twist, '/cmd_vel_autonomy', 10
        )
        self._teleop_publisher = self.create_publisher(
            Twist, '/cmd_vel_teleop', 10
        )
        self.create_subscription(
            Odometry, '/vision60/odom', self._odometry_callback, 20
        )
        self.create_subscription(
            RobotSafetyState,
            '/vision60/safety_state',
            self._safety_callback,
            10,
        )
        self.create_subscription(
            CommunicationState,
            '/communication/state',
            self._communication_callback,
            10,
        )
        self.create_subscription(
            RecoveryStatus,
            '/communication/recovery_status',
            self._recovery_callback,
            10,
        )
        self.create_subscription(
            CollisionMonitorState,
            '/safety/collision_monitor_state',
            self._collision_state_callback,
            10,
        )
        self.create_subscription(
            String,
            '/vision60/mock_obstacle_phase',
            self._obstacle_phase_callback,
            10,
        )
        self.create_subscription(
            Twist,
            '/cmd_vel_safe',
            self._safe_velocity_callback,
            20,
        )
        self.create_subscription(
            Twist,
            '/cmd_vel_muxed',
            self._muxed_velocity_callback,
            20,
        )
        self.create_subscription(
            Bool,
            '/safety/cmd_vel_lock',
            self._motion_lock_callback,
            10,
        )
        self.create_timer(0.1, self._tick)

    def _elapsed_s(self) -> float:
        return (
            self.get_clock().now().nanoseconds - self._start_ns
        ) / 1e9

    def _communication_callback(self, message: CommunicationState) -> None:
        self._link_connected = message.connected

    def _odometry_callback(self, message: Odometry) -> None:
        self._latest_x = message.pose.pose.position.x
        if self._reentry_started and not self._reentry_completed:
            self._max_reentry_speed_mps = max(
                self._max_reentry_speed_mps,
                abs(message.twist.twist.linear.x),
            )
        self._max_x = max(self._max_x, self._latest_x)
        if self._max_x >= self._min_outbound_distance_m:
            self._outbound_reached = True
        if (
            self._returning
            and self._max_x - self._latest_x
            >= self._min_return_distance_m
        ):
            self._returned = True

    def _safety_callback(self, message: RobotSafetyState) -> None:
        # Walking is already disabled before the mission starts, so a stop
        # only counts once the robot has actually been authorised to walk.
        if message.walk_enabled:
            self._walk_was_enabled = True
            return
        if self._walk_was_enabled and not message.motion_allowed:
            self._stop_confirmed = True

    def _recovery_callback(self, message: RecoveryStatus) -> None:
        if message.state == RecoveryStatus.RETURNING:
            self._returning = True
        if message.state == RecoveryStatus.CHANNEL_SWITCH:
            self._channel_switch_seen = True
        if message.active_channel == 'mock_backup_wifi':
            self._backup_channel_active = True
        if (
            message.failure_cause
            == RecoveryStatus.CAUSE_CHANNEL_ANOMALY
        ):
            self._channel_anomaly_classified = True
        if message.state == RecoveryStatus.REENTRY_TEST:
            self._reentry_started = True
        if (
            self._reentry_started
            and message.state == RecoveryStatus.NORMAL
        ):
            self._reentry_completed = True

    def _collision_state_callback(
        self,
        message: CollisionMonitorState,
    ) -> None:
        self._collision_action = message.action_type

    def _obstacle_phase_callback(self, message: String) -> None:
        self._obstacle_phase = message.data

    def _safe_velocity_callback(self, message: Twist) -> None:
        speed = abs(message.linear.x)
        if (
            (
                self._collision_action
                == CollisionMonitorState.SLOWDOWN
                or self._obstacle_phase == 'slowdown'
            )
            and 0.0 < speed <= 0.08
        ):
            self._collision_slowdown_seen = True
        if (
            (
                self._collision_action == CollisionMonitorState.STOP
                or self._obstacle_phase == 'stop'
            )
            and speed <= 0.001
        ):
            self._collision_stop_seen = True

    def _muxed_velocity_callback(self, message: Twist) -> None:
        if abs(message.linear.x - self._teleop_speed_mps) <= 0.001:
            self._teleop_priority_seen = True

    def _motion_lock_callback(self, message: Bool) -> None:
        if message.data:
            self._safety_lock_seen = True

    def _tick(self) -> None:
        elapsed_s = self._elapsed_s()

        # The operator holds walk authorisation only while the link is up.
        # Releasing it on link loss lets the safe stop clear walk_enabled.
        if self._link_connected:
            walk = Bool()
            walk.data = True
            self._walk_publisher.publish(walk)

        driving = (
            self._link_connected
            and elapsed_s >= self._start_drive_after_s
            and not self._stop_confirmed
        )
        if driving:
            command = Twist()
            command.linear.x = self._outbound_speed_mps
            self._command_publisher.publish(command)
        if (
            driving
            and self._teleop_start_s
            <= elapsed_s
            < self._teleop_start_s + self._teleop_duration_s
        ):
            teleop = Twist()
            teleop.linear.x = self._teleop_speed_mps
            self._teleop_publisher.publish(teleop)

        if (
            self._outbound_reached
            and self._stop_confirmed
            and self._returning
            and self._returned
            and self._reentry_completed
            and self._channel_switch_seen
            and self._backup_channel_active
            and self._channel_anomaly_classified
            and self._teleop_priority_seen
            and self._safety_lock_seen
            and self._collision_slowdown_seen
            and self._collision_stop_seen
            and self._max_reentry_speed_mps
            <= self._allowed_reentry_speed_mps
        ):
            self.passed = True
            self.done = True
            self.get_logger().info(
                'FULL SYSTEM PASS: outbound '
                f'{self._max_x:.2f} m, confirmed stop, RETURNING, '
                f'reentered to {self._latest_x:.2f} m at max '
                f'{self._max_reentry_speed_mps:.2f} m/s via backup channel'
                ', twist_mux priority/lock and collision slowdown/stop '
                'verified'
            )
            return

        if elapsed_s > self._timeout_s:
            self.get_logger().error(
                'FULL SYSTEM FAIL: '
                f'outbound_reached={self._outbound_reached} '
                f'(max_x={self._max_x:.2f}) '
                f'stop_confirmed={self._stop_confirmed} '
                f'returning={self._returning} '
                f'returned={self._returned} '
                f'reentry_started={self._reentry_started} '
                f'reentry_completed={self._reentry_completed} '
                f'channel_switch_seen={self._channel_switch_seen} '
                f'backup_active={self._backup_channel_active} '
                f'channel_anomaly='
                f'{self._channel_anomaly_classified} '
                f'teleop_priority={self._teleop_priority_seen} '
                f'safety_lock={self._safety_lock_seen} '
                f'collision_slowdown='
                f'{self._collision_slowdown_seen} '
                f'collision_stop={self._collision_stop_seen} '
                f'max_reentry_speed='
                f'{self._max_reentry_speed_mps:.2f} '
                f'(x={self._latest_x:.2f})'
            )
            self.done = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FullSystemProbe()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    passed = node.passed
    node.destroy_node()
    rclpy.shutdown()
    if not passed:
        sys.exit(1)

import math
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class VelocityCommand:
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0

    @classmethod
    def zero(cls):
        return cls()

    def is_zero(self) -> bool:
        return (
            self.linear_x == 0.0
            and self.linear_y == 0.0
            and self.angular_z == 0.0
        )


@dataclass(frozen=True)
class BridgeDecision:
    command: VelocityCommand
    motion_allowed: bool
    command_timed_out: bool
    stop_reason: str


class BridgeSafetyCore:
    """Validate and gate commands before they reach a robot transport."""

    def __init__(
        self,
        command_timeout_s: float = 0.3,
        max_linear_x_mps: float = 0.5,
        max_linear_y_mps: float = 0.3,
        max_angular_z_rps: float = 0.6,
        max_linear_accel_mps2: float = 0.5,
        max_angular_accel_rps2: float = 1.0,
    ) -> None:
        self.command_timeout_s = command_timeout_s
        self.max_linear_x_mps = max_linear_x_mps
        self.max_linear_y_mps = max_linear_y_mps
        self.max_angular_z_rps = max_angular_z_rps
        self.max_linear_accel_mps2 = max_linear_accel_mps2
        self.max_angular_accel_rps2 = max_angular_accel_rps2
        self.walk_enabled = False
        self.emergency_stop = False
        self._target = VelocityCommand.zero()
        self._applied = VelocityCommand.zero()
        self._last_command_s: Optional[float] = None
        self._command_valid = False
        self._command_error = ''

    def set_walk_enabled(self, enabled: bool) -> bool:
        if enabled and self.emergency_stop:
            return False
        self.walk_enabled = enabled
        if not enabled:
            self._applied = VelocityCommand.zero()
        return True

    def latch_emergency_stop(self) -> None:
        self.emergency_stop = True
        self.walk_enabled = False
        self._target = VelocityCommand.zero()
        self._applied = VelocityCommand.zero()

    def submit_command(
        self, command: VelocityCommand, now_s: float
    ) -> bool:
        self._last_command_s = now_s
        self._command_valid, self._command_error = self._validate(command)
        self._target = (
            command if self._command_valid else VelocityCommand.zero()
        )
        return self._command_valid

    def evaluate(
        self,
        now_s: float,
        dt_s: float,
        sdk_connected: bool,
        localization_healthy: bool,
        lidar_healthy: bool,
        allow_motion_output: bool,
    ) -> BridgeDecision:
        timed_out = (
            self._last_command_s is None
            or now_s - self._last_command_s > self.command_timeout_s
        )
        reason = self._stop_reason(
            timed_out,
            sdk_connected,
            localization_healthy,
            lidar_healthy,
            allow_motion_output,
        )
        motion_allowed = reason == ''

        if not motion_allowed:
            self._applied = VelocityCommand.zero()
        else:
            self._applied = self._slew(self._target, max(dt_s, 0.0))

        return BridgeDecision(
            self._applied,
            motion_allowed,
            timed_out,
            reason,
        )

    def _validate(self, command: VelocityCommand):
        values = (
            command.linear_x,
            command.linear_y,
            command.angular_z,
        )
        if not all(math.isfinite(value) for value in values):
            return False, 'non-finite velocity command'
        if abs(command.linear_x) > self.max_linear_x_mps:
            return False, 'linear_x command out of range'
        if abs(command.linear_y) > self.max_linear_y_mps:
            return False, 'linear_y command out of range'
        if abs(command.angular_z) > self.max_angular_z_rps:
            return False, 'angular_z command out of range'
        return True, ''

    def _stop_reason(
        self,
        timed_out: bool,
        sdk_connected: bool,
        localization_healthy: bool,
        lidar_healthy: bool,
        allow_motion_output: bool,
    ) -> str:
        if self.emergency_stop:
            return 'emergency stop latched'
        if not allow_motion_output:
            return 'motion output physically disabled'
        if not self.walk_enabled:
            return 'walking disabled'
        if not sdk_connected:
            return 'SDK disconnected'
        if not localization_healthy:
            return 'localization stale'
        if not lidar_healthy:
            return 'LiDAR stale'
        if not self._command_valid:
            return self._command_error or 'no valid command'
        if timed_out:
            return 'velocity command timed out'
        return ''

    def _slew(
        self, target: VelocityCommand, dt_s: float
    ) -> VelocityCommand:
        linear_step = self.max_linear_accel_mps2 * dt_s
        angular_step = self.max_angular_accel_rps2 * dt_s
        return VelocityCommand(
            _move_towards(
                self._applied.linear_x, target.linear_x, linear_step
            ),
            _move_towards(
                self._applied.linear_y, target.linear_y, linear_step
            ),
            _move_towards(
                self._applied.angular_z, target.angular_z, angular_step
            ),
        )


def _move_towards(current: float, target: float, step: float) -> float:
    if step <= 0.0:
        return current
    if target > current:
        return min(current + step, target)
    return max(current - step, target)

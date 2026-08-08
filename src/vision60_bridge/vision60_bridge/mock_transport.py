import math

from vision60_bridge.core import VelocityCommand
from vision60_bridge.interface import RobotState, Vision60Interface


class MockVision60Interface(Vision60Interface):
    """In-memory transport used while the KRM SDK is unavailable."""

    def __init__(self) -> None:
        self.connected = False
        self.walk_enabled = False
        self.estop = False
        self.command_log = []
        self._state = RobotState()

    def connect(self) -> bool:
        self.connected = True
        return True

    def is_connected(self) -> bool:
        return self.connected

    def set_walk_enabled(self, enabled: bool) -> bool:
        if self.estop and enabled:
            return False
        self.walk_enabled = enabled
        return True

    def emergency_stop(self) -> bool:
        self.estop = True
        self.walk_enabled = False
        return True

    def send_velocity(
        self, command: VelocityCommand, dt_s: float
    ) -> bool:
        if not self.connected:
            return False
        applied = (
            command
            if self.walk_enabled and not self.estop
            else VelocityCommand.zero()
        )
        self.command_log.append(applied)
        yaw = self._state.yaw + applied.angular_z * dt_s
        x = self._state.x + (
            applied.linear_x * math.cos(yaw)
            - applied.linear_y * math.sin(yaw)
        ) * dt_s
        y = self._state.y + (
            applied.linear_x * math.sin(yaw)
            + applied.linear_y * math.cos(yaw)
        ) * dt_s
        self._state = RobotState(
            x=x,
            y=y,
            z=self._state.z,
            yaw=yaw,
            linear_x=applied.linear_x,
            linear_y=applied.linear_y,
            angular_z=applied.angular_z,
            battery_ratio=self._state.battery_ratio,
            fault=self._state.fault,
        )
        return True

    def read_state(self) -> RobotState:
        return self._state

    def close(self) -> None:
        if self.connected:
            self.send_velocity(VelocityCommand.zero(), 0.0)
        self.walk_enabled = False
        self.connected = False

from vision60_bridge.core import VelocityCommand
from vision60_bridge.interface import RobotState, Vision60Interface


class KrmVision60Interface(Vision60Interface):
    """Fail-closed placeholder until the KRM SDK contract is received."""

    def connect(self) -> bool:
        raise RuntimeError('KRM SDK transport is not implemented')

    def is_connected(self) -> bool:
        return False

    def set_walk_enabled(self, enabled: bool) -> bool:
        return False

    def emergency_stop(self) -> bool:
        return False

    def send_velocity(
        self, command: VelocityCommand, dt_s: float
    ) -> bool:
        return False

    def read_state(self) -> RobotState:
        return RobotState(fault='KRM SDK transport is not implemented')

    def close(self) -> None:
        return None

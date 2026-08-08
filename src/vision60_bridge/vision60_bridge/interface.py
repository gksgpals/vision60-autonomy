from abc import ABC, abstractmethod
from dataclasses import dataclass

from vision60_bridge.core import VelocityCommand


@dataclass(frozen=True)
class RobotState:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0
    battery_ratio: float = 1.0
    fault: str = ''


class Vision60Interface(ABC):
    """Transport boundary replaced by the KRM SDK implementation."""

    @abstractmethod
    def connect(self) -> bool:
        """Connect to the robot control transport."""

    @abstractmethod
    def is_connected(self) -> bool:
        """Return whether command and state transport is connected."""

    @abstractmethod
    def set_walk_enabled(self, enabled: bool) -> bool:
        """Enable or disable walking."""

    @abstractmethod
    def emergency_stop(self) -> bool:
        """Latch emergency stop at the transport layer."""

    @abstractmethod
    def send_velocity(
        self, command: VelocityCommand, dt_s: float
    ) -> bool:
        """Send one body velocity command."""

    @abstractmethod
    def read_state(self) -> RobotState:
        """Read the latest robot state."""

    @abstractmethod
    def close(self) -> None:
        """Stop and close the transport."""

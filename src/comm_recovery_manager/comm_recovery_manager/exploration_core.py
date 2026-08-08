"""Fail-closed policy for pausing and resuming frontier exploration."""


class ExplorationGateCore:
    """Allow new frontier goals only during safe mission states."""

    NORMAL = 1
    DEGRADED = 2

    def __init__(self) -> None:
        self.recovery_state = None
        self.walk_enabled = False
        self.motion_allowed = False
        self.emergency_stop = True
        self.safety_received = False

    def observe_recovery(self, state: int) -> None:
        """Store the latest communication-recovery state."""
        self.recovery_state = int(state)

    def observe_safety(
        self,
        walk_enabled: bool,
        motion_allowed: bool,
        emergency_stop: bool,
    ) -> None:
        """Store the latest robot safety state."""
        self.walk_enabled = bool(walk_enabled)
        self.motion_allowed = bool(motion_allowed)
        self.emergency_stop = bool(emergency_stop)
        self.safety_received = True

    @property
    def exploration_allowed(self) -> bool:
        """Return true only when autonomous frontier goals are safe."""
        link_state_allows_exploration = self.recovery_state in (
            self.NORMAL,
            self.DEGRADED,
        )
        return bool(
            self.safety_received
            and link_state_allows_exploration
            and self.walk_enabled
            and self.motion_allowed
            and not self.emergency_stop
        )

    @property
    def reason(self) -> str:
        """Explain the active gate decision for logs and operators."""
        if not self.safety_received:
            return 'waiting for robot safety state'
        if self.emergency_stop:
            return 'emergency stop active'
        if not self.walk_enabled or not self.motion_allowed:
            return 'robot motion is not permitted'
        if self.recovery_state not in (self.NORMAL, self.DEGRADED):
            return 'communication recovery owns robot motion'
        return 'frontier exploration permitted'

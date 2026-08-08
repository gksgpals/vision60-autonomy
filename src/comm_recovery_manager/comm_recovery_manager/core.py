from enum import IntEnum
from typing import Optional


class RecoveryState(IntEnum):
    IDLE = 0
    NORMAL = 1
    DEGRADED = 2
    LINK_LOST = 3
    STOPPING = 4
    RETURNING = 5
    LINK_RECOVERED = 6
    SYNCING = 7
    REENTRY_TEST = 8
    SAFE_STOP = 9
    CHANNEL_SWITCH = 10
    CLASSIFYING = 11


class FailureCause(IntEnum):
    UNKNOWN = 0
    LOCATION_RADIO_SHADOW = 1
    CHANNEL_ANOMALY = 2
    TOTAL_LINK_FAILURE = 3
    TRANSIENT_NETWORK_LOSS = 4
    DATA_SYNC_FAILURE = 5
    REENTRY_NAVIGATION_FAILURE = 6


class RecoveryEventType(IntEnum):
    RETURN_SUCCEEDED = 1
    RETURN_FAILED = 2
    SYNC_STARTED = 3
    SYNC_SUCCEEDED = 4
    SYNC_FAILED = 5
    REENTRY_SUCCEEDED = 6
    REENTRY_LINK_LOST = 7
    CHANNEL_SWITCH_SUCCEEDED = 8
    CHANNEL_SWITCH_FAILED = 9
    CLASSIFICATION_RECORDED = 10
    REENTRY_FAILED = 11


class InvalidTransition(RuntimeError):
    pass


class RecoveryManagerCore:
    def __init__(
        self,
        lost_timeout_s: float = 2.0,
        degraded_packet_loss_ratio: float = 0.20,
        degraded_latency_ms: float = 500.0,
        max_channel_switch_attempts: int = 2,
    ) -> None:
        self.lost_timeout_s = lost_timeout_s
        self.degraded_packet_loss_ratio = degraded_packet_loss_ratio
        self.degraded_latency_ms = degraded_latency_ms
        self.max_channel_switch_attempts = max_channel_switch_attempts
        self.state = RecoveryState.NORMAL
        self._disconnected_since: Optional[float] = None
        self.link_connected = True
        self.active_channel = ''
        self.original_channel = ''
        self.channel_switch_attempts = 0
        self.failure_cause = FailureCause.UNKNOWN
        self.failure_confidence = 0.0
        self.detail = 'communication healthy'

    def observe_link(
        self,
        connected: bool,
        packet_loss_ratio: float,
        latency_ms: float,
        now_s: float,
        channel: str = '',
    ) -> RecoveryState:
        self.link_connected = connected
        if channel:
            self.active_channel = channel
            if not self.original_channel:
                self.original_channel = channel

        if self.state not in (
            RecoveryState.NORMAL,
            RecoveryState.DEGRADED,
            RecoveryState.LINK_LOST,
        ):
            return self.state

        if connected:
            self._disconnected_since = None
            degraded = (
                packet_loss_ratio >= self.degraded_packet_loss_ratio
                or latency_ms >= self.degraded_latency_ms
            )
            self.state = (
                RecoveryState.DEGRADED
                if degraded
                else RecoveryState.NORMAL
            )
            self.detail = (
                'communication quality degraded'
                if degraded
                else 'communication healthy'
            )
            return self.state

        if self._disconnected_since is None:
            self._disconnected_since = now_s

        elapsed = now_s - self._disconnected_since
        if elapsed >= self.lost_timeout_s:
            self.state = RecoveryState.LINK_LOST
            self.detail = 'communication loss confirmed'
        else:
            self.state = RecoveryState.DEGRADED
            self.detail = 'waiting for link-loss confirmation'
        return self.state

    def start_stopping(self) -> None:
        self._transition(
            RecoveryState.LINK_LOST,
            RecoveryState.STOPPING,
            'stopping before route recovery',
        )

    def confirm_stopped(self) -> None:
        self._transition(
            RecoveryState.STOPPING,
            RecoveryState.RETURNING,
            'following recorded route to recovery waypoint',
        )

    def confirm_link_recovered(self) -> None:
        self.link_connected = True
        self._transition(
            RecoveryState.RETURNING,
            RecoveryState.LINK_RECOVERED,
            'communication restored',
        )

    def finish_return(self) -> None:
        if self.state != RecoveryState.RETURNING:
            raise InvalidTransition(
                f'expected RETURNING, current {self.state.name}'
            )
        if self.link_connected:
            self.state = RecoveryState.LINK_RECOVERED
            self.detail = 'communication restored at recovery waypoint'
            return
        self.state = RecoveryState.CHANNEL_SWITCH
        self.detail = 'primary channel unavailable; alternate channel required'

    def start_sync(self) -> None:
        self._transition(
            RecoveryState.LINK_RECOVERED,
            RecoveryState.SYNCING,
            'synchronizing onboard mission data',
        )

    def finish_sync(self, success: bool) -> None:
        if not success:
            self.failure_cause = FailureCause.DATA_SYNC_FAILURE
            self.failure_confidence = 1.0
            self.fail('mission data synchronization failed')
            return
        self._transition(
            RecoveryState.SYNCING,
            RecoveryState.REENTRY_TEST,
            'ready for low-speed reentry test',
        )

    def finish_reentry(self, success: bool) -> None:
        if not success:
            self.state = RecoveryState.CLASSIFYING
            self.failure_cause = FailureCause.LOCATION_RADIO_SHADOW
            self.failure_confidence = 0.8
            self.detail = (
                'link loss repeated during low-speed reentry; '
                'location-based radio shadow suspected'
            )
            return
        self._transition(
            RecoveryState.REENTRY_TEST,
            RecoveryState.NORMAL,
            'reentry test completed',
        )
        self.failure_cause = FailureCause.TRANSIENT_NETWORK_LOSS
        self.failure_confidence = 0.6
        self._disconnected_since = None
        self.channel_switch_attempts = 0

    def finish_channel_switch(
        self,
        success: bool,
        selected_channel: str = '',
    ) -> None:
        if self.state != RecoveryState.CHANNEL_SWITCH:
            raise InvalidTransition(
                f'expected CHANNEL_SWITCH, current {self.state.name}'
            )

        self.channel_switch_attempts += 1
        if success:
            self.link_connected = True
            if selected_channel:
                self.active_channel = selected_channel
            self.state = RecoveryState.LINK_RECOVERED
            self.failure_cause = FailureCause.CHANNEL_ANOMALY
            self.failure_confidence = 0.8
            self.detail = (
                f'communication restored on alternate channel '
                f'{self.active_channel or "unknown"}'
            )
            return

        if self.channel_switch_attempts < self.max_channel_switch_attempts:
            self.detail = (
                'alternate channel failed; another configured channel '
                'may be attempted'
            )
            return

        self.state = RecoveryState.CLASSIFYING
        self.failure_cause = FailureCause.TOTAL_LINK_FAILURE
        self.failure_confidence = 0.8
        self.detail = (
            'all configured channels failed at recovery waypoint; '
            'total link failure suspected'
        )

    def finish_classification(self) -> None:
        self._transition(
            RecoveryState.CLASSIFYING,
            RecoveryState.SAFE_STOP,
            f'{self.detail}; autonomous search remains disabled',
        )

    def fail(self, reason: str) -> None:
        self.state = RecoveryState.SAFE_STOP
        self.detail = reason

    def _transition(
        self,
        expected: RecoveryState,
        target: RecoveryState,
        detail: str,
    ) -> None:
        if self.state != expected:
            raise InvalidTransition(
                f'expected {expected.name}, current {self.state.name}'
            )
        self.state = target
        self.detail = detail


def apply_recovery_event(
    manager: RecoveryManagerCore,
    event: RecoveryEventType,
    channel: str = '',
    detail: str = '',
) -> RecoveryState:
    if event == RecoveryEventType.RETURN_SUCCEEDED:
        manager.finish_return()
    elif event == RecoveryEventType.RETURN_FAILED:
        manager.fail(detail or 'recorded-route return failed')
    elif event == RecoveryEventType.SYNC_STARTED:
        manager.start_sync()
    elif event == RecoveryEventType.SYNC_SUCCEEDED:
        manager.finish_sync(True)
    elif event == RecoveryEventType.SYNC_FAILED:
        manager.finish_sync(False)
    elif event == RecoveryEventType.REENTRY_SUCCEEDED:
        manager.finish_reentry(True)
    elif event == RecoveryEventType.REENTRY_LINK_LOST:
        manager.finish_reentry(False)
    elif event == RecoveryEventType.REENTRY_FAILED:
        manager.failure_cause = (
            FailureCause.REENTRY_NAVIGATION_FAILURE
        )
        manager.failure_confidence = 1.0
        manager.fail(detail or 'low-speed reentry navigation failed')
    elif event == RecoveryEventType.CHANNEL_SWITCH_SUCCEEDED:
        manager.finish_channel_switch(True, channel)
    elif event == RecoveryEventType.CHANNEL_SWITCH_FAILED:
        manager.finish_channel_switch(False, channel)
    elif event == RecoveryEventType.CLASSIFICATION_RECORDED:
        manager.finish_classification()
    else:
        raise ValueError(f'unknown recovery event: {event}')
    return manager.state

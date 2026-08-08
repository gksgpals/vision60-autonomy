import pytest

from comm_recovery_manager.core import (
    InvalidTransition,
    RecoveryManagerCore,
    RecoveryState,
)
from comm_recovery_manager.node import is_confirmed_stopped
from vision60_msgs.msg import RobotSafetyState


def test_healthy_link_remains_normal():
    manager = RecoveryManagerCore()
    state = manager.observe_link(True, 0.01, 20.0, 0.0)
    assert state == RecoveryState.NORMAL


def test_poor_connected_link_becomes_degraded():
    manager = RecoveryManagerCore()
    state = manager.observe_link(True, 0.30, 20.0, 0.0)
    assert state == RecoveryState.DEGRADED


def test_short_disconnect_does_not_confirm_link_loss():
    manager = RecoveryManagerCore(lost_timeout_s=2.0)
    manager.observe_link(False, 1.0, 999.0, 10.0)
    state = manager.observe_link(False, 1.0, 999.0, 11.9)
    assert state == RecoveryState.DEGRADED


def test_sustained_disconnect_confirms_link_loss():
    manager = RecoveryManagerCore(lost_timeout_s=2.0)
    manager.observe_link(False, 1.0, 999.0, 10.0)
    state = manager.observe_link(False, 1.0, 999.0, 12.0)
    assert state == RecoveryState.LINK_LOST


def test_complete_recovery_sequence_returns_to_normal():
    manager = RecoveryManagerCore(lost_timeout_s=1.0)
    manager.observe_link(False, 1.0, 999.0, 0.0)
    manager.observe_link(False, 1.0, 999.0, 1.0)
    manager.start_stopping()
    manager.confirm_stopped()
    manager.confirm_link_recovered()
    manager.start_sync()
    manager.finish_sync(True)
    manager.finish_reentry(True)
    assert manager.state == RecoveryState.NORMAL


def test_sync_failure_causes_safe_stop():
    manager = RecoveryManagerCore()
    manager.state = RecoveryState.SYNCING
    manager.finish_sync(False)
    assert manager.state == RecoveryState.SAFE_STOP


def test_invalid_transition_is_rejected():
    manager = RecoveryManagerCore()
    with pytest.raises(InvalidTransition):
        manager.confirm_stopped()


def test_stop_confirmation_requires_motion_disabled():
    state = RobotSafetyState()
    state.walk_enabled = False
    state.motion_allowed = False
    assert is_confirmed_stopped(state)

    state.motion_allowed = True
    assert not is_confirmed_stopped(state)

import pytest

from comm_recovery_manager.core import (
    FailureCause,
    InvalidTransition,
    RecoveryEventType,
    RecoveryManagerCore,
    RecoveryState,
    apply_recovery_event,
)


def advance_to_returning(manager):
    manager.observe_link(
        False, 1.0, 999.0, 0.0, channel='primary'
    )
    manager.observe_link(
        False, 1.0, 999.0, manager.lost_timeout_s,
        channel='primary',
    )
    manager.start_stopping()
    manager.confirm_stopped()


def test_return_with_recovered_link_starts_sync_sequence():
    manager = RecoveryManagerCore(lost_timeout_s=1.0)
    advance_to_returning(manager)
    manager.observe_link(
        True, 0.01, 20.0, 2.0, channel='primary'
    )

    manager.finish_return()
    assert manager.state == RecoveryState.LINK_RECOVERED

    manager.start_sync()
    manager.finish_sync(True)
    manager.finish_reentry(True)

    assert manager.state == RecoveryState.NORMAL
    assert manager.failure_cause == FailureCause.TRANSIENT_NETWORK_LOSS


def test_return_without_link_requests_channel_switch():
    manager = RecoveryManagerCore(lost_timeout_s=1.0)
    advance_to_returning(manager)

    manager.finish_return()

    assert manager.state == RecoveryState.CHANNEL_SWITCH
    assert manager.active_channel == 'primary'


def test_successful_alternate_channel_is_classified():
    manager = RecoveryManagerCore(lost_timeout_s=1.0)
    advance_to_returning(manager)
    manager.finish_return()

    manager.finish_channel_switch(True, 'backup_wifi')

    assert manager.state == RecoveryState.LINK_RECOVERED
    assert manager.active_channel == 'backup_wifi'
    assert manager.failure_cause == FailureCause.CHANNEL_ANOMALY


def test_all_channel_attempts_fail_then_safe_stop():
    manager = RecoveryManagerCore(
        lost_timeout_s=1.0,
        max_channel_switch_attempts=2,
    )
    advance_to_returning(manager)
    manager.finish_return()

    manager.finish_channel_switch(False)
    assert manager.state == RecoveryState.CHANNEL_SWITCH

    manager.finish_channel_switch(False)
    assert manager.state == RecoveryState.CLASSIFYING
    assert manager.failure_cause == FailureCause.TOTAL_LINK_FAILURE

    manager.finish_classification()
    assert manager.state == RecoveryState.SAFE_STOP


def test_repeated_loss_during_reentry_is_location_candidate():
    manager = RecoveryManagerCore()
    manager.state = RecoveryState.REENTRY_TEST

    manager.finish_reentry(False)

    assert manager.state == RecoveryState.CLASSIFYING
    assert manager.failure_cause == FailureCause.LOCATION_RADIO_SHADOW
    assert manager.failure_confidence < 1.0


def test_sync_failure_is_fail_closed_and_classified():
    manager = RecoveryManagerCore()
    manager.state = RecoveryState.SYNCING

    manager.finish_sync(False)

    assert manager.state == RecoveryState.SAFE_STOP
    assert manager.failure_cause == FailureCause.DATA_SYNC_FAILURE


def test_channel_result_outside_switch_state_is_rejected():
    manager = RecoveryManagerCore()
    with pytest.raises(InvalidTransition):
        manager.finish_channel_switch(True, 'backup_wifi')


def test_typed_events_drive_post_return_sequence():
    manager = RecoveryManagerCore(lost_timeout_s=1.0)
    advance_to_returning(manager)
    manager.observe_link(
        True, 0.01, 10.0, 2.0, channel='primary'
    )

    apply_recovery_event(
        manager, RecoveryEventType.RETURN_SUCCEEDED
    )
    apply_recovery_event(manager, RecoveryEventType.SYNC_STARTED)
    apply_recovery_event(manager, RecoveryEventType.SYNC_SUCCEEDED)
    apply_recovery_event(
        manager, RecoveryEventType.REENTRY_SUCCEEDED
    )

    assert manager.state == RecoveryState.NORMAL


def test_typed_channel_events_reach_fail_closed_state():
    manager = RecoveryManagerCore(
        lost_timeout_s=1.0,
        max_channel_switch_attempts=1,
    )
    advance_to_returning(manager)
    apply_recovery_event(
        manager, RecoveryEventType.RETURN_SUCCEEDED
    )

    apply_recovery_event(
        manager, RecoveryEventType.CHANNEL_SWITCH_FAILED
    )
    assert manager.state == RecoveryState.CLASSIFYING

    apply_recovery_event(
        manager, RecoveryEventType.CLASSIFICATION_RECORDED
    )
    assert manager.state == RecoveryState.SAFE_STOP

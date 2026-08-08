from comm_recovery_manager.motion_lock_adapter import (
    motion_lock_required,
)


def test_motion_lock_is_inverse_of_motion_permission():
    assert motion_lock_required(False)
    assert not motion_lock_required(True)

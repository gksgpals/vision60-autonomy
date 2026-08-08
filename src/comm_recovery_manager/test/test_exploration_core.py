from comm_recovery_manager.exploration_core import ExplorationGateCore


def safe_robot(core):
    core.observe_safety(True, True, False)


def test_gate_is_fail_closed_until_both_inputs_arrive():
    core = ExplorationGateCore()
    assert not core.exploration_allowed
    core.observe_recovery(core.NORMAL)
    assert not core.exploration_allowed
    safe_robot(core)
    assert core.exploration_allowed


def test_degraded_link_can_finish_current_frontier_navigation():
    core = ExplorationGateCore()
    safe_robot(core)
    core.observe_recovery(core.DEGRADED)
    assert core.exploration_allowed


def test_link_loss_and_every_recovery_state_pause_exploration():
    core = ExplorationGateCore()
    safe_robot(core)
    for state in range(3, 12):
        core.observe_recovery(state)
        assert not core.exploration_allowed


def test_normal_after_recovery_resumes_exploration():
    core = ExplorationGateCore()
    safe_robot(core)
    core.observe_recovery(5)
    assert not core.exploration_allowed
    core.observe_recovery(core.NORMAL)
    assert core.exploration_allowed


def test_any_motion_safety_block_pauses_exploration():
    core = ExplorationGateCore()
    core.observe_recovery(core.NORMAL)
    core.observe_safety(False, True, False)
    assert not core.exploration_allowed
    core.observe_safety(True, False, False)
    assert not core.exploration_allowed
    core.observe_safety(True, True, True)
    assert not core.exploration_allowed

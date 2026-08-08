import math

from vision60_bridge.core import BridgeSafetyCore, VelocityCommand


def _ready_core():
    core = BridgeSafetyCore()
    assert core.set_walk_enabled(True)
    assert core.submit_command(VelocityCommand(0.2, 0.0, 0.1), 1.0)
    return core


def _evaluate(core, now_s=1.1, dt_s=0.1, allow_motion=True):
    return core.evaluate(
        now_s=now_s,
        dt_s=dt_s,
        sdk_connected=True,
        localization_healthy=True,
        lidar_healthy=True,
        allow_motion_output=allow_motion,
    )


def test_valid_command_is_acceleration_limited():
    decision = _evaluate(_ready_core())
    assert decision.motion_allowed
    assert math.isclose(decision.command.linear_x, 0.05)
    assert math.isclose(decision.command.angular_z, 0.1)


def test_walk_disabled_blocks_motion():
    core = BridgeSafetyCore()
    core.submit_command(VelocityCommand(0.2, 0.0, 0.0), 1.0)
    decision = _evaluate(core)
    assert decision.command.is_zero()
    assert decision.stop_reason == 'walking disabled'


def test_emergency_stop_is_latched_and_blocks_walk_enable():
    core = _ready_core()
    core.latch_emergency_stop()
    assert not core.set_walk_enabled(True)
    decision = _evaluate(core)
    assert decision.command.is_zero()
    assert decision.stop_reason == 'emergency stop latched'


def test_command_timeout_stops_immediately():
    core = _ready_core()
    moving = _evaluate(core)
    stopped = _evaluate(core, now_s=1.4)
    assert not moving.command.is_zero()
    assert stopped.command.is_zero()
    assert stopped.command_timed_out


def test_non_finite_command_is_rejected():
    core = BridgeSafetyCore()
    assert not core.submit_command(
        VelocityCommand(math.nan, 0.0, 0.0), 1.0
    )


def test_out_of_range_command_is_rejected():
    core = BridgeSafetyCore(max_linear_x_mps=0.5)
    assert not core.submit_command(
        VelocityCommand(0.51, 0.0, 0.0), 1.0
    )


def test_stale_localization_blocks_motion():
    core = _ready_core()
    decision = core.evaluate(
        now_s=1.1,
        dt_s=0.1,
        sdk_connected=True,
        localization_healthy=False,
        lidar_healthy=True,
        allow_motion_output=True,
    )
    assert decision.command.is_zero()
    assert decision.stop_reason == 'localization stale'


def test_hardware_no_motion_mode_blocks_output():
    decision = _evaluate(_ready_core(), allow_motion=False)
    assert decision.command.is_zero()
    assert decision.stop_reason == 'motion output physically disabled'

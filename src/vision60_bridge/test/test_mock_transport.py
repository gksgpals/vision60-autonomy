from vision60_bridge.core import VelocityCommand
from vision60_bridge.mock_transport import MockVision60Interface


def test_mock_transport_integrates_accepted_command():
    transport = MockVision60Interface()
    assert transport.connect()
    assert transport.set_walk_enabled(True)
    assert transport.send_velocity(VelocityCommand(0.2, 0.0, 0.0), 1.0)
    state = transport.read_state()
    assert state.x == 0.2
    assert state.linear_x == 0.2


def test_mock_transport_estop_forces_zero():
    transport = MockVision60Interface()
    transport.connect()
    transport.set_walk_enabled(True)
    transport.emergency_stop()
    transport.send_velocity(VelocityCommand(0.2, 0.0, 0.0), 1.0)
    assert transport.command_log[-1].is_zero()
    assert transport.read_state().x == 0.0

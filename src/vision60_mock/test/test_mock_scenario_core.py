from vision60_mock.core import MockScenarioCore


def test_normal_degraded_and_lost_sequence():
    scenario = MockScenarioCore()
    normal = scenario.advance(1.0, 0.1)
    degraded = scenario.advance(4.0, 0.1)
    lost = scenario.advance(6.0, 0.1)

    assert normal.connected
    assert normal.communication_state == scenario.NORMAL
    assert degraded.connected
    assert degraded.communication_state == scenario.DEGRADED
    assert not lost.connected
    assert lost.communication_state == scenario.LOST


def test_safe_stop_freezes_position():
    scenario = MockScenarioCore()
    before = scenario.advance(1.0, 1.0)
    scenario.request_stop()
    after = scenario.advance(2.0, 1.0)
    assert after.x == before.x


def test_optional_reconnection_restores_normal_link():
    scenario = MockScenarioCore(
        degraded_after_s=2.0,
        disconnected_after_s=4.0,
        reconnected_after_s=6.0,
    )

    lost = scenario.advance(5.0, 0.1)
    restored = scenario.advance(6.0, 0.1)

    assert not lost.connected
    assert restored.connected
    assert restored.communication_state == scenario.NORMAL

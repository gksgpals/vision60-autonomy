from math import pi

from route_recorder.core import RecordedPoint, RouteRecorderCore


def point(
    x=0.0,
    y=0.0,
    yaw=0.0,
    state=1,
    loss=0.0,
    safe=True,
):
    return RecordedPoint(
        timestamp_ns=0,
        x=x,
        y=y,
        z=0.0,
        yaw=yaw,
        communication_state=state,
        packet_loss_ratio=loss,
        latency_ms=10.0,
        safe_to_return=safe,
    )


def test_first_point_is_recorded():
    recorder = RouteRecorderCore()
    assert recorder.add(point())
    assert len(recorder.points) == 1


def test_small_stationary_noise_is_ignored():
    recorder = RouteRecorderCore()
    recorder.add(point())
    assert not recorder.add(point(x=0.02, y=0.01, yaw=0.02))


def test_distance_heading_and_communication_changes_are_recorded():
    recorder = RouteRecorderCore()
    recorder.add(point())
    assert recorder.add(point(x=0.3))
    assert recorder.add(point(x=0.3, yaw=0.3))
    assert recorder.add(point(x=0.3, yaw=0.3, state=2, loss=0.2))


def test_recovery_segment_ends_at_latest_safe_point():
    recorder = RouteRecorderCore()
    recorder.add(point(x=0.0, safe=True))
    recorder.add(point(x=0.3, safe=True))
    recorder.add(point(x=0.6, state=2, safe=False))
    recorder.add(point(x=0.9, state=3, safe=False))

    recovery = recorder.recovery_segment()

    assert [item.x for item in recovery] == [0.9, 0.6, 0.3]
    assert abs(abs(recovery[0].yaw) - pi) < 1e-6


def test_no_safe_waypoint_returns_empty_path():
    recorder = RouteRecorderCore()
    recorder.add(point(safe=False))
    assert recorder.recovery_segment() == []

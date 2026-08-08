from math import hypot

from comm_recovery_manager.safety_velocity_gate import (
    clamp_planar_speed,
    is_fresh,
    source_stamp_is_fresh,
)
from geometry_msgs.msg import Twist
import pytest


def test_never_received_is_not_fresh():
    assert not is_fresh(1_000_000_000, 0, 0.5)


def test_recent_sample_is_fresh():
    assert is_fresh(1_000_000_000, 750_000_000, 0.5)


def test_expired_sample_is_not_fresh():
    assert not is_fresh(1_000_000_000, 400_000_000, 0.5)


def test_zero_or_old_sensor_stamp_is_rejected():
    assert not source_stamp_is_fresh(1_000_000_000, 0, 0.5)
    assert not source_stamp_is_fresh(
        1_000_000_000, 400_000_000, 0.5
    )


def test_reentry_planar_speed_is_hard_limited():
    command = Twist()
    command.linear.x = 0.18
    command.linear.y = 0.24
    command.angular.z = 0.4
    limited = clamp_planar_speed(command, 0.10)
    assert hypot(limited.linear.x, limited.linear.y) == pytest.approx(0.10)
    assert limited.angular.z == 0.4
    assert hypot(command.linear.x, command.linear.y) == pytest.approx(0.30)

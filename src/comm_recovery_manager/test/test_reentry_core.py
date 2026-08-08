from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from comm_recovery_manager.reentry_core import (
    LinkLossMonitor,
    build_reentry_path,
)


def test_reentry_path_reverses_recovery_path_and_heading():
    recovery = Path()
    for x in (2.0, 1.0, 0.0):
        pose = PoseStamped()
        pose.pose.position.x = x
        recovery.poses.append(pose)

    reentry = build_reentry_path(recovery)

    assert [pose.pose.position.x for pose in reentry.poses] == [
        0.0, 1.0, 2.0
    ]
    assert reentry.poses[0].pose.orientation.z == 0.0
    assert reentry.poses[0].pose.orientation.w == 1.0


def test_link_loss_requires_sustained_timeout():
    monitor = LinkLossMonitor(timeout_s=1.0)

    assert not monitor.observe(False, 10.0)
    assert not monitor.observe(False, 10.9)
    assert monitor.observe(False, 11.0)

    assert not monitor.observe(True, 11.1)
    assert not monitor.observe(False, 11.2)

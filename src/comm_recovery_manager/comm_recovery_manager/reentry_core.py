from copy import deepcopy
from math import atan2, cos, sin
from typing import Optional

from nav_msgs.msg import Path


class LinkLossMonitor:
    def __init__(self, timeout_s: float) -> None:
        self.timeout_s = timeout_s
        self._disconnected_since_s: Optional[float] = None

    def observe(self, connected: bool, now_s: float) -> bool:
        if connected:
            self._disconnected_since_s = None
            return False
        if self._disconnected_since_s is None:
            self._disconnected_since_s = now_s
            return False
        return (
            now_s - self._disconnected_since_s
            >= self.timeout_s
        )

    def reset(self) -> None:
        self._disconnected_since_s = None


def build_reentry_path(recovery_path: Path) -> Path:
    result = Path()
    result.header = deepcopy(recovery_path.header)
    result.poses = [
        deepcopy(pose)
        for pose in reversed(recovery_path.poses)
    ]
    if len(result.poses) < 2:
        return result

    for index in range(len(result.poses) - 1):
        current = result.poses[index].pose
        following = result.poses[index + 1].pose
        yaw = atan2(
            following.position.y - current.position.y,
            following.position.x - current.position.x,
        )
        current.orientation.x = 0.0
        current.orientation.y = 0.0
        current.orientation.z = sin(yaw / 2.0)
        current.orientation.w = cos(yaw / 2.0)

    result.poses[-1].pose.orientation = deepcopy(
        result.poses[-2].pose.orientation
    )
    return result

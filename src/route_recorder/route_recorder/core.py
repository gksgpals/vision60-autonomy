from dataclasses import dataclass, replace
from math import atan2, hypot, pi
from typing import Iterable, List, Optional


def normalize_angle(angle: float) -> float:
    while angle > pi:
        angle -= 2.0 * pi
    while angle < -pi:
        angle += 2.0 * pi
    return angle


@dataclass(frozen=True)
class RecordedPoint:
    timestamp_ns: int
    x: float
    y: float
    z: float
    yaw: float
    communication_state: int
    packet_loss_ratio: float
    latency_ms: float
    safe_to_return: bool


class RouteRecorderCore:
    def __init__(
        self,
        distance_threshold_m: float = 0.25,
        heading_threshold_rad: float = 0.25,
        communication_change_threshold: float = 0.15,
    ) -> None:
        self.distance_threshold_m = distance_threshold_m
        self.heading_threshold_rad = heading_threshold_rad
        self.communication_change_threshold = communication_change_threshold
        self.points: List[RecordedPoint] = []

    def should_record(self, candidate: RecordedPoint) -> bool:
        if not self.points:
            return True

        previous = self.points[-1]
        distance = hypot(candidate.x - previous.x, candidate.y - previous.y)
        heading_change = abs(normalize_angle(candidate.yaw - previous.yaw))
        communication_changed = (
            candidate.communication_state != previous.communication_state
            or abs(candidate.packet_loss_ratio - previous.packet_loss_ratio)
            >= self.communication_change_threshold
        )
        return (
            distance >= self.distance_threshold_m
            or heading_change >= self.heading_threshold_rad
            or communication_changed
        )

    def add(self, candidate: RecordedPoint) -> bool:
        if not self.should_record(candidate):
            return False
        self.points.append(candidate)
        return True

    def latest_safe_index(self) -> Optional[int]:
        for index in range(len(self.points) - 1, -1, -1):
            if self.points[index].safe_to_return:
                return index
        return None

    def recovery_segment(self) -> List[RecordedPoint]:
        safe_index = self.latest_safe_index()
        if safe_index is None:
            return []
        segment = list(reversed(self.points[safe_index:]))
        return recalculate_return_headings(segment)


def recalculate_return_headings(
    points: Iterable[RecordedPoint],
) -> List[RecordedPoint]:
    result = list(points)
    if len(result) < 2:
        return result

    updated: List[RecordedPoint] = []
    for index, point in enumerate(result):
        if index < len(result) - 1:
            next_point = result[index + 1]
            yaw = atan2(next_point.y - point.y, next_point.x - point.x)
        else:
            yaw = updated[-1].yaw
        updated.append(replace(point, yaw=yaw))
    return updated

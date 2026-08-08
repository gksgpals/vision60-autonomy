from dataclasses import dataclass


@dataclass(frozen=True)
class ScenarioSample:
    x: float
    connected: bool
    communication_state: int
    packet_loss_ratio: float
    latency_ms: float


class MockScenarioCore:
    NORMAL = 1
    DEGRADED = 2
    LOST = 3

    def __init__(
        self,
        speed_mps: float = 0.5,
        degraded_after_s: float = 3.0,
        disconnected_after_s: float = 5.0,
        reconnected_after_s: float = -1.0,
    ) -> None:
        self.speed_mps = speed_mps
        self.degraded_after_s = degraded_after_s
        self.disconnected_after_s = disconnected_after_s
        self.reconnected_after_s = reconnected_after_s
        self.x = 0.0
        self.stopped = False

    def advance(self, elapsed_s: float, dt_s: float) -> ScenarioSample:
        if not self.stopped:
            self.x += self.speed_mps * dt_s

        if (
            self.reconnected_after_s >= 0.0
            and elapsed_s >= self.reconnected_after_s
        ):
            return ScenarioSample(
                self.x, True, self.NORMAL, 0.01, 25.0
            )
        if elapsed_s < self.degraded_after_s:
            return ScenarioSample(
                self.x, True, self.NORMAL, 0.01, 20.0
            )
        if elapsed_s < self.disconnected_after_s:
            return ScenarioSample(
                self.x, True, self.DEGRADED, 0.35, 650.0
            )
        return ScenarioSample(
            self.x, False, self.LOST, 1.0, 2000.0
        )

    def request_stop(self) -> None:
        self.stopped = True

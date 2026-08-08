from dataclasses import dataclass
from typing import Optional, Sequence


@dataclass(frozen=True)
class ChannelAttempt:
    attempt_index: int
    channel: str


class ChannelPlan:
    def __init__(self, channels: Sequence[str]) -> None:
        normalized = [channel.strip() for channel in channels]
        self.channels = [
            channel for channel in normalized if channel
        ]
        if len(set(self.channels)) != len(self.channels):
            raise ValueError('candidate channels must be unique')

    def attempt_for(
        self,
        completed_attempts: int,
    ) -> Optional[ChannelAttempt]:
        if completed_attempts < 0:
            raise ValueError(
                'completed_attempts must be non-negative'
            )
        if completed_attempts >= len(self.channels):
            return None
        return ChannelAttempt(
            completed_attempts,
            self.channels[completed_attempts],
        )

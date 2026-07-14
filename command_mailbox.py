from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from threading import Lock

from pydantic import ValidationError

from models import Command, PostureCommand, VelocityCommand, decode_command


@dataclass(frozen=True)
class ReceivedCommand:
    command: Command
    received_at: float


@dataclass(frozen=True)
class CommandBatch:
    postures: tuple[ReceivedCommand, ...]
    velocity: ReceivedCommand | None
    invalid_error: str | None


class CommandMailbox:
    """Thread-safe, bounded handoff from Zenoh callbacks to the control loop."""

    def __init__(self, *, posture_capacity: int = 8) -> None:
        if posture_capacity < 1:
            raise ValueError("posture_capacity must be positive")
        self._postures: deque[ReceivedCommand] = deque(maxlen=posture_capacity)
        self._velocity: ReceivedCommand | None = None
        self._invalid_error: str | None = None
        self._lock = Lock()

    def submit(self, payload: bytes | str, *, received_at: float) -> None:
        try:
            command = decode_command(payload)
        except (ValidationError, ValueError, TypeError) as error:
            with self._lock:
                self._invalid_error = f"invalid command: {error}"
            return

        received = ReceivedCommand(command=command, received_at=received_at)
        with self._lock:
            if isinstance(command, VelocityCommand):
                self._velocity = received
            elif isinstance(command, PostureCommand):
                self._postures.append(received)

    def take(self, *, posture_limit: int = 4) -> CommandBatch:
        if posture_limit < 1:
            raise ValueError("posture_limit must be positive")
        with self._lock:
            postures = tuple(
                self._postures.popleft()
                for _ in range(min(posture_limit, len(self._postures)))
            )
            velocity = self._velocity
            invalid_error = self._invalid_error
            self._velocity = None
            self._invalid_error = None
        return CommandBatch(
            postures=postures,
            velocity=velocity,
            invalid_error=invalid_error,
        )

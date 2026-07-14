from __future__ import annotations

from dataclasses import dataclass
from threading import Lock

from pydantic import ValidationError

from models import Command, PostureCommand, VelocityCommand, decode_command


@dataclass(frozen=True)
class ReceivedCommand:
    command: Command
    received_at: float
    sequence: int


@dataclass(frozen=True)
class InvalidCommand:
    error: str
    received_at: float
    sequence: int


PendingEvent = ReceivedCommand | InvalidCommand


@dataclass(frozen=True)
class PendingCommands:
    posture: ReceivedCommand | None
    velocity: ReceivedCommand | None
    invalid: InvalidCommand | None

    def in_receive_order(self) -> tuple[PendingEvent, ...]:
        events = tuple(
            event
            for event in (self.posture, self.velocity, self.invalid)
            if event is not None
        )
        return tuple(
            sorted(events, key=lambda event: (event.received_at, event.sequence))
        )


class CommandMailbox:
    """Thread-safe latest-value handoff from callbacks to the control loop."""

    def __init__(self) -> None:
        self._posture: ReceivedCommand | None = None
        self._velocity: ReceivedCommand | None = None
        self._invalid: InvalidCommand | None = None
        self._sequence = 0
        self._last_completed_order: tuple[float, int] | None = None
        self._last_drained_order: tuple[float, int] | None = None
        self._lock = Lock()

    def submit(self, payload: bytes | str, *, received_at: float) -> None:
        with self._lock:
            sequence = self._next_sequence()
        try:
            command = decode_command(payload)
        except (ValidationError, ValueError, TypeError) as error:
            invalid = InvalidCommand(
                error=f"invalid command: {error}",
                received_at=received_at,
                sequence=sequence,
            )
            with self._lock:
                if self._was_already_drained(invalid):
                    return
                self._mark_completed(invalid)
                if self._is_newer(invalid, self._invalid):
                    self._invalid = invalid
            return

        received = ReceivedCommand(
            command=command,
            received_at=received_at,
            sequence=sequence,
        )
        with self._lock:
            if self._was_already_drained(received):
                return
            self._mark_completed(received)
            if isinstance(command, VelocityCommand):
                if self._is_newer(received, self._velocity):
                    self._velocity = received
            elif isinstance(command, PostureCommand):
                self._coalesce_posture(received)

    def _next_sequence(self) -> int:
        sequence = self._sequence
        self._sequence += 1
        return sequence

    @staticmethod
    def _order(event: PendingEvent) -> tuple[float, int]:
        return (event.received_at, event.sequence)

    def _was_already_drained(self, event: PendingEvent) -> bool:
        return (
            self._last_drained_order is not None
            and self._order(event) <= self._last_drained_order
        )

    def _mark_completed(self, event: PendingEvent) -> None:
        order = self._order(event)
        if self._last_completed_order is None or order > self._last_completed_order:
            self._last_completed_order = order

    @classmethod
    def _is_newer(cls, candidate: PendingEvent, current: PendingEvent | None) -> bool:
        if current is None:
            return True
        return cls._order(candidate) > cls._order(current)

    def _coalesce_posture(self, received: ReceivedCommand) -> None:
        current = self._posture
        command = received.command
        assert isinstance(command, PostureCommand)
        down_is_pending = current is not None and (
            isinstance(current.command, PostureCommand)
            and current.command.posture == "down"
        )
        if down_is_pending:
            assert current is not None
            if command.posture == "down" and self._is_newer(current, received):
                self._posture = received
            return
        if (
            current is None
            or command.posture == "down"
            or self._is_newer(received, current)
        ):
            self._posture = received

    def drain(self) -> PendingCommands:
        with self._lock:
            pending = PendingCommands(
                posture=self._posture,
                velocity=self._velocity,
                invalid=self._invalid,
            )
            drained_orders = [
                self._order(event)
                for event in (pending.posture, pending.velocity, pending.invalid)
                if event is not None
            ]
            completed_orders = [
                order
                for order in (*drained_orders, self._last_completed_order)
                if order is not None
            ]
            if completed_orders:
                newest_drained = max(completed_orders)
                if (
                    self._last_drained_order is None
                    or newest_drained > self._last_drained_order
                ):
                    self._last_drained_order = newest_drained
            self._posture = None
            self._velocity = None
            self._invalid = None
        return pending

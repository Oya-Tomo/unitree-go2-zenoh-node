from __future__ import annotations

import json
import time
from collections.abc import Callable
from dataclasses import dataclass
from threading import Lock
from typing import Protocol

import zenoh

from keyspace import RobotKeyspace
from models import Posture, PostureCommand, PostureTarget, VelocityCommand


@dataclass(frozen=True)
class ObservedState:
    value: dict[str, object]
    received_at: float

    def is_stale(self, *, now: float, maximum_age: float) -> bool:
        return now - self.received_at > maximum_age


@dataclass(frozen=True)
class PostureObservation:
    posture: Posture
    received_at: float


class PostureObserver(Protocol):
    def posture(self) -> PostureObservation | None: ...


class SamplePayload(Protocol):
    def to_bytes(self) -> bytes: ...


class StateSample(Protocol):
    @property
    def payload(self) -> SamplePayload: ...

    @property
    def key_expr(self) -> object: ...


class RobotStateCache:
    """Thread-safe cache of state samples and their local arrival times."""

    def __init__(
        self,
        keyspace: RobotKeyspace,
        *,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.keyspace = keyspace
        self._clock = clock
        self._states: dict[str, ObservedState] = {}
        self._lock = Lock()

    def update(self, sample: StateSample) -> None:
        try:
            value = json.loads(sample.payload.to_bytes())
        except (UnicodeDecodeError, json.JSONDecodeError):
            return
        if not isinstance(value, dict):
            return
        key = str(sample.key_expr)
        if key not in self.keyspace.state_keys:
            return
        observed = ObservedState(value=value, received_at=self._clock())
        with self._lock:
            self._states[key] = observed

    def snapshot(self) -> dict[str, ObservedState]:
        with self._lock:
            return {
                key: ObservedState(
                    value=state.value.copy(), received_at=state.received_at
                )
                for key, state in self._states.items()
            }

    def posture(self) -> PostureObservation | None:
        with self._lock:
            state = self._states.get(self.keyspace.posture)
            if state is None:
                return None
            posture = state.value.get("posture")
            if not isinstance(posture, str):
                return None
            try:
                parsed = Posture(posture)
            except ValueError:
                return None
            return PostureObservation(posture=parsed, received_at=state.received_at)


class CommandPublisher:
    def __init__(
        self,
        velocity_put: Callable[[str], object],
        posture_put: Callable[[str], object],
    ) -> None:
        self._velocity_put = velocity_put
        self._posture_put = posture_put

    def publish_velocity(self, velocity: tuple[float, float, float]) -> None:
        self._velocity_put(
            VelocityCommand(
                vx=velocity[0], vy=velocity[1], vyaw=velocity[2]
            ).model_dump_json()
        )

    def publish_posture(self, posture: PostureTarget) -> None:
        self._posture_put(PostureCommand(posture=posture).model_dump_json())


def fetch_initial_state(session: zenoh.Session, cache: RobotStateCache) -> None:
    for key in cache.keyspace.state_keys:
        for reply in session.get(key, timeout=0.5):
            sample = reply.ok
            if sample is not None:
                cache.update(sample)

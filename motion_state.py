from __future__ import annotations

import logging
import math
import time
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from dataclasses import dataclass
from enum import IntEnum, StrEnum
from threading import Lock
from typing import Protocol

from models import MotionTelemetryState

LOGGER = logging.getLogger(__name__)


class Go2SportMode(IntEnum):
    """Values published in Unitree's Go2 SportModeState.mode field."""

    IDLE = 0
    BALANCE_STAND = 1
    POSE = 2
    LOCOMOTION = 3
    RESERVED_4 = 4
    LIE_DOWN = 5
    JOINT_LOCK = 6
    DAMPING = 7
    RECOVERY_STAND = 8
    RESERVED_9 = 9
    SIT = 10
    FRONT_FLIP = 11
    FRONT_JUMP = 12
    FRONT_POUNCE = 13


@dataclass(frozen=True)
class MotionStateObservation:
    mode: int
    error_code: int
    received_at: float

    def is_fresh(self, *, now: float, maximum_age: float) -> bool:
        # The callback may update the cache just after the caller captures
        # ``now``. Accept that small race, but reject timestamps that are far
        # in either direction so a mismatched clock cannot bypass StopMove.
        return abs(now - self.received_at) <= maximum_age

    @property
    def mode_name(self) -> str:
        try:
            return Go2SportMode(self.mode).name.lower()
        except ValueError:
            return "unknown"


class MotionStateSource(Protocol):
    def snapshot(self) -> MotionStateObservation | None: ...


@dataclass(frozen=True)
class MotionStateAssessment:
    telemetry: MotionTelemetryState
    received_at: float | None = None

    @property
    def is_trusted(self) -> bool:
        return self.telemetry.fresh and self.telemetry.error_code == 0

    @property
    def confirms_down(self) -> bool:
        return self.is_trusted and self.telemetry.mode == Go2SportMode.LIE_DOWN

    @property
    def confirms_balance_stand(self) -> bool:
        return self.is_trusted and self.telemetry.mode == Go2SportMode.BALANCE_STAND

    def confirms_down_since(self, since: float) -> bool:
        return (
            self.confirms_down
            and self.received_at is not None
            and self.received_at >= since
        )

    def confirms_balance_stand_since(self, since: float) -> bool:
        return (
            self.confirms_balance_stand
            and self.received_at is not None
            and self.received_at >= since
        )

    def confirms_not_down_since(self, since: float) -> bool:
        return (
            self.is_trusted
            and self.telemetry.mode != Go2SportMode.LIE_DOWN
            and self.received_at is not None
            and self.received_at >= since
        )

    @property
    def permits_walking(self) -> bool:
        return self.is_trusted and self.telemetry.mode in (
            Go2SportMode.BALANCE_STAND,
            Go2SportMode.LOCOMOTION,
        )

    @property
    def walking_rejection_reason(self) -> str:
        state = self.telemetry
        if state.mode is None:
            return "motion telemetry is unavailable"
        if not state.fresh:
            return "motion telemetry is stale"
        if state.error_code != 0:
            return f"motion telemetry reports error code {state.error_code}"
        return f"motion mode {state.mode_name} ({state.mode}) is not walking-capable"

    @property
    def description(self) -> str:
        state = self.telemetry
        if state.mode is None:
            return "motion telemetry unavailable"
        if not state.fresh:
            return f"stale motion mode {state.mode_name} ({state.mode})"
        if state.error_code != 0:
            return f"motion telemetry error {state.error_code}"
        return f"motion mode {state.mode_name} ({state.mode})"


class _ObservationRequirement(StrEnum):
    NONE = "none"
    NON_DOWN = "non_down"
    DOWN = "down"


@dataclass
class PostureObservationGate:
    """Keep telemetry causally ordered with posture RPC attempts.

    A posture RPC can be followed by a newly received sample that still
    describes the pre-RPC mode. The gate suppresses that ambiguous direction
    until telemetry demonstrates progress. If an uncertain call never ran,
    multiple continuous source-mode samples eventually recover the controller;
    repeating one cached sample or observing gaps cannot satisfy recovery.
    """

    stable_recovery_seconds: float
    maximum_sample_gap: float
    not_before: float | None = None
    _requirement: _ObservationRequirement = _ObservationRequirement.NONE
    _allow_stable_recovery: bool = False
    _recovery_first_received_at: float | None = None
    _recovery_last_received_at: float | None = None

    def __post_init__(self) -> None:
        for name in ("stable_recovery_seconds", "maximum_sample_gap"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")

    def begin_attempt(self, attempted_at: float) -> None:
        self.not_before = attempted_at
        self._reset_recovery_samples()

    def stand_up_succeeded(self, completed_at: float) -> None:
        self._require(
            _ObservationRequirement.NON_DOWN,
            since=completed_at,
            allow_stable_recovery=True,
        )

    def stand_down_succeeded(self, completed_at: float) -> None:
        self._require(
            _ObservationRequirement.DOWN,
            since=completed_at,
            allow_stable_recovery=True,
        )

    def stand_up_failed(self, failed_at: float, *, uncertain: bool) -> None:
        self._require(
            _ObservationRequirement.NON_DOWN
            if uncertain
            else _ObservationRequirement.NONE,
            since=failed_at,
            allow_stable_recovery=uncertain,
        )

    def stand_down_failed(self, failed_at: float, *, uncertain: bool) -> None:
        self._require(
            _ObservationRequirement.DOWN if uncertain else _ObservationRequirement.NONE,
            since=failed_at,
            allow_stable_recovery=uncertain,
        )

    def balance_stand_failed(self, failed_at: float, *, uncertain: bool) -> None:
        self.not_before = failed_at
        self._reset_recovery_samples()
        if not uncertain:
            self.confirmed()

    def begin_recovery(self, started_at: float) -> None:
        self.not_before = started_at
        self.confirmed()

    def observe(self, motion: MotionStateAssessment) -> None:
        not_before = self.not_before
        if not_before is None or self._requirement is _ObservationRequirement.NONE:
            return
        if self._observed_expected_progress(motion, since=not_before):
            self.confirmed()
            return
        if not self._allow_stable_recovery or not self._observed_source_mode(
            motion,
            since=not_before,
        ):
            self._reset_recovery_samples()
            return
        self._record_recovery_sample(motion)

    def confirms_down(self, motion: MotionStateAssessment) -> bool:
        if self._requirement is _ObservationRequirement.NON_DOWN:
            return False
        return self._confirms_since_barrier(motion, down=True)

    def confirms_balance_stand(self, motion: MotionStateAssessment) -> bool:
        if self._requirement is _ObservationRequirement.DOWN:
            return False
        return self._confirms_since_barrier(motion, down=False)

    def confirmed(self) -> None:
        self._requirement = _ObservationRequirement.NONE
        self._allow_stable_recovery = False
        self._reset_recovery_samples()

    def _require(
        self,
        requirement: _ObservationRequirement,
        *,
        since: float,
        allow_stable_recovery: bool,
    ) -> None:
        self.not_before = since
        self._requirement = requirement
        self._allow_stable_recovery = allow_stable_recovery
        self._reset_recovery_samples()

    def _observed_expected_progress(
        self,
        motion: MotionStateAssessment,
        *,
        since: float,
    ) -> bool:
        if self._requirement is _ObservationRequirement.NON_DOWN:
            return motion.confirms_not_down_since(since)
        return motion.confirms_down_since(since)

    def _observed_source_mode(
        self,
        motion: MotionStateAssessment,
        *,
        since: float,
    ) -> bool:
        if self._requirement is _ObservationRequirement.NON_DOWN:
            return motion.confirms_down_since(since)
        return motion.confirms_balance_stand_since(since)

    def _record_recovery_sample(self, motion: MotionStateAssessment) -> None:
        received_at = motion.received_at
        if received_at is None or received_at == self._recovery_last_received_at:
            return
        last_received_at = self._recovery_last_received_at
        if (
            last_received_at is None
            or received_at < last_received_at
            or received_at - last_received_at > self.maximum_sample_gap
        ):
            self._recovery_first_received_at = received_at
        self._recovery_last_received_at = received_at
        first_received_at = self._recovery_first_received_at
        if (
            first_received_at is not None
            and received_at > first_received_at
            and received_at - first_received_at >= self.stable_recovery_seconds
        ):
            self.confirmed()

    def _confirms_since_barrier(
        self,
        motion: MotionStateAssessment,
        *,
        down: bool,
    ) -> bool:
        if self.not_before is None:
            return motion.confirms_down if down else motion.confirms_balance_stand
        if down:
            return motion.confirms_down_since(self.not_before)
        return motion.confirms_balance_stand_since(self.not_before)

    def _reset_recovery_samples(self) -> None:
        self._recovery_first_received_at = None
        self._recovery_last_received_at = None


class MotionStateCache:
    """Thread-safe latest-value cache for the DDS SportModeState callback."""

    def __init__(self, *, clock: Callable[[], float] = time.monotonic) -> None:
        self._clock = clock
        self._observation: MotionStateObservation | None = None
        self._lock = Lock()

    def update(self, *, mode: int, error_code: int) -> None:
        observation = MotionStateObservation(
            mode=mode,
            error_code=error_code,
            received_at=self._clock(),
        )
        with self._lock:
            self._observation = observation

    def snapshot(self) -> MotionStateObservation | None:
        with self._lock:
            return self._observation


class MotionStateMonitor:
    """Convert raw locally-timestamped samples into controller telemetry."""

    def __init__(
        self,
        source: MotionStateSource | None,
        *,
        maximum_age: float,
    ) -> None:
        self._source = source
        self._maximum_age = maximum_age

    def observe(self, *, now: float) -> MotionStateAssessment:
        if self._source is None:
            return MotionStateAssessment(MotionTelemetryState())
        try:
            observation = self._source.snapshot()
        except Exception:
            LOGGER.exception("Could not read motion telemetry")
            return MotionStateAssessment(MotionTelemetryState())
        if observation is None:
            return MotionStateAssessment(MotionTelemetryState())
        return MotionStateAssessment(
            MotionTelemetryState(
                mode=observation.mode,
                mode_name=observation.mode_name,
                error_code=observation.error_code,
                fresh=observation.is_fresh(now=now, maximum_age=self._maximum_age),
            ),
            received_at=observation.received_at,
        )


@contextmanager
def subscribe_sport_mode(
    topic: str,
    cache: MotionStateCache,
) -> Iterator[None]:
    """Feed a cache from the Go2 DDS SportModeState topic."""
    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

    subscriber = ChannelSubscriber(topic, SportModeState_)

    def update_cache(state: SportModeState_) -> None:
        cache.update(mode=int(state.mode), error_code=int(state.error_code))

    try:
        subscriber.Init(update_cache)
        yield
    finally:
        subscriber.Close()

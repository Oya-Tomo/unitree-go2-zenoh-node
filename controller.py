from __future__ import annotations

import logging
import math
import time
from collections.abc import Callable
from dataclasses import dataclass
from enum import StrEnum
from typing import Protocol

from models import (
    Command,
    HealthState,
    NodeStatus,
    Posture,
    PostureCommand,
    PostureState,
    VelocityCommand,
    VelocityState,
)

LOGGER = logging.getLogger(__name__)


class SportClientProtocol(Protocol):
    def Move(self, vx: float, vy: float, vyaw: float) -> int: ...

    def StopMove(self) -> int: ...

    def StandUp(self) -> int: ...

    def StandDown(self) -> int: ...

    def BalanceStand(self) -> int: ...


class StateSink(Protocol):
    def publish_requested(self, state: VelocityState) -> None: ...

    def publish_applied(self, state: VelocityState) -> None: ...

    def publish_posture(self, state: PostureState) -> None: ...

    def publish_health(self, state: HealthState) -> None: ...


@dataclass(frozen=True)
class ControllerTiming:
    command_timeout_seconds: float
    posture_transition_seconds: float
    shutdown_stop_delay_seconds: float
    stop_retry_interval_seconds: float = 0.25

    def __post_init__(self) -> None:
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")


class _StopReason(StrEnum):
    STARTUP = "startup"
    WATCHDOG = "watchdog"
    MOVE_FAILURE = "move_failure"
    POSTURE_DOWN = "posture_down"


@dataclass(frozen=True)
class _PendingStop:
    reason: _StopReason
    next_attempt_at: float
    preceding_error: str | None


class _Unset:
    pass


_UNSET = _Unset()


class RobotController:
    """Serialize SDK calls and enforce the high-level safety state machine."""

    def __init__(
        self,
        sport_client: SportClientProtocol,
        state_sink: StateSink,
        timing: ControllerTiming,
        *,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self._sport_client = sport_client
        self._state_sink = state_sink
        self._timing = timing
        self._clock = clock
        self._sleep = sleep

        self.requested = VelocityState()
        self.applied = VelocityState()
        self.posture = PostureState()
        self.health = HealthState()

        self._stand_balance_deadline: float | None = None
        self._last_forwarded_velocity_at: float | None = None
        self._pending_stop: _PendingStop | None = None
        self._started = False
        self._shutdown = False

    def publish_state(self) -> None:
        self._publish(
            "requested velocity",
            lambda: self._state_sink.publish_requested(self.requested),
        )
        self._publish(
            "applied velocity", lambda: self._state_sink.publish_applied(self.applied)
        )
        self._publish("posture", lambda: self._state_sink.publish_posture(self.posture))
        self._publish("health", lambda: self._state_sink.publish_health(self.health))

    @staticmethod
    def _publish(name: str, operation: Callable[[], None]) -> None:
        try:
            operation()
        except Exception:
            LOGGER.exception("Could not publish %s state", name)

    def _set_requested(self, state: VelocityState) -> None:
        self.requested = state
        self._publish(
            "requested velocity", lambda: self._state_sink.publish_requested(state)
        )

    def _set_applied(self, state: VelocityState) -> None:
        self.applied = state
        self._publish(
            "applied velocity", lambda: self._state_sink.publish_applied(state)
        )

    def _set_posture(self, posture: Posture) -> None:
        self.posture = PostureState(posture=posture)
        self._publish("posture", lambda: self._state_sink.publish_posture(self.posture))

    def _update_health(
        self,
        *,
        status: NodeStatus | None = None,
        walking_enabled: bool | None = None,
        watchdog_triggered: bool | None = None,
        last_error: str | None | _Unset = _UNSET,
    ) -> None:
        current = self.health
        self.health = HealthState(
            status=current.status if status is None else status,
            walking_enabled=(
                current.walking_enabled if walking_enabled is None else walking_enabled
            ),
            watchdog_triggered=(
                current.watchdog_triggered
                if watchdog_triggered is None
                else watchdog_triggered
            ),
            last_error=current.last_error
            if isinstance(last_error, _Unset)
            else last_error,
        )
        self._publish("health", lambda: self._state_sink.publish_health(self.health))

    def _set_ready(
        self, *, walking_enabled: bool, watchdog_triggered: bool = False
    ) -> None:
        self._update_health(
            status=NodeStatus.READY,
            walking_enabled=walking_enabled,
            watchdog_triggered=watchdog_triggered,
            last_error=None,
        )

    def _set_error(self, message: str) -> None:
        self._update_health(status=NodeStatus.DEGRADED, last_error=message)

    def report_command_error(self, message: str) -> None:
        self._set_error(message)

    def _invoke(self, name: str, operation: Callable[[], int]) -> bool:
        try:
            code = operation()
        except Exception as error:
            self._set_error(f"{name} raised {type(error).__name__}: {error}")
            return False
        if code != 0:
            self._set_error(f"{name} failed with SDK code {code}")
            return False
        return True

    def startup(self) -> bool:
        if self._started:
            return self.health.status is not NodeStatus.DEGRADED
        self.publish_state()
        self._started = True
        return self._request_motion_stop(_StopReason.STARTUP, now=self._clock())

    def handle_command(
        self,
        command: Command,
        *,
        received_at: float,
        now: float | None = None,
    ) -> bool:
        now = self._clock() if now is None else now
        if isinstance(command, VelocityCommand):
            return self._handle_velocity(command, received_at=received_at, now=now)
        return self._handle_posture(command, now=now)

    def _handle_velocity(
        self, command: VelocityCommand, *, received_at: float, now: float
    ) -> bool:
        requested = VelocityState.from_command(command)
        self._set_requested(requested)
        age = now - received_at
        if age > self._timing.command_timeout_seconds:
            self._set_error(f"stale velocity command dropped after {age:.3f} seconds")
            return False
        if self._pending_stop is not None:
            return False
        if (
            self.posture.posture is not Posture.STANDING
            or not self.health.walking_enabled
        ):
            return False

        if not self._invoke(
            "Move",
            lambda: self._sport_client.Move(command.vx, command.vy, command.vyaw),
        ):
            self._request_motion_stop(
                _StopReason.MOVE_FAILURE,
                now=now,
                preceding_error=self.health.last_error,
            )
            return False

        self._set_applied(requested)
        self._last_forwarded_velocity_at = max(now, self._clock())
        self._set_ready(walking_enabled=True)
        return True

    def _handle_posture(self, command: PostureCommand, *, now: float) -> bool:
        if command.posture == "stand":
            return self._start_standing(now=now)
        return self._stand_down(now=now)

    def _start_standing(self, *, now: float) -> bool:
        if self._pending_stop is not None:
            return False
        if self.posture.posture is Posture.STANDING and self.health.walking_enabled:
            return True
        if self.posture.posture is Posture.STANDING_UP:
            return True

        self._stand_balance_deadline = None
        self._last_forwarded_velocity_at = None
        self._update_health(walking_enabled=False, watchdog_triggered=False)
        if not self._stop_before_posture_change():
            self._set_posture(Posture.UNKNOWN)
            return False

        self._set_posture(Posture.STANDING_UP)
        if not self._invoke("StandUp", self._sport_client.StandUp):
            self._set_posture(Posture.UNKNOWN)
            return False

        completed_at = max(now, self._clock())
        self._stand_balance_deadline = (
            completed_at + self._timing.posture_transition_seconds
        )
        self._set_ready(walking_enabled=False)
        return True

    def _finish_standing(self) -> bool:
        self._stand_balance_deadline = None
        if not self._invoke("BalanceStand", self._sport_client.BalanceStand):
            self._stop_before_posture_change()
            self._set_posture(Posture.UNKNOWN)
            self._update_health(walking_enabled=False)
            return False

        self._set_posture(Posture.STANDING)
        self._set_ready(walking_enabled=True)
        return True

    def _stand_down(self, *, now: float) -> bool:
        if self.posture.posture is Posture.DOWN:
            self._set_ready(walking_enabled=False)
            return True
        if (
            self._pending_stop is not None
            and self._pending_stop.reason is _StopReason.POSTURE_DOWN
        ):
            self._retry_pending_stop(now=now)
            return self.posture.posture is Posture.DOWN

        self._stand_balance_deadline = None
        self._last_forwarded_velocity_at = None
        self._update_health(walking_enabled=False, watchdog_triggered=False)
        self._set_posture(Posture.STANDING_DOWN)
        return self._request_motion_stop(_StopReason.POSTURE_DOWN, now=now)

    def _complete_stand_down(self) -> bool:
        if self._invoke("StandDown", self._sport_client.StandDown):
            self._set_posture(Posture.DOWN)
            self._set_ready(walking_enabled=False)
            return True
        self._set_posture(Posture.UNKNOWN)
        return False

    def _stop_before_posture_change(self) -> bool:
        stopped = self._invoke("StopMove", self._sport_client.StopMove)
        if stopped:
            self._set_applied(VelocityState())
        return stopped

    def _request_motion_stop(
        self,
        reason: _StopReason,
        *,
        now: float,
        preceding_error: str | None = None,
    ) -> bool:
        self._last_forwarded_velocity_at = None
        self._pending_stop = _PendingStop(
            reason=reason,
            next_attempt_at=now,
            preceding_error=preceding_error,
        )
        self._update_health(
            walking_enabled=False,
            watchdog_triggered=reason is _StopReason.WATCHDOG,
        )
        return self._retry_pending_stop(now=now)

    def _retry_pending_stop(self, *, now: float) -> bool:
        pending = self._pending_stop
        if pending is None or now < pending.next_attempt_at:
            return False
        if not self._invoke("StopMove", self._sport_client.StopMove):
            attempted_at = max(now, self._clock())
            self._pending_stop = _PendingStop(
                reason=pending.reason,
                next_attempt_at=(
                    attempted_at + self._timing.stop_retry_interval_seconds
                ),
                preceding_error=pending.preceding_error,
            )
            self._update_health(
                walking_enabled=False,
                watchdog_triggered=pending.reason is _StopReason.WATCHDOG,
            )
            return False

        self._set_applied(VelocityState())
        self._pending_stop = None
        if pending.reason is _StopReason.POSTURE_DOWN:
            return self._complete_stand_down()
        if pending.reason is _StopReason.STARTUP:
            self._set_ready(walking_enabled=False)
            return True

        walking_enabled = self.posture.posture is Posture.STANDING
        if pending.reason is _StopReason.WATCHDOG:
            self._set_ready(
                walking_enabled=walking_enabled,
                watchdog_triggered=True,
            )
        else:
            self._update_health(
                status=(
                    NodeStatus.DEGRADED
                    if pending.preceding_error is not None
                    else NodeStatus.READY
                ),
                walking_enabled=walking_enabled,
                watchdog_triggered=False,
                last_error=pending.preceding_error,
            )
        return True

    def tick(self, *, now: float | None = None) -> None:
        now = self._clock() if now is None else now
        if (
            self._stand_balance_deadline is not None
            and now >= self._stand_balance_deadline
        ):
            self._finish_standing()

        if self._pending_stop is not None:
            self._retry_pending_stop(now=now)
            return

        last_velocity = self._last_forwarded_velocity_at
        if (
            self.health.walking_enabled
            and last_velocity is not None
            and now - last_velocity >= self._timing.command_timeout_seconds
        ):
            self._request_motion_stop(_StopReason.WATCHDOG, now=now)

    def graceful_shutdown(self) -> None:
        if self._shutdown:
            return
        self._shutdown = True
        self._stand_balance_deadline = None
        self._pending_stop = None
        self._update_health(status=NodeStatus.STOPPING, walking_enabled=False)

        stopped = self._stop_before_posture_change()
        sleep_interruption: BaseException | None = None
        try:
            self._sleep(self._timing.shutdown_stop_delay_seconds)
        except BaseException as error:
            sleep_interruption = error
        if not stopped:
            stopped = self._stop_before_posture_change()
        if not stopped:
            self._set_posture(Posture.UNKNOWN)
            self._update_health(status=NodeStatus.STOPPING, walking_enabled=False)
        else:
            self._set_posture(Posture.STANDING_DOWN)
            if self._invoke("StandDown", self._sport_client.StandDown):
                self._set_posture(Posture.DOWN)
            else:
                self._set_posture(Posture.UNKNOWN)
        self._update_health(status=NodeStatus.STOPPING, walking_enabled=False)
        if sleep_interruption is not None:
            raise sleep_interruption

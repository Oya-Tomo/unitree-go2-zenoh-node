from __future__ import annotations

import logging
import time
from collections.abc import Callable
from typing import Protocol

from pydantic import ValidationError

from models import (
    Command,
    HealthState,
    NodeStatus,
    Posture,
    PostureCommand,
    PostureState,
    VelocityCommand,
    VelocityState,
    decode_command,
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


class RobotController:
    """Serialize SDK calls and enforce the high-level safety state machine."""

    def __init__(
        self,
        sport_client: SportClientProtocol,
        state_sink: StateSink,
        *,
        command_timeout_seconds: float,
        posture_transition_seconds: float,
        shutdown_stop_delay_seconds: float,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self._sport_client = sport_client
        self._state_sink = state_sink
        self._command_timeout_seconds = command_timeout_seconds
        self._posture_transition_seconds = posture_transition_seconds
        self._shutdown_stop_delay_seconds = shutdown_stop_delay_seconds
        self._clock = clock
        self._sleep = sleep

        self.requested = VelocityState()
        self.applied = VelocityState()
        self.posture = PostureState()
        self.health = HealthState()

        self._stand_balance_deadline: float | None = None
        self._last_forwarded_velocity_at: float | None = None
        self._watchdog_stopped = False
        self._started = False
        self._shutdown = False

    def _publish_initial_state(self) -> None:
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

    def _set_health(self, **updates: object) -> None:
        self.health = self.health.model_copy(update=updates)
        self._publish("health", lambda: self._state_sink.publish_health(self.health))

    def _set_ready(self, *, walking_enabled: bool) -> None:
        self._set_health(
            status=NodeStatus.READY,
            walking_enabled=walking_enabled,
            last_error=None,
        )

    def _set_error(self, message: str) -> None:
        self._set_health(status=NodeStatus.DEGRADED, last_error=message)

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
        self._publish_initial_state()
        self._started = True
        stopped = self._invoke("StopMove", self._sport_client.StopMove)
        if stopped:
            self._set_applied(VelocityState())
            self._set_ready(walking_enabled=False)
        return stopped

    def handle_payload(self, payload: bytes | str, *, now: float | None = None) -> bool:
        try:
            command = decode_command(payload)
        except (ValidationError, ValueError, TypeError) as error:
            self._set_error(f"invalid command: {error}")
            return False
        return self.handle_command(command, now=now)

    def handle_command(self, command: Command, *, now: float | None = None) -> bool:
        if isinstance(command, VelocityCommand):
            return self._handle_velocity(
                command, now=self._clock() if now is None else now
            )
        return self._handle_posture(command, now=self._clock() if now is None else now)

    def _handle_velocity(self, command: VelocityCommand, *, now: float) -> bool:
        requested = VelocityState.from_command(command)
        self._set_requested(requested)
        if (
            self.posture.posture is not Posture.STANDING
            or not self.health.walking_enabled
        ):
            return False

        if not self._invoke(
            "Move",
            lambda: self._sport_client.Move(command.vx, command.vy, command.vyaw),
        ):
            self._stop_motion(mark_watchdog=True)
            return False

        self._set_applied(requested)
        self._last_forwarded_velocity_at = now
        self._watchdog_stopped = False
        self._set_health(
            status=NodeStatus.READY,
            watchdog_triggered=False,
            last_error=None,
        )
        return True

    def _handle_posture(self, command: PostureCommand, *, now: float) -> bool:
        if command.posture == "stand":
            return self._start_standing(now=now)
        return self._stand_down()

    def _start_standing(self, *, now: float) -> bool:
        if self.posture.posture is Posture.STANDING and self.health.walking_enabled:
            return True
        if self.posture.posture is Posture.STANDING_UP:
            return True

        self._stand_balance_deadline = None
        self._last_forwarded_velocity_at = None
        self._watchdog_stopped = False
        self._set_health(walking_enabled=False, watchdog_triggered=False)
        if not self._stop_motion(mark_watchdog=False):
            self._set_posture(Posture.UNKNOWN)
            return False

        self._set_posture(Posture.STANDING_UP)
        if not self._invoke("StandUp", self._sport_client.StandUp):
            self._set_posture(Posture.UNKNOWN)
            return False

        completed_at = max(now, self._clock())
        self._stand_balance_deadline = completed_at + self._posture_transition_seconds
        self._set_ready(walking_enabled=False)
        return True

    def _finish_standing(self) -> bool:
        self._stand_balance_deadline = None
        if not self._invoke("BalanceStand", self._sport_client.BalanceStand):
            self._stop_motion(mark_watchdog=False)
            self._set_posture(Posture.UNKNOWN)
            self._set_health(walking_enabled=False)
            return False

        self._set_posture(Posture.STANDING)
        self._set_ready(walking_enabled=True)
        return True

    def _stand_down(self) -> bool:
        if self.posture.posture is Posture.DOWN:
            self._set_ready(walking_enabled=False)
            return True
        self._stand_balance_deadline = None
        self._last_forwarded_velocity_at = None
        self._watchdog_stopped = False
        self._set_health(walking_enabled=False, watchdog_triggered=False)
        stopped = self._stop_motion(mark_watchdog=False)
        if not stopped:
            self._set_posture(Posture.UNKNOWN)
            return False

        self._set_posture(Posture.STANDING_DOWN)
        down = self._invoke("StandDown", self._sport_client.StandDown)
        if down:
            self._set_applied(VelocityState())
            self._set_posture(Posture.DOWN)
            self._set_ready(walking_enabled=False)
            return True
        self._set_posture(Posture.UNKNOWN)
        return False

    def _stop_motion(self, *, mark_watchdog: bool) -> bool:
        stopped = self._invoke("StopMove", self._sport_client.StopMove)
        if stopped:
            self._set_applied(VelocityState())
        self._last_forwarded_velocity_at = None
        self._watchdog_stopped = mark_watchdog
        self._set_health(
            watchdog_triggered=mark_watchdog,
            walking_enabled=(
                False if mark_watchdog and not stopped else self.health.walking_enabled
            ),
        )
        return stopped

    def tick(self, *, now: float | None = None) -> None:
        now = self._clock() if now is None else now
        if (
            self._stand_balance_deadline is not None
            and now >= self._stand_balance_deadline
        ):
            self._finish_standing()

        last_velocity = self._last_forwarded_velocity_at
        if (
            self.health.walking_enabled
            and last_velocity is not None
            and not self._watchdog_stopped
            and now - last_velocity >= self._command_timeout_seconds
        ):
            self._stop_motion(mark_watchdog=True)

    def graceful_shutdown(self) -> None:
        if self._shutdown:
            return
        self._shutdown = True
        self._stand_balance_deadline = None
        self._set_health(status=NodeStatus.STOPPING, walking_enabled=False)
        try:
            self._stop_motion(mark_watchdog=False)
        finally:
            try:
                self._sleep(self._shutdown_stop_delay_seconds)
            finally:
                self._set_posture(Posture.STANDING_DOWN)
                if self._invoke("StandDown", self._sport_client.StandDown):
                    self._set_applied(VelocityState())
                    self._set_posture(Posture.DOWN)

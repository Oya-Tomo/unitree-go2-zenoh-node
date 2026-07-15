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
    MotionTelemetryState,
    NodeStatus,
    Posture,
    PostureCommand,
    PostureState,
    VelocityCommand,
    VelocityState,
)
from motion_state import (
    MotionStateAssessment,
    MotionStateMonitor,
    MotionStateSource,
    PostureObservationGate,
)

LOGGER = logging.getLogger(__name__)
_UNITREE_RPC_INFRASTRUCTURE_ERRORS = range(3000, 4000)


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

    def publish_motion(self, state: MotionTelemetryState) -> None: ...

    def publish_health(self, state: HealthState) -> None: ...


@dataclass(frozen=True)
class ControllerTiming:
    command_timeout_seconds: float
    posture_transition_seconds: float
    shutdown_stop_delay_seconds: float
    motion_state_max_age_seconds: float = 0.5
    balance_confirmation_timeout_seconds: float = 1.0
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
    UNSAFE_TELEMETRY = "unsafe_telemetry"


@dataclass(frozen=True)
class _PendingStop:
    reason: _StopReason
    next_attempt_at: float
    preceding_error: str | None


@dataclass(frozen=True)
class _PendingBalanceConfirmation:
    deadline: float


class _SdkCallOutcome(StrEnum):
    SUCCEEDED = "succeeded"
    REJECTED = "rejected"
    UNCERTAIN = "uncertain"

    @property
    def succeeded(self) -> bool:
        return self is _SdkCallOutcome.SUCCEEDED

    @property
    def uncertain(self) -> bool:
        return self is _SdkCallOutcome.UNCERTAIN


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
        motion_state_source: MotionStateSource | None = None,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self._sport_client = sport_client
        self._state_sink = state_sink
        self._timing = timing
        self._motion_monitor = MotionStateMonitor(
            motion_state_source,
            maximum_age=timing.motion_state_max_age_seconds,
        )
        self._clock = clock
        self._sleep = sleep

        self.requested = VelocityState()
        self.applied = VelocityState()
        self.posture = PostureState()
        self.motion = MotionTelemetryState()
        self.health = HealthState()
        self._motion_assessment = MotionStateAssessment(self.motion)

        self._stand_balance_deadline: float | None = None
        self._pending_balance_confirmation: _PendingBalanceConfirmation | None = None
        self._posture_observations = PostureObservationGate(
            stable_recovery_seconds=(
                timing.posture_transition_seconds + timing.motion_state_max_age_seconds
            ),
            maximum_sample_gap=timing.motion_state_max_age_seconds,
        )
        self._last_forwarded_velocity_at: float | None = None
        self._pending_stop: _PendingStop | None = None
        self._started = False
        self._shutdown = False

    def publish_state(self) -> None:
        now = self._clock()
        motion = self._observe_motion(now=now, force_publish=True)
        self._reconcile_stable_down(motion)
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

    def _set_motion(self, state: MotionTelemetryState) -> None:
        self.motion = state
        self._publish("motion", lambda: self._state_sink.publish_motion(state))

    def _observe_motion(
        self,
        *,
        now: float,
        force_publish: bool = False,
    ) -> MotionStateAssessment:
        motion = self._motion_monitor.observe(now=now)
        self._motion_assessment = motion
        self._posture_observations.observe(motion)
        if force_publish or motion.telemetry != self.motion:
            self._set_motion(motion.telemetry)
        return motion

    def _accept_observed_down(self, motion: MotionStateAssessment) -> bool:
        if not self._posture_observations.confirms_down(motion):
            return False
        self._stand_balance_deadline = None
        self._pending_balance_confirmation = None
        self._last_forwarded_velocity_at = None
        stopped = VelocityState()
        if self.applied != stopped:
            self._set_applied(stopped)
        if self.posture.posture is not Posture.DOWN:
            self._set_posture(Posture.DOWN)
        if self.health.walking_enabled:
            self._update_health(walking_enabled=False)
        self._posture_observations.confirmed()
        return True

    def _confirms_current_down(self, motion: MotionStateAssessment) -> bool:
        return self._posture_observations.confirms_down(motion)

    def _confirms_current_balance_stand(
        self,
        motion: MotionStateAssessment,
    ) -> bool:
        return self._posture_observations.confirms_balance_stand(motion)

    def _reconcile_stable_down(self, motion: MotionStateAssessment) -> bool:
        if self.posture.posture in (Posture.STANDING_UP, Posture.STANDING_DOWN):
            return False
        return self._accept_observed_down(motion)

    def _enforce_stable_motion_safety(
        self,
        motion: MotionStateAssessment,
        *,
        now: float,
    ) -> bool:
        if self._reconcile_stable_down(motion):
            return True
        return self._stop_for_unsafe_walking(motion, now=now)

    def _stop_for_unsafe_walking(
        self,
        motion: MotionStateAssessment,
        *,
        now: float,
    ) -> bool:
        if not self.health.walking_enabled or motion.permits_walking:
            return False
        self._request_motion_stop(
            _StopReason.UNSAFE_TELEMETRY,
            now=now,
            motion=motion,
            preceding_error=motion.walking_rejection_reason,
        )
        return True

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

    def _invoke_outcome(
        self,
        name: str,
        operation: Callable[[], int],
    ) -> _SdkCallOutcome:
        try:
            code = operation()
        except Exception as error:
            self._set_error(f"{name} raised {type(error).__name__}: {error}")
            return _SdkCallOutcome.UNCERTAIN
        if code != 0:
            self._set_error(f"{name} failed with SDK code {code}")
            # Unitree SDK2 reserves the 3xxx families for RPC transport,
            # client, and server infrastructure errors.  They do not prove
            # that a posture request was rejected before execution.
            return (
                _SdkCallOutcome.UNCERTAIN
                if code in _UNITREE_RPC_INFRASTRUCTURE_ERRORS
                else _SdkCallOutcome.REJECTED
            )
        return _SdkCallOutcome.SUCCEEDED

    def _invoke(self, name: str, operation: Callable[[], int]) -> bool:
        return self._invoke_outcome(name, operation).succeeded

    def startup(self) -> bool:
        if self._started:
            return self.health.status is not NodeStatus.DEGRADED
        self.publish_state()
        self._started = True
        if self._confirms_current_down(self._motion_assessment):
            self._set_ready(walking_enabled=False)
            return True
        return self._request_motion_stop(
            _StopReason.STARTUP,
            now=self._clock(),
            motion=self._motion_assessment,
        )

    def handle_command(
        self,
        command: Command,
        *,
        received_at: float,
        now: float | None = None,
    ) -> bool:
        now = self._clock() if now is None else now
        motion = self._observe_motion(now=now)
        if isinstance(command, VelocityCommand):
            return self._handle_velocity(
                command,
                received_at=received_at,
                now=now,
                motion=motion,
            )
        return self._handle_posture(command, now=now, motion=motion)

    def _handle_velocity(
        self,
        command: VelocityCommand,
        *,
        received_at: float,
        now: float,
        motion: MotionStateAssessment,
    ) -> bool:
        requested = VelocityState.from_command(command)
        self._set_requested(requested)
        if self._enforce_stable_motion_safety(motion, now=now):
            return False
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
                motion=motion,
            )
            return False

        self._set_applied(requested)
        self._last_forwarded_velocity_at = max(now, self._clock())
        self._set_ready(walking_enabled=True)
        return True

    def _handle_posture(
        self,
        command: PostureCommand,
        *,
        now: float,
        motion: MotionStateAssessment,
    ) -> bool:
        if command.posture == "stand":
            return self._start_standing(now=now, motion=motion)
        return self._stand_down(now=now, motion=motion)

    def _start_standing(
        self,
        *,
        now: float,
        motion: MotionStateAssessment,
    ) -> bool:
        if self._pending_stop is not None:
            self._retry_pending_stop(now=now, motion=motion)
            return False
        if self.posture.posture is Posture.STANDING_UP:
            return True
        confirmed_down = self._accept_observed_down(motion)
        if not confirmed_down and self._stop_for_unsafe_walking(motion, now=now):
            return False
        if self.posture.posture is Posture.STANDING and self.health.walking_enabled:
            return True
        if not confirmed_down and self._confirms_current_balance_stand(motion):
            self._complete_standing()
            return True

        self._stand_balance_deadline = None
        self._pending_balance_confirmation = None
        self._last_forwarded_velocity_at = None
        self._update_health(walking_enabled=False, watchdog_triggered=False)
        if not confirmed_down and not self._stop_before_posture_change():
            self._set_posture(Posture.UNKNOWN)
            return False

        self._set_posture(Posture.STANDING_UP)
        completed_at = self._invoke_stand_up(now=now)
        if completed_at is None:
            self._set_posture(Posture.UNKNOWN)
            return False

        self._stand_balance_deadline = (
            completed_at + self._timing.posture_transition_seconds
        )
        self._set_ready(walking_enabled=False)
        return True

    def _invoke_stand_up(self, *, now: float) -> float | None:
        attempted_at = max(now, self._clock())
        self._posture_observations.begin_attempt(attempted_at)
        outcome = self._invoke_outcome("StandUp", self._sport_client.StandUp)
        if not outcome.succeeded:
            self._posture_observations.stand_up_failed(
                max(attempted_at, self._clock()),
                uncertain=outcome.uncertain,
            )
            return None
        completed_at = max(attempted_at, self._clock())
        self._posture_observations.stand_up_succeeded(completed_at)
        return completed_at

    def _begin_balance_confirmation(self, *, now: float) -> bool:
        self._stand_balance_deadline = None
        attempted_at = max(now, self._clock())
        self._posture_observations.begin_attempt(attempted_at)
        outcome = self._invoke_outcome(
            "BalanceStand",
            self._sport_client.BalanceStand,
        )
        if not outcome.succeeded:
            self._posture_observations.balance_stand_failed(
                max(attempted_at, self._clock()),
                uncertain=outcome.uncertain,
            )
            if self._stop_before_posture_change():
                self._posture_observations.begin_recovery(
                    max(attempted_at, self._clock())
                )
            self._set_posture(Posture.UNKNOWN)
            self._update_health(walking_enabled=False)
            return False

        requested_at = max(attempted_at, self._clock())
        self._posture_observations.begin_attempt(requested_at)
        self._pending_balance_confirmation = _PendingBalanceConfirmation(
            deadline=(requested_at + self._timing.balance_confirmation_timeout_seconds),
        )
        return True

    def _advance_balance_confirmation(
        self,
        motion: MotionStateAssessment,
        *,
        now: float,
    ) -> bool:
        pending = self._pending_balance_confirmation
        if pending is None:
            return False
        if self._accept_observed_down(motion):
            self._set_ready(walking_enabled=False)
            return True
        if self._confirms_current_balance_stand(motion):
            self._complete_standing()
            return True
        if now < pending.deadline:
            return False

        self._pending_balance_confirmation = None
        self._set_posture(Posture.UNKNOWN)
        self._posture_observations.begin_recovery(max(now, self._clock()))
        self._request_motion_stop(
            _StopReason.UNSAFE_TELEMETRY,
            now=now,
            motion=motion,
            preceding_error=f"BalanceStand was not confirmed: {motion.description}",
        )
        return False

    def _complete_standing(self) -> None:
        self._pending_balance_confirmation = None
        self._posture_observations.confirmed()
        self._set_posture(Posture.STANDING)
        self._set_ready(walking_enabled=True)

    def _stand_down(
        self,
        *,
        now: float,
        motion: MotionStateAssessment,
    ) -> bool:
        if self._accept_observed_down(motion):
            self._set_ready(walking_enabled=False)
            return True
        if (
            self._pending_stop is not None
            and self._pending_stop.reason is _StopReason.POSTURE_DOWN
        ):
            self._retry_pending_stop(now=now, motion=motion)
            return self.posture.posture is Posture.DOWN

        self._stand_balance_deadline = None
        self._pending_balance_confirmation = None
        self._last_forwarded_velocity_at = None
        self._update_health(walking_enabled=False, watchdog_triggered=False)
        self._set_posture(Posture.STANDING_DOWN)
        return self._request_motion_stop(
            _StopReason.POSTURE_DOWN,
            now=now,
            motion=motion,
        )

    def _complete_stand_down(self, *, now: float) -> bool:
        if self._invoke_stand_down(now=now):
            self._set_posture(Posture.DOWN)
            self._set_ready(walking_enabled=False)
            return True
        self._set_posture(Posture.UNKNOWN)
        return False

    def _invoke_stand_down(self, *, now: float) -> bool:
        attempted_at = max(now, self._clock())
        self._posture_observations.begin_attempt(attempted_at)
        outcome = self._invoke_outcome("StandDown", self._sport_client.StandDown)
        if not outcome.succeeded:
            self._posture_observations.stand_down_failed(
                max(attempted_at, self._clock()),
                uncertain=outcome.uncertain,
            )
            return False
        self._posture_observations.stand_down_succeeded(
            max(attempted_at, self._clock())
        )
        return True

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
        motion: MotionStateAssessment,
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
        return self._retry_pending_stop(now=now, motion=motion)

    def _retry_pending_stop(
        self,
        *,
        now: float,
        motion: MotionStateAssessment,
    ) -> bool:
        pending = self._pending_stop
        if pending is None:
            return False
        if self._accept_observed_down(motion):
            self._pending_stop = None
            if pending.reason is _StopReason.WATCHDOG:
                self._set_ready(walking_enabled=False, watchdog_triggered=True)
            elif pending.reason is _StopReason.MOVE_FAILURE:
                self._update_health(
                    status=NodeStatus.DEGRADED,
                    walking_enabled=False,
                    watchdog_triggered=False,
                    last_error=pending.preceding_error,
                )
            else:
                self._set_ready(walking_enabled=False)
            return True
        if now < pending.next_attempt_at:
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
            return self._complete_stand_down(now=now)
        if pending.reason is _StopReason.STARTUP:
            self._set_ready(walking_enabled=False)
            return True
        if pending.reason is _StopReason.UNSAFE_TELEMETRY:
            self._update_health(
                status=NodeStatus.DEGRADED,
                walking_enabled=False,
                watchdog_triggered=False,
                last_error=pending.preceding_error,
            )
            return True

        walking_enabled = (
            self.posture.posture is Posture.STANDING and motion.permits_walking
        )
        if pending.reason is _StopReason.WATCHDOG:
            if walking_enabled:
                self._set_ready(
                    walking_enabled=True,
                    watchdog_triggered=True,
                )
            else:
                self._update_health(
                    status=NodeStatus.DEGRADED,
                    walking_enabled=False,
                    watchdog_triggered=True,
                    last_error=motion.walking_rejection_reason,
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
        motion = self._observe_motion(now=now)
        if self._pending_stop is not None:
            self._retry_pending_stop(now=now, motion=motion)
            return
        if self._pending_balance_confirmation is not None:
            self._advance_balance_confirmation(motion, now=now)
            return
        if self._enforce_stable_motion_safety(motion, now=now):
            return
        if (
            self._stand_balance_deadline is not None
            and now >= self._stand_balance_deadline
        ):
            self._begin_balance_confirmation(now=now)
            return

        last_velocity = self._last_forwarded_velocity_at
        if (
            self.health.walking_enabled
            and last_velocity is not None
            and now - last_velocity >= self._timing.command_timeout_seconds
        ):
            self._request_motion_stop(
                _StopReason.WATCHDOG,
                now=now,
                motion=motion,
            )

    def graceful_shutdown(self) -> None:
        if self._shutdown:
            return
        self._shutdown = True
        self._stand_balance_deadline = None
        self._pending_balance_confirmation = None
        self._pending_stop = None
        self._update_health(status=NodeStatus.STOPPING, walking_enabled=False)

        motion = self._observe_motion(now=self._clock())
        if self._accept_observed_down(motion):
            return

        stopped = self._stop_before_posture_change()
        sleep_interruption: BaseException | None = None
        try:
            self._sleep(self._timing.shutdown_stop_delay_seconds)
        except BaseException as error:
            sleep_interruption = error
        motion = self._observe_motion(now=self._clock())
        already_down = self._accept_observed_down(motion)
        if already_down:
            stopped = True
        elif not stopped:
            stopped = self._stop_before_posture_change()
        if not stopped:
            self._set_posture(Posture.UNKNOWN)
            self._update_health(status=NodeStatus.STOPPING, walking_enabled=False)
        elif not already_down:
            self._set_posture(Posture.STANDING_DOWN)
            if self._invoke_stand_down(now=self._clock()):
                self._set_posture(Posture.DOWN)
            else:
                self._set_posture(Posture.UNKNOWN)
        self._update_health(status=NodeStatus.STOPPING, walking_enabled=False)
        if sleep_interruption is not None:
            raise sleep_interruption

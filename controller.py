"""Fresh-State-driven command selection for one Unitree Go2."""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from dataclasses import dataclass, replace
from enum import IntEnum
from functools import partial
from threading import Condition
from typing import Protocol

from wire import (
    Lifecycle,
    Motion,
    PostureCommand,
    PostureTarget,
    PublishedNodeState,
    PublishedRobotState,
    RobotState,
    SdkCommand,
    SdkDiagnostic,
    UnknownReason,
    VelocityCommand,
)

ZERO_VELOCITY = VelocityCommand(vx=0.0, vy=0.0, vyaw=0.0)


class Go2SportMode(IntEnum):
    IDLE = 0
    BALANCE_STAND = 1
    POSE = 2
    LOCOMOTION = 3
    LIE_DOWN = 5
    RECOVERY_STAND = 8


class Go2MotionStateMachine(IntEnum):
    """Go2 V2.0 motion state machine IDs carried in ``error_code``."""

    AGILE = 100
    DAMPING = 1001
    STANDING_LOCK = 1002
    CROUCH = 1004
    SPECIAL_ACTION = 1006
    SIT = 1007
    FRONT_JUMP = 1008
    LUNGE = 1009
    BALANCE_STANDING = 1013
    REGULAR_WALKING = 1015
    REGULAR_RUNNING = 1016
    REGULAR_ENDURANCE = 1017
    POSE = 1091
    ALTERNATE_CROUCH = 2006
    DODGE = 2007
    BOUND_RUN = 2008
    JUMP_RUN = 2009
    CLASSIC = 2010
    HANDSTAND = 2011
    FRONT_FLIP = 2012
    BACK_FLIP = 2013
    LEFT_FLIP = 2014
    CROSS_STEP = 2016
    UPRIGHT = 2017
    TOWING = 2019


_ROBOT_STATES: dict[tuple[int, int], RobotState] = {
    (Go2MotionStateMachine.DAMPING, Go2SportMode.IDLE): RobotState.DAMPING,
    (Go2MotionStateMachine.CROUCH, Go2SportMode.LIE_DOWN): RobotState.DOWN,
    (Go2MotionStateMachine.ALTERNATE_CROUCH, Go2SportMode.LIE_DOWN): RobotState.DOWN,
    (Go2MotionStateMachine.STANDING_LOCK, Go2SportMode.IDLE): RobotState.LOCKED_STAND,
    (Go2MotionStateMachine.AGILE, Go2SportMode.IDLE): RobotState.READY_STAND,
    (Go2MotionStateMachine.AGILE, Go2SportMode.BALANCE_STAND): RobotState.READY_STAND,
    (
        Go2MotionStateMachine.BALANCE_STANDING,
        Go2SportMode.BALANCE_STAND,
    ): RobotState.READY_STAND,
    (Go2MotionStateMachine.AGILE, Go2SportMode.LOCOMOTION): RobotState.LOCOMOTION,
    (
        Go2MotionStateMachine.BALANCE_STANDING,
        Go2SportMode.LOCOMOTION,
    ): RobotState.LOCOMOTION,
}
_POSTURE_REACHED = {
    PostureTarget.STAND: RobotState.READY_STAND,
    PostureTarget.DOWN: RobotState.DOWN,
}


class SportClientProtocol(Protocol):
    def StandUp(self) -> int: ...

    def RecoveryStand(self) -> int: ...

    def BalanceStand(self) -> int: ...

    def StopMove(self) -> int: ...

    def StandDown(self) -> int: ...

    def Move(self, vx: float, vy: float, vyaw: float) -> int: ...


@dataclass(frozen=True, slots=True)
class RobotObservation:
    received_at: float
    stamp_sec: int
    stamp_nanosec: int
    state_machine_code: int
    mode: int
    velocity: tuple[float, float, float]
    yaw_speed: float

    def __post_init__(self) -> None:
        values = (self.received_at, *self.velocity, self.yaw_speed)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("State contains a non-finite value")
        if not 0 <= self.stamp_nanosec < 1_000_000_000:
            raise ValueError("State nanosecond stamp is outside its valid range")


def classify(
    observation: RobotObservation,
    *,
    quiescent_linear_speed_mps: float,
    quiescent_yaw_rate_rad_s: float,
) -> tuple[RobotState, Motion]:
    """Classify one valid State sample without inferring from SDK results."""

    moving = (
        max(abs(component) for component in observation.velocity)
        > quiescent_linear_speed_mps
        or abs(observation.yaw_speed) > quiescent_yaw_rate_rad_s
    )
    motion = Motion.MOVING if moving else Motion.QUIESCENT
    robot = _ROBOT_STATES.get(
        (observation.state_machine_code, observation.mode),
        RobotState.UNSUPPORTED,
    )
    if robot is RobotState.DOWN and moving:
        robot = RobotState.UNSUPPORTED
    return (robot, motion)


def posture_action(
    target: PostureTarget,
    robot: RobotState,
    motion: Motion,
    last_action: SdkCommand | None,
) -> SdkCommand | None:
    """Select the next posture SDK action, or None to wait for a later State."""

    if target is PostureTarget.STAND:
        if robot is RobotState.DAMPING:
            return SdkCommand.RECOVERY_STAND if motion is Motion.QUIESCENT else None
        if robot is RobotState.DOWN:
            return SdkCommand.STAND_UP
        if robot in {RobotState.LOCKED_STAND, RobotState.LOCOMOTION}:
            return SdkCommand.BALANCE_STAND
        return None
    if robot not in {
        RobotState.LOCKED_STAND,
        RobotState.READY_STAND,
        RobotState.LOCOMOTION,
    }:
        return None
    if last_action is None:
        return SdkCommand.STOP_MOVE
    if last_action is SdkCommand.STOP_MOVE and motion is Motion.QUIESCENT:
        return SdkCommand.STAND_DOWN
    return None


def _member_name(enum_type: type[IntEnum], value: int) -> str:
    return enum_type(value).name.lower() if value in enum_type else "unsupported"


@dataclass(frozen=True, slots=True)
class ReceivedCommand[C: VelocityCommand | PostureCommand]:
    command: C
    received_at: float


@dataclass(frozen=True, slots=True)
class NodeState:
    """Canonical in-process state shared by every runtime execution context."""

    lifecycle: Lifecycle = Lifecycle.STARTING
    observation: RobotObservation | None = None
    requested_posture: ReceivedCommand[PostureCommand] | None = None
    requested_velocity: ReceivedCommand[VelocityCommand] | None = None
    active_posture: PostureTarget | None = None
    last_posture_action: SdkCommand | None = None
    awaiting_observation_after: float | None = None
    last_sdk_diagnostic: SdkDiagnostic | None = None
    last_node_error: str | None = None


@dataclass(frozen=True, slots=True)
class _SdkInvocation:
    command: SdkCommand
    velocity: VelocityCommand | None = None


class Controller:
    """Own the shared Node State and run one control step per fresh DDS State."""

    def __init__(
        self,
        client: SportClientProtocol,
        *,
        state_freshness_seconds: float,
        quiescent_linear_speed_mps: float,
        quiescent_yaw_rate_rad_s: float,
        velocity_command_timeout_seconds: float,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._client = client
        self._posture_calls: dict[SdkCommand, Callable[[], int]] = {
            SdkCommand.STAND_UP: client.StandUp,
            SdkCommand.RECOVERY_STAND: client.RecoveryStand,
            SdkCommand.BALANCE_STAND: client.BalanceStand,
            SdkCommand.STOP_MOVE: client.StopMove,
            SdkCommand.STAND_DOWN: client.StandDown,
        }
        self._state_freshness_seconds = state_freshness_seconds
        self._velocity_command_timeout_seconds = velocity_command_timeout_seconds
        self._classify = partial(
            classify,
            quiescent_linear_speed_mps=quiescent_linear_speed_mps,
            quiescent_yaw_rate_rad_s=quiescent_yaw_rate_rad_s,
        )
        self._clock = clock
        self._condition = Condition()
        self._state = NodeState()

    @property
    def shutdown_complete(self) -> bool:
        with self._condition:
            return (
                self._state.lifecycle is Lifecycle.SHUTTING_DOWN
                and self._state.active_posture is None
            )

    def receive_observation(self, observation: RobotObservation) -> bool:
        """Replace the newest DDS observation without running control or I/O."""

        with self._condition:
            current = self._state.observation
            if current is not None and observation.received_at <= current.received_at:
                return False
            self._state = replace(self._state, observation=observation)
            self._condition.notify_all()
            return True

    def reject_observation(self, error: str) -> None:
        """Record a malformed DDS sample without refreshing connection freshness."""

        with self._condition:
            self._state = replace(self._state, last_node_error=error)

    def receive_command(
        self,
        command: VelocityCommand | PostureCommand,
        *,
        received_at: float,
    ) -> bool:
        """Apply the shared command gate and keep the newest request of each kind."""

        with self._condition:
            state = self._state
            if isinstance(command, VelocityCommand):
                current = state.requested_velocity
                updated = replace(
                    state,
                    requested_velocity=ReceivedCommand(command, received_at),
                )
            else:
                current = state.requested_posture
                updated = replace(
                    state,
                    requested_posture=ReceivedCommand(command, received_at),
                )
            if not self._accepting(state, self._clock()) or (
                current is not None and received_at < current.received_at
            ):
                return False
            self._state = updated
            return True

    def step(self, *, after: float | None, timeout: float) -> float | None:
        """Consume the newest observation and issue at most one SDK command.

        Waits up to ``timeout`` for an observation received after ``after`` and
        enforces freshness either way. Returns the newest observation's receive
        time, which the single control worker passes back as ``after``.
        """

        with self._condition:
            self._condition.wait_for(
                lambda: (
                    self._state.observation is not None
                    and (after is None or self._state.observation.received_at > after)
                ),
                timeout=timeout,
            )
            state = self._state
            observation = state.observation
            if observation is None:
                return None
            now = self._clock()
            if not self._connected(state, now):
                self._state = self._disconnected(state)
                return observation.received_at
            barrier = state.awaiting_observation_after
            if (after is not None and observation.received_at <= after) or (
                barrier is not None and observation.received_at <= barrier
            ):
                return observation.received_at

            robot, motion = self._classify(observation)
            state = replace(
                state,
                awaiting_observation_after=None,
                lifecycle=(
                    Lifecycle.RUNNING
                    if state.lifecycle is Lifecycle.STARTING
                    else state.lifecycle
                ),
            )
            self._state, invocation = self._select(state, robot, motion, now)

        if invocation is not None:
            diagnostic = self._invoke(invocation)
            with self._condition:
                state = self._state
                self._state = replace(
                    state,
                    last_sdk_diagnostic=diagnostic,
                    awaiting_observation_after=(
                        state.awaiting_observation_after
                        if invocation.command is SdkCommand.MOVE
                        else self._clock()
                    ),
                )
        return observation.received_at

    def begin_shutdown(self) -> None:
        with self._condition:
            self._state = replace(
                self._state,
                lifecycle=Lifecycle.SHUTTING_DOWN,
                requested_posture=None,
                requested_velocity=None,
                active_posture=PostureTarget.DOWN,
                last_posture_action=None,
            )

    def finish_shutdown(self, error: str | None = None) -> None:
        with self._condition:
            self._state = replace(
                self._state,
                lifecycle=Lifecycle.STOPPED,
                requested_posture=None,
                requested_velocity=None,
                active_posture=None,
                last_node_error=(
                    error if error is not None else self._state.last_node_error
                ),
            )

    def published_state(self) -> PublishedNodeState:
        """Project a coherent public snapshot without exposing monotonic timestamps."""

        with self._condition:
            state = self._state
        now = self._clock()
        observation = state.observation
        connected = self._connected(state, now)
        if observation is None or not connected:
            robot_state = PublishedRobotState.unknown(
                UnknownReason.NO_SAMPLE if observation is None else UnknownReason.STALE
            )
        elif state.awaiting_observation_after is not None:
            robot_state = PublishedRobotState.unknown(UnknownReason.AWAITING_STATE)
        else:
            robot, motion = self._classify(observation)
            robot_state = PublishedRobotState(
                state=robot,
                motion=motion,
                state_machine_code=observation.state_machine_code,
                state_machine_name=_member_name(
                    Go2MotionStateMachine, observation.state_machine_code
                ),
                mode=observation.mode,
                mode_name=_member_name(Go2SportMode, observation.mode),
                velocity=observation.velocity,
                yaw_speed=observation.yaw_speed,
                stamp_sec=observation.stamp_sec,
                stamp_nanosec=observation.stamp_nanosec,
            )

        posture = state.requested_posture
        velocity = state.requested_velocity
        return PublishedNodeState(
            lifecycle=state.lifecycle,
            robot_connected=connected,
            robot_state=robot_state,
            accepting_commands=self._accepting(state, now),
            requested_posture=(
                (
                    posture.command.posture
                    if posture is not None
                    else state.active_posture
                )
                if connected
                else None
            ),
            requested_velocity=(
                velocity.command if connected and velocity is not None else None
            ),
            last_posture_action=state.last_posture_action,
            last_sdk_diagnostic=state.last_sdk_diagnostic,
            last_node_error=state.last_node_error,
        )

    def _connected(self, state: NodeState, now: float) -> bool:
        """Return whether the newest valid DDS observation is still fresh."""

        observation = state.observation
        return (
            observation is not None
            and now < observation.received_at + self._state_freshness_seconds
        )

    def _accepting(self, state: NodeState, now: float) -> bool:
        return (
            state.lifecycle is Lifecycle.RUNNING
            and state.awaiting_observation_after is None
            and self._connected(state, now)
        )

    def _disconnected(self, state: NodeState) -> NodeState:
        shutting_down = state.lifecycle is Lifecycle.SHUTTING_DOWN
        return replace(
            state,
            requested_posture=None,
            requested_velocity=None,
            active_posture=PostureTarget.DOWN if shutting_down else None,
            last_posture_action=state.last_posture_action if shutting_down else None,
            awaiting_observation_after=None,
        )

    def _select(
        self,
        state: NodeState,
        robot: RobotState,
        motion: Motion,
        now: float,
    ) -> tuple[NodeState, _SdkInvocation | None]:
        """Apply posture priority and select at most one SDK effect."""

        posture = state.requested_posture
        if posture is not None:
            state = replace(
                state,
                requested_posture=None,
                active_posture=posture.command.posture,
                last_posture_action=None,
            )

        target = state.active_posture
        if target is not None:
            state = replace(state, requested_velocity=None)
            if robot is _POSTURE_REACHED[target] or (
                posture is not None and robot is RobotState.UNSUPPORTED
            ):
                return (
                    replace(state, active_posture=None, last_posture_action=None),
                    None,
                )
            action = posture_action(target, robot, motion, state.last_posture_action)
            if action is None or action is state.last_posture_action:
                return (state, None)
            return (
                replace(
                    state,
                    last_posture_action=action,
                    awaiting_observation_after=now,
                ),
                _SdkInvocation(action),
            )

        velocity = state.requested_velocity
        if velocity is None:
            return (state, None)
        if robot not in {RobotState.READY_STAND, RobotState.LOCOMOTION}:
            return (replace(state, requested_velocity=None), None)
        if now >= velocity.received_at + self._velocity_command_timeout_seconds:
            return (
                replace(state, requested_velocity=None),
                _SdkInvocation(SdkCommand.MOVE, ZERO_VELOCITY),
            )
        return (state, _SdkInvocation(SdkCommand.MOVE, velocity.command))

    def _invoke(self, invocation: _SdkInvocation) -> SdkDiagnostic:
        command = invocation.command
        velocity = invocation.velocity
        try:
            code = (
                self._posture_calls[command]()
                if velocity is None
                else self._client.Move(velocity.vx, velocity.vy, velocity.vyaw)
            )
        except Exception as error:
            return SdkDiagnostic(
                command=command,
                velocity=velocity,
                error=f"{command} raised {type(error).__name__}: {error}",
            )
        return SdkDiagnostic(
            command=command,
            velocity=velocity,
            code=code,
            error=None if code == 0 else f"{command} failed with SDK code {code}",
        )

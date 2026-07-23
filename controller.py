"""Fresh-State-driven command selection for one Unitree Go2."""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from dataclasses import dataclass, replace
from enum import IntEnum, StrEnum
from threading import Condition
from typing import Annotated, Literal, Protocol

from pydantic import BaseModel, ConfigDict, Field, TypeAdapter

VX_MIN = -2.5
VX_MAX = 3.8
VY_MAX = 1.0
VYAW_MAX = 4.0
JSON_ENCODING = "application/json"


@dataclass(frozen=True, slots=True)
class Keyspace:
    prefix: str

    @property
    def command(self) -> str:
        return f"{self.prefix}/command"

    @property
    def state(self) -> str:
        return f"{self.prefix}/state"


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


class PostureTarget(StrEnum):
    STAND = "stand"
    DOWN = "down"


class RobotState(StrEnum):
    DAMPING = "damping"
    DOWN = "down"
    LOCKED_STAND = "locked_stand"
    READY_STAND = "ready_stand"
    LOCOMOTION = "locomotion"
    UNSUPPORTED = "unsupported"
    UNKNOWN = "unknown"


class Motion(StrEnum):
    QUIESCENT = "quiescent"
    MOVING = "moving"
    UNKNOWN = "unknown"


class UnknownReason(StrEnum):
    NO_SAMPLE = "no_sample"
    STALE = "stale"
    AWAITING_STATE = "awaiting_state"


class SdkCommand(StrEnum):
    STAND_UP = "stand_up"
    RECOVERY_STAND = "recovery_stand"
    BALANCE_STAND = "balance_stand"
    STOP_MOVE = "stop_move"
    STAND_DOWN = "stand_down"
    MOVE = "move"


class Lifecycle(StrEnum):
    STARTING = "starting"
    RUNNING = "running"
    SHUTTING_DOWN = "shutting_down"
    STOPPED = "stopped"


class WireModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


class VelocityCommand(WireModel):
    type: Literal["velocity"] = "velocity"
    vx: Annotated[float, Field(strict=True, ge=VX_MIN, le=VX_MAX)]
    vy: Annotated[float, Field(strict=True, ge=-VY_MAX, le=VY_MAX)]
    vyaw: Annotated[float, Field(strict=True, ge=-VYAW_MAX, le=VYAW_MAX)]


class PostureCommand(WireModel):
    type: Literal["posture"] = "posture"
    posture: PostureTarget


Command = Annotated[VelocityCommand | PostureCommand, Field(discriminator="type")]
COMMAND_ADAPTER = TypeAdapter(Command)


def decode_command(payload: bytes | str) -> VelocityCommand | PostureCommand:
    return COMMAND_ADAPTER.validate_json(payload)


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


class PhysicalState(WireModel):
    state: RobotState
    reason: UnknownReason | None
    motion: Motion
    state_machine_code: int | None
    state_machine_name: str | None
    mode: int | None
    mode_name: str | None
    velocity: tuple[float, float, float] | None
    yaw_speed: float | None
    stamp_sec: int | None
    stamp_nanosec: int | None

    @classmethod
    def unknown(cls, reason: UnknownReason) -> PhysicalState:
        return cls(
            state=RobotState.UNKNOWN,
            reason=reason,
            motion=Motion.UNKNOWN,
            state_machine_code=None,
            state_machine_name=None,
            mode=None,
            mode_name=None,
            velocity=None,
            yaw_speed=None,
            stamp_sec=None,
            stamp_nanosec=None,
        )


def mode_name(mode: int) -> str:
    try:
        return Go2SportMode(mode).name.lower()
    except ValueError:
        return "unsupported"


def state_machine_name(code: int) -> str:
    try:
        return Go2MotionStateMachine(code).name.lower()
    except ValueError:
        return "unsupported"


def classify_state(
    observation: RobotObservation,
    *,
    quiescent_linear_speed_mps: float,
    quiescent_yaw_rate_rad_s: float,
) -> PhysicalState:
    """Classify one valid State sample without inferring from SDK results."""

    moving = (
        max(abs(component) for component in observation.velocity)
        > quiescent_linear_speed_mps
        or abs(observation.yaw_speed) > quiescent_yaw_rate_rad_s
    )
    motion = Motion.MOVING if moving else Motion.QUIESCENT
    machine = observation.state_machine_code
    mode = observation.mode

    if machine == Go2MotionStateMachine.DAMPING and mode == Go2SportMode.IDLE:
        state = RobotState.DAMPING
    elif (
        machine
        in {Go2MotionStateMachine.CROUCH, Go2MotionStateMachine.ALTERNATE_CROUCH}
        and mode == Go2SportMode.LIE_DOWN
        and motion is Motion.QUIESCENT
    ):
        state = RobotState.DOWN
    elif machine == Go2MotionStateMachine.STANDING_LOCK and mode == Go2SportMode.IDLE:
        state = RobotState.LOCKED_STAND
    elif (
        machine == Go2MotionStateMachine.AGILE
        and mode in {Go2SportMode.IDLE, Go2SportMode.BALANCE_STAND}
    ) or (
        machine == Go2MotionStateMachine.BALANCE_STANDING
        and mode == Go2SportMode.BALANCE_STAND
    ):
        state = RobotState.READY_STAND
    elif (
        machine
        in {
            Go2MotionStateMachine.AGILE,
            Go2MotionStateMachine.BALANCE_STANDING,
        }
        and mode == Go2SportMode.LOCOMOTION
    ):
        state = RobotState.LOCOMOTION
    else:
        state = RobotState.UNSUPPORTED

    return PhysicalState(
        state=state,
        reason=None,
        motion=motion,
        state_machine_code=machine,
        state_machine_name=state_machine_name(machine),
        mode=mode,
        mode_name=mode_name(mode),
        velocity=observation.velocity,
        yaw_speed=observation.yaw_speed,
        stamp_sec=observation.stamp_sec,
        stamp_nanosec=observation.stamp_nanosec,
    )


@dataclass(frozen=True, slots=True)
class VelocityRequest:
    vx: float
    vy: float
    vyaw: float
    received_at: float

    @property
    def values(self) -> tuple[float, float, float]:
        return (self.vx, self.vy, self.vyaw)


@dataclass(frozen=True, slots=True)
class PostureRequest:
    target: PostureTarget
    received_at: float


class SportClientProtocol(Protocol):
    def StandUp(self) -> int: ...

    def RecoveryStand(self) -> int: ...

    def BalanceStand(self) -> int: ...

    def StopMove(self) -> int: ...

    def StandDown(self) -> int: ...

    def Move(self, vx: float, vy: float, vyaw: float) -> int: ...


class SdkDiagnostic(WireModel):
    command: SdkCommand
    velocity: tuple[float, float, float] | None = None
    code: int | None = None
    error: str | None = None


class VelocitySnapshot(WireModel):
    vx: float
    vy: float
    vyaw: float


class PublishedNodeState(WireModel):
    lifecycle: Lifecycle
    robot_connected: bool
    robot_state: PhysicalState
    accepting_commands: bool
    requested_posture: PostureTarget | None
    requested_velocity: VelocitySnapshot | None
    last_posture_action: SdkCommand | None
    last_sdk_diagnostic: SdkDiagnostic | None
    last_node_error: str | None


@dataclass(frozen=True, slots=True)
class NodeState:
    """Canonical in-process state shared by every runtime execution context."""

    observation: RobotObservation | None
    lifecycle: Lifecycle
    requested_posture: PostureRequest | None
    requested_velocity: VelocityRequest | None
    active_posture: PostureTarget | None
    last_posture_action: SdkCommand | None
    awaiting_observation_after: float | None
    last_sdk_diagnostic: SdkDiagnostic | None
    last_node_error: str | None

    @classmethod
    def initial(cls) -> NodeState:
        return cls(
            observation=None,
            lifecycle=Lifecycle.STARTING,
            requested_posture=None,
            requested_velocity=None,
            active_posture=None,
            last_posture_action=None,
            awaiting_observation_after=None,
            last_sdk_diagnostic=None,
            last_node_error=None,
        )


@dataclass(frozen=True, slots=True)
class _SdkInvocation:
    command: SdkCommand
    velocity: tuple[float, float, float] | None = None


def is_robot_connected(
    state: NodeState,
    *,
    now: float,
    state_freshness_seconds: float,
) -> bool:
    """Return whether the newest valid DDS observation is still fresh."""

    observation = state.observation
    return (
        observation is not None
        and now < observation.received_at + state_freshness_seconds
    )


def _accepting_commands(
    state: NodeState,
    *,
    now: float,
    state_freshness_seconds: float,
) -> bool:
    return (
        state.lifecycle is Lifecycle.RUNNING
        and state.awaiting_observation_after is None
        and is_robot_connected(
            state,
            now=now,
            state_freshness_seconds=state_freshness_seconds,
        )
    )


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
        self._state_freshness_seconds = state_freshness_seconds
        self._quiescent_linear_speed_mps = quiescent_linear_speed_mps
        self._quiescent_yaw_rate_rad_s = quiescent_yaw_rate_rad_s
        self._velocity_command_timeout_seconds = velocity_command_timeout_seconds
        self._clock = clock
        self._condition = Condition()
        self._state = NodeState.initial()

    @property
    def has_observation(self) -> bool:
        with self._condition:
            return self._state.observation is not None

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
        """Atomically validate the shared command gate and keep the newest request."""

        with self._condition:
            state = self._state
            if not _accepting_commands(
                state,
                now=self._clock(),
                state_freshness_seconds=self._state_freshness_seconds,
            ):
                return False

            if isinstance(command, VelocityCommand):
                current = state.requested_velocity
                if current is not None and received_at < current.received_at:
                    return False
                request = VelocityRequest(
                    command.vx,
                    command.vy,
                    command.vyaw,
                    received_at,
                )
                self._state = replace(state, requested_velocity=request)
            else:
                current = state.requested_posture
                if current is not None and received_at < current.received_at:
                    return False
                request = PostureRequest(command.posture, received_at)
                self._state = replace(state, requested_posture=request)
            return True

    def wait_for_observation(
        self,
        *,
        after: float | None,
        timeout: float,
    ) -> bool:
        """Wait until DDS has replaced the latest unprocessed observation."""

        with self._condition:
            return self._condition.wait_for(
                lambda: (
                    self._state.observation is not None
                    and (after is None or self._state.observation.received_at > after)
                ),
                timeout=timeout,
            )

    def process_observation(self, *, after: float | None) -> float | None:
        """Process the newest DDS observation and issue at most one SDK command."""

        invocation: _SdkInvocation | None = None
        with self._condition:
            state = self._state
            if state.observation is None or (
                after is not None and state.observation.received_at <= after
            ):
                return None

            observation = state.observation
            processed_received_at = observation.received_at
            now = self._clock()
            if not is_robot_connected(
                state,
                now=now,
                state_freshness_seconds=self._state_freshness_seconds,
            ):
                self._state = self._disconnected_state(state)
                return processed_received_at

            if (
                state.awaiting_observation_after is not None
                and observation.received_at <= state.awaiting_observation_after
            ):
                return processed_received_at

            robot_state = classify_state(
                observation,
                quiescent_linear_speed_mps=self._quiescent_linear_speed_mps,
                quiescent_yaw_rate_rad_s=self._quiescent_yaw_rate_rad_s,
            )
            state = replace(
                state,
                awaiting_observation_after=None,
                lifecycle=(
                    Lifecycle.RUNNING
                    if state.lifecycle is Lifecycle.STARTING
                    else state.lifecycle
                ),
            )
            if state.lifecycle is Lifecycle.SHUTTING_DOWN:
                state, invocation = self._process_posture(
                    state,
                    robot_state,
                    new_request=False,
                    now=now,
                )
            elif state.lifecycle is Lifecycle.RUNNING:
                state, invocation = self._process_commands(
                    state,
                    robot_state,
                    now=now,
                )
            self._state = state

        if invocation is not None:
            diagnostic = self._invoke(invocation)
            with self._condition:
                state = self._state
                self._state = replace(
                    state,
                    last_sdk_diagnostic=diagnostic,
                    awaiting_observation_after=(
                        self._clock()
                        if invocation.command is not SdkCommand.MOVE
                        else state.awaiting_observation_after
                    ),
                )
        return processed_received_at

    def enforce_observation_freshness(self) -> None:
        """Clear unsafe control state when the latest observation is stale."""

        with self._condition:
            state = self._state
            if state.observation is None or is_robot_connected(
                state,
                now=self._clock(),
                state_freshness_seconds=self._state_freshness_seconds,
            ):
                return
            disconnected = self._disconnected_state(state)
            if disconnected != state:
                self._state = disconnected

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

    def published_state(self, *, now: float | None = None) -> PublishedNodeState:
        """Project a coherent public snapshot without exposing monotonic timestamps."""

        now = self._clock() if now is None else now
        with self._condition:
            state = self._state

        connected = is_robot_connected(
            state,
            now=now,
            state_freshness_seconds=self._state_freshness_seconds,
        )
        if connected:
            observation = state.observation
            assert observation is not None
            if state.awaiting_observation_after is None:
                robot_state = classify_state(
                    observation,
                    quiescent_linear_speed_mps=self._quiescent_linear_speed_mps,
                    quiescent_yaw_rate_rad_s=self._quiescent_yaw_rate_rad_s,
                )
            else:
                robot_state = PhysicalState.unknown(UnknownReason.AWAITING_STATE)
            posture = (
                state.requested_posture.target
                if state.requested_posture is not None
                else state.active_posture
            )
            velocity = state.requested_velocity
        else:
            reason = (
                UnknownReason.NO_SAMPLE
                if state.observation is None
                else UnknownReason.STALE
            )
            robot_state = PhysicalState.unknown(reason)
            posture = None
            velocity = None

        return PublishedNodeState(
            lifecycle=state.lifecycle,
            robot_connected=connected,
            robot_state=robot_state,
            accepting_commands=_accepting_commands(
                state,
                now=now,
                state_freshness_seconds=self._state_freshness_seconds,
            ),
            requested_posture=posture,
            requested_velocity=(
                VelocitySnapshot(vx=velocity.vx, vy=velocity.vy, vyaw=velocity.vyaw)
                if velocity is not None
                else None
            ),
            last_posture_action=state.last_posture_action,
            last_sdk_diagnostic=state.last_sdk_diagnostic,
            last_node_error=state.last_node_error,
        )

    def _disconnected_state(self, state: NodeState) -> NodeState:
        shutting_down = state.lifecycle is Lifecycle.SHUTTING_DOWN
        return replace(
            state,
            requested_posture=None,
            requested_velocity=None,
            active_posture=PostureTarget.DOWN if shutting_down else None,
            last_posture_action=state.last_posture_action if shutting_down else None,
            awaiting_observation_after=None,
        )

    def _process_commands(
        self,
        state: NodeState,
        robot_state: PhysicalState,
        *,
        now: float,
    ) -> tuple[NodeState, _SdkInvocation | None]:
        posture = state.requested_posture
        if posture is not None:
            state = replace(
                state,
                requested_posture=None,
                requested_velocity=None,
                active_posture=posture.target,
                last_posture_action=None,
            )

        if state.active_posture is not None:
            state = replace(state, requested_velocity=None)
            return self._process_posture(
                state,
                robot_state,
                new_request=posture is not None,
                now=now,
            )

        if state.requested_velocity is not None:
            return self._process_velocity(state, robot_state, now=now)
        return (state, None)

    def _process_posture(
        self,
        node_state: NodeState,
        robot_state: PhysicalState,
        *,
        new_request: bool,
        now: float,
    ) -> tuple[NodeState, _SdkInvocation | None]:
        target = node_state.active_posture
        state = robot_state.state
        if target is None:
            return (node_state, None)
        if state is RobotState.UNSUPPORTED:
            if new_request:
                node_state = self._complete_posture(node_state)
            return (node_state, None)

        action: SdkCommand | None = None
        if target is PostureTarget.STAND:
            if state is RobotState.READY_STAND:
                return (self._complete_posture(node_state), None)
            if state is RobotState.DAMPING and robot_state.motion is Motion.QUIESCENT:
                action = SdkCommand.RECOVERY_STAND
            elif state is RobotState.DOWN:
                action = SdkCommand.STAND_UP
            elif state in {RobotState.LOCKED_STAND, RobotState.LOCOMOTION}:
                action = SdkCommand.BALANCE_STAND
        else:
            if state is RobotState.DOWN:
                return (self._complete_posture(node_state), None)
            if state in {
                RobotState.LOCKED_STAND,
                RobotState.READY_STAND,
                RobotState.LOCOMOTION,
            }:
                if node_state.last_posture_action is None:
                    action = SdkCommand.STOP_MOVE
                elif (
                    node_state.last_posture_action is SdkCommand.STOP_MOVE
                    and robot_state.motion is Motion.QUIESCENT
                ):
                    action = SdkCommand.STAND_DOWN

        if action is None or action is node_state.last_posture_action:
            return (node_state, None)
        return self._dispatch_posture(node_state, action, now=now)

    def _complete_posture(self, state: NodeState) -> NodeState:
        return replace(
            state,
            active_posture=None,
            last_posture_action=None,
        )

    def _process_velocity(
        self,
        state: NodeState,
        robot_state: PhysicalState,
        *,
        now: float,
    ) -> tuple[NodeState, _SdkInvocation | None]:
        request = state.requested_velocity
        assert request is not None
        if robot_state.state not in {
            RobotState.READY_STAND,
            RobotState.LOCOMOTION,
        }:
            return (replace(state, requested_velocity=None), None)
        expired = now >= request.received_at + self._velocity_command_timeout_seconds
        velocity = (0.0, 0.0, 0.0) if expired else request.values
        if expired:
            state = replace(state, requested_velocity=None)
        return (state, _SdkInvocation(SdkCommand.MOVE, velocity))

    def _dispatch_posture(
        self,
        state: NodeState,
        command: SdkCommand,
        *,
        now: float,
    ) -> tuple[NodeState, _SdkInvocation]:
        return (
            replace(
                state,
                requested_posture=None,
                requested_velocity=None,
                last_posture_action=command,
                awaiting_observation_after=now,
            ),
            _SdkInvocation(command),
        )

    def _invoke(
        self,
        invocation: _SdkInvocation,
    ) -> SdkDiagnostic:
        command = invocation.command
        velocity = invocation.velocity
        try:
            if command is SdkCommand.STAND_UP:
                code = self._client.StandUp()
            elif command is SdkCommand.RECOVERY_STAND:
                code = self._client.RecoveryStand()
            elif command is SdkCommand.BALANCE_STAND:
                code = self._client.BalanceStand()
            elif command is SdkCommand.STOP_MOVE:
                code = self._client.StopMove()
            elif command is SdkCommand.STAND_DOWN:
                code = self._client.StandDown()
            else:
                assert velocity is not None
                code = self._client.Move(*velocity)
            error = None if code == 0 else f"{command} failed with SDK code {code}"
            return SdkDiagnostic(
                command=command,
                velocity=velocity,
                code=code,
                error=error,
            )
        except Exception as error:
            message = f"{command} raised {type(error).__name__}: {error}"
            return SdkDiagnostic(
                command=command,
                velocity=velocity,
                error=message,
            )

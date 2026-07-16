"""Fresh-State-driven command selection for one Unitree Go2."""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from dataclasses import dataclass
from enum import IntEnum, StrEnum
from threading import Lock
from typing import Annotated, Literal, Protocol

from pydantic import BaseModel, ConfigDict, Field, TypeAdapter

VX_MIN = -2.5
VX_MAX = 3.8
VY_MAX = 1.0
VYAW_MAX = 4.0
JSON_ENCODING = "application/json"


@dataclass(frozen=True, slots=True)
class Keyspace:
    robot_key: str

    @property
    def command(self) -> str:
        return f"{self.robot_key}/command"

    @property
    def state(self) -> str:
        return f"{self.robot_key}/state"


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
    INVALID_SAMPLE = "invalid_sample"
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


class PhysicalState(BaseModel):
    model_config = ConfigDict(frozen=True)

    state: RobotState
    reason: UnknownReason | None
    motion: Motion
    state_machine_code: int | None
    state_machine_name: str | None
    mode: int | None
    mode_name: str | None
    velocity: tuple[float, float, float] | None
    yaw_speed: float | None
    received_at: float | None
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
            received_at=None,
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
    linear_velocity_quiescent_threshold: float,
    yaw_speed_quiescent_threshold: float,
) -> PhysicalState:
    """Classify one valid State sample without inferring from SDK results."""

    moving = (
        max(abs(component) for component in observation.velocity)
        > linear_velocity_quiescent_threshold
        or abs(observation.yaw_speed) > yaw_speed_quiescent_threshold
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
        received_at=observation.received_at,
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
class BufferedCommands:
    velocity: VelocityRequest | None
    posture: PostureTarget | None
    accepting: bool


class CommandBuffer:
    """Keep only the latest command of each type behind one ingress gate."""

    def __init__(self) -> None:
        self._velocity: VelocityRequest | None = None
        self._posture: PostureTarget | None = None
        self._accepting = False
        self._lock = Lock()

    def update_velocity(self, command: VelocityCommand, *, received_at: float) -> bool:
        with self._lock:
            if not self._accepting:
                return False
            self._velocity = VelocityRequest(
                command.vx,
                command.vy,
                command.vyaw,
                received_at,
            )
            return True

    def update_posture(self, command: PostureCommand) -> bool:
        with self._lock:
            if not self._accepting:
                return False
            self._posture = command.posture
            return True

    def snapshot(self) -> BufferedCommands:
        with self._lock:
            return BufferedCommands(self._velocity, self._posture, self._accepting)

    def set_accepting(self, accepting: bool, *, clear: bool = False) -> None:
        with self._lock:
            self._accepting = accepting
            if clear:
                self._velocity = None
                self._posture = None

    def take_posture(self) -> PostureTarget | None:
        """Take the newest posture and discard velocity behind its priority."""

        with self._lock:
            posture = self._posture
            self._posture = None
            if posture is not None:
                self._velocity = None
            return posture

    def claim_velocity(self, expected: VelocityRequest) -> bool:
        with self._lock:
            return (
                self._accepting and self._posture is None and self._velocity is expected
            )

    def clear_velocity(self, expected: VelocityRequest | None = None) -> bool:
        with self._lock:
            if expected is not None and self._velocity is not expected:
                return False
            self._velocity = None
            return True


class SportClientProtocol(Protocol):
    def StandUp(self) -> int: ...

    def RecoveryStand(self) -> int: ...

    def BalanceStand(self) -> int: ...

    def StopMove(self) -> int: ...

    def StandDown(self) -> int: ...

    def Move(self, vx: float, vy: float, vyaw: float) -> int: ...


class SdkDiagnostic(BaseModel):
    model_config = ConfigDict(frozen=True)

    command: SdkCommand
    velocity: tuple[float, float, float] | None = None
    code: int | None = None
    error: str | None = None


class VelocitySnapshot(BaseModel):
    model_config = ConfigDict(frozen=True)

    vx: float
    vy: float
    vyaw: float
    received_at: float


class NodeState(BaseModel):
    model_config = ConfigDict(frozen=True)

    revision: int
    lifecycle: Lifecycle
    connected: bool
    robot: PhysicalState
    accepting_commands: bool
    requested_posture: PostureTarget | None
    requested_velocity: VelocitySnapshot | None
    posture_action: SdkCommand | None
    last_sdk: SdkDiagnostic | None
    last_error: str | None


class StateSink(Protocol):
    def publish(self, state: NodeState) -> None: ...


class Controller:
    """Run the command State machine once for each fresh DDS State."""

    def __init__(
        self,
        client: SportClientProtocol,
        commands: CommandBuffer,
        state_sink: StateSink,
        *,
        maximum_state_age_seconds: float,
        linear_velocity_quiescent_threshold: float,
        yaw_speed_quiescent_threshold: float,
        velocity_deadman_seconds: float,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._client = client
        self._commands = commands
        self._state_sink = state_sink
        self._maximum_state_age_seconds = maximum_state_age_seconds
        self._linear_velocity_threshold = linear_velocity_quiescent_threshold
        self._yaw_speed_threshold = yaw_speed_quiescent_threshold
        self._velocity_deadman_seconds = velocity_deadman_seconds
        self._clock = clock

        self._state = PhysicalState.unknown(UnknownReason.NO_SAMPLE)
        self._connected = False
        self._last_valid_state_at: float | None = None
        self._awaiting_state_after: float | None = None
        self._active_posture: PostureTarget | None = None
        self._last_posture_action: SdkCommand | None = None
        self._last_sdk: SdkDiagnostic | None = None
        self._last_error: str | None = None
        self._lifecycle = Lifecycle.STARTING
        self._shutdown_complete = False
        self._revision = 0

    @property
    def has_state(self) -> bool:
        return self._last_valid_state_at is not None

    @property
    def shutdown_complete(self) -> bool:
        return self._shutdown_complete

    def on_state(self, observation: RobotObservation) -> None:
        """Process one valid observation and at most one authorized SDK call."""

        now = self._clock()
        self._last_valid_state_at = observation.received_at
        if now >= observation.received_at + self._maximum_state_age_seconds:
            self._disconnect(UnknownReason.STALE)
            return

        self._connected = True
        if (
            self._awaiting_state_after is not None
            and observation.received_at <= self._awaiting_state_after
        ):
            self.publish()
            return

        self._awaiting_state_after = None
        self._state = classify_state(
            observation,
            linear_velocity_quiescent_threshold=self._linear_velocity_threshold,
            yaw_speed_quiescent_threshold=self._yaw_speed_threshold,
        )
        if self._lifecycle is Lifecycle.STARTING:
            self._lifecycle = Lifecycle.RUNNING

        accepting = self._lifecycle is Lifecycle.RUNNING
        self._commands.set_accepting(accepting)
        if self._lifecycle is Lifecycle.SHUTTING_DOWN:
            self._process_posture(new_request=False)
        elif accepting:
            self._process_commands(now)
        self.publish()

    def reject_state(self, error: str) -> None:
        """Record a malformed sample without refreshing the freshness deadline."""

        self._last_error = error
        if self._last_valid_state_at is None:
            self._state = PhysicalState.unknown(UnknownReason.INVALID_SAMPLE)
            self._commands.set_accepting(False, clear=True)
        elif self.expire_state():
            return
        self.publish()

    def expire_state(self) -> bool:
        last_state_at = self._last_valid_state_at
        if (
            self._connected
            and last_state_at is not None
            and self._clock() >= last_state_at + self._maximum_state_age_seconds
        ):
            self._disconnect(UnknownReason.STALE)
            return True
        return False

    def begin_shutdown(self) -> None:
        self._commands.set_accepting(False, clear=True)
        self._lifecycle = Lifecycle.SHUTTING_DOWN
        self._active_posture = PostureTarget.DOWN
        self._last_posture_action = None
        self.publish()

    def finish_shutdown(self, error: str | None = None) -> None:
        self._commands.set_accepting(False, clear=True)
        self._lifecycle = Lifecycle.STOPPED
        if error is not None:
            self._last_error = error
        self.publish()

    def publish(self) -> None:
        self._revision += 1
        buffered = self._commands.snapshot()
        velocity = buffered.velocity
        self._state_sink.publish(
            NodeState(
                revision=self._revision,
                lifecycle=self._lifecycle,
                connected=self._connected,
                robot=self._state,
                accepting_commands=buffered.accepting,
                requested_posture=buffered.posture or self._active_posture,
                requested_velocity=(
                    VelocitySnapshot(
                        vx=velocity.vx,
                        vy=velocity.vy,
                        vyaw=velocity.vyaw,
                        received_at=velocity.received_at,
                    )
                    if velocity is not None
                    else None
                ),
                posture_action=self._last_posture_action,
                last_sdk=self._last_sdk,
                last_error=self._last_error,
            )
        )

    def _disconnect(self, reason: UnknownReason) -> None:
        self._connected = False
        self._awaiting_state_after = None
        self._state = PhysicalState.unknown(reason)
        self._commands.set_accepting(False, clear=True)
        self._active_posture = None
        self._last_posture_action = None
        self.publish()

    def _process_commands(self, now: float) -> None:
        posture = self._commands.take_posture()
        if posture is not None:
            self._active_posture = posture
            self._last_posture_action = None

        if self._active_posture is not None:
            self._commands.clear_velocity()
            self._process_posture(new_request=posture is not None)
            return

        buffered = self._commands.snapshot()
        if buffered.velocity is not None:
            self._process_velocity(buffered.velocity, now)

    def _process_posture(self, *, new_request: bool) -> None:
        target = self._active_posture
        state = self._state.state
        if target is None:
            return
        if state is RobotState.UNSUPPORTED:
            if new_request:
                self._complete_posture()
            return

        action: SdkCommand | None = None
        if target is PostureTarget.STAND:
            if state is RobotState.READY_STAND:
                self._complete_posture()
                return
            if state is RobotState.DAMPING and self._state.motion is Motion.QUIESCENT:
                action = SdkCommand.RECOVERY_STAND
            elif state is RobotState.DOWN:
                action = SdkCommand.STAND_UP
            elif state in {RobotState.LOCKED_STAND, RobotState.LOCOMOTION}:
                action = SdkCommand.BALANCE_STAND
        else:
            if state is RobotState.DOWN:
                self._complete_posture()
                if self._lifecycle is Lifecycle.SHUTTING_DOWN:
                    self._shutdown_complete = True
                return
            if state in {
                RobotState.LOCKED_STAND,
                RobotState.READY_STAND,
                RobotState.LOCOMOTION,
            }:
                if self._last_posture_action is None:
                    action = SdkCommand.STOP_MOVE
                elif (
                    self._last_posture_action is SdkCommand.STOP_MOVE
                    and self._state.motion is Motion.QUIESCENT
                ):
                    action = SdkCommand.STAND_DOWN

        if action is not None and action is not self._last_posture_action:
            self._dispatch_posture(action)

    def _complete_posture(self) -> None:
        self._active_posture = None
        self._last_posture_action = None

    def _process_velocity(self, request: VelocityRequest, now: float) -> None:
        if self._state.state not in {
            RobotState.READY_STAND,
            RobotState.LOCOMOTION,
        }:
            self._commands.clear_velocity(request)
            return
        expired = now >= request.received_at + self._velocity_deadman_seconds
        velocity = (0.0, 0.0, 0.0) if expired else request.values
        if not self._commands.claim_velocity(request):
            return
        self._invoke(SdkCommand.MOVE, velocity)
        if expired:
            self._commands.clear_velocity(request)

    def _dispatch_posture(self, command: SdkCommand) -> None:
        self._last_posture_action = command
        self._state = PhysicalState.unknown(UnknownReason.AWAITING_STATE)
        self._commands.set_accepting(False, clear=True)
        self.publish()
        self._invoke(command)
        self._awaiting_state_after = self._clock()

    def _invoke(
        self,
        command: SdkCommand,
        velocity: tuple[float, float, float] | None = None,
    ) -> None:
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
            self._last_sdk = SdkDiagnostic(
                command=command,
                velocity=velocity,
                code=code,
                error=error,
            )
        except Exception as error:
            message = f"{command} raised {type(error).__name__}: {error}"
            self._last_sdk = SdkDiagnostic(
                command=command,
                velocity=velocity,
                error=message,
            )

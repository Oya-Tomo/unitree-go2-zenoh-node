"""State-driven command selection for one Unitree Go2."""

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


class ModeClass(StrEnum):
    DOWN = "down"
    IDLE_STAND = "idle_stand"
    READY_STAND = "ready_stand"
    LOCOMOTION = "locomotion"
    TRANSITION = "transition"
    UNSUPPORTED = "unsupported"
    UNKNOWN = "unknown"


class Motion(StrEnum):
    QUIESCENT = "quiescent"
    MOVING = "moving"
    UNKNOWN = "unknown"


class StateValidity(StrEnum):
    CONFIRMED = "confirmed"
    UNKNOWN = "unknown"


class UnknownReason(StrEnum):
    NO_SAMPLE = "no_sample"
    STALE = "stale"
    INVALID_SAMPLE = "invalid_sample"
    AWAITING_STATE = "awaiting_state"


class SdkCommand(StrEnum):
    STAND_UP = "stand_up"
    BALANCE_STAND = "balance_stand"
    STOP_MOVE = "stop_move"
    STAND_DOWN = "stand_down"
    MOVE = "move"


class PosturePhase(StrEnum):
    STAND_UP_SENT = "stand_up_sent"
    BALANCE_STAND_SENT = "balance_stand_sent"
    STOP_SENT = "stop_sent"
    DOWN_SENT = "down_sent"


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

    validity: StateValidity
    reason: UnknownReason | None
    mode_class: ModeClass
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
            validity=StateValidity.UNKNOWN,
            reason=reason,
            mode_class=ModeClass.UNKNOWN,
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

    @property
    def is_actionable(self) -> bool:
        return self.validity is StateValidity.CONFIRMED and self.mode_class in {
            ModeClass.DOWN,
            ModeClass.IDLE_STAND,
            ModeClass.READY_STAND,
            ModeClass.LOCOMOTION,
        }


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
    now: float,
    maximum_age_seconds: float,
    linear_velocity_quiescent_threshold: float,
    yaw_speed_quiescent_threshold: float,
) -> PhysicalState:
    """Classify one fresh State sample without inferring from SDK results."""

    age = now - observation.received_at
    if age < 0 or age >= maximum_age_seconds:
        return PhysicalState.unknown(UnknownReason.STALE)

    moving = (
        max(abs(component) for component in observation.velocity)
        > linear_velocity_quiescent_threshold
        or abs(observation.yaw_speed) > yaw_speed_quiescent_threshold
    )
    motion = Motion.MOVING if moving else Motion.QUIESCENT
    state_machine = observation.state_machine_code
    if state_machine in {
        Go2MotionStateMachine.CROUCH,
        Go2MotionStateMachine.ALTERNATE_CROUCH,
    }:
        mode_class = (
            ModeClass.DOWN
            if observation.mode == Go2SportMode.LIE_DOWN and motion is Motion.QUIESCENT
            else ModeClass.TRANSITION
        )
    elif state_machine == Go2MotionStateMachine.STANDING_LOCK:
        mode_class = (
            ModeClass.IDLE_STAND
            if observation.mode == Go2SportMode.IDLE and motion is Motion.QUIESCENT
            else ModeClass.TRANSITION
        )
    elif state_machine == Go2MotionStateMachine.BALANCE_STANDING:
        if observation.mode == Go2SportMode.LOCOMOTION:
            mode_class = ModeClass.LOCOMOTION
        elif (
            observation.mode == Go2SportMode.BALANCE_STAND
            and motion is Motion.QUIESCENT
        ):
            mode_class = ModeClass.READY_STAND
        else:
            mode_class = ModeClass.TRANSITION
    elif state_machine == Go2MotionStateMachine.AGILE:
        if observation.mode == Go2SportMode.IDLE and motion is Motion.QUIESCENT:
            mode_class = ModeClass.IDLE_STAND
        elif (
            observation.mode == Go2SportMode.BALANCE_STAND
            and motion is Motion.QUIESCENT
        ):
            mode_class = ModeClass.READY_STAND
        elif observation.mode == Go2SportMode.LOCOMOTION:
            mode_class = ModeClass.LOCOMOTION
        else:
            mode_class = ModeClass.TRANSITION
    else:
        mode_class = ModeClass.UNSUPPORTED

    return PhysicalState(
        validity=StateValidity.CONFIRMED,
        reason=None,
        mode_class=mode_class,
        motion=motion,
        state_machine_code=observation.state_machine_code,
        state_machine_name=state_machine_name(observation.state_machine_code),
        mode=observation.mode,
        mode_name=mode_name(observation.mode),
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
class PostureRequest:
    target: PostureTarget
    received_at: float


@dataclass(frozen=True, slots=True)
class BufferedCommands:
    velocity: VelocityRequest | None
    posture: PostureRequest | None
    accepting: bool


class CommandBuffer:
    """Keep only the latest velocity and posture received from Zenoh."""

    def __init__(self) -> None:
        self._velocity: VelocityRequest | None = None
        self._posture: PostureRequest | None = None
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

    def update_posture(self, command: PostureCommand, *, received_at: float) -> bool:
        with self._lock:
            if not self._accepting:
                return False
            self._posture = PostureRequest(command.posture, received_at)
            return True

    def snapshot(self) -> BufferedCommands:
        with self._lock:
            return BufferedCommands(
                velocity=self._velocity,
                posture=self._posture,
                accepting=self._accepting,
            )

    def resume(self) -> None:
        with self._lock:
            self._accepting = True

    def pause(self, *, clear: bool = False) -> None:
        with self._lock:
            self._accepting = False
            if clear:
                self._velocity = None
                self._posture = None

    def begin_posture(self, expected: PostureRequest) -> bool:
        """Atomically block new commands before one posture SDK call."""

        with self._lock:
            if not self._accepting or self._posture is not expected:
                return False
            self._accepting = False
            self._velocity = None
            return True

    def begin_velocity(self, expected: VelocityRequest) -> bool:
        with self._lock:
            return (
                self._accepting and self._posture is None and self._velocity is expected
            )

    def clear_posture(self, expected: PostureRequest) -> bool:
        with self._lock:
            if self._posture is not expected:
                return False
            self._posture = None
            return True

    def clear_velocity(self, expected: VelocityRequest) -> bool:
        with self._lock:
            if self._velocity is not expected:
                return False
            self._velocity = None
            return True


class SportClientProtocol(Protocol):
    def StandUp(self) -> int: ...

    def BalanceStand(self) -> int: ...

    def StopMove(self) -> int: ...

    def StandDown(self) -> int: ...

    def Move(self, vx: float, vy: float, vyaw: float) -> int: ...


class StateSink(Protocol):
    def publish(self, state: NodeState) -> None: ...


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
    robot: PhysicalState
    accepting_commands: bool
    requested_posture: PostureTarget | None
    requested_velocity: VelocitySnapshot | None
    posture_phase: PosturePhase | None
    last_sdk: SdkDiagnostic | None
    last_error: str | None


class Controller:
    """Run at most one State-authorized SDK call for each State update."""

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
        self._last_observation_at: float | None = None
        self._active_posture: PostureRequest | None = None
        self._posture_phase: PosturePhase | None = None
        self._last_sdk: SdkDiagnostic | None = None
        self._last_error: str | None = None
        self._lifecycle = Lifecycle.STARTING
        self._state_required_after: float | None = None
        self._shutdown_complete = False
        self._revision = 0

    @property
    def has_state(self) -> bool:
        return self._last_observation_at is not None

    @property
    def shutdown_complete(self) -> bool:
        return self._shutdown_complete

    def on_state(self, observation: RobotObservation) -> None:
        now = self._clock()
        self._last_observation_at = observation.received_at
        if (
            self._state_required_after is not None
            and observation.received_at <= self._state_required_after
        ):
            self.publish()
            return
        self._state = classify_state(
            observation,
            now=now,
            maximum_age_seconds=self._maximum_state_age_seconds,
            linear_velocity_quiescent_threshold=self._linear_velocity_threshold,
            yaw_speed_quiescent_threshold=self._yaw_speed_threshold,
        )
        if self._state.validity is StateValidity.CONFIRMED:
            self._state_required_after = None

        if self._lifecycle is Lifecycle.SHUTTING_DOWN:
            self._process_shutdown_state()
        else:
            if self._state.validity is StateValidity.CONFIRMED:
                self._lifecycle = Lifecycle.RUNNING
            if self._state.is_actionable:
                self._commands.resume()
                self._process_buffered_commands(now)
            else:
                self._commands.pause()
        self.publish()

    def reject_state(self, error: str) -> None:
        self._state = PhysicalState.unknown(UnknownReason.INVALID_SAMPLE)
        self._last_error = error
        clear_request = (
            self._state_required_after is None
            and self._lifecycle is not Lifecycle.SHUTTING_DOWN
        )
        self._commands.pause(clear=clear_request)
        if clear_request:
            self._active_posture = None
            self._posture_phase = None
        self.publish()

    def expire_state(self) -> bool:
        now = self._clock()
        if (
            self._state_required_after is None
            and self._state.received_at is not None
            and now - self._state.received_at >= self._maximum_state_age_seconds
        ):
            self._state = PhysicalState.unknown(UnknownReason.STALE)
            self._commands.pause(clear=True)
            if self._lifecycle is not Lifecycle.SHUTTING_DOWN:
                self._active_posture = None
                self._posture_phase = None
            self.publish()
            return True
        return False

    def begin_shutdown(self) -> None:
        self._commands.pause(clear=True)
        self._lifecycle = Lifecycle.SHUTTING_DOWN
        self._active_posture = PostureRequest(PostureTarget.DOWN, self._clock())
        self._posture_phase = None
        self.publish()

    def finish_shutdown(self, error: str | None = None) -> None:
        self._commands.pause(clear=True)
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
                robot=self._state,
                accepting_commands=buffered.accepting,
                requested_posture=(
                    buffered.posture.target if buffered.posture is not None else None
                ),
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
                posture_phase=self._posture_phase,
                last_sdk=self._last_sdk,
                last_error=self._last_error,
            )
        )

    def _process_buffered_commands(self, now: float) -> None:
        buffered = self._commands.snapshot()
        posture = buffered.posture
        if posture is not self._active_posture:
            self._active_posture = posture
            self._posture_phase = None

        if posture is not None:
            if not self._process_posture(posture):
                return
            if not self._commands.clear_posture(posture):
                return
            self._active_posture = None
            self._posture_phase = None

        buffered = self._commands.snapshot()
        if buffered.posture is not None or buffered.velocity is None:
            return
        self._process_velocity(buffered.velocity, now)

    def _process_posture(self, request: PostureRequest) -> bool:
        state = self._state
        if not state.is_actionable:
            return False

        if request.target is PostureTarget.STAND:
            if state.mode_class is ModeClass.READY_STAND:
                return True
            if self._posture_phase is PosturePhase.BALANCE_STAND_SENT:
                return False
            if state.mode_class is ModeClass.DOWN:
                if self._posture_phase is None:
                    self._dispatch_posture(
                        request,
                        SdkCommand.STAND_UP,
                        PosturePhase.STAND_UP_SENT,
                    )
                return False
            if state.mode_class in {ModeClass.IDLE_STAND, ModeClass.LOCOMOTION}:
                self._dispatch_posture(
                    request,
                    SdkCommand.BALANCE_STAND,
                    PosturePhase.BALANCE_STAND_SENT,
                )
            return False

        if state.mode_class is ModeClass.DOWN:
            return True
        if self._posture_phase is None:
            self._dispatch_posture(
                request,
                SdkCommand.STOP_MOVE,
                PosturePhase.STOP_SENT,
            )
        elif (
            self._posture_phase is PosturePhase.STOP_SENT
            and state.motion is Motion.QUIESCENT
            and state.mode_class
            in {ModeClass.IDLE_STAND, ModeClass.READY_STAND, ModeClass.LOCOMOTION}
        ):
            self._dispatch_posture(
                request,
                SdkCommand.STAND_DOWN,
                PosturePhase.DOWN_SENT,
            )
        return False

    def _process_velocity(self, request: VelocityRequest, now: float) -> None:
        if self._state.mode_class not in {
            ModeClass.READY_STAND,
            ModeClass.LOCOMOTION,
        }:
            return
        expired = now - request.received_at >= self._velocity_deadman_seconds
        velocity = (0.0, 0.0, 0.0) if expired else request.values
        if not self._commands.begin_velocity(request):
            return
        self._invoke(SdkCommand.MOVE, velocity)
        if expired:
            self._commands.clear_velocity(request)

    def _dispatch_posture(
        self,
        request: PostureRequest,
        command: SdkCommand,
        phase: PosturePhase,
    ) -> None:
        if (
            self._lifecycle is not Lifecycle.SHUTTING_DOWN
            and not self._commands.begin_posture(request)
        ):
            return
        self._posture_phase = phase
        self._state = PhysicalState.unknown(UnknownReason.AWAITING_STATE)
        self.publish()
        self._invoke(command)
        self._state_required_after = self._clock()

    def _process_shutdown_state(self) -> None:
        if not self._state.is_actionable or self._active_posture is None:
            return
        if self._process_posture(self._active_posture):
            self._shutdown_complete = True

    def _invoke(
        self,
        command: SdkCommand,
        velocity: tuple[float, float, float] | None = None,
    ) -> None:
        try:
            if command is SdkCommand.STAND_UP:
                code = self._client.StandUp()
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
            self._last_error = error
        except Exception as error:
            message = f"{command} raised {type(error).__name__}: {error}"
            self._last_sdk = SdkDiagnostic(
                command=command,
                velocity=velocity,
                error=message,
            )
            self._last_error = message

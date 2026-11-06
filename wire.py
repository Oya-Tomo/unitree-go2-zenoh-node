"""Zenoh wire contract shared by the Go2 node and its clients."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from typing import Annotated

from pydantic import BaseModel, ConfigDict, Field

VX_MIN = -2.5
VX_MAX = 3.8
VY_MAX = 1.0
VYAW_MAX = 4.0
JSON_ENCODING = "application/json"


@dataclass(frozen=True, slots=True)
class Keyspace:
    prefix: str

    @property
    def velocity(self) -> str:
        return f"{self.prefix}/command/velocity"

    @property
    def posture(self) -> str:
        return f"{self.prefix}/command/posture"

    @property
    def state(self) -> str:
        return f"{self.prefix}/state"


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
    """Payload of `{prefix}/command/velocity`."""

    vx: Annotated[float, Field(strict=True, ge=VX_MIN, le=VX_MAX)]
    vy: Annotated[float, Field(strict=True, ge=-VY_MAX, le=VY_MAX)]
    vyaw: Annotated[float, Field(strict=True, ge=-VYAW_MAX, le=VYAW_MAX)]


class PostureCommand(WireModel):
    """Payload of `{prefix}/command/posture`."""

    posture: PostureTarget


class PublishedRobotState(WireModel):
    state: RobotState
    reason: UnknownReason | None = None
    motion: Motion
    state_machine_code: int | None = None
    state_machine_name: str | None = None
    mode: int | None = None
    mode_name: str | None = None
    velocity: tuple[float, float, float] | None = None
    yaw_speed: float | None = None
    stamp_sec: int | None = None
    stamp_nanosec: int | None = None

    @classmethod
    def unknown(cls, reason: UnknownReason) -> PublishedRobotState:
        return cls(state=RobotState.UNKNOWN, reason=reason, motion=Motion.UNKNOWN)


class SdkDiagnostic(WireModel):
    command: SdkCommand
    velocity: VelocityCommand | None = None
    code: int | None = None
    error: str | None = None


class PublishedNodeState(WireModel):
    """Payload of `{prefix}/state`."""

    lifecycle: Lifecycle
    robot_connected: bool
    robot_state: PublishedRobotState
    accepting_commands: bool
    requested_posture: PostureTarget | None
    requested_velocity: VelocityCommand | None
    last_posture_action: SdkCommand | None
    last_sdk_diagnostic: SdkDiagnostic | None
    last_node_error: str | None

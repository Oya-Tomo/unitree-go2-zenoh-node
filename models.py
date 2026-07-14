from enum import StrEnum
from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, TypeAdapter

VX_MIN = -2.5
VX_MAX = 3.8
VY_MIN = -1.0
VY_MAX = 1.0
VYAW_MIN = -4.0
VYAW_MAX = 4.0

Vx = Annotated[float, Field(strict=True, ge=VX_MIN, le=VX_MAX, allow_inf_nan=False)]
Vy = Annotated[float, Field(strict=True, ge=VY_MIN, le=VY_MAX, allow_inf_nan=False)]
Vyaw = Annotated[
    float,
    Field(strict=True, ge=VYAW_MIN, le=VYAW_MAX, allow_inf_nan=False),
]


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


class VelocityCommand(StrictModel):
    type: Literal["velocity"] = "velocity"
    vx: Vx
    vy: Vy
    vyaw: Vyaw


class PostureCommand(StrictModel):
    type: Literal["posture"] = "posture"
    posture: Literal["stand", "down"]


Command = Annotated[VelocityCommand | PostureCommand, Field(discriminator="type")]
COMMAND_ADAPTER = TypeAdapter(Command)


class Posture(StrEnum):
    UNKNOWN = "unknown"
    STANDING_UP = "standing_up"
    STANDING = "standing"
    STANDING_DOWN = "standing_down"
    DOWN = "down"


class NodeStatus(StrEnum):
    STARTING = "starting"
    READY = "ready"
    DEGRADED = "degraded"
    STOPPING = "stopping"


class VelocityState(StrictModel):
    vx: Vx = 0.0
    vy: Vy = 0.0
    vyaw: Vyaw = 0.0
    active: bool = False

    @classmethod
    def from_command(
        cls, command: VelocityCommand, *, active: bool = True
    ) -> "VelocityState":
        return cls(
            vx=command.vx,
            vy=command.vy,
            vyaw=command.vyaw,
            active=active,
        )


class PostureState(StrictModel):
    posture: Posture = Posture.UNKNOWN


class HealthState(StrictModel):
    status: NodeStatus = NodeStatus.STARTING
    walking_enabled: bool = False
    watchdog_triggered: bool = False
    last_error: str | None = None


def decode_command(payload: bytes | str) -> Command:
    return COMMAND_ADAPTER.validate_json(payload)

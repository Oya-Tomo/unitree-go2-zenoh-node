"""Validated JSON5 settings shared by the node and keyboard client."""

from __future__ import annotations

from pathlib import Path
from typing import Annotated

import json5
import zenoh
from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    StrictStr,
    ValidationInfo,
    field_validator,
)

from controller import VX_MAX, VX_MIN, VY_MAX, VYAW_MAX

NonNegativeInt = Annotated[int, Field(strict=True, ge=0)]
PositiveFloat = Annotated[float, Field(strict=True, gt=0, allow_inf_nan=False)]


class ConfigModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


def validate_robot_key(value: str) -> str:
    if not value.strip():
        raise ValueError("robot_key must be a non-empty string")
    try:
        zenoh.KeyExpr(value)
    except zenoh.ZError as error:
        raise ValueError(f"robot_key must be a valid Zenoh key: {error}") from error
    if "*" in value:
        raise ValueError("robot_key must not contain wildcards")
    return value.rstrip("/")


class DdsConfig(ConfigModel):
    domain_id: NonNegativeInt
    network_interface: StrictStr
    rpc_timeout_seconds: PositiveFloat
    sport_mode_state_topic: StrictStr = "rt/sportmodestate"

    @field_validator("network_interface", "sport_mode_state_topic")
    @classmethod
    def validate_non_empty(cls, value: str, info: ValidationInfo) -> str:
        if not value.strip():
            raise ValueError(f"{info.field_name} must be a non-empty string")
        return value


class StateConfig(ConfigModel):
    startup_timeout_seconds: PositiveFloat
    maximum_age_seconds: PositiveFloat
    linear_velocity_quiescent_threshold: PositiveFloat
    yaw_speed_quiescent_threshold: PositiveFloat


class ControlConfig(ConfigModel):
    velocity_deadman_seconds: PositiveFloat
    shutdown_timeout_seconds: PositiveFloat


class NodeConfig(ConfigModel):
    robot_key: StrictStr
    state_heartbeat_seconds: PositiveFloat
    dds: DdsConfig
    state: StateConfig
    control: ControlConfig

    @field_validator("robot_key")
    @classmethod
    def validate_key(cls, value: str) -> str:
        return validate_robot_key(value)


class VelocityTargets(ConfigModel):
    vx: Annotated[
        float,
        Field(strict=True, gt=0, le=min(abs(VX_MIN), VX_MAX), allow_inf_nan=False),
    ]
    vy: Annotated[float, Field(strict=True, gt=0, le=VY_MAX, allow_inf_nan=False)]
    vyaw: Annotated[float, Field(strict=True, gt=0, le=VYAW_MAX, allow_inf_nan=False)]


class RampRates(ConfigModel):
    vx: PositiveFloat
    vy: PositiveFloat
    vyaw: PositiveFloat


class KeyboardConfig(ConfigModel):
    robot_key: StrictStr
    publish_frequency_hz: PositiveFloat
    state_stale_after_seconds: PositiveFloat
    targets: VelocityTargets
    ramp_rates: RampRates

    @field_validator("robot_key")
    @classmethod
    def validate_key(cls, value: str) -> str:
        return validate_robot_key(value)


def _load(path: Path) -> object:
    with path.open(encoding="utf-8") as stream:
        return json5.load(stream)


def load_node_config(path: Path) -> NodeConfig:
    return NodeConfig.model_validate(_load(path))


def load_keyboard_config(path: Path) -> KeyboardConfig:
    return KeyboardConfig.model_validate(_load(path))

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
PositiveInt = Annotated[int, Field(strict=True, gt=0)]
PositiveFloat = Annotated[float, Field(strict=True, gt=0, allow_inf_nan=False)]


class ConfigModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


def validate_zenoh_key_prefix(value: str) -> str:
    if not value.strip():
        raise ValueError("zenoh_key_prefix must be a non-empty string")
    try:
        zenoh.KeyExpr(value)
    except zenoh.ZError as error:
        raise ValueError(
            f"zenoh_key_prefix must be a valid Zenoh key: {error}"
        ) from error
    if "*" in value:
        raise ValueError("zenoh_key_prefix must not contain wildcards")
    return value.rstrip("/")


class DdsConfig(ConfigModel):
    domain_id: NonNegativeInt
    network_interface: StrictStr
    sport_mode_state_topic: StrictStr = "rt/sportmodestate"
    first_state_timeout_seconds: PositiveFloat
    state_freshness_seconds: PositiveFloat

    @field_validator("network_interface", "sport_mode_state_topic")
    @classmethod
    def validate_non_empty(cls, value: str, info: ValidationInfo) -> str:
        if not value.strip():
            raise ValueError(f"{info.field_name} must be a non-empty string")
        return value


class SportClientConfig(ConfigModel):
    rpc_timeout_seconds: PositiveFloat


class RobotStateConfig(ConfigModel):
    quiescent_linear_speed_mps: PositiveFloat
    quiescent_yaw_rate_rad_s: PositiveFloat


class ControlConfig(ConfigModel):
    velocity_command_timeout_seconds: PositiveFloat
    shutdown_timeout_seconds: PositiveFloat


class NodeConfig(ConfigModel):
    zenoh_key_prefix: StrictStr
    node_state_publish_frequency_hz: PositiveFloat
    dds: DdsConfig
    sport_client: SportClientConfig
    robot_state: RobotStateConfig
    control: ControlConfig

    @field_validator("zenoh_key_prefix")
    @classmethod
    def validate_key(cls, value: str) -> str:
        return validate_zenoh_key_prefix(value)


class VelocityTargets(ConfigModel):
    vx: Annotated[
        float,
        Field(strict=True, gt=0, le=min(abs(VX_MIN), VX_MAX), allow_inf_nan=False),
    ]
    vy: Annotated[float, Field(strict=True, gt=0, le=VY_MAX, allow_inf_nan=False)]
    vyaw: Annotated[float, Field(strict=True, gt=0, le=VYAW_MAX, allow_inf_nan=False)]


class VelocityRampRates(ConfigModel):
    vx_mps2: PositiveFloat
    vy_mps2: PositiveFloat
    vyaw_rad_s2: PositiveFloat


class KeyboardConfig(ConfigModel):
    zenoh_key_prefix: StrictStr
    loop_frequency_hz: PositiveInt
    node_state_timeout_seconds: PositiveFloat
    velocity_targets: VelocityTargets
    velocity_ramp_rates: VelocityRampRates

    @field_validator("zenoh_key_prefix")
    @classmethod
    def validate_key(cls, value: str) -> str:
        return validate_zenoh_key_prefix(value)


def _load(path: Path) -> object:
    with path.open(encoding="utf-8") as stream:
        return json5.load(stream)


def load_node_config(path: Path) -> NodeConfig:
    return NodeConfig.model_validate(_load(path))


def load_keyboard_config(path: Path) -> KeyboardConfig:
    return KeyboardConfig.model_validate(_load(path))

from __future__ import annotations

from pathlib import Path
from typing import Annotated, Self

import json5
import zenoh
from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    StrictStr,
    ValidationInfo,
    field_validator,
    model_validator,
)

from models import VX_MAX, VX_MIN, VY_MAX, VYAW_MAX

NonNegativeInt = Annotated[int, Field(strict=True, ge=0)]
PositiveFiniteFloat = Annotated[float, Field(strict=True, gt=0, allow_inf_nan=False)]


class ConfigModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


def validate_concrete_key(value: str, location: str) -> str:
    if not value.strip():
        raise ValueError(f"{location} must be a non-empty string")
    try:
        zenoh.KeyExpr(value)
    except zenoh.ZError as error:
        raise ValueError(
            f"{location} must be a valid Zenoh key expression: {error}"
        ) from error
    if "*" in value:
        raise ValueError(f"{location} must be a concrete Zenoh key without wildcards")
    return value


class DdsConfig(ConfigModel):
    domain_id: NonNegativeInt
    network_interface: StrictStr
    rpc_timeout_seconds: PositiveFiniteFloat
    sport_mode_state_topic: StrictStr = "rt/sportmodestate"

    @field_validator("network_interface", "sport_mode_state_topic")
    @classmethod
    def validate_non_empty_string(cls, value: str, info: ValidationInfo) -> str:
        if not value.strip():
            raise ValueError(f"{info.field_name} must be a non-empty string")
        return value


class SafetyConfig(ConfigModel):
    command_timeout_seconds: PositiveFiniteFloat
    posture_transition_seconds: PositiveFiniteFloat
    shutdown_stop_delay_seconds: PositiveFiniteFloat
    motion_state_max_age_seconds: PositiveFiniteFloat = 0.5
    balance_confirmation_timeout_seconds: PositiveFiniteFloat = 1.0


class NodeConfig(ConfigModel):
    robot_key: StrictStr
    state_heartbeat_seconds: PositiveFiniteFloat
    dds: DdsConfig
    safety: SafetyConfig

    @field_validator("robot_key")
    @classmethod
    def validate_robot_key(cls, value: str) -> str:
        return validate_concrete_key(value, "robot_key")


class VelocityTargets(ConfigModel):
    # W and S share one magnitude, so vx honors the smaller reverse limit.
    vx: Annotated[
        float,
        Field(strict=True, gt=0, le=min(abs(VX_MIN), VX_MAX), allow_inf_nan=False),
    ]
    vy: Annotated[float, Field(strict=True, gt=0, le=VY_MAX, allow_inf_nan=False)]
    vyaw: Annotated[float, Field(strict=True, gt=0, le=VYAW_MAX, allow_inf_nan=False)]


class RampRates(ConfigModel):
    vx: PositiveFiniteFloat
    vy: PositiveFiniteFloat
    vyaw: PositiveFiniteFloat


class PostureRequestConfig(ConfigModel):
    retry_interval_seconds: PositiveFiniteFloat
    timeout_seconds: PositiveFiniteFloat

    @model_validator(mode="after")
    def validate_timeout(self) -> Self:
        if self.timeout_seconds <= self.retry_interval_seconds:
            raise ValueError("timeout_seconds must exceed retry_interval_seconds")
        return self


class KeyboardConfig(ConfigModel):
    robot_key: StrictStr
    publish_frequency_hz: PositiveFiniteFloat
    state_stale_after_seconds: PositiveFiniteFloat
    targets: VelocityTargets
    ramp_rates: RampRates
    posture_requests: PostureRequestConfig

    @field_validator("robot_key")
    @classmethod
    def validate_robot_key(cls, value: str) -> str:
        return validate_concrete_key(value, "robot_key")


def load_node_config(path: Path) -> NodeConfig:
    with path.open(encoding="utf-8") as config_file:
        return NodeConfig.model_validate(json5.load(config_file))


def load_keyboard_config(path: Path) -> KeyboardConfig:
    with path.open(encoding="utf-8") as config_file:
        return KeyboardConfig.model_validate(json5.load(config_file))

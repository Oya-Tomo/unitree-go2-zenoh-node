"""Validated JSON5 settings shared by the node and keyboard client."""

from __future__ import annotations

from pathlib import Path
from typing import Annotated

import json5
import zenoh
from pydantic import (
    AfterValidator,
    BaseModel,
    ConfigDict,
    Field,
    NonNegativeInt,
    PositiveFloat,
    PositiveInt,
)

from wire import VX_MAX, VX_MIN, VY_MAX, VYAW_MAX


def _non_blank(value: str) -> str:
    if not value.strip():
        raise ValueError("must be a non-empty string")
    return value


def _key_prefix(value: str) -> str:
    try:
        zenoh.KeyExpr(_non_blank(value))
    except zenoh.ZError as error:
        raise ValueError(f"must be a valid Zenoh key: {error}") from error
    if "*" in value:
        raise ValueError("must not contain wildcards")
    return value


NonBlankStr = Annotated[str, AfterValidator(_non_blank)]
KeyPrefix = Annotated[str, AfterValidator(_key_prefix)]


class ConfigModel(BaseModel):
    model_config = ConfigDict(
        extra="forbid",
        frozen=True,
        strict=True,
        allow_inf_nan=False,
    )


class DdsConfig(ConfigModel):
    domain_id: NonNegativeInt
    network_interface: NonBlankStr
    sport_mode_state_topic: NonBlankStr = "rt/sportmodestate"
    first_state_timeout_seconds: PositiveFloat
    state_freshness_seconds: PositiveFloat


class SportClientConfig(ConfigModel):
    rpc_timeout_seconds: PositiveFloat


class RobotStateConfig(ConfigModel):
    quiescent_linear_speed_mps: PositiveFloat
    quiescent_yaw_rate_rad_s: PositiveFloat


class ControlConfig(ConfigModel):
    velocity_command_timeout_seconds: PositiveFloat
    shutdown_timeout_seconds: PositiveFloat


class NodeConfig(ConfigModel):
    zenoh_key_prefix: KeyPrefix
    node_state_publish_frequency_hz: PositiveFloat
    dds: DdsConfig
    sport_client: SportClientConfig
    robot_state: RobotStateConfig
    control: ControlConfig


class VelocityTargets(ConfigModel):
    vx: Annotated[PositiveFloat, Field(le=min(-VX_MIN, VX_MAX))]
    vy: Annotated[PositiveFloat, Field(le=VY_MAX)]
    vyaw: Annotated[PositiveFloat, Field(le=VYAW_MAX)]


class VelocityRampRates(ConfigModel):
    vx_mps2: PositiveFloat
    vy_mps2: PositiveFloat
    vyaw_rad_s2: PositiveFloat


class KeyboardConfig(ConfigModel):
    zenoh_key_prefix: KeyPrefix
    loop_frequency_hz: PositiveInt
    node_state_timeout_seconds: PositiveFloat
    velocity_targets: VelocityTargets
    velocity_ramp_rates: VelocityRampRates


def load_config[M: ConfigModel](model: type[M], path: Path) -> M:
    with path.open(encoding="utf-8") as stream:
        return model.model_validate(json5.load(stream))

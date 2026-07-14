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
    field_validator,
    model_validator,
)

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

    @field_validator("network_interface")
    @classmethod
    def validate_network_interface(cls, value: str) -> str:
        if not value.strip():
            raise ValueError("network_interface must be a non-empty string")
        return value


class SafetyConfig(ConfigModel):
    command_timeout_seconds: PositiveFiniteFloat
    posture_transition_seconds: PositiveFiniteFloat
    shutdown_stop_delay_seconds: PositiveFiniteFloat


class NodeConfig(ConfigModel):
    robot_key: StrictStr
    dds: DdsConfig
    safety: SafetyConfig

    @field_validator("robot_key")
    @classmethod
    def validate_robot_key(cls, value: str) -> str:
        return validate_concrete_key(value, "robot_key")

    @model_validator(mode="after")
    def validate_rpc_timeout(self) -> Self:
        if self.dds.rpc_timeout_seconds > self.safety.command_timeout_seconds:
            raise ValueError(
                "dds.rpc_timeout_seconds must not exceed safety.command_timeout_seconds"
            )
        return self


def load_node_config(path: Path) -> NodeConfig:
    with path.open(encoding="utf-8") as config_file:
        return NodeConfig.model_validate(json5.load(config_file))

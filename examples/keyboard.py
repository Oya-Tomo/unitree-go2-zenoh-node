"""Control a Go2 Zenoh node with a pygame keyboard deadman."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections.abc import Sequence
from contextlib import ExitStack
from dataclasses import dataclass
from pathlib import Path
from threading import Lock
from typing import Annotated, Literal, Self

import json5
import pygame
import zenoh
from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    ValidationError,
    field_validator,
    model_validator,
)

from models import PostureCommand, VelocityCommand, VelocityState
from settings import validate_concrete_key

DEFAULT_KEYBOARD_CONFIG_PATH = Path("examples/keyboard-config.json5")
DEFAULT_ZENOH_CONFIG_PATH = Path("examples/keyboard-zenoh-config.json5")
JSON_ENCODING = "application/json"
WINDOW_SIZE = (900, 520)
ZERO_VELOCITY = (0.0, 0.0, 0.0)
BACKGROUND = (20, 23, 28)
FOREGROUND = (232, 236, 241)
ACCENT = (86, 182, 194)
WARNING = (244, 180, 0)

PositiveFiniteFloat = Annotated[float, Field(strict=True, gt=0, allow_inf_nan=False)]


class ConfigModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


class VelocityTargets(ConfigModel):
    # One magnitude is used for W and S, so it must honor the smaller reverse limit.
    vx: Annotated[float, Field(strict=True, gt=0, le=2.5, allow_inf_nan=False)]
    vy: Annotated[float, Field(strict=True, gt=0, le=1.0, allow_inf_nan=False)]
    vyaw: Annotated[float, Field(strict=True, gt=0, le=4.0, allow_inf_nan=False)]


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
    robot_key: str
    publish_frequency_hz: PositiveFiniteFloat
    targets: VelocityTargets
    ramp_rates: RampRates
    posture_requests: PostureRequestConfig

    @field_validator("robot_key")
    @classmethod
    def validate_robot_key(cls, value: str) -> str:
        return validate_concrete_key(value, "robot_key")

    @field_validator("publish_frequency_hz", mode="before")
    @classmethod
    def validate_frequency(cls, value: object) -> object:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError("publish_frequency_hz must be a number")
        if not math.isfinite(float(value)):
            raise ValueError("publish_frequency_hz must be finite")
        return value


def load_keyboard_config(path: Path) -> KeyboardConfig:
    with path.open(encoding="utf-8") as config_file:
        return KeyboardConfig.model_validate(json5.load(config_file))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="unitree-go2-keyboard",
        description="Publish deadman-controlled Go2 commands over Zenoh.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--keyboard-config",
        type=Path,
        default=DEFAULT_KEYBOARD_CONFIG_PATH,
        help="path to the keyboard JSON5 configuration",
    )
    parser.add_argument(
        "--zenoh-config",
        type=Path,
        default=DEFAULT_ZENOH_CONFIG_PATH,
        help="path to the Zenoh JSON5 configuration",
    )
    return parser


def approach(current: float, target: float, maximum_delta: float) -> float:
    if current < target:
        return min(current + maximum_delta, target)
    if current > target:
        return max(current - maximum_delta, target)
    return current


class StateMonitor:
    def __init__(self) -> None:
        self._states: dict[str, dict[str, object]] = {}
        self._lock = Lock()

    def update(self, sample: zenoh.Sample) -> None:
        try:
            value = json.loads(sample.payload.to_bytes())
        except (UnicodeDecodeError, json.JSONDecodeError):
            return
        if not isinstance(value, dict):
            return
        with self._lock:
            self._states[str(sample.key_expr)] = value

    def snapshot(self) -> dict[str, dict[str, object]]:
        with self._lock:
            return {key: value.copy() for key, value in self._states.items()}

    def posture(self, robot_key: str) -> str | None:
        with self._lock:
            value = self._states.get(f"{robot_key}/state/posture")
            if value is None:
                return None
            posture = value.get("posture")
            return posture if isinstance(posture, str) else None


class CommandPublisher:
    def __init__(
        self,
        velocity_publisher: zenoh.Publisher,
        posture_publisher: zenoh.Publisher,
    ) -> None:
        self._velocity_publisher = velocity_publisher
        self._posture_publisher = posture_publisher

    def velocity(self, velocity: tuple[float, float, float]) -> None:
        self._velocity_publisher.put(
            VelocityCommand(
                vx=velocity[0], vy=velocity[1], vyaw=velocity[2]
            ).model_dump_json()
        )

    def posture(self, posture: Literal["stand", "down"]) -> None:
        self._posture_publisher.put(PostureCommand(posture=posture).model_dump_json())


class DeadmanState:
    def __init__(self) -> None:
        self._focused = True
        self._release_required = False

    def focus_lost(self) -> None:
        self._focused = False
        self._release_required = True

    def focus_gained(self) -> None:
        self._focused = True

    def active(self, *, shift_pressed: bool) -> bool:
        if not shift_pressed:
            self._release_required = False
        return self._focused and shift_pressed and not self._release_required


@dataclass
class PendingPosture:
    target: Literal["stand", "down"]
    next_retry_at: float
    deadline: float


class PostureRequester:
    def __init__(
        self,
        config: KeyboardConfig,
        publisher: CommandPublisher,
        monitor: StateMonitor,
    ) -> None:
        self._config = config
        self._publisher = publisher
        self._monitor = monitor
        self._pending: PendingPosture | None = None
        self.error: str | None = None

    def request(self, target: Literal["stand", "down"], *, now: float) -> None:
        self._publisher.posture(target)
        retry = self._config.posture_requests.retry_interval_seconds
        self._pending = PendingPosture(
            target=target,
            next_retry_at=now + retry,
            deadline=now + self._config.posture_requests.timeout_seconds,
        )
        self.error = None

    def service(self, *, now: float) -> None:
        pending = self._pending
        if pending is None:
            return
        expected = "standing" if pending.target == "stand" else "down"
        if self._monitor.posture(self._config.robot_key) == expected:
            self._pending = None
            self.error = None
            return
        if now >= pending.deadline:
            self._pending = None
            self.error = f"posture request '{pending.target}' timed out"
            return
        if now >= pending.next_retry_at:
            self._publisher.posture(pending.target)
            pending.next_retry_at = (
                now + self._config.posture_requests.retry_interval_seconds
            )


def target_velocity(
    keys: pygame.key.ScancodeWrapper,
    config: KeyboardConfig,
    *,
    deadman: bool,
) -> tuple[float, float, float]:
    if not deadman:
        return (0.0, 0.0, 0.0)
    vx = float(keys[pygame.K_w] - keys[pygame.K_s]) * config.targets.vx
    vy = float(keys[pygame.K_a] - keys[pygame.K_d]) * config.targets.vy
    vyaw = float(keys[pygame.K_q] - keys[pygame.K_e]) * config.targets.vyaw
    return (vx, vy, vyaw)


def ramp_velocity(
    current: tuple[float, float, float],
    target: tuple[float, float, float],
    config: KeyboardConfig,
    elapsed_seconds: float,
) -> tuple[float, float, float]:
    return (
        approach(current[0], target[0], config.ramp_rates.vx * elapsed_seconds),
        approach(current[1], target[1], config.ramp_rates.vy * elapsed_seconds),
        approach(
            current[2],
            target[2],
            config.ramp_rates.vyaw * elapsed_seconds,
        ),
    )


def _format_velocity(state: dict[str, object] | None) -> str:
    if state is None:
        return "waiting"
    try:
        velocity = VelocityState.model_validate(state)
        return (
            f"vx={velocity.vx:+.3f}  vy={velocity.vy:+.3f}  "
            f"vyaw={velocity.vyaw:+.3f}  active={velocity.active}"
        )
    except ValidationError:
        return "invalid state payload"


class Dashboard:
    def __init__(self, config: KeyboardConfig, monitor: StateMonitor) -> None:
        self._config = config
        self._monitor = monitor
        pygame.display.set_caption("Unitree Go2 Zenoh keyboard controller")
        self._screen = pygame.display.set_mode(WINDOW_SIZE)
        self._font = pygame.font.Font(None, 34)
        self._small_font = pygame.font.Font(None, 25)

    def _state_lines(
        self,
        velocity: tuple[float, float, float],
        *,
        deadman: bool,
        operator_error: str | None,
    ) -> list[tuple[pygame.font.Font, str, tuple[int, int, int]]]:
        states = self._monitor.snapshot()
        base = self._config.robot_key
        requested = states.get(f"{base}/state/command/requested")
        applied = states.get(f"{base}/state/command/applied")
        posture = states.get(f"{base}/state/posture", {}).get("posture", "waiting")
        health = states.get(f"{base}/state/health", {})
        last_error = health.get("last_error") or "none"
        return [
            (self._font, f"Go2 keyboard node: {base}", ACCENT),
            (
                self._font,
                f"Deadman: {'ACTIVE' if deadman else 'released'}",
                ACCENT if deadman else WARNING,
            ),
            (
                self._small_font,
                f"Local output: vx={velocity[0]:+.3f}  vy={velocity[1]:+.3f}  vyaw={velocity[2]:+.3f}",
                FOREGROUND,
            ),
            (
                self._small_font,
                f"Requested: {_format_velocity(requested)}",
                FOREGROUND,
            ),
            (
                self._small_font,
                f"Applied:   {_format_velocity(applied)}",
                FOREGROUND,
            ),
            (self._small_font, f"Posture: {posture}", FOREGROUND),
            (
                self._small_font,
                "Health: "
                f"{health.get('status', 'waiting')}  "
                f"walking_enabled={health.get('walking_enabled', 'waiting')}  "
                f"watchdog={health.get('watchdog_triggered', 'waiting')}",
                FOREGROUND,
            ),
            (
                self._small_font,
                f"Last error: {last_error}",
                WARNING if last_error != "none" else FOREGROUND,
            ),
            (
                self._small_font,
                f"Operator error: {operator_error or 'none'}",
                WARNING if operator_error else FOREGROUND,
            ),
            (
                self._small_font,
                "Shift+W/S: vx   Shift+A/D: vy   Shift+Q/E: yaw",
                FOREGROUND,
            ),
            (
                self._small_font,
                "Shift+R: stand   Shift+F: down   Space: zero   Esc: zero and exit",
                FOREGROUND,
            ),
        ]

    def draw(
        self,
        velocity: tuple[float, float, float],
        *,
        deadman: bool,
        operator_error: str | None,
    ) -> None:
        self._screen.fill(BACKGROUND)
        y = 24
        for font, text, color in self._state_lines(
            velocity,
            deadman=deadman,
            operator_error=operator_error,
        ):
            rendered = font.render(text, True, color)
            self._screen.blit(rendered, (24, y))
            y += rendered.get_height() + 14
        pygame.display.flip()


def fetch_initial_state(
    session: zenoh.Session, monitor: StateMonitor, robot_key: str
) -> None:
    for suffix in (
        "state/command/requested",
        "state/command/applied",
        "state/posture",
        "state/health",
    ):
        for reply in session.get(f"{robot_key}/{suffix}", timeout=0.5):
            sample = reply.ok
            if sample is not None:
                monitor.update(sample)


class KeyboardController:
    def __init__(
        self,
        config: KeyboardConfig,
        publisher: CommandPublisher,
        posture_requester: PostureRequester,
        dashboard: Dashboard,
    ) -> None:
        self._config = config
        self._publisher = publisher
        self._posture_requester = posture_requester
        self._dashboard = dashboard
        self._deadman = DeadmanState()
        self._velocity = ZERO_VELOCITY
        self._last_update = time.monotonic()

    def _process_events(self) -> tuple[bool, bool]:
        running = True
        immediate_zero = False
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                immediate_zero = True
            elif event.type == pygame.WINDOWFOCUSLOST:
                self._deadman.focus_lost()
                immediate_zero = True
            elif event.type == pygame.WINDOWFOCUSGAINED:
                self._deadman.focus_gained()
            elif event.type == pygame.KEYDOWN:
                shift = bool(event.mod & pygame.KMOD_SHIFT)
                if event.key == pygame.K_ESCAPE:
                    running = False
                    immediate_zero = True
                elif event.key == pygame.K_SPACE:
                    immediate_zero = True
                elif shift and event.key == pygame.K_r:
                    self._posture_requester.request("stand", now=time.monotonic())
                elif shift and event.key == pygame.K_f:
                    self._posture_requester.request("down", now=time.monotonic())
            elif event.type == pygame.KEYUP and event.key in (
                pygame.K_LSHIFT,
                pygame.K_RSHIFT,
            ):
                immediate_zero = True
        return running, immediate_zero

    def _update_velocity(self, *, immediate_zero: bool) -> bool:
        keys = pygame.key.get_pressed()
        shift_pressed = bool(keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT])
        deadman = self._deadman.active(shift_pressed=shift_pressed)
        now = time.monotonic()
        period = 1.0 / self._config.publish_frequency_hz
        elapsed = min(now - self._last_update, period * 2.0)
        self._last_update = now

        if immediate_zero or keys[pygame.K_SPACE] or not deadman:
            self._velocity = ZERO_VELOCITY
            if immediate_zero:
                self._publisher.velocity(ZERO_VELOCITY)
        else:
            self._velocity = ramp_velocity(
                self._velocity,
                target_velocity(keys, self._config, deadman=True),
                self._config,
                elapsed,
            )
        return deadman

    def run(self) -> None:
        frame_clock = pygame.time.Clock()
        try:
            running = True
            while running:
                running, immediate_zero = self._process_events()
                deadman = self._update_velocity(immediate_zero=immediate_zero)
                self._publisher.velocity(self._velocity)
                self._posture_requester.service(now=time.monotonic())
                self._dashboard.draw(
                    self._velocity,
                    deadman=deadman,
                    operator_error=self._posture_requester.error,
                )
                frame_clock.tick(max(1, round(self._config.publish_frequency_hz)))
        finally:
            self._publisher.velocity(ZERO_VELOCITY)


def run(zenoh_config: zenoh.Config, config: KeyboardConfig) -> None:
    command_key = f"{config.robot_key}/command"
    state_key = f"{config.robot_key}/state/**"
    monitor = StateMonitor()

    zenoh.init_log_from_env_or("error")
    pygame.init()

    try:
        with zenoh.open(zenoh_config) as session, ExitStack() as resources:
            velocity_publisher = resources.enter_context(
                session.declare_publisher(
                    command_key,
                    encoding=JSON_ENCODING,
                    congestion_control=zenoh.CongestionControl.DROP,
                    reliability=zenoh.Reliability.BEST_EFFORT,
                )
            )
            posture_publisher = resources.enter_context(
                session.declare_publisher(
                    command_key,
                    encoding=JSON_ENCODING,
                    congestion_control=zenoh.CongestionControl.BLOCK,
                    reliability=zenoh.Reliability.RELIABLE,
                )
            )
            resources.enter_context(
                session.declare_subscriber(state_key, monitor.update)
            )
            fetch_initial_state(session, monitor, config.robot_key)
            publisher = CommandPublisher(velocity_publisher, posture_publisher)
            posture_requester = PostureRequester(config, publisher, monitor)
            dashboard = Dashboard(config, monitor)
            KeyboardController(config, publisher, posture_requester, dashboard).run()
    finally:
        pygame.quit()


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        keyboard_config = load_keyboard_config(args.keyboard_config)
        zenoh_config = zenoh.Config.from_file(args.zenoh_config)
        run(zenoh_config, keyboard_config)
    except KeyboardInterrupt:
        print("\n[INFO] Stopped.")
    except (OSError, ValueError, ValidationError, pygame.error, zenoh.ZError) as error:
        print(f"[ERROR] {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

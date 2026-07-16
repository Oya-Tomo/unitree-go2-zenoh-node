"""Control the standalone Go2 node with a pygame keyboard deadman."""

from __future__ import annotations

import argparse
import sys
import time
from collections.abc import Callable, Sequence
from contextlib import ExitStack
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import pygame
import zenoh
from pydantic import ValidationError

from config import KeyboardConfig, load_keyboard_config
from controller import (
    JSON_ENCODING,
    Keyspace,
    PostureCommand,
    PostureTarget,
    VelocityCommand,
)
from examples.keyboard_dashboard import Dashboard, RobotStateCache

DEFAULT_KEYBOARD_CONFIG_PATH = Path("examples/keyboard-config.json5")
DEFAULT_ZENOH_CONFIG_PATH = Path("examples/keyboard-zenoh-config.json5")
ZERO_VELOCITY = (0.0, 0.0, 0.0)


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


class CommandPublisher:
    def __init__(
        self,
        velocity_put: Callable[[str], object],
        action_put: Callable[[str], object],
    ) -> None:
        self._velocity_put = velocity_put
        self._action_put = action_put

    def publish_velocity(self, velocity: tuple[float, float, float]) -> None:
        self._velocity_put(
            VelocityCommand(
                vx=velocity[0],
                vy=velocity[1],
                vyaw=velocity[2],
            ).model_dump_json()
        )

    def publish_posture(self, posture: PostureTarget) -> None:
        # The node owns the request workflow. The keyboard never retries.
        self._action_put(PostureCommand(posture=posture).model_dump_json())


def fetch_initial_state(session: zenoh.Session, cache: RobotStateCache) -> None:
    for reply in session.get(cache.keyspace.state, timeout=0.5):
        sample = reply.ok
        if sample is not None:
            cache.update(sample, initial_reply=True)


def approach(current: float, target: float, maximum_delta: float) -> float:
    if current < target:
        return min(current + maximum_delta, target)
    if current > target:
        return max(current - maximum_delta, target)
    return current


class DeadmanState:
    def __init__(self) -> None:
        self._focused = True
        self._release_required = False

    def focus_lost(self) -> None:
        self._focused = False
        self._release_required = True

    def focus_gained(self) -> None:
        self._focused = True

    def is_armed(self, *, shift_pressed: bool) -> bool:
        if not shift_pressed:
            self._release_required = False
        return self._focused and shift_pressed and not self._release_required


def target_velocity(
    keys: pygame.key.ScancodeWrapper,
    config: KeyboardConfig,
    *,
    deadman: bool,
) -> tuple[float, float, float]:
    if not deadman:
        return ZERO_VELOCITY
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


@dataclass(frozen=True, slots=True)
class EventResult:
    running: bool = True
    zero_requested: bool = False


class DashboardProtocol(Protocol):
    def draw(
        self,
        velocity: tuple[float, float, float],
        *,
        deadman: bool,
        now: float | None = None,
    ) -> None: ...


class KeyboardController:
    def __init__(
        self,
        config: KeyboardConfig,
        publisher: CommandPublisher,
        dashboard: DashboardProtocol,
    ) -> None:
        self._config = config
        self._publisher = publisher
        self._dashboard = dashboard
        self._deadman = DeadmanState()
        self._velocity = ZERO_VELOCITY
        self._last_update = time.monotonic()

    def _handle_keydown(self, event: pygame.event.Event) -> EventResult:
        if event.key == pygame.K_ESCAPE:
            return EventResult(running=False, zero_requested=True)
        if event.key == pygame.K_SPACE:
            return EventResult(zero_requested=True)
        if not self._deadman.is_armed(
            shift_pressed=bool(event.mod & pygame.KMOD_SHIFT)
        ):
            return EventResult()
        if event.key == pygame.K_r:
            self._publisher.publish_posture(PostureTarget.STAND)
        elif event.key == pygame.K_f:
            self._publisher.publish_posture(PostureTarget.DOWN)
        return EventResult()

    def _handle_event(self, event: pygame.event.Event) -> EventResult:
        if event.type == pygame.QUIT:
            return EventResult(running=False, zero_requested=True)
        if event.type == pygame.WINDOWFOCUSLOST:
            self._deadman.focus_lost()
            return EventResult(zero_requested=True)
        if event.type == pygame.WINDOWFOCUSGAINED:
            self._deadman.focus_gained()
        elif event.type == pygame.KEYDOWN:
            return self._handle_keydown(event)
        elif event.type == pygame.KEYUP and event.key in (
            pygame.K_LSHIFT,
            pygame.K_RSHIFT,
        ):
            self._deadman.is_armed(shift_pressed=bool(event.mod & pygame.KMOD_SHIFT))
            return EventResult(zero_requested=True)
        return EventResult()

    def _process_events(self) -> EventResult:
        running = True
        zero_requested = False
        for event in pygame.event.get():
            result = self._handle_event(event)
            running = running and result.running
            zero_requested = zero_requested or result.zero_requested
        return EventResult(running=running, zero_requested=zero_requested)

    def _update_velocity(self, *, zero_requested: bool) -> bool:
        keys = pygame.key.get_pressed()
        shift_pressed = bool(keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT])
        deadman = self._deadman.is_armed(shift_pressed=shift_pressed)
        now = time.monotonic()
        period = 1.0 / self._config.publish_frequency_hz
        elapsed = min(now - self._last_update, period * 2.0)
        self._last_update = now
        if zero_requested or keys[pygame.K_SPACE] or not deadman:
            self._velocity = ZERO_VELOCITY
            if zero_requested:
                self._publisher.publish_velocity(ZERO_VELOCITY)
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
            while True:
                event_result = self._process_events()
                if not event_result.running:
                    break
                deadman = self._update_velocity(
                    zero_requested=event_result.zero_requested
                )
                self._publisher.publish_velocity(self._velocity)
                self._dashboard.draw(self._velocity, deadman=deadman)
                frame_clock.tick(max(1, round(self._config.publish_frequency_hz)))
        finally:
            self._publisher.publish_velocity(ZERO_VELOCITY)


def run(zenoh_config: zenoh.Config, config: KeyboardConfig) -> None:
    keyspace = Keyspace(config.robot_key)
    cache = RobotStateCache(keyspace)
    zenoh.init_log_from_env_or("error")
    pygame.init()

    try:
        with zenoh.open(zenoh_config) as session, ExitStack() as resources:
            velocity_publisher = resources.enter_context(
                session.declare_publisher(
                    keyspace.command,
                    encoding=JSON_ENCODING,
                    congestion_control=zenoh.CongestionControl.DROP,
                    reliability=zenoh.Reliability.BEST_EFFORT,
                )
            )
            action_publisher = resources.enter_context(
                session.declare_publisher(
                    keyspace.command,
                    encoding=JSON_ENCODING,
                    congestion_control=zenoh.CongestionControl.BLOCK,
                    reliability=zenoh.Reliability.RELIABLE,
                )
            )
            resources.enter_context(
                session.declare_subscriber(keyspace.state, cache.update)
            )
            fetch_initial_state(session, cache)
            publisher = CommandPublisher(
                velocity_publisher.put,
                action_publisher.put,
            )
            KeyboardController(
                config,
                publisher,
                Dashboard(config, cache),
            ).run()
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

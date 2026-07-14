"""Control a Go2 Zenoh node with a pygame keyboard deadman."""

from __future__ import annotations

import argparse
import sys
import time
from collections.abc import Sequence
from contextlib import ExitStack
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import pygame
import zenoh
from pydantic import ValidationError

from examples.keyboard_dashboard import Dashboard
from examples.keyboard_io import (
    CommandPublisher,
    PostureObserver,
    RobotStateCache,
    fetch_initial_state,
)
from keyspace import RobotKeyspace
from models import Posture, PostureTarget
from settings import KeyboardConfig, load_keyboard_config

DEFAULT_KEYBOARD_CONFIG_PATH = Path("examples/keyboard-config.json5")
DEFAULT_ZENOH_CONFIG_PATH = Path("examples/keyboard-zenoh-config.json5")
JSON_ENCODING = "application/json"
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


@dataclass
class PendingPosture:
    target: PostureTarget
    requested_at: float
    next_retry_at: float
    deadline: float


class PostureRequester:
    def __init__(
        self,
        config: KeyboardConfig,
        publisher: CommandPublisher,
        observer: PostureObserver,
    ) -> None:
        self._config = config
        self._publisher = publisher
        self._observer = observer
        self._pending: PendingPosture | None = None
        self.error: str | None = None

    def request(self, target: PostureTarget, *, now: float) -> None:
        self._publisher.publish_posture(target)
        retry = self._config.posture_requests.retry_interval_seconds
        self._pending = PendingPosture(
            target=target,
            requested_at=now,
            next_retry_at=now + retry,
            deadline=now + self._config.posture_requests.timeout_seconds,
        )
        self.error = None

    def cancel_stand(self) -> None:
        if self._pending is not None and self._pending.target == "stand":
            self._pending = None

    def service(self, *, now: float) -> None:
        pending = self._pending
        if pending is None:
            return
        expected = Posture.STANDING if pending.target == "stand" else Posture.DOWN
        observed = self._observer.posture()
        if (
            observed is not None
            and observed.received_at > pending.requested_at
            and observed.posture == expected
        ):
            self._pending = None
            self.error = None
            return
        if now >= pending.deadline:
            self._pending = None
            self.error = f"posture request '{pending.target}' timed out"
            return
        if now >= pending.next_retry_at:
            self._publisher.publish_posture(pending.target)
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


@dataclass(frozen=True)
class EventResult:
    running: bool = True
    immediate_zero: bool = False


class DashboardProtocol(Protocol):
    def draw(
        self,
        velocity: tuple[float, float, float],
        *,
        deadman: bool,
        operator_error: str | None,
        now: float | None = None,
    ) -> None: ...


class KeyboardController:
    def __init__(
        self,
        config: KeyboardConfig,
        publisher: CommandPublisher,
        posture_requester: PostureRequester,
        dashboard: DashboardProtocol,
    ) -> None:
        self._config = config
        self._publisher = publisher
        self._posture_requester = posture_requester
        self._dashboard = dashboard
        self._deadman = DeadmanState()
        self._velocity = ZERO_VELOCITY
        self._last_update = time.monotonic()

    def _handle_keydown(self, event: pygame.event.Event) -> EventResult:
        if event.key == pygame.K_ESCAPE:
            self._posture_requester.cancel_stand()
            return EventResult(running=False, immediate_zero=True)
        if event.key == pygame.K_SPACE:
            return EventResult(immediate_zero=True)

        shift_pressed = bool(event.mod & pygame.KMOD_SHIFT)
        if not self._deadman.is_armed(shift_pressed=shift_pressed):
            return EventResult()
        if event.key == pygame.K_r:
            self._posture_requester.request("stand", now=time.monotonic())
        elif event.key == pygame.K_f:
            self._posture_requester.request("down", now=time.monotonic())
        return EventResult()

    def _handle_event(self, event: pygame.event.Event) -> EventResult:
        if event.type == pygame.QUIT:
            self._posture_requester.cancel_stand()
            return EventResult(running=False, immediate_zero=True)
        if event.type == pygame.WINDOWFOCUSLOST:
            self._deadman.focus_lost()
            self._posture_requester.cancel_stand()
            return EventResult(immediate_zero=True)
        if event.type == pygame.WINDOWFOCUSGAINED:
            self._deadman.focus_gained()
        elif event.type == pygame.KEYDOWN:
            return self._handle_keydown(event)
        elif event.type == pygame.KEYUP and event.key in (
            pygame.K_LSHIFT,
            pygame.K_RSHIFT,
        ):
            remaining_shift_pressed = bool(event.mod & pygame.KMOD_SHIFT)
            self._deadman.is_armed(shift_pressed=remaining_shift_pressed)
            self._posture_requester.cancel_stand()
            return EventResult(immediate_zero=True)
        return EventResult()

    def _process_events(self) -> EventResult:
        running = True
        immediate_zero = False
        for event in pygame.event.get():
            result = self._handle_event(event)
            running = running and result.running
            immediate_zero = immediate_zero or result.immediate_zero
        return EventResult(running=running, immediate_zero=immediate_zero)

    def _update_velocity(self, *, immediate_zero: bool) -> bool:
        keys = pygame.key.get_pressed()
        shift_pressed = bool(keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT])
        deadman = self._deadman.is_armed(shift_pressed=shift_pressed)
        now = time.monotonic()
        period = 1.0 / self._config.publish_frequency_hz
        elapsed = min(now - self._last_update, period * 2.0)
        self._last_update = now

        if immediate_zero or keys[pygame.K_SPACE] or not deadman:
            self._velocity = ZERO_VELOCITY
            if immediate_zero:
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
                    immediate_zero=event_result.immediate_zero
                )
                self._publisher.publish_velocity(self._velocity)
                self._posture_requester.service(now=time.monotonic())
                self._dashboard.draw(
                    self._velocity,
                    deadman=deadman,
                    operator_error=self._posture_requester.error,
                )
                frame_clock.tick(max(1, round(self._config.publish_frequency_hz)))
        finally:
            self._posture_requester.cancel_stand()
            self._publisher.publish_velocity(ZERO_VELOCITY)


def run(zenoh_config: zenoh.Config, config: KeyboardConfig) -> None:
    keyspace = RobotKeyspace(config.robot_key)
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
            posture_publisher = resources.enter_context(
                session.declare_publisher(
                    keyspace.command,
                    encoding=JSON_ENCODING,
                    congestion_control=zenoh.CongestionControl.BLOCK,
                    reliability=zenoh.Reliability.RELIABLE,
                )
            )
            resources.enter_context(
                session.declare_subscriber(keyspace.state_selector, cache.update)
            )
            fetch_initial_state(session, cache)
            publisher = CommandPublisher(
                velocity_publisher.put,
                posture_publisher.put,
            )
            posture_requester = PostureRequester(config, publisher, cache)
            dashboard = Dashboard(config, cache)
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

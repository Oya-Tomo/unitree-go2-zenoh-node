"""Control the standalone Go2 node with a pygame keyboard deadman."""

from __future__ import annotations

import argparse
import sys
from collections.abc import Sequence
from pathlib import Path

# Direct script execution bootstraps repository-local imports below.
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pygame
import zenoh

from config import KeyboardConfig, load_config
from examples.keyboard_dashboard import Dashboard, NodeStateCache
from wire import (
    JSON_ENCODING,
    Keyspace,
    PostureCommand,
    PostureTarget,
    VelocityCommand,
)

DEFAULT_KEYBOARD_CONFIG_PATH = Path("examples/keyboard-config.json5")
DEFAULT_ZENOH_CONFIG_PATH = Path("examples/keyboard-zenoh-config.json5")
ZERO_VELOCITY = (0.0, 0.0, 0.0)
POSTURE_KEYS = {pygame.K_r: PostureTarget.STAND, pygame.K_f: PostureTarget.DOWN}
SHIFT_KEYS = (pygame.K_LSHIFT, pygame.K_RSHIFT)


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


def velocity_payload(velocity: tuple[float, float, float]) -> str:
    vx, vy, vyaw = velocity
    return VelocityCommand(vx=vx, vy=vy, vyaw=vyaw).model_dump_json()


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

    def update(self, *, shift_pressed: bool) -> bool:
        """Record the Shift state and return whether the deadman is armed."""

        if not shift_pressed:
            self._release_required = False
        return self._focused and shift_pressed and not self._release_required


def target_velocity(
    keys: pygame.key.ScancodeWrapper,
    config: KeyboardConfig,
) -> tuple[float, float, float]:
    targets = config.velocity_targets
    return (
        float(keys[pygame.K_w] - keys[pygame.K_s]) * targets.vx,
        float(keys[pygame.K_a] - keys[pygame.K_d]) * targets.vy,
        float(keys[pygame.K_q] - keys[pygame.K_e]) * targets.vyaw,
    )


def ramp_velocity(
    current: tuple[float, float, float],
    target: tuple[float, float, float],
    config: KeyboardConfig,
    elapsed_seconds: float,
) -> tuple[float, float, float]:
    rates = config.velocity_ramp_rates
    vx, vy, vyaw = (
        approach(value, goal, rate * elapsed_seconds)
        for value, goal, rate in zip(
            current,
            target,
            (rates.vx_mps2, rates.vy_mps2, rates.vyaw_rad_s2),
            strict=True,
        )
    )
    return (vx, vy, vyaw)


def handle_events(
    deadman: DeadmanState,
    posture_publisher: zenoh.Publisher,
) -> tuple[bool, bool]:
    """Handle pending window events and return (running, zero_requested)."""

    running = True
    zero_requested = False
    for event in pygame.event.get():
        if event.type == pygame.QUIT or (
            event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE
        ):
            running = False
            zero_requested = True
        elif event.type == pygame.WINDOWFOCUSLOST:
            deadman.focus_lost()
            zero_requested = True
        elif event.type == pygame.WINDOWFOCUSGAINED:
            deadman.focus_gained()
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
            zero_requested = True
        elif event.type == pygame.KEYDOWN and event.key in POSTURE_KEYS:
            if deadman.update(shift_pressed=bool(event.mod & pygame.KMOD_SHIFT)):
                # The node owns the request workflow. The keyboard never retries.
                posture = POSTURE_KEYS[event.key]
                posture_publisher.put(PostureCommand(posture=posture).model_dump_json())
        elif event.type == pygame.KEYUP and event.key in SHIFT_KEYS:
            deadman.update(shift_pressed=bool(event.mod & pygame.KMOD_SHIFT))
            zero_requested = True
    return (running, zero_requested)


def control_loop(
    config: KeyboardConfig,
    velocity_publisher: zenoh.Publisher,
    posture_publisher: zenoh.Publisher,
    dashboard: Dashboard,
) -> None:
    deadman = DeadmanState()
    velocity = ZERO_VELOCITY
    elapsed_seconds = 0.0
    max_elapsed_seconds = 2.0 / config.loop_frequency_hz
    frame_clock = pygame.time.Clock()
    try:
        while True:
            running, zero_requested = handle_events(deadman, posture_publisher)
            if not running:
                return
            keys = pygame.key.get_pressed()
            armed = deadman.update(shift_pressed=any(keys[key] for key in SHIFT_KEYS))
            if zero_requested or keys[pygame.K_SPACE] or not armed:
                velocity = ZERO_VELOCITY
            else:
                velocity = ramp_velocity(
                    velocity,
                    target_velocity(keys, config),
                    config,
                    elapsed_seconds,
                )
            velocity_publisher.put(velocity_payload(velocity))
            dashboard.draw(velocity, deadman=armed)
            elapsed_seconds = min(
                frame_clock.tick(config.loop_frequency_hz) / 1000.0,
                max_elapsed_seconds,
            )
    finally:
        velocity_publisher.put(velocity_payload(ZERO_VELOCITY))


def run(zenoh_config: zenoh.Config, config: KeyboardConfig) -> None:
    keyspace = Keyspace(config.zenoh_key_prefix)
    cache = NodeStateCache()
    zenoh.init_log_from_env_or("error")
    pygame.init()
    try:
        with (
            zenoh.open(zenoh_config) as session,
            session.declare_publisher(
                keyspace.velocity,
                encoding=JSON_ENCODING,
                congestion_control=zenoh.CongestionControl.DROP,
                reliability=zenoh.Reliability.BEST_EFFORT,
            ) as velocity_publisher,
            session.declare_publisher(
                keyspace.posture,
                encoding=JSON_ENCODING,
                congestion_control=zenoh.CongestionControl.BLOCK,
                reliability=zenoh.Reliability.RELIABLE,
            ) as posture_publisher,
            session.declare_subscriber(keyspace.state, cache.update),
        ):
            control_loop(
                config,
                velocity_publisher,
                posture_publisher,
                Dashboard(config, cache),
            )
    finally:
        pygame.quit()


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        keyboard_config = load_config(KeyboardConfig, args.keyboard_config)
        zenoh_config = zenoh.Config.from_file(args.zenoh_config)
        run(zenoh_config, keyboard_config)
    except KeyboardInterrupt:
        print("\n[INFO] Stopped.")
    except (OSError, ValueError, pygame.error, zenoh.ZError) as error:
        print(f"[ERROR] {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

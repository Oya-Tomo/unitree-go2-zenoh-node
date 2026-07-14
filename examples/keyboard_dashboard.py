from __future__ import annotations

import time

import pygame
from pydantic import ValidationError

from examples.keyboard_io import ObservedState, RobotStateCache
from models import VelocityState
from settings import KeyboardConfig

WINDOW_SIZE = (900, 520)
BACKGROUND = (20, 23, 28)
FOREGROUND = (232, 236, 241)
ACCENT = (86, 182, 194)
WARNING = (244, 180, 0)


def _format_velocity(
    state: ObservedState | None, *, now: float, maximum_age: float
) -> str:
    if state is None:
        return "waiting"
    if state.is_stale(now=now, maximum_age=maximum_age):
        return "stale"
    try:
        velocity = VelocityState.model_validate(state.value)
        return (
            f"vx={velocity.vx:+.3f}  vy={velocity.vy:+.3f}  vyaw={velocity.vyaw:+.3f}"
        )
    except ValidationError:
        return "invalid state payload"


class Dashboard:
    def __init__(self, config: KeyboardConfig, cache: RobotStateCache) -> None:
        self._config = config
        self._cache = cache
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
        now: float,
    ) -> list[tuple[pygame.font.Font, str, tuple[int, int, int]]]:
        states = self._cache.snapshot()
        keys = self._cache.keyspace
        requested = states.get(keys.requested_velocity)
        applied = states.get(keys.applied_velocity)
        posture_state = states.get(keys.posture)
        health_state = states.get(keys.health)
        posture_stale = posture_state is None or posture_state.is_stale(
            now=now, maximum_age=self._config.state_stale_after_seconds
        )
        posture = "stale"
        if not posture_stale and posture_state is not None:
            posture = posture_state.value.get("posture", "invalid")
        health = health_state.value if health_state is not None else {}
        health_stale = health_state is None or health_state.is_stale(
            now=now, maximum_age=self._config.state_stale_after_seconds
        )
        health_status = "STALE" if health_stale else health.get("status", "waiting")
        last_error = health.get("last_error") or "none"
        return [
            (self._font, f"Go2 keyboard node: {keys.robot_key}", ACCENT),
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
                "Requested: "
                f"{_format_velocity(requested, now=now, maximum_age=self._config.state_stale_after_seconds)}",
                FOREGROUND,
            ),
            (
                self._small_font,
                "Applied:   "
                f"{_format_velocity(applied, now=now, maximum_age=self._config.state_stale_after_seconds)}",
                FOREGROUND,
            ),
            (self._small_font, f"Posture: {posture}", FOREGROUND),
            (
                self._small_font,
                "Health: "
                f"{health_status}  "
                f"walking_enabled={health.get('walking_enabled', 'waiting')}  "
                f"watchdog={health.get('watchdog_triggered', 'waiting')}",
                WARNING if health_stale else FOREGROUND,
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
        now: float | None = None,
    ) -> None:
        now = time.monotonic() if now is None else now
        self._screen.fill(BACKGROUND)
        y = 24
        for font, text, color in self._state_lines(
            velocity,
            deadman=deadman,
            operator_error=operator_error,
            now=now,
        ):
            rendered = font.render(text, True, color)
            self._screen.blit(rendered, (24, y))
            y += rendered.get_height() + 14
        pygame.display.flip()

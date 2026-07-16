"""Pygame projection of the node's canonical State snapshot."""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass
from threading import Lock
from typing import Protocol

import pygame
from pydantic import ValidationError

from config import KeyboardConfig
from controller import Keyspace, NodeState

WINDOW_SIZE = (920, 620)
BACKGROUND = (20, 23, 28)
FOREGROUND = (232, 236, 241)
ACCENT = (86, 182, 194)
WARNING = (244, 180, 0)


class SamplePayload(Protocol):
    def to_bytes(self) -> bytes: ...


class StateSample(Protocol):
    @property
    def payload(self) -> SamplePayload: ...

    @property
    def key_expr(self) -> object: ...


@dataclass(frozen=True, slots=True)
class ObservedSnapshot:
    value: NodeState
    received_at: float

    def is_stale(self, *, now: float, maximum_age: float) -> bool:
        return now - self.received_at >= maximum_age


class RobotStateCache:
    """Thread-safe read model for the node's canonical State snapshot."""

    def __init__(
        self,
        keyspace: Keyspace,
        *,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.keyspace = keyspace
        self._clock = clock
        self._state: ObservedSnapshot | None = None
        self._lock = Lock()

    def update(self, sample: StateSample, *, initial_reply: bool = False) -> None:
        if str(sample.key_expr) != self.keyspace.state:
            return
        try:
            state = NodeState.model_validate_json(sample.payload.to_bytes())
        except (UnicodeDecodeError, ValidationError, ValueError):
            return
        with self._lock:
            if initial_reply and self._state is not None:
                return
            self._state = ObservedSnapshot(state, self._clock())

    def latest(self) -> ObservedSnapshot | None:
        with self._lock:
            return self._state


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
        now: float,
    ) -> list[tuple[pygame.font.Font, str, tuple[int, int, int]]]:
        observed = self._cache.latest()
        if observed is None:
            state_lines = [
                (self._small_font, "Node State: waiting", WARNING),
            ]
        elif observed.is_stale(
            now=now,
            maximum_age=self._config.state_stale_after_seconds,
        ):
            state_lines = [(self._small_font, "Node State: stale", WARNING)]
        else:
            snapshot = observed.value
            state = snapshot.robot
            velocity_text = (
                "unknown"
                if state.velocity is None
                else "vx={:+.3f}  vy={:+.3f}  vz={:+.3f}  yaw={:+.3f}".format(
                    *state.velocity,
                    state.yaw_speed or 0.0,
                )
            )
            requested_velocity = snapshot.requested_velocity
            diagnostic = snapshot.last_sdk
            state_lines = [
                (
                    self._small_font,
                    f"Lifecycle: {snapshot.lifecycle}",
                    FOREGROUND,
                ),
                (
                    self._small_font,
                    f"Robot State: {state.state} / {state.motion}",
                    WARNING if state.state == "unknown" else FOREGROUND,
                ),
                (
                    self._small_font,
                    f"State reason: {state.reason or 'none'}",
                    WARNING if state.reason else FOREGROUND,
                ),
                (
                    self._small_font,
                    "State machine: "
                    f"{state.state_machine_code} ({state.state_machine_name})  "
                    f"mode={state.mode} ({state.mode_name})",
                    FOREGROUND,
                ),
                (self._small_font, f"Observed: {velocity_text}", FOREGROUND),
                (
                    self._small_font,
                    "Requested posture: "
                    f"{snapshot.requested_posture or 'none'}  "
                    f"action={snapshot.posture_action or 'none'}",
                    FOREGROUND,
                ),
                (
                    self._small_font,
                    "Requested velocity: "
                    + (
                        f"vx={requested_velocity.vx:+.3f}  "
                        f"vy={requested_velocity.vy:+.3f}  "
                        f"vyaw={requested_velocity.vyaw:+.3f}"
                        if requested_velocity
                        else "none"
                    ),
                    FOREGROUND,
                ),
                (
                    self._small_font,
                    "DDS connected: "
                    f"{snapshot.connected}  "
                    f"accepting commands={snapshot.accepting_commands}",
                    FOREGROUND,
                ),
                (
                    self._small_font,
                    "Last RPC: "
                    + (
                        f"{diagnostic.command} code={diagnostic.code} "
                        f"error={diagnostic.error or 'none'}"
                        if diagnostic
                        else "none"
                    ),
                    FOREGROUND,
                ),
                (
                    self._small_font,
                    f"Node error: {snapshot.last_error or 'none'}",
                    WARNING if snapshot.last_error else FOREGROUND,
                ),
            ]
        return [
            (
                self._font,
                f"Go2 keyboard node: {self._cache.keyspace.robot_key}",
                ACCENT,
            ),
            (
                self._font,
                f"Deadman: {'ACTIVE' if deadman else 'released'}",
                ACCENT if deadman else WARNING,
            ),
            (
                self._small_font,
                "Local command: "
                f"vx={velocity[0]:+.3f}  vy={velocity[1]:+.3f}  "
                f"vyaw={velocity[2]:+.3f}",
                FOREGROUND,
            ),
            *state_lines,
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
        now: float | None = None,
    ) -> None:
        now = time.monotonic() if now is None else now
        self._screen.fill(BACKGROUND)
        y = 18
        for font, text, color in self._state_lines(
            velocity,
            deadman=deadman,
            now=now,
        ):
            rendered = font.render(text, True, color)
            self._screen.blit(rendered, (20, y))
            y += rendered.get_height() + 10
        pygame.display.flip()

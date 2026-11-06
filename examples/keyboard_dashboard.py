"""Pygame projection of the node's published State."""

from __future__ import annotations

import time
from dataclasses import dataclass
from threading import Lock

import pygame
import zenoh

from config import KeyboardConfig
from wire import PublishedNodeState, RobotState

WINDOW_SIZE = (920, 620)
BACKGROUND = (20, 23, 28)
FOREGROUND = (232, 236, 241)
ACCENT = (86, 182, 194)
WARNING = (244, 180, 0)

type Color = tuple[int, int, int]


@dataclass(frozen=True, slots=True)
class ReceivedNodeState:
    value: PublishedNodeState
    received_at: float


class NodeStateCache:
    """Thread-safe cache of the latest published Node State."""

    def __init__(self) -> None:
        self._state: ReceivedNodeState | None = None
        self._lock = Lock()

    def update(self, sample: zenoh.Sample) -> None:
        try:
            state = PublishedNodeState.model_validate_json(sample.payload.to_bytes())
        except ValueError:
            return
        with self._lock:
            self._state = ReceivedNodeState(state, time.monotonic())

    def latest(self) -> ReceivedNodeState | None:
        with self._lock:
            return self._state


def format_velocity(vx: float, vy: float, vyaw: float) -> str:
    return f"vx={vx:+.3f}  vy={vy:+.3f}  vyaw={vyaw:+.3f}"


class Dashboard:
    def __init__(self, config: KeyboardConfig, cache: NodeStateCache) -> None:
        self._config = config
        self._cache = cache
        pygame.display.set_caption("Unitree Go2 Zenoh keyboard controller")
        self._screen = pygame.display.set_mode(WINDOW_SIZE)
        self._font = pygame.font.Font(None, 34)
        self._small_font = pygame.font.Font(None, 25)

    def _node_lines(self) -> list[tuple[str, Color]]:
        received = self._cache.latest()
        if received is None:
            return [("Node State: waiting", WARNING)]
        if (
            time.monotonic() - received.received_at
            >= self._config.node_state_timeout_seconds
        ):
            return [("Node State: stale", WARNING)]

        snapshot = received.value
        robot = snapshot.robot_state
        observed = (
            "unknown"
            if robot.velocity is None
            else "vx={:+.3f}  vy={:+.3f}  vz={:+.3f}  yaw={:+.3f}".format(
                *robot.velocity,
                robot.yaw_speed or 0.0,
            )
        )
        requested = snapshot.requested_velocity
        diagnostic = snapshot.last_sdk_diagnostic
        return [
            (f"Lifecycle: {snapshot.lifecycle}", FOREGROUND),
            (
                f"Robot State: {robot.state} / {robot.motion}",
                WARNING if robot.state is RobotState.UNKNOWN else FOREGROUND,
            ),
            (
                f"State reason: {robot.reason or 'none'}",
                WARNING if robot.reason else FOREGROUND,
            ),
            (
                f"State machine: {robot.state_machine_code} "
                f"({robot.state_machine_name})  "
                f"mode={robot.mode} ({robot.mode_name})",
                FOREGROUND,
            ),
            (f"Observed: {observed}", FOREGROUND),
            (
                f"Requested posture: {snapshot.requested_posture or 'none'}  "
                f"action={snapshot.last_posture_action or 'none'}",
                FOREGROUND,
            ),
            (
                "Requested velocity: "
                + (
                    format_velocity(requested.vx, requested.vy, requested.vyaw)
                    if requested
                    else "none"
                ),
                FOREGROUND,
            ),
            (
                f"Robot connected: {snapshot.robot_connected}  "
                f"accepting commands={snapshot.accepting_commands}",
                FOREGROUND,
            ),
            (
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
                f"Node error: {snapshot.last_node_error or 'none'}",
                WARNING if snapshot.last_node_error else FOREGROUND,
            ),
        ]

    def draw(self, velocity: tuple[float, float, float], *, deadman: bool) -> None:
        small_lines = [
            (f"Local command: {format_velocity(*velocity)}", FOREGROUND),
            *self._node_lines(),
            ("Shift+W/S: vx   Shift+A/D: vy   Shift+Q/E: yaw", FOREGROUND),
            (
                "Shift+R: stand   Shift+F: down   Space: zero   Esc: zero and exit",
                FOREGROUND,
            ),
        ]
        lines = [
            (self._font, f"Go2 keyboard node: {self._config.zenoh_key_prefix}", ACCENT),
            (
                self._font,
                f"Deadman: {'ACTIVE' if deadman else 'released'}",
                ACCENT if deadman else WARNING,
            ),
            *((self._small_font, text, color) for text, color in small_lines),
        ]
        self._screen.fill(BACKGROUND)
        y = 18
        for font, text, color in lines:
            rendered = font.render(text, True, color)
            self._screen.blit(rendered, (20, y))
            y += rendered.get_height() + 10
        pygame.display.flip()

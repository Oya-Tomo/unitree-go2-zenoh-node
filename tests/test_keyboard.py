from __future__ import annotations

import json
import unittest
from dataclasses import dataclass
from typing import cast

import pygame

from examples.keyboard import (
    DeadmanState,
    KeyboardController,
    PostureRequester,
    ramp_velocity,
    target_velocity,
)
from examples.keyboard_io import CommandPublisher, PostureObservation
from models import Posture
from settings import KeyboardConfig


def keyboard_config() -> KeyboardConfig:
    return KeyboardConfig.model_validate(
        {
            "robot_key": "unitree/go2",
            "publish_frequency_hz": 20.0,
            "state_stale_after_seconds": 2.5,
            "targets": {"vx": 0.5, "vy": 0.3, "vyaw": 1.0},
            "ramp_rates": {"vx": 1.0, "vy": 0.8, "vyaw": 2.0},
            "posture_requests": {
                "retry_interval_seconds": 0.5,
                "timeout_seconds": 10.0,
            },
        }
    )


class FakeKeys:
    def __init__(self, *pressed: int) -> None:
        self._pressed = set(pressed)

    def __getitem__(self, key: int) -> bool:
        return key in self._pressed


class FakePut:
    def __init__(self) -> None:
        self.payloads: list[str] = []

    def __call__(self, payload: str) -> None:
        self.payloads.append(payload)


class FakeObserver:
    def __init__(self) -> None:
        self.observation: PostureObservation | None = None

    def posture(self) -> PostureObservation | None:
        return self.observation


@dataclass
class FakeDashboard:
    draws: int = 0

    def draw(
        self,
        velocity: tuple[float, float, float],
        *,
        deadman: bool,
        operator_error: str | None,
        now: float | None = None,
    ) -> None:
        self.draws += 1


class KeyboardFixture:
    def __init__(self) -> None:
        self.velocity_put = FakePut()
        self.posture_put = FakePut()
        self.publisher = CommandPublisher(self.velocity_put, self.posture_put)
        self.observer = FakeObserver()
        self.requester = PostureRequester(
            keyboard_config(),
            self.publisher,
            self.observer,
        )
        self.controller = KeyboardController(
            keyboard_config(),
            self.publisher,
            self.requester,
            FakeDashboard(),
        )


class KeyboardTests(unittest.TestCase):
    def test_target_velocity_combines_axes(self) -> None:
        keys = cast(
            pygame.key.ScancodeWrapper,
            FakeKeys(pygame.K_w, pygame.K_a, pygame.K_e),
        )

        self.assertEqual(
            target_velocity(keys, keyboard_config(), deadman=True),
            (0.5, 0.3, -1.0),
        )
        self.assertEqual(
            target_velocity(keys, keyboard_config(), deadman=False),
            (0.0, 0.0, 0.0),
        )

    def test_ramp_velocity_respects_per_axis_rates(self) -> None:
        velocity = ramp_velocity(
            (0.0, 0.0, 0.0),
            (0.5, -0.3, 1.0),
            keyboard_config(),
            0.25,
        )

        self.assertEqual(velocity, (0.25, -0.2, 0.5))

    def test_command_publisher_uses_discriminated_json_models(self) -> None:
        velocity_put = FakePut()
        posture_put = FakePut()
        publisher = CommandPublisher(velocity_put, posture_put)

        publisher.publish_velocity((0.5, 0.0, -0.25))
        publisher.publish_posture("down")

        self.assertEqual(json.loads(velocity_put.payloads[0])["type"], "velocity")
        self.assertEqual(
            json.loads(posture_put.payloads[0]),
            {"type": "posture", "posture": "down"},
        )

    def test_focus_loss_requires_shift_release_before_rearming(self) -> None:
        deadman = DeadmanState()
        self.assertTrue(deadman.is_armed(shift_pressed=True))

        deadman.focus_lost()
        deadman.focus_gained()

        self.assertFalse(deadman.is_armed(shift_pressed=True))
        self.assertFalse(deadman.is_armed(shift_pressed=False))
        self.assertTrue(deadman.is_armed(shift_pressed=True))

    def test_focus_rearm_gate_also_blocks_posture_shortcuts(self) -> None:
        fixture = KeyboardFixture()
        fixture.controller._handle_event(pygame.event.Event(pygame.WINDOWFOCUSLOST))
        fixture.controller._handle_event(pygame.event.Event(pygame.WINDOWFOCUSGAINED))

        fixture.controller._handle_event(
            pygame.event.Event(
                pygame.KEYDOWN,
                key=pygame.K_r,
                mod=pygame.KMOD_SHIFT,
            )
        )
        self.assertEqual(fixture.posture_put.payloads, [])

        fixture.controller._handle_event(
            pygame.event.Event(
                pygame.KEYUP,
                key=pygame.K_LSHIFT,
                mod=pygame.KMOD_NONE,
            )
        )
        fixture.controller._handle_event(
            pygame.event.Event(
                pygame.KEYDOWN,
                key=pygame.K_r,
                mod=pygame.KMOD_SHIFT,
            )
        )
        self.assertEqual(len(fixture.posture_put.payloads), 1)

    def test_rearming_requires_all_shift_keys_to_be_released(self) -> None:
        fixture = KeyboardFixture()
        fixture.controller._handle_event(pygame.event.Event(pygame.WINDOWFOCUSLOST))
        fixture.controller._handle_event(pygame.event.Event(pygame.WINDOWFOCUSGAINED))

        fixture.controller._handle_event(
            pygame.event.Event(
                pygame.KEYUP,
                key=pygame.K_LSHIFT,
                mod=pygame.KMOD_SHIFT,
            )
        )
        fixture.controller._handle_event(
            pygame.event.Event(
                pygame.KEYDOWN,
                key=pygame.K_r,
                mod=pygame.KMOD_SHIFT,
            )
        )
        self.assertEqual(fixture.posture_put.payloads, [])

        fixture.controller._handle_event(
            pygame.event.Event(
                pygame.KEYUP,
                key=pygame.K_RSHIFT,
                mod=pygame.KMOD_NONE,
            )
        )
        fixture.controller._handle_event(
            pygame.event.Event(
                pygame.KEYDOWN,
                key=pygame.K_r,
                mod=pygame.KMOD_SHIFT,
            )
        )
        self.assertEqual(len(fixture.posture_put.payloads), 1)

    def test_focus_loss_cancels_pending_stand_retries(self) -> None:
        fixture = KeyboardFixture()
        fixture.requester.request("stand", now=0.0)

        fixture.controller._handle_event(pygame.event.Event(pygame.WINDOWFOCUSLOST))
        fixture.requester.service(now=1.0)

        self.assertEqual(len(fixture.posture_put.payloads), 1)

    def test_escape_cancels_pending_stand_retries(self) -> None:
        fixture = KeyboardFixture()
        fixture.requester.request("stand", now=0.0)

        result = fixture.controller._handle_event(
            pygame.event.Event(
                pygame.KEYDOWN,
                key=pygame.K_ESCAPE,
                mod=pygame.KMOD_NONE,
            )
        )
        fixture.requester.service(now=1.0)

        self.assertFalse(result.running)
        self.assertTrue(result.immediate_zero)
        self.assertEqual(len(fixture.posture_put.payloads), 1)

    def test_shift_release_cancels_pending_stand_retries(self) -> None:
        fixture = KeyboardFixture()
        fixture.requester.request("stand", now=0.0)

        fixture.controller._handle_event(
            pygame.event.Event(
                pygame.KEYUP,
                key=pygame.K_LSHIFT,
                mod=pygame.KMOD_NONE,
            )
        )
        fixture.requester.service(now=1.0)

        self.assertEqual(len(fixture.posture_put.payloads), 1)

    def test_focus_loss_keeps_pending_safety_down_retry(self) -> None:
        fixture = KeyboardFixture()
        fixture.requester.request("down", now=0.0)

        fixture.controller._handle_event(pygame.event.Event(pygame.WINDOWFOCUSLOST))
        fixture.requester.service(now=1.0)

        self.assertEqual(len(fixture.posture_put.payloads), 2)

    def test_posture_request_rejects_observation_older_than_request(self) -> None:
        fixture = KeyboardFixture()
        fixture.observer.observation = PostureObservation(
            posture=Posture.DOWN,
            received_at=-1.0,
        )

        fixture.requester.request("down", now=0.0)
        fixture.requester.service(now=0.5)

        self.assertEqual(len(fixture.posture_put.payloads), 2)

        fixture.observer.observation = PostureObservation(
            posture=Posture.DOWN,
            received_at=0.6,
        )
        fixture.requester.service(now=0.6)
        fixture.requester.service(now=1.5)
        self.assertEqual(len(fixture.posture_put.payloads), 2)

    def test_posture_request_timeout_is_reported(self) -> None:
        fixture = KeyboardFixture()

        fixture.requester.request("stand", now=0.0)
        fixture.requester.service(now=10.0)

        self.assertEqual(
            fixture.requester.error,
            "posture request 'stand' timed out",
        )


if __name__ == "__main__":
    unittest.main()

from __future__ import annotations

import json
import unittest
from typing import cast

import pygame
import zenoh

from examples.keyboard import (
    CommandPublisher,
    DeadmanState,
    KeyboardConfig,
    PostureRequester,
    StateMonitor,
    ramp_velocity,
    target_velocity,
)


def keyboard_config() -> KeyboardConfig:
    return KeyboardConfig.model_validate(
        {
            "robot_key": "unitree/go2",
            "publish_frequency_hz": 20.0,
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


class FakePublisher:
    def __init__(self) -> None:
        self.payloads: list[str] = []

    def put(self, payload: str) -> None:
        self.payloads.append(payload)


class FakeMonitor:
    def __init__(self) -> None:
        self.value: str | None = None

    def posture(self, _robot_key: str) -> str | None:
        return self.value


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
        velocity = FakePublisher()
        posture = FakePublisher()
        publisher = CommandPublisher(
            cast(zenoh.Publisher, velocity),
            cast(zenoh.Publisher, posture),
        )

        publisher.velocity((0.5, 0.0, -0.25))
        publisher.posture("down")

        self.assertEqual(json.loads(velocity.payloads[0])["type"], "velocity")
        self.assertEqual(
            json.loads(posture.payloads[0]),
            {"type": "posture", "posture": "down"},
        )

    def test_focus_loss_requires_shift_release_before_rearming(self) -> None:
        deadman = DeadmanState()
        self.assertTrue(deadman.active(shift_pressed=True))

        deadman.focus_lost()
        deadman.focus_gained()

        self.assertFalse(deadman.active(shift_pressed=True))
        self.assertFalse(deadman.active(shift_pressed=False))
        self.assertTrue(deadman.active(shift_pressed=True))

    def test_posture_request_retries_until_observed(self) -> None:
        raw_posture = FakePublisher()
        publisher = CommandPublisher(
            cast(zenoh.Publisher, FakePublisher()),
            cast(zenoh.Publisher, raw_posture),
        )
        monitor = FakeMonitor()
        requester = PostureRequester(
            keyboard_config(),
            publisher,
            cast(StateMonitor, monitor),
        )

        requester.request("down", now=0.0)
        requester.service(now=0.5)
        monitor.value = "down"
        requester.service(now=0.6)
        requester.service(now=1.5)

        self.assertEqual(len(raw_posture.payloads), 2)
        self.assertIsNone(requester.error)

    def test_posture_request_timeout_is_reported(self) -> None:
        requester = PostureRequester(
            keyboard_config(),
            CommandPublisher(
                cast(zenoh.Publisher, FakePublisher()),
                cast(zenoh.Publisher, FakePublisher()),
            ),
            cast(StateMonitor, FakeMonitor()),
        )

        requester.request("stand", now=0.0)
        requester.service(now=10.0)

        self.assertEqual(requester.error, "posture request 'stand' timed out")


if __name__ == "__main__":
    unittest.main()

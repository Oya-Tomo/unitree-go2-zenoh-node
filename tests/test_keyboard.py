from __future__ import annotations

import json
import unittest
from pathlib import Path

from controller import PostureTarget
from examples.keyboard import (
    ZERO_VELOCITY,
    CommandPublisher,
    DeadmanState,
    approach,
    ramp_velocity,
)
from settings import load_keyboard_config

ROOT = Path(__file__).parents[1]


class PutRecorder:
    def __init__(self) -> None:
        self.payloads: list[str] = []

    def __call__(self, payload: str) -> None:
        self.payloads.append(payload)


class KeyboardTests(unittest.TestCase):
    def test_approach_does_not_overshoot(self) -> None:
        self.assertEqual(approach(0.0, 1.0, 0.25), 0.25)
        self.assertEqual(approach(0.9, 1.0, 0.25), 1.0)
        self.assertEqual(approach(0.0, -1.0, 0.25), -0.25)

    def test_ramp_uses_each_axis_rate(self) -> None:
        config = load_keyboard_config(ROOT / "examples/keyboard-config.example.json5")
        velocity = ramp_velocity(
            ZERO_VELOCITY,
            (1.0, 1.0, 1.0),
            config,
            0.1,
        )
        self.assertEqual(
            velocity,
            (
                min(config.ramp_rates.vx * 0.1, 1.0),
                min(config.ramp_rates.vy * 0.1, 1.0),
                min(config.ramp_rates.vyaw * 0.1, 1.0),
            ),
        )

    def test_publisher_preserves_the_two_existing_command_types(self) -> None:
        velocity_put = PutRecorder()
        posture_put = PutRecorder()
        publisher = CommandPublisher(velocity_put, posture_put)

        publisher.publish_velocity((0.1, 0.2, 0.3))
        publisher.publish_posture(PostureTarget.DOWN)

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


if __name__ == "__main__":
    unittest.main()

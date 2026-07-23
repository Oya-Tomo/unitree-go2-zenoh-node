from __future__ import annotations

import importlib
import json
import os
import subprocess
import sys
import unittest
from pathlib import Path

from config import load_keyboard_config
from controller import PostureTarget
from examples import keyboard
from examples.keyboard import (
    ZERO_VELOCITY,
    CommandPublisher,
    DeadmanState,
    approach,
    ramp_velocity,
)

ROOT = Path(__file__).parents[1]


class PutRecorder:
    def __init__(self) -> None:
        self.payloads: list[str] = []

    def __call__(self, payload: str) -> None:
        self.payloads.append(payload)


class KeyboardTests(unittest.TestCase):
    def test_direct_script_help_succeeds(self) -> None:
        environment = os.environ.copy()
        environment["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"

        result = subprocess.run(
            [sys.executable, str(ROOT / "examples/keyboard.py"), "--help"],
            cwd=ROOT,
            env=environment,
            capture_output=True,
            text=True,
            check=False,
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("unitree-go2-keyboard", result.stdout)
        self.assertIn("--keyboard-config", result.stdout)
        self.assertIn("--zenoh-config", result.stdout)

    def test_module_import_does_not_modify_sys_path(self) -> None:
        original_path = sys.path.copy()

        importlib.reload(keyboard)

        self.assertEqual(sys.path, original_path)

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
                min(config.velocity_ramp_rates.vx_mps2 * 0.1, 1.0),
                min(config.velocity_ramp_rates.vy_mps2 * 0.1, 1.0),
                min(config.velocity_ramp_rates.vyaw_rad_s2 * 0.1, 1.0),
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

from __future__ import annotations

import unittest

from pydantic import ValidationError

from models import PostureCommand, VelocityCommand, decode_command


class CommandModelTests(unittest.TestCase):
    def test_velocity_accepts_official_boundaries(self) -> None:
        lower = VelocityCommand(vx=-2.5, vy=-1.0, vyaw=-4.0)
        upper = VelocityCommand(vx=3.8, vy=1.0, vyaw=4.0)

        self.assertEqual(lower.vx, -2.5)
        self.assertEqual(upper.vyaw, 4.0)

    def test_velocity_rejects_values_outside_official_boundaries(self) -> None:
        for values in (
            {"vx": -2.5001, "vy": 0.0, "vyaw": 0.0},
            {"vx": 3.8001, "vy": 0.0, "vyaw": 0.0},
            {"vx": 0.0, "vy": -1.0001, "vyaw": 0.0},
            {"vx": 0.0, "vy": 1.0001, "vyaw": 0.0},
            {"vx": 0.0, "vy": 0.0, "vyaw": -4.0001},
            {"vx": 0.0, "vy": 0.0, "vyaw": 4.0001},
        ):
            with self.subTest(values=values), self.assertRaises(ValidationError):
                VelocityCommand.model_validate(values)

    def test_velocity_rejects_non_finite_values(self) -> None:
        for value in (float("inf"), float("-inf"), float("nan")):
            with self.subTest(value=value), self.assertRaises(ValidationError):
                VelocityCommand(vx=value, vy=0.0, vyaw=0.0)

    def test_discriminated_union_decodes_both_commands(self) -> None:
        velocity = decode_command(b'{"type":"velocity","vx":0.5,"vy":0.0,"vyaw":-0.25}')
        posture = decode_command(b'{"type":"posture","posture":"down"}')

        self.assertIsInstance(velocity, VelocityCommand)
        self.assertEqual(posture, PostureCommand(posture="down"))

    def test_commands_forbid_unknown_fields(self) -> None:
        with self.assertRaises(ValidationError):
            decode_command(b'{"type":"velocity","vx":0,"vy":0,"vyaw":0,"speed":1}')

    def test_velocity_rejects_numeric_strings(self) -> None:
        with self.assertRaises(ValidationError):
            decode_command(b'{"type":"velocity","vx":"0.5","vy":0,"vyaw":0}')


if __name__ == "__main__":
    unittest.main()

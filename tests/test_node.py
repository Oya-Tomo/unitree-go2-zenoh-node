from __future__ import annotations

import unittest
from dataclasses import dataclass

from pydantic import ValidationError

from controller import PostureCommand, VelocityCommand, decode_command
from node import StateEvent, StateInbox, to_observation


@dataclass(frozen=True)
class FakeStamp:
    sec: int = 12
    nanosec: int = 34


@dataclass(frozen=True)
class FakeState:
    stamp: FakeStamp = FakeStamp()
    error_code: int = 100
    mode: int = 5
    velocity: tuple[float, float, float] = (0.1, 0.2, 0.3)
    yaw_speed: float = 0.4


class NodeBoundaryTests(unittest.TestCase):
    def test_wire_contract_has_only_velocity_and_posture(self) -> None:
        velocity = decode_command(b'{"type":"velocity","vx":0.1,"vy":0.0,"vyaw":-0.2}')
        posture = decode_command(b'{"type":"posture","posture":"down"}')
        self.assertIsInstance(velocity, VelocityCommand)
        self.assertIsInstance(posture, PostureCommand)
        with self.assertRaises(ValidationError):
            decode_command(b'{"type":"stop"}')

    def test_state_conversion_uses_the_full_velocity_vector(self) -> None:
        observation = to_observation(FakeState(), received_at=1.5)
        self.assertEqual(observation.velocity, (0.1, 0.2, 0.3))
        self.assertEqual(observation.stamp_sec, 12)
        self.assertEqual(observation.stamp_nanosec, 34)
        self.assertEqual(observation.state_machine_code, 100)

    def test_state_inbox_keeps_only_the_latest_update(self) -> None:
        inbox = StateInbox()
        inbox.put(StateEvent(error="first"))
        inbox.put(StateEvent(error="second"))

        result = inbox.wait_after(0, timeout=0.0)
        self.assertIsNotNone(result)
        assert result is not None
        revision, event = result
        self.assertEqual(revision, 2)
        self.assertEqual(event.error, "second")


if __name__ == "__main__":
    unittest.main()

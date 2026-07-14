from __future__ import annotations

import unittest
from unittest.mock import Mock

from command_mailbox import CommandMailbox
from controller import RobotController
from models import PostureCommand, VelocityCommand
from node import dispatch_commands


class CommandMailboxTests(unittest.TestCase):
    def test_velocity_slot_keeps_only_the_newest_command(self) -> None:
        mailbox = CommandMailbox()
        mailbox.submit(
            b'{"type":"velocity","vx":0.1,"vy":0,"vyaw":0}',
            received_at=1.0,
        )
        mailbox.submit(
            b'{"type":"velocity","vx":0.2,"vy":0,"vyaw":0}',
            received_at=2.0,
        )

        batch = mailbox.take()

        self.assertIsNotNone(batch.velocity)
        assert batch.velocity is not None
        self.assertEqual(batch.velocity.received_at, 2.0)
        self.assertEqual(
            batch.velocity.command, VelocityCommand(vx=0.2, vy=0.0, vyaw=0.0)
        )

    def test_posture_queue_is_bounded_and_separate_from_velocity(self) -> None:
        mailbox = CommandMailbox(posture_capacity=2)
        mailbox.submit(b'{"type":"posture","posture":"stand"}', received_at=1.0)
        mailbox.submit(b'{"type":"posture","posture":"down"}', received_at=2.0)
        mailbox.submit(b'{"type":"posture","posture":"stand"}', received_at=3.0)
        mailbox.submit(
            b'{"type":"velocity","vx":0.2,"vy":0,"vyaw":0}',
            received_at=3.1,
        )

        batch = mailbox.take()

        self.assertEqual(
            [received.command for received in batch.postures],
            [PostureCommand(posture="down"), PostureCommand(posture="stand")],
        )
        self.assertIsNotNone(batch.velocity)

    def test_invalid_payload_is_bounded_to_latest_error(self) -> None:
        mailbox = CommandMailbox()
        mailbox.submit(b"not-json", received_at=1.0)
        mailbox.submit(b'{"type":"velocity","vx":99,"vy":0,"vyaw":0}', received_at=2.0)

        batch = mailbox.take()

        self.assertIn("invalid command", batch.invalid_error or "")
        self.assertIsNone(mailbox.take().invalid_error)

    def test_dispatch_drops_velocity_that_was_already_stale(self) -> None:
        controller = Mock(spec=RobotController)
        mailbox = CommandMailbox()
        mailbox.submit(
            b'{"type":"velocity","vx":0.2,"vy":0,"vyaw":0}',
            received_at=1.0,
        )

        dispatch_commands(
            controller,
            mailbox.take(),
            command_timeout_seconds=0.25,
            clock=lambda: 1.3,
        )

        controller.handle_command.assert_not_called()
        controller.report_command_error.assert_called_once()
        self.assertIn(
            "stale velocity",
            controller.report_command_error.call_args.args[0],
        )

    def test_dispatch_recaptures_time_after_forwarding_velocity(self) -> None:
        controller = Mock(spec=RobotController)
        mailbox = CommandMailbox()
        mailbox.submit(
            b'{"type":"velocity","vx":0.2,"vy":0,"vyaw":0}',
            received_at=1.0,
        )
        times = iter((1.1, 1.4))

        dispatch_commands(
            controller,
            mailbox.take(),
            command_timeout_seconds=0.25,
            clock=lambda: next(times),
        )

        controller.handle_command.assert_called_once()
        controller.tick.assert_called_once_with(now=1.4)


if __name__ == "__main__":
    unittest.main()

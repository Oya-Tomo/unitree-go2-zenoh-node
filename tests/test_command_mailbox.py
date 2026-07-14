from __future__ import annotations

import unittest
from threading import Event, Thread
from unittest.mock import Mock, call, patch

from command_mailbox import CommandMailbox, InvalidCommand, ReceivedCommand
from controller import RobotController
from models import (
    PostureCommand,
    VelocityCommand,
)
from models import (
    decode_command as real_decode_command,
)
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

        velocity = mailbox.drain().velocity

        self.assertIsNotNone(velocity)
        assert velocity is not None
        self.assertEqual(velocity.received_at, 2.0)
        self.assertEqual(velocity.command, VelocityCommand(vx=0.2, vy=0.0, vyaw=0.0))

    def test_slow_older_decode_cannot_overwrite_newer_velocity(self) -> None:
        mailbox = CommandMailbox()
        older_started = Event()
        release_older = Event()
        older_payload = b'{"type":"velocity","vx":0.1,"vy":0,"vyaw":0}'
        newer_payload = b'{"type":"velocity","vx":0.2,"vy":0,"vyaw":0}'

        def delayed_decode(payload: bytes | str) -> object:
            if payload == older_payload:
                older_started.set()
                release_older.wait(timeout=1.0)
            return real_decode_command(payload)

        with patch("command_mailbox.decode_command", side_effect=delayed_decode):
            older_thread = Thread(
                target=mailbox.submit,
                args=(older_payload,),
                kwargs={"received_at": 1.0},
            )
            older_thread.start()
            self.assertTrue(older_started.wait(timeout=1.0))
            mailbox.submit(newer_payload, received_at=2.0)
            first_velocity = mailbox.drain().velocity
            release_older.set()
            older_thread.join(timeout=1.0)

        self.assertFalse(older_thread.is_alive())
        self.assertIsNotNone(first_velocity)
        assert first_velocity is not None
        self.assertEqual(
            first_velocity.command,
            VelocityCommand(vx=0.2, vy=0.0, vyaw=0.0),
        )
        self.assertIsNone(mailbox.drain().velocity)

    def test_slow_unique_command_survives_an_empty_drain(self) -> None:
        mailbox = CommandMailbox()
        decode_started = Event()
        release_decode = Event()
        payload = b'{"type":"velocity","vx":0.1,"vy":0,"vyaw":0}'

        def delayed_decode(raw_payload: bytes | str) -> object:
            decode_started.set()
            release_decode.wait(timeout=1.0)
            return real_decode_command(raw_payload)

        with patch("command_mailbox.decode_command", side_effect=delayed_decode):
            submit_thread = Thread(
                target=mailbox.submit,
                args=(payload,),
                kwargs={"received_at": 1.0},
            )
            submit_thread.start()
            self.assertTrue(decode_started.wait(timeout=1.0))
            self.assertIsNone(mailbox.drain().velocity)
            release_decode.set()
            submit_thread.join(timeout=1.0)

        self.assertFalse(submit_thread.is_alive())
        velocity = mailbox.drain().velocity
        self.assertIsNotNone(velocity)
        assert velocity is not None
        self.assertEqual(
            velocity.command,
            VelocityCommand(vx=0.1, vy=0.0, vyaw=0.0),
        )

    def test_slow_older_invalid_payload_cannot_overwrite_newer_error(self) -> None:
        mailbox = CommandMailbox()
        older_started = Event()
        release_older = Event()

        def delayed_error(payload: bytes | str) -> object:
            if payload == b"older-invalid":
                older_started.set()
                release_older.wait(timeout=1.0)
                raise ValueError("older error")
            raise ValueError("newer error")

        with patch("command_mailbox.decode_command", side_effect=delayed_error):
            older_thread = Thread(
                target=mailbox.submit,
                args=(b"older-invalid",),
                kwargs={"received_at": 1.0},
            )
            older_thread.start()
            self.assertTrue(older_started.wait(timeout=1.0))
            mailbox.submit(b"newer-invalid", received_at=2.0)
            first_invalid = mailbox.drain().invalid
            release_older.set()
            older_thread.join(timeout=1.0)

        self.assertFalse(older_thread.is_alive())
        self.assertIsNotNone(first_invalid)
        assert first_invalid is not None
        self.assertIn("newer error", first_invalid.error)
        self.assertIsNone(mailbox.drain().invalid)

    def test_posture_slot_keeps_latest_stand(self) -> None:
        mailbox = CommandMailbox()
        mailbox.submit(b'{"type":"posture","posture":"stand"}', received_at=1.0)
        mailbox.submit(b'{"type":"posture","posture":"stand"}', received_at=2.0)

        posture = mailbox.drain().posture

        self.assertIsNotNone(posture)
        assert posture is not None
        self.assertEqual(posture.received_at, 2.0)

    def test_pending_down_cannot_be_superseded_by_stand_burst(self) -> None:
        mailbox = CommandMailbox()
        mailbox.submit(b'{"type":"posture","posture":"down"}', received_at=1.0)
        for received_at in range(2, 102):
            mailbox.submit(
                b'{"type":"posture","posture":"stand"}',
                received_at=float(received_at),
            )

        posture = mailbox.drain().posture

        self.assertIsNotNone(posture)
        assert posture is not None
        self.assertEqual(posture.command, PostureCommand(posture="down"))

    def test_duplicate_down_preserves_first_down_chronology(self) -> None:
        mailbox = CommandMailbox()
        mailbox.submit(b'{"type":"posture","posture":"down"}', received_at=1.0)
        mailbox.submit(
            b'{"type":"velocity","vx":0.2,"vy":0,"vyaw":0}',
            received_at=2.0,
        )
        mailbox.submit(b'{"type":"posture","posture":"down"}', received_at=3.0)

        events = mailbox.drain().in_receive_order()

        self.assertIsInstance(events[0], ReceivedCommand)
        assert isinstance(events[0], ReceivedCommand)
        self.assertEqual(events[0].command, PostureCommand(posture="down"))
        self.assertEqual(events[0].received_at, 1.0)

    def test_down_decoded_after_its_drain_cycle_cannot_reappear(self) -> None:
        mailbox = CommandMailbox()
        older_started = Event()
        release_older = Event()
        down_payload = b'{"type":"posture","posture":"down"}'

        def delayed_first_decode(payload: bytes | str) -> object:
            if not older_started.is_set():
                older_started.set()
                release_older.wait(timeout=1.0)
            return real_decode_command(payload)

        with patch("command_mailbox.decode_command", side_effect=delayed_first_decode):
            older_thread = Thread(
                target=mailbox.submit,
                args=(down_payload,),
                kwargs={"received_at": 1.0},
            )
            older_thread.start()
            self.assertTrue(older_started.wait(timeout=1.0))
            mailbox.submit(down_payload, received_at=2.0)
            first_posture = mailbox.drain().posture
            release_older.set()
            older_thread.join(timeout=1.0)

        self.assertFalse(older_thread.is_alive())
        self.assertIsNotNone(first_posture)
        assert first_posture is not None
        self.assertEqual(first_posture.received_at, 2.0)
        self.assertIsNone(mailbox.drain().posture)

    def test_slow_stand_cannot_reappear_after_down_suppressed_newer_stand(self) -> None:
        mailbox = CommandMailbox()
        slow_started = Event()
        release_slow = Event()
        down_payload = b'{"type":"posture","posture":"down"}'
        slow_stand = b'{"type":"posture", "posture":"stand"}'
        newer_stand = b'{"type":"posture","posture":"stand"}'
        mailbox.submit(down_payload, received_at=1.0)

        def delayed_stand_decode(payload: bytes | str) -> object:
            if payload == slow_stand:
                slow_started.set()
                release_slow.wait(timeout=1.0)
            return real_decode_command(payload)

        with patch("command_mailbox.decode_command", side_effect=delayed_stand_decode):
            slow_thread = Thread(
                target=mailbox.submit,
                args=(slow_stand,),
                kwargs={"received_at": 2.0},
            )
            slow_thread.start()
            self.assertTrue(slow_started.wait(timeout=1.0))
            mailbox.submit(newer_stand, received_at=3.0)
            drained_posture = mailbox.drain().posture
            release_slow.set()
            slow_thread.join(timeout=1.0)

        self.assertFalse(slow_thread.is_alive())
        self.assertIsNotNone(drained_posture)
        assert drained_posture is not None
        self.assertEqual(drained_posture.command, PostureCommand(posture="down"))
        self.assertIsNone(mailbox.drain().posture)

    def test_new_stand_is_accepted_after_down_was_drained(self) -> None:
        mailbox = CommandMailbox()
        mailbox.submit(b'{"type":"posture","posture":"down"}', received_at=1.0)
        mailbox.drain()
        mailbox.submit(b'{"type":"posture","posture":"stand"}', received_at=2.0)

        posture = mailbox.drain().posture

        self.assertIsNotNone(posture)
        assert posture is not None
        self.assertEqual(posture.command, PostureCommand(posture="stand"))

    def test_invalid_payload_is_bounded_to_latest_error(self) -> None:
        mailbox = CommandMailbox()
        mailbox.submit(b"not-json", received_at=1.0)
        mailbox.submit(b'{"type":"velocity","vx":99,"vy":0,"vyaw":0}', received_at=2.0)

        invalid = mailbox.drain().invalid

        self.assertIsNotNone(invalid)
        assert invalid is not None
        self.assertIn("invalid command", invalid.error)
        self.assertIsNone(mailbox.drain().invalid)

    def test_pending_events_preserve_receive_order(self) -> None:
        mailbox = CommandMailbox()
        mailbox.submit(b"not-json", received_at=1.0)
        mailbox.submit(
            b'{"type":"velocity","vx":0.2,"vy":0,"vyaw":0}',
            received_at=2.0,
        )

        events = mailbox.drain().in_receive_order()

        self.assertIsInstance(events[0], InvalidCommand)
        self.assertIsInstance(events[1], ReceivedCommand)

    def test_dispatch_applies_validation_and_commands_chronologically(self) -> None:
        controller = Mock(spec=RobotController)
        mailbox = CommandMailbox()
        mailbox.submit(b"not-json", received_at=1.0)
        mailbox.submit(
            b'{"type":"velocity","vx":0.2,"vy":0,"vyaw":0}',
            received_at=2.0,
        )
        times = iter((2.1, 2.2))

        dispatch_commands(controller, mailbox.drain(), clock=lambda: next(times))

        self.assertEqual(
            controller.method_calls,
            [
                call.report_command_error(controller.method_calls[0].args[0]),
                call.handle_command(
                    VelocityCommand(vx=0.2, vy=0.0, vyaw=0.0),
                    received_at=2.0,
                    now=2.1,
                ),
                call.tick(now=2.2),
            ],
        )
        self.assertIn("invalid command", controller.method_calls[0].args[0])


if __name__ == "__main__":
    unittest.main()

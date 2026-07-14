from __future__ import annotations

import unittest
from dataclasses import dataclass, field

from controller import RobotController
from models import (
    HealthState,
    NodeStatus,
    Posture,
    PostureCommand,
    PostureState,
    VelocityCommand,
    VelocityState,
)


@dataclass
class FakeSportClient:
    calls: list[tuple[object, ...]] = field(default_factory=list)
    return_codes: dict[str, int] = field(default_factory=dict)

    def _call(self, name: str, *arguments: float) -> int:
        self.calls.append((name, *arguments))
        return self.return_codes.get(name, 0)

    def Move(self, vx: float, vy: float, vyaw: float) -> int:
        return self._call("Move", vx, vy, vyaw)

    def StopMove(self) -> int:
        return self._call("StopMove")

    def StandUp(self) -> int:
        return self._call("StandUp")

    def StandDown(self) -> int:
        return self._call("StandDown")

    def BalanceStand(self) -> int:
        return self._call("BalanceStand")


@dataclass
class FakeStateSink:
    requested: list[VelocityState] = field(default_factory=list)
    applied: list[VelocityState] = field(default_factory=list)
    postures: list[PostureState] = field(default_factory=list)
    health: list[HealthState] = field(default_factory=list)

    def publish_requested(self, state: VelocityState) -> None:
        self.requested.append(state)

    def publish_applied(self, state: VelocityState) -> None:
        self.applied.append(state)

    def publish_posture(self, state: PostureState) -> None:
        self.postures.append(state)

    def publish_health(self, state: HealthState) -> None:
        self.health.append(state)


class FailingStateSink(FakeStateSink):
    @staticmethod
    def _fail() -> None:
        raise RuntimeError("Zenoh unavailable")

    def publish_requested(self, state: VelocityState) -> None:
        self._fail()

    def publish_applied(self, state: VelocityState) -> None:
        self._fail()

    def publish_posture(self, state: PostureState) -> None:
        self._fail()

    def publish_health(self, state: HealthState) -> None:
        self._fail()


class RobotControllerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.client = FakeSportClient()
        self.sink = FakeStateSink()
        self.sleeps: list[float] = []
        self.controller = RobotController(
            self.client,
            self.sink,
            command_timeout_seconds=0.25,
            posture_transition_seconds=3.0,
            shutdown_stop_delay_seconds=1.0,
            clock=lambda: 0.0,
            sleep=self.sleeps.append,
        )

    def stand(self) -> None:
        self.assertTrue(self.controller.startup())
        self.assertTrue(
            self.controller.handle_command(PostureCommand(posture="stand"), now=1.0)
        )
        self.controller.tick(now=3.999)
        self.assertEqual(self.controller.posture.posture, Posture.STANDING_UP)
        self.controller.tick(now=4.0)
        self.assertEqual(self.controller.posture.posture, Posture.STANDING)

    def test_startup_stops_motion_and_requires_explicit_stand(self) -> None:
        self.assertTrue(self.controller.startup())

        self.assertEqual(self.client.calls, [("StopMove",)])
        self.assertEqual(self.controller.posture.posture, Posture.UNKNOWN)
        self.assertFalse(self.controller.health.walking_enabled)
        self.assertEqual(self.controller.health.status, NodeStatus.READY)

        accepted = self.controller.handle_command(
            VelocityCommand(vx=0.5, vy=0.0, vyaw=0.0), now=0.1
        )
        self.assertFalse(accepted)
        self.assertNotIn(("Move", 0.5, 0.0, 0.0), self.client.calls)
        self.assertEqual(self.controller.requested.vx, 0.5)

    def test_stand_waits_then_enters_balance_mode_before_walking(self) -> None:
        self.stand()

        self.assertEqual(
            self.client.calls,
            [("StopMove",), ("StopMove",), ("StandUp",), ("BalanceStand",)],
        )
        self.assertTrue(self.controller.health.walking_enabled)

        accepted = self.controller.handle_command(
            VelocityCommand(vx=0.5, vy=-0.2, vyaw=0.3), now=4.1
        )
        self.assertTrue(accepted)
        self.assertEqual(self.client.calls[-1], ("Move", 0.5, -0.2, 0.3))
        self.assertEqual(self.controller.applied.vx, 0.5)

    def test_repeated_stand_during_transition_is_idempotent(self) -> None:
        self.assertTrue(self.controller.startup())
        stand = PostureCommand(posture="stand")
        self.assertTrue(self.controller.handle_command(stand, now=1.0))
        calls_after_first_request = self.client.calls.copy()

        self.assertTrue(self.controller.handle_command(stand, now=2.0))

        self.assertEqual(self.client.calls, calls_after_first_request)
        self.controller.tick(now=4.0)
        self.assertEqual(self.controller.posture.posture, Posture.STANDING)

    def test_stand_delay_begins_after_stand_up_returns(self) -> None:
        controller = RobotController(
            self.client,
            self.sink,
            command_timeout_seconds=0.25,
            posture_transition_seconds=3.0,
            shutdown_stop_delay_seconds=1.0,
            clock=lambda: 10.0,
            sleep=self.sleeps.append,
        )
        self.assertTrue(controller.startup())

        self.assertTrue(
            controller.handle_command(PostureCommand(posture="stand"), now=1.0)
        )
        controller.tick(now=12.999)
        self.assertEqual(controller.posture.posture, Posture.STANDING_UP)
        controller.tick(now=13.0)
        self.assertEqual(controller.posture.posture, Posture.STANDING)

    def test_watchdog_stops_once_and_fresh_velocity_resumes(self) -> None:
        self.stand()
        command = VelocityCommand(vx=0.4, vy=0.0, vyaw=0.0)
        self.assertTrue(self.controller.handle_command(command, now=4.0))

        self.controller.tick(now=4.249)
        self.assertEqual(self.client.calls[-1], ("Move", 0.4, 0.0, 0.0))
        self.controller.tick(now=4.25)
        self.assertEqual(self.client.calls[-1], ("StopMove",))
        self.assertTrue(self.controller.health.watchdog_triggered)
        self.assertTrue(self.controller.health.walking_enabled)
        watchdog_call_count = self.client.calls.count(("StopMove",))

        self.controller.tick(now=5.0)
        self.assertEqual(self.client.calls.count(("StopMove",)), watchdog_call_count)

        self.assertTrue(self.controller.handle_command(command, now=5.01))
        self.assertEqual(self.client.calls[-1], ("Move", 0.4, 0.0, 0.0))
        self.assertFalse(self.controller.health.watchdog_triggered)

    def test_move_failure_immediately_stops_previous_motion(self) -> None:
        self.stand()
        self.assertTrue(
            self.controller.handle_command(
                VelocityCommand(vx=0.4, vy=0.0, vyaw=0.0), now=4.0
            )
        )
        self.client.return_codes["Move"] = 8

        self.assertFalse(
            self.controller.handle_command(
                VelocityCommand(vx=0.2, vy=0.0, vyaw=0.0), now=4.1
            )
        )

        self.assertEqual(
            self.client.calls[-2:],
            [("Move", 0.2, 0.0, 0.0), ("StopMove",)],
        )
        self.assertTrue(self.controller.health.watchdog_triggered)
        self.assertFalse(self.controller.applied.active)

    def test_down_stops_before_stand_down_and_ignores_velocity(self) -> None:
        self.stand()
        self.assertTrue(
            self.controller.handle_command(
                VelocityCommand(vx=0.2, vy=0.0, vyaw=0.0), now=4.1
            )
        )
        self.assertTrue(
            self.controller.handle_command(PostureCommand(posture="down"), now=4.2)
        )

        self.assertEqual(self.client.calls[-2:], [("StopMove",), ("StandDown",)])
        self.assertEqual(self.controller.posture.posture, Posture.DOWN)
        self.assertFalse(self.controller.health.walking_enabled)

        move_count = sum(call[0] == "Move" for call in self.client.calls)
        self.assertFalse(
            self.controller.handle_command(
                VelocityCommand(vx=0.3, vy=0.0, vyaw=0.0), now=4.3
            )
        )
        self.assertEqual(
            sum(call[0] == "Move" for call in self.client.calls), move_count
        )

    def test_down_aborts_when_stop_move_fails(self) -> None:
        self.stand()
        self.client.return_codes["StopMove"] = 6

        self.assertFalse(
            self.controller.handle_command(PostureCommand(posture="down"), now=4.1)
        )

        self.assertEqual(self.client.calls[-1], ("StopMove",))
        self.assertNotEqual(self.client.calls[-1], ("StandDown",))
        self.assertEqual(self.controller.posture.posture, Posture.UNKNOWN)
        self.assertEqual(self.controller.health.status, NodeStatus.DEGRADED)

    def test_repeated_down_after_success_is_idempotent(self) -> None:
        self.stand()
        down = PostureCommand(posture="down")
        self.assertTrue(self.controller.handle_command(down, now=4.1))
        calls_after_first_request = self.client.calls.copy()

        self.assertTrue(self.controller.handle_command(down, now=4.2))

        self.assertEqual(self.client.calls, calls_after_first_request)

    def test_invalid_payload_does_not_refresh_or_reach_sdk(self) -> None:
        self.stand()
        self.assertTrue(
            self.controller.handle_command(
                VelocityCommand(vx=0.2, vy=0.0, vyaw=0.0), now=4.0
            )
        )

        self.assertFalse(
            self.controller.handle_payload(
                b'{"type":"velocity","vx":99,"vy":0,"vyaw":0}', now=4.2
            )
        )
        self.assertEqual(self.controller.health.status, NodeStatus.DEGRADED)
        self.assertIn("invalid command", self.controller.health.last_error or "")
        self.controller.tick(now=4.25)
        self.assertEqual(self.client.calls[-1], ("StopMove",))

    def test_failed_balance_keeps_velocity_disabled(self) -> None:
        self.client.return_codes["BalanceStand"] = 7
        self.assertTrue(self.controller.startup())
        self.assertTrue(
            self.controller.handle_command(PostureCommand(posture="stand"), now=1.0)
        )

        self.controller.tick(now=4.0)

        self.assertEqual(self.controller.posture.posture, Posture.UNKNOWN)
        self.assertFalse(self.controller.health.walking_enabled)
        self.assertEqual(self.controller.health.status, NodeStatus.DEGRADED)
        self.assertIn("BalanceStand failed", self.controller.health.last_error or "")

    def test_failed_watchdog_stop_requires_a_new_stand_sequence(self) -> None:
        self.stand()
        command = VelocityCommand(vx=0.2, vy=0.0, vyaw=0.0)
        self.assertTrue(self.controller.handle_command(command, now=4.0))
        self.client.return_codes["StopMove"] = 9

        self.controller.tick(now=4.25)

        self.assertFalse(self.controller.health.walking_enabled)
        self.assertEqual(self.controller.health.status, NodeStatus.DEGRADED)
        self.assertFalse(self.controller.handle_command(command, now=4.3))

    def test_graceful_shutdown_stops_waits_then_stands_down(self) -> None:
        self.stand()

        self.controller.graceful_shutdown()

        self.assertEqual(self.sleeps, [1.0])
        self.assertEqual(self.client.calls[-2:], [("StopMove",), ("StandDown",)])
        self.assertEqual(self.controller.posture.posture, Posture.DOWN)
        self.assertEqual(self.controller.health.status, NodeStatus.STOPPING)
        self.assertFalse(self.controller.health.walking_enabled)

    def test_state_publication_failure_cannot_abort_shutdown(self) -> None:
        controller = RobotController(
            self.client,
            FailingStateSink(),
            command_timeout_seconds=0.25,
            posture_transition_seconds=3.0,
            shutdown_stop_delay_seconds=1.0,
            sleep=self.sleeps.append,
        )
        with self.assertLogs("controller", level="ERROR"):
            self.assertTrue(controller.startup())
            controller.graceful_shutdown()

        self.assertEqual(self.client.calls[-2:], [("StopMove",), ("StandDown",)])
        self.assertEqual(self.sleeps, [1.0])


if __name__ == "__main__":
    unittest.main()

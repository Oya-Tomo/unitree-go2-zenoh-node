from __future__ import annotations

import unittest
from threading import Event, Thread

from controller import (
    CommandBuffer,
    Controller,
    Go2SportMode,
    ModeClass,
    Motion,
    NodeState,
    PostureCommand,
    PosturePhase,
    PostureTarget,
    RobotObservation,
    StateValidity,
    VelocityCommand,
    classify_state,
)


class Clock:
    def __init__(self, value: float = 1.0) -> None:
        self.value = value

    def __call__(self) -> float:
        return self.value

    def advance(self, seconds: float = 0.01) -> None:
        self.value += seconds


class FakeClient:
    def __init__(self) -> None:
        self.calls: list[tuple[object, ...]] = []
        self.stop_code = 0

    def StandUp(self) -> int:
        self.calls.append(("StandUp",))
        return 0

    def BalanceStand(self) -> int:
        self.calls.append(("BalanceStand",))
        return 0

    def StopMove(self) -> int:
        self.calls.append(("StopMove",))
        return self.stop_code

    def StandDown(self) -> int:
        self.calls.append(("StandDown",))
        return 0

    def Move(self, vx: float, vy: float, vyaw: float) -> int:
        self.calls.append(("Move", vx, vy, vyaw))
        return 0


class BlockingStandClient(FakeClient):
    def __init__(self) -> None:
        super().__init__()
        self.entered = Event()
        self.release = Event()

    def StandUp(self) -> int:
        self.calls.append(("StandUp",))
        self.entered.set()
        if not self.release.wait(timeout=1.0):
            raise TimeoutError("test did not release StandUp")
        return 0


class StateRecorder:
    def __init__(self) -> None:
        self.states: list[NodeState] = []

    def publish(self, state: NodeState) -> None:
        self.states.append(state)


class Harness:
    def __init__(self) -> None:
        self.clock = Clock()
        self.client = FakeClient()
        self.commands = CommandBuffer()
        self.states = StateRecorder()
        self.controller = Controller(
            self.client,
            self.commands,
            self.states,
            maximum_state_age_seconds=0.5,
            linear_velocity_quiescent_threshold=0.03,
            yaw_speed_quiescent_threshold=0.05,
            velocity_deadman_seconds=0.25,
            clock=self.clock,
        )

    def state(
        self,
        mode: int,
        *,
        vx: float = 0.0,
        error_code: int = 0,
    ) -> None:
        self.controller.on_state(
            RobotObservation(
                received_at=self.clock(),
                stamp_sec=10,
                stamp_nanosec=20,
                error_code=error_code,
                mode=mode,
                velocity=(vx, 0.0, 0.0),
                yaw_speed=0.0,
            )
        )

    def next_state(self, mode: int, *, vx: float = 0.0) -> None:
        self.clock.advance()
        self.state(mode, vx=vx)

    def posture(self, posture: str) -> bool:
        return self.commands.update_posture(
            PostureCommand(posture=PostureTarget(posture)),
            received_at=self.clock(),
        )

    def velocity(self, vx: float, vy: float = 0.0, vyaw: float = 0.0) -> bool:
        return self.commands.update_velocity(
            VelocityCommand(vx=vx, vy=vy, vyaw=vyaw),
            received_at=self.clock(),
        )


class ControllerTests(unittest.TestCase):
    def test_one_fresh_sample_is_immediately_classified(self) -> None:
        observation = RobotObservation(
            received_at=1.0,
            stamp_sec=2,
            stamp_nanosec=3,
            error_code=0,
            mode=Go2SportMode.LIE_DOWN,
            velocity=(0.03, 0.0, 0.0),
            yaw_speed=0.05,
        )
        state = classify_state(
            observation,
            now=1.0,
            maximum_age_seconds=0.5,
            linear_velocity_quiescent_threshold=0.03,
            yaw_speed_quiescent_threshold=0.05,
        )
        self.assertEqual(state.validity, StateValidity.CONFIRMED)
        self.assertEqual(state.mode_class, ModeClass.DOWN)
        self.assertEqual(state.motion, Motion.QUIESCENT)

    def test_stand_workflow_waits_for_state_after_each_sdk_call(self) -> None:
        harness = Harness()
        harness.state(Go2SportMode.LIE_DOWN)
        self.assertTrue(harness.posture("stand"))

        harness.next_state(Go2SportMode.LIE_DOWN)
        self.assertEqual(harness.client.calls, [("StandUp",)])
        self.assertEqual(harness.states.states[-1].robot.validity, "unknown")
        self.assertFalse(harness.states.states[-1].accepting_commands)
        self.assertFalse(harness.velocity(1.0))

        harness.next_state(Go2SportMode.IDLE)
        self.assertEqual(
            harness.client.calls,
            [("StandUp",), ("BalanceStand",)],
        )
        self.assertEqual(
            harness.states.states[-1].posture_phase,
            PosturePhase.BALANCE_STAND_SENT,
        )

        harness.next_state(Go2SportMode.BALANCE_STAND)
        self.assertIsNone(harness.states.states[-1].requested_posture)
        self.assertTrue(harness.states.states[-1].accepting_commands)

    def test_unknown_is_published_before_a_blocking_posture_rpc(self) -> None:
        clock = Clock()
        client = BlockingStandClient()
        commands = CommandBuffer()
        states = StateRecorder()
        controller = Controller(
            client,
            commands,
            states,
            maximum_state_age_seconds=0.5,
            linear_velocity_quiescent_threshold=0.03,
            yaw_speed_quiescent_threshold=0.05,
            velocity_deadman_seconds=0.25,
            clock=clock,
        )
        controller.on_state(
            RobotObservation(
                received_at=clock(),
                stamp_sec=10,
                stamp_nanosec=20,
                error_code=0,
                mode=Go2SportMode.LIE_DOWN,
                velocity=(0.0, 0.0, 0.0),
                yaw_speed=0.0,
            )
        )
        self.assertTrue(
            commands.update_posture(
                PostureCommand(posture=PostureTarget.STAND),
                received_at=clock(),
            )
        )
        clock.advance()
        observation = RobotObservation(
            received_at=clock(),
            stamp_sec=10,
            stamp_nanosec=21,
            error_code=0,
            mode=Go2SportMode.LIE_DOWN,
            velocity=(0.0, 0.0, 0.0),
            yaw_speed=0.0,
        )
        worker = Thread(target=controller.on_state, args=(observation,))
        worker.start()
        self.assertTrue(client.entered.wait(timeout=1.0))
        self.assertEqual(states.states[-1].robot.validity, StateValidity.UNKNOWN)
        self.assertFalse(states.states[-1].accepting_commands)
        client.release.set()
        worker.join(timeout=1.0)
        self.assertFalse(worker.is_alive())

    def test_down_stops_once_then_waits_for_quiescent_state(self) -> None:
        harness = Harness()
        harness.client.stop_code = -1
        harness.state(Go2SportMode.BALANCE_STAND)
        self.assertTrue(harness.posture("down"))

        harness.next_state(Go2SportMode.LOCOMOTION, vx=0.2)
        self.assertEqual(harness.client.calls, [("StopMove",)])
        diagnostic = harness.states.states[-1].last_sdk
        self.assertIsNotNone(diagnostic)
        assert diagnostic is not None
        self.assertEqual(diagnostic.code, -1)

        harness.next_state(Go2SportMode.LOCOMOTION, vx=0.1)
        self.assertEqual(harness.client.calls, [("StopMove",)])

        harness.next_state(Go2SportMode.BALANCE_STAND)
        self.assertEqual(
            harness.client.calls,
            [("StopMove",), ("StandDown",)],
        )

        harness.next_state(Go2SportMode.LIE_DOWN)
        self.assertIsNone(harness.states.states[-1].requested_posture)

    def test_latest_posture_replaces_older_posture(self) -> None:
        harness = Harness()
        harness.state(Go2SportMode.LIE_DOWN)
        self.assertTrue(harness.posture("stand"))
        self.assertTrue(harness.posture("down"))

        harness.next_state(Go2SportMode.LIE_DOWN)
        self.assertEqual(harness.client.calls, [])
        self.assertIsNone(harness.states.states[-1].requested_posture)

    def test_posture_has_priority_and_clears_buffered_velocity(self) -> None:
        harness = Harness()
        harness.state(Go2SportMode.BALANCE_STAND)
        self.assertTrue(harness.velocity(1.0))
        self.assertTrue(harness.posture("down"))
        self.assertTrue(harness.velocity(2.0))

        harness.next_state(Go2SportMode.BALANCE_STAND)
        self.assertEqual(harness.client.calls, [("StopMove",)])
        self.assertIsNone(harness.states.states[-1].requested_velocity)

    def test_latest_velocity_is_sent_only_from_a_new_state(self) -> None:
        harness = Harness()
        harness.state(Go2SportMode.BALANCE_STAND)
        self.assertTrue(harness.velocity(0.5))
        self.assertTrue(harness.velocity(1.0, 0.2, -0.3))
        self.assertEqual(harness.client.calls, [])

        harness.next_state(Go2SportMode.BALANCE_STAND)
        self.assertEqual(harness.client.calls, [("Move", 1.0, 0.2, -0.3)])

    def test_zero_velocity_uses_move_not_stop_move(self) -> None:
        harness = Harness()
        harness.state(Go2SportMode.BALANCE_STAND)
        self.assertTrue(harness.velocity(0.0))

        harness.next_state(Go2SportMode.BALANCE_STAND)
        self.assertEqual(harness.client.calls, [("Move", 0.0, 0.0, 0.0)])

    def test_expired_velocity_sends_one_zero_and_is_cleared(self) -> None:
        harness = Harness()
        harness.state(Go2SportMode.BALANCE_STAND)
        self.assertTrue(harness.velocity(1.0))
        harness.clock.advance(0.3)

        harness.state(Go2SportMode.LOCOMOTION, vx=1.0)
        self.assertEqual(harness.client.calls, [("Move", 0.0, 0.0, 0.0)])
        self.assertIsNone(harness.states.states[-1].requested_velocity)

    def test_pre_command_state_cannot_clear_unknown(self) -> None:
        harness = Harness()
        harness.state(Go2SportMode.LIE_DOWN)
        self.assertTrue(harness.posture("stand"))
        harness.next_state(Go2SportMode.LIE_DOWN)
        command_time = harness.clock()

        harness.controller.on_state(
            RobotObservation(
                received_at=command_time,
                stamp_sec=10,
                stamp_nanosec=21,
                error_code=0,
                mode=Go2SportMode.IDLE,
                velocity=(0.0, 0.0, 0.0),
                yaw_speed=0.0,
            )
        )
        self.assertEqual(harness.states.states[-1].robot.validity, "unknown")
        self.assertEqual(harness.client.calls, [("StandUp",)])

    def test_stale_state_disables_and_clears_commands(self) -> None:
        harness = Harness()
        harness.state(Go2SportMode.BALANCE_STAND)
        self.assertTrue(harness.velocity(1.0))
        harness.clock.advance(0.5)

        self.assertTrue(harness.controller.expire_state())
        self.assertEqual(harness.states.states[-1].robot.reason, "stale")
        self.assertFalse(harness.velocity(1.0))

    def test_shutdown_uses_the_same_stop_then_down_workflow(self) -> None:
        harness = Harness()
        harness.state(Go2SportMode.BALANCE_STAND)
        harness.controller.begin_shutdown()

        harness.next_state(Go2SportMode.BALANCE_STAND)
        harness.next_state(Go2SportMode.IDLE)
        harness.next_state(Go2SportMode.LIE_DOWN)

        self.assertEqual(
            harness.client.calls,
            [("StopMove",), ("StandDown",)],
        )
        self.assertTrue(harness.controller.shutdown_complete)

    def test_shutdown_while_down_sends_no_redundant_stop(self) -> None:
        harness = Harness()
        harness.state(Go2SportMode.LIE_DOWN)
        harness.controller.begin_shutdown()
        harness.next_state(Go2SportMode.LIE_DOWN)

        self.assertEqual(harness.client.calls, [])
        self.assertTrue(harness.controller.shutdown_complete)

    def test_moving_mode_five_is_not_actionable_for_stand_or_down(self) -> None:
        for posture in ("stand", "down"):
            with self.subTest(posture=posture):
                harness = Harness()
                harness.state(Go2SportMode.BALANCE_STAND)
                self.assertTrue(harness.posture(posture))

                harness.next_state(Go2SportMode.LIE_DOWN, vx=0.1)

                self.assertEqual(harness.client.calls, [])
                self.assertEqual(
                    harness.states.states[-1].robot.mode_class,
                    ModeClass.TRANSITION,
                )
                self.assertIsNotNone(harness.states.states[-1].requested_posture)

    def test_shutdown_waits_for_quiescent_mode_five(self) -> None:
        harness = Harness()
        harness.state(Go2SportMode.BALANCE_STAND)
        harness.controller.begin_shutdown()

        harness.next_state(Go2SportMode.LIE_DOWN, vx=0.1)
        self.assertFalse(harness.controller.shutdown_complete)
        self.assertEqual(harness.client.calls, [])

        harness.next_state(Go2SportMode.LIE_DOWN)
        self.assertTrue(harness.controller.shutdown_complete)
        self.assertEqual(harness.client.calls, [])

    def test_invalid_state_does_not_discard_shutdown_workflow(self) -> None:
        harness = Harness()
        harness.state(Go2SportMode.BALANCE_STAND)
        harness.controller.begin_shutdown()

        harness.controller.reject_state("malformed")
        harness.next_state(Go2SportMode.BALANCE_STAND)

        self.assertEqual(harness.client.calls, [("StopMove",)])


if __name__ == "__main__":
    unittest.main()

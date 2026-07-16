from __future__ import annotations

import unittest
from threading import Event, Thread

from controller import (
    CommandBuffer,
    Controller,
    Go2MotionStateMachine,
    Go2SportMode,
    Lifecycle,
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
        mode: Go2SportMode,
        *,
        vx: float = 0.0,
        state_machine_code: int | None = None,
    ) -> None:
        if state_machine_code is None:
            state_machine_code = {
                Go2SportMode.IDLE: Go2MotionStateMachine.STANDING_LOCK,
                Go2SportMode.BALANCE_STAND: Go2MotionStateMachine.BALANCE_STANDING,
                Go2SportMode.LOCOMOTION: Go2MotionStateMachine.AGILE,
                Go2SportMode.LIE_DOWN: Go2MotionStateMachine.CROUCH,
            }.get(mode, Go2MotionStateMachine.AGILE)
        self.controller.on_state(
            RobotObservation(
                received_at=self.clock(),
                stamp_sec=10,
                stamp_nanosec=20,
                state_machine_code=state_machine_code,
                mode=mode,
                velocity=(vx, 0.0, 0.0),
                yaw_speed=0.0,
            )
        )

    def next_state(
        self,
        mode: Go2SportMode,
        *,
        vx: float = 0.0,
        state_machine_code: int | None = None,
    ) -> None:
        self.clock.advance()
        self.state(mode, vx=vx, state_machine_code=state_machine_code)

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
    def test_state_machine_mode_and_motion_classification(self) -> None:
        fsm = Go2MotionStateMachine
        sport = Go2SportMode
        physical = ModeClass
        cases = (
            ("crouch", fsm.CROUCH, sport.LIE_DOWN, 0.0, physical.DOWN),
            (
                "alternate crouch",
                fsm.ALTERNATE_CROUCH,
                sport.LIE_DOWN,
                0.0,
                physical.DOWN,
            ),
            ("moving crouch", fsm.CROUCH, sport.LIE_DOWN, 0.1, physical.TRANSITION),
            ("standing lock", fsm.STANDING_LOCK, sport.IDLE, 0.0, physical.IDLE_STAND),
            (
                "moving standing lock",
                fsm.STANDING_LOCK,
                sport.IDLE,
                0.1,
                physical.TRANSITION,
            ),
            (
                "balance standing",
                fsm.BALANCE_STANDING,
                sport.BALANCE_STAND,
                0.0,
                physical.READY_STAND,
            ),
            (
                "moving balance standing",
                fsm.BALANCE_STANDING,
                sport.BALANCE_STAND,
                0.1,
                physical.TRANSITION,
            ),
            (
                "balance locomotion",
                fsm.BALANCE_STANDING,
                sport.LOCOMOTION,
                0.1,
                physical.LOCOMOTION,
            ),
            (
                "inconsistent balance mode",
                fsm.BALANCE_STANDING,
                sport.IDLE,
                0.0,
                physical.TRANSITION,
            ),
            ("agile idle", fsm.AGILE, sport.IDLE, 0.0, physical.IDLE_STAND),
            ("agile ready", fsm.AGILE, sport.BALANCE_STAND, 0.0, physical.READY_STAND),
            ("agile locomotion", fsm.AGILE, sport.LOCOMOTION, 0.1, physical.LOCOMOTION),
            (
                "inconsistent agile mode",
                fsm.AGILE,
                sport.LIE_DOWN,
                0.0,
                physical.TRANSITION,
            ),
            (
                "known unsupported",
                fsm.DAMPING,
                sport.IDLE,
                0.1,
                physical.UNSUPPORTED,
            ),
            ("unknown", 0, sport.IDLE, 0.0, physical.UNSUPPORTED),
        )

        for name, state_machine, mode, vx, expected in cases:
            with self.subTest(name=name):
                state = classify_state(
                    RobotObservation(
                        received_at=1.0,
                        stamp_sec=2,
                        stamp_nanosec=3,
                        state_machine_code=state_machine,
                        mode=mode,
                        velocity=(vx, 0.0, 0.0),
                        yaw_speed=0.0,
                    ),
                    now=1.0,
                    maximum_age_seconds=0.5,
                    linear_velocity_quiescent_threshold=0.03,
                    yaw_speed_quiescent_threshold=0.05,
                )
                self.assertEqual(state.validity, StateValidity.CONFIRMED)
                self.assertEqual(state.mode_class, expected)
                self.assertEqual(
                    state.motion,
                    Motion.MOVING if vx > 0.03 else Motion.QUIESCENT,
                )
                self.assertEqual(state.state_machine_code, state_machine)

    def test_agile_idle_startup_accepts_stand(self) -> None:
        harness = Harness()

        harness.state(
            Go2SportMode.IDLE,
            state_machine_code=Go2MotionStateMachine.AGILE,
        )

        state = harness.states.states[-1]
        self.assertEqual(state.lifecycle, Lifecycle.RUNNING)
        self.assertEqual(state.robot.mode_class, ModeClass.IDLE_STAND)
        self.assertEqual(state.robot.state_machine_code, 100)
        self.assertEqual(state.robot.state_machine_name, "agile")
        self.assertTrue(state.accepting_commands)

        self.assertTrue(harness.posture("stand"))
        harness.next_state(
            Go2SportMode.IDLE,
            state_machine_code=Go2MotionStateMachine.AGILE,
        )
        self.assertEqual(harness.client.calls, [("BalanceStand",)])
        self.assertEqual(
            harness.states.states[-1].robot.validity,
            StateValidity.UNKNOWN,
        )

        harness.next_state(
            Go2SportMode.BALANCE_STAND,
            state_machine_code=Go2MotionStateMachine.BALANCE_STANDING,
        )
        self.assertIsNone(harness.states.states[-1].requested_posture)
        self.assertTrue(harness.states.states[-1].accepting_commands)

    def test_unrecognized_state_machine_is_preserved_but_not_actionable(self) -> None:
        harness = Harness()

        harness.state(
            Go2SportMode.IDLE,
            state_machine_code=0,
        )

        state = harness.states.states[-1]
        self.assertEqual(state.lifecycle, Lifecycle.RUNNING)
        self.assertEqual(state.robot.mode_class, ModeClass.UNSUPPORTED)
        self.assertEqual(state.robot.state_machine_code, 0)
        self.assertEqual(state.robot.state_machine_name, "unsupported")
        self.assertFalse(state.accepting_commands)

    def test_stand_workflow_waits_for_state_after_each_sdk_call(self) -> None:
        harness = Harness()
        harness.state(Go2SportMode.LIE_DOWN)
        self.assertTrue(harness.posture("stand"))

        harness.next_state(Go2SportMode.LIE_DOWN)
        self.assertEqual(harness.client.calls, [("StandUp",)])
        self.assertEqual(harness.states.states[-1].robot.validity, "unknown")
        self.assertFalse(harness.states.states[-1].accepting_commands)
        self.assertFalse(harness.velocity(1.0))

        harness.next_state(Go2SportMode.IDLE, vx=0.1)
        self.assertEqual(harness.client.calls, [("StandUp",)])
        self.assertEqual(
            harness.states.states[-1].robot.mode_class,
            ModeClass.TRANSITION,
        )

        harness.next_state(Go2SportMode.IDLE)
        self.assertEqual(
            harness.client.calls,
            [("StandUp",), ("BalanceStand",)],
        )
        self.assertEqual(
            harness.states.states[-1].posture_phase,
            PosturePhase.BALANCE_STAND_SENT,
        )

        harness.next_state(Go2SportMode.BALANCE_STAND, vx=0.1)
        self.assertIsNotNone(harness.states.states[-1].requested_posture)
        self.assertEqual(
            harness.states.states[-1].robot.mode_class,
            ModeClass.TRANSITION,
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
                state_machine_code=Go2MotionStateMachine.CROUCH,
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
            state_machine_code=Go2MotionStateMachine.CROUCH,
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

    def test_state_received_before_rpc_returns_cannot_clear_unknown(self) -> None:
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
                state_machine_code=Go2MotionStateMachine.STANDING_LOCK,
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

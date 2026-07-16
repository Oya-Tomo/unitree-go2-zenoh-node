from __future__ import annotations

import unittest

from controller import (
    CommandBuffer,
    Controller,
    Go2MotionStateMachine,
    Go2SportMode,
    Lifecycle,
    Motion,
    NodeState,
    PostureCommand,
    PostureTarget,
    RobotObservation,
    RobotState,
    SdkCommand,
    UnknownReason,
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

    def RecoveryStand(self) -> int:
        self.calls.append(("RecoveryStand",))
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
            maximum_state_age_seconds=0.2,
            linear_velocity_quiescent_threshold=0.03,
            yaw_speed_quiescent_threshold=0.05,
            velocity_deadman_seconds=0.25,
            clock=self.clock,
        )

    @property
    def latest(self) -> NodeState:
        return self.states.states[-1]

    def state(
        self,
        mode: Go2SportMode,
        *,
        vx: float = 0.0,
        yaw_speed: float = 0.0,
        state_machine_code: int | None = None,
        received_at: float | None = None,
    ) -> None:
        if state_machine_code is None:
            state_machine_code = {
                Go2SportMode.IDLE: Go2MotionStateMachine.AGILE,
                Go2SportMode.BALANCE_STAND: Go2MotionStateMachine.BALANCE_STANDING,
                Go2SportMode.LOCOMOTION: Go2MotionStateMachine.AGILE,
                Go2SportMode.LIE_DOWN: Go2MotionStateMachine.CROUCH,
            }.get(mode, Go2MotionStateMachine.AGILE)
        self.controller.on_state(
            RobotObservation(
                received_at=self.clock() if received_at is None else received_at,
                stamp_sec=10,
                stamp_nanosec=20,
                state_machine_code=state_machine_code,
                mode=mode,
                velocity=(vx, 0.0, 0.0),
                yaw_speed=yaw_speed,
            )
        )

    def next_state(self, mode: Go2SportMode, **kwargs: object) -> None:
        self.clock.advance()
        self.state(mode, **kwargs)  # type: ignore[arg-type]

    def posture(self, posture: str) -> bool:
        return self.commands.update_posture(
            PostureCommand(posture=PostureTarget(posture))
        )

    def velocity(self, vx: float, vy: float = 0.0, vyaw: float = 0.0) -> bool:
        return self.commands.update_velocity(
            VelocityCommand(vx=vx, vy=vy, vyaw=vyaw),
            received_at=self.clock(),
        )


class ControllerTests(unittest.TestCase):
    def test_classifier_is_one_direct_observation_table(self) -> None:
        fsm = Go2MotionStateMachine
        mode = Go2SportMode
        cases = (
            (fsm.DAMPING, mode.IDLE, 0.1, RobotState.DAMPING),
            (fsm.CROUCH, mode.LIE_DOWN, 0.0, RobotState.DOWN),
            (fsm.ALTERNATE_CROUCH, mode.LIE_DOWN, 0.0, RobotState.DOWN),
            (fsm.CROUCH, mode.LIE_DOWN, 0.1, RobotState.UNSUPPORTED),
            (fsm.STANDING_LOCK, mode.IDLE, 0.1, RobotState.LOCKED_STAND),
            (fsm.AGILE, mode.IDLE, 0.1, RobotState.READY_STAND),
            (fsm.AGILE, mode.BALANCE_STAND, 0.1, RobotState.READY_STAND),
            (fsm.BALANCE_STANDING, mode.BALANCE_STAND, 0.1, RobotState.READY_STAND),
            (fsm.AGILE, mode.LOCOMOTION, 0.1, RobotState.LOCOMOTION),
            (fsm.BALANCE_STANDING, mode.LOCOMOTION, 0.1, RobotState.LOCOMOTION),
            (fsm.SIT, mode.IDLE, 0.0, RobotState.UNSUPPORTED),
            (0, mode.IDLE, 0.0, RobotState.UNSUPPORTED),
        )

        for machine, sport_mode, vx, expected in cases:
            with self.subTest(machine=machine, mode=sport_mode, vx=vx):
                state = classify_state(
                    RobotObservation(
                        received_at=1.0,
                        stamp_sec=2,
                        stamp_nanosec=3,
                        state_machine_code=machine,
                        mode=sport_mode,
                        velocity=(vx, 0.0, 0.0),
                        yaw_speed=0.0,
                    ),
                    linear_velocity_quiescent_threshold=0.03,
                    yaw_speed_quiescent_threshold=0.05,
                )
                self.assertEqual(state.state, expected)
                self.assertEqual(
                    state.motion,
                    Motion.MOVING if vx > 0.03 else Motion.QUIESCENT,
                )

    def test_command_gate_depends_on_fresh_dds_not_robot_state(self) -> None:
        harness = Harness()
        self.assertFalse(harness.velocity(0.5))

        harness.state(Go2SportMode.IDLE, state_machine_code=0)
        self.assertTrue(harness.latest.connected)
        self.assertEqual(harness.latest.robot.state, RobotState.UNSUPPORTED)
        self.assertTrue(harness.latest.accepting_commands)
        self.assertTrue(harness.velocity(0.5))

        harness.next_state(Go2SportMode.IDLE, state_machine_code=0)
        self.assertIsNone(harness.latest.requested_velocity)
        self.assertEqual(harness.client.calls, [])
        self.assertTrue(harness.latest.accepting_commands)

    def test_200_ms_state_gap_disconnects_and_clears_commands(self) -> None:
        harness = Harness()
        harness.state(Go2SportMode.IDLE)
        self.assertTrue(harness.velocity(0.5))

        harness.clock.advance(0.2)
        self.assertTrue(harness.controller.expire_state())
        self.assertFalse(harness.latest.connected)
        self.assertEqual(harness.latest.robot.state, RobotState.UNKNOWN)
        self.assertEqual(harness.latest.robot.reason, UnknownReason.STALE)
        self.assertFalse(harness.latest.accepting_commands)
        self.assertIsNone(harness.latest.requested_velocity)
        self.assertFalse(harness.velocity(0.5))

        harness.next_state(Go2SportMode.IDLE)
        self.assertTrue(harness.latest.connected)
        self.assertTrue(harness.latest.accepting_commands)

    def test_invalid_sample_does_not_refresh_freshness(self) -> None:
        harness = Harness()
        harness.controller.reject_state("bad startup sample")
        self.assertEqual(harness.latest.robot.reason, UnknownReason.INVALID_SAMPLE)
        self.assertFalse(harness.controller.has_state)

        harness.state(Go2SportMode.IDLE)
        harness.clock.advance(0.19)
        harness.controller.reject_state("bad later sample")
        self.assertTrue(harness.latest.connected)

        harness.clock.advance(0.01)
        self.assertTrue(harness.controller.expire_state())
        self.assertFalse(harness.latest.connected)

    def test_moving_agile_accepts_and_executes_latest_velocity(self) -> None:
        harness = Harness()
        harness.state(Go2SportMode.IDLE, vx=0.1)
        self.assertEqual(harness.latest.robot.state, RobotState.READY_STAND)
        self.assertTrue(harness.velocity(0.5))
        self.assertTrue(harness.velocity(1.0, 0.2, -0.3))
        self.assertEqual(harness.client.calls, [])

        harness.next_state(Go2SportMode.IDLE, vx=0.2)
        self.assertEqual(harness.client.calls, [("Move", 1.0, 0.2, -0.3)])
        self.assertTrue(harness.latest.accepting_commands)

    def test_velocity_illegal_in_current_state_is_not_delayed(self) -> None:
        harness = Harness()
        harness.state(
            Go2SportMode.IDLE,
            state_machine_code=Go2MotionStateMachine.STANDING_LOCK,
        )
        self.assertTrue(harness.velocity(0.5))
        harness.next_state(
            Go2SportMode.IDLE,
            state_machine_code=Go2MotionStateMachine.STANDING_LOCK,
        )
        self.assertIsNone(harness.latest.requested_velocity)

        harness.next_state(Go2SportMode.IDLE)
        self.assertEqual(harness.client.calls, [])

    def test_stand_action_is_selected_only_from_observed_state(self) -> None:
        cases = (
            (
                Go2SportMode.LIE_DOWN,
                Go2MotionStateMachine.CROUCH,
                "StandUp",
            ),
            (
                Go2SportMode.IDLE,
                Go2MotionStateMachine.DAMPING,
                "RecoveryStand",
            ),
            (
                Go2SportMode.IDLE,
                Go2MotionStateMachine.STANDING_LOCK,
                "BalanceStand",
            ),
            (
                Go2SportMode.LOCOMOTION,
                Go2MotionStateMachine.AGILE,
                "BalanceStand",
            ),
        )
        for sport_mode, machine, expected in cases:
            with self.subTest(machine=machine):
                harness = Harness()
                harness.state(sport_mode, state_machine_code=machine)
                self.assertTrue(harness.posture("stand"))
                harness.next_state(sport_mode, state_machine_code=machine)
                self.assertEqual(harness.client.calls, [(expected,)])
                self.assertEqual(harness.latest.robot.state, RobotState.UNKNOWN)
                self.assertFalse(harness.latest.accepting_commands)

        ready = Harness()
        ready.state(Go2SportMode.IDLE)
        self.assertTrue(ready.posture("stand"))
        ready.next_state(Go2SportMode.IDLE)
        self.assertEqual(ready.client.calls, [])
        self.assertIsNone(ready.latest.requested_posture)

    def test_stand_workflow_advances_from_each_new_observation(self) -> None:
        harness = Harness()
        harness.state(Go2SportMode.LIE_DOWN)
        self.assertTrue(harness.posture("stand"))
        harness.next_state(Go2SportMode.LIE_DOWN)
        self.assertEqual(harness.client.calls, [("StandUp",)])

        harness.next_state(
            Go2SportMode.IDLE,
            state_machine_code=Go2MotionStateMachine.STANDING_LOCK,
        )
        self.assertEqual(harness.client.calls, [("StandUp",), ("BalanceStand",)])

        harness.next_state(Go2SportMode.IDLE)
        self.assertIsNone(harness.latest.requested_posture)
        self.assertTrue(harness.latest.accepting_commands)

    def test_posture_rpc_requires_state_received_after_rpc_return(self) -> None:
        harness = Harness()
        harness.state(Go2SportMode.LIE_DOWN)
        self.assertTrue(harness.posture("stand"))
        harness.next_state(Go2SportMode.LIE_DOWN)
        rpc_returned_at = harness.clock()

        harness.state(
            Go2SportMode.IDLE,
            state_machine_code=Go2MotionStateMachine.STANDING_LOCK,
            received_at=rpc_returned_at,
        )
        self.assertEqual(harness.latest.robot.state, RobotState.UNKNOWN)
        self.assertEqual(harness.client.calls, [("StandUp",)])

        harness.next_state(
            Go2SportMode.IDLE,
            state_machine_code=Go2MotionStateMachine.STANDING_LOCK,
        )
        self.assertEqual(harness.client.calls, [("StandUp",), ("BalanceStand",)])

    def test_down_stops_once_then_waits_for_quiescent_state(self) -> None:
        harness = Harness()
        harness.client.stop_code = -1
        harness.state(Go2SportMode.IDLE)
        self.assertTrue(harness.posture("down"))

        harness.next_state(Go2SportMode.IDLE)
        self.assertEqual(harness.client.calls, [("StopMove",)])
        self.assertEqual(harness.latest.last_sdk.code, -1)  # type: ignore[union-attr]

        harness.next_state(Go2SportMode.IDLE, vx=0.2)
        self.assertEqual(harness.client.calls, [("StopMove",)])
        self.assertEqual(harness.latest.robot.state, RobotState.READY_STAND)

        harness.next_state(Go2SportMode.IDLE)
        self.assertEqual(
            harness.client.calls,
            [("StopMove",), ("StandDown",)],
        )

        harness.next_state(Go2SportMode.LIE_DOWN)
        self.assertIsNone(harness.latest.requested_posture)
        self.assertTrue(harness.latest.accepting_commands)

    def test_damping_never_completes_down_and_stand_can_replace_it(self) -> None:
        harness = Harness()
        harness.state(Go2SportMode.IDLE)
        self.assertTrue(harness.posture("down"))
        harness.next_state(Go2SportMode.IDLE)

        harness.next_state(
            Go2SportMode.IDLE,
            state_machine_code=Go2MotionStateMachine.DAMPING,
        )
        self.assertEqual(harness.latest.requested_posture, PostureTarget.DOWN)
        self.assertTrue(harness.latest.accepting_commands)

        self.assertTrue(harness.posture("stand"))
        harness.next_state(
            Go2SportMode.IDLE,
            state_machine_code=Go2MotionStateMachine.DAMPING,
        )
        self.assertEqual(
            harness.client.calls,
            [("StopMove",), ("RecoveryStand",)],
        )

    def test_latest_posture_replaces_older_and_has_velocity_priority(self) -> None:
        harness = Harness()
        harness.state(Go2SportMode.LIE_DOWN)
        self.assertTrue(harness.posture("stand"))
        self.assertTrue(harness.posture("down"))
        harness.next_state(Go2SportMode.LIE_DOWN)
        self.assertEqual(harness.client.calls, [])
        self.assertIsNone(harness.latest.requested_posture)

        moving = Harness()
        moving.state(Go2SportMode.IDLE)
        self.assertTrue(moving.velocity(1.0))
        self.assertTrue(moving.posture("down"))
        self.assertTrue(moving.velocity(2.0))
        moving.next_state(Go2SportMode.IDLE)
        self.assertEqual(moving.client.calls, [("StopMove",)])
        self.assertIsNone(moving.latest.requested_velocity)

    def test_velocity_deadman_sends_one_zero_without_closing_input(self) -> None:
        harness = Harness()
        harness.state(Go2SportMode.IDLE)
        self.assertTrue(harness.velocity(1.0))
        harness.clock.advance(0.19)
        harness.state(Go2SportMode.IDLE)
        self.assertEqual(harness.client.calls, [("Move", 1.0, 0.0, 0.0)])

        harness.clock.advance(0.06)
        harness.state(Go2SportMode.IDLE)
        self.assertEqual(
            harness.client.calls,
            [("Move", 1.0, 0.0, 0.0), ("Move", 0.0, 0.0, 0.0)],
        )
        self.assertIsNone(harness.latest.requested_velocity)
        self.assertTrue(harness.latest.accepting_commands)

        harness.next_state(Go2SportMode.IDLE)
        self.assertEqual(len(harness.client.calls), 2)

    def test_shutdown_uses_same_down_workflow_and_requires_observed_down(self) -> None:
        harness = Harness()
        harness.state(Go2SportMode.IDLE)
        harness.controller.begin_shutdown()
        self.assertEqual(harness.latest.lifecycle, Lifecycle.SHUTTING_DOWN)
        self.assertFalse(harness.latest.accepting_commands)

        harness.next_state(Go2SportMode.IDLE)
        harness.next_state(Go2SportMode.IDLE)
        self.assertFalse(harness.controller.shutdown_complete)
        harness.next_state(
            Go2SportMode.IDLE,
            state_machine_code=Go2MotionStateMachine.DAMPING,
        )
        self.assertFalse(harness.controller.shutdown_complete)
        harness.next_state(Go2SportMode.LIE_DOWN)
        self.assertTrue(harness.controller.shutdown_complete)
        self.assertEqual(
            harness.client.calls,
            [("StopMove",), ("StandDown",)],
        )

    def test_public_state_keeps_sdk_result_separate_from_robot_state(self) -> None:
        harness = Harness()
        harness.client.stop_code = -1
        harness.state(Go2SportMode.IDLE)
        self.assertTrue(harness.posture("down"))
        harness.next_state(Go2SportMode.IDLE)

        self.assertEqual(harness.latest.robot.state, RobotState.UNKNOWN)
        self.assertEqual(harness.latest.posture_action, SdkCommand.STOP_MOVE)
        diagnostic = harness.latest.last_sdk
        self.assertIsNotNone(diagnostic)
        assert diagnostic is not None
        self.assertEqual(diagnostic.code, -1)
        self.assertIsNone(harness.latest.last_error)


if __name__ == "__main__":
    unittest.main()

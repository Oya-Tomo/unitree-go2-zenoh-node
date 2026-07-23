from __future__ import annotations

import unittest
from threading import Event, Thread

from pydantic import ValidationError

from controller import (
    Controller,
    Go2MotionStateMachine,
    Go2SportMode,
    Lifecycle,
    Motion,
    PostureCommand,
    PostureTarget,
    PublishedNodeState,
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


class BlockingMoveClient(FakeClient):
    def __init__(self) -> None:
        super().__init__()
        self.move_started = Event()
        self.release_move = Event()

    def Move(self, vx: float, vy: float, vyaw: float) -> int:
        self.calls.append(("Move", vx, vy, vyaw))
        self.move_started.set()
        if not self.release_move.wait(timeout=1.0):
            raise TimeoutError("test did not release Move")
        return 0


class Harness:
    def __init__(self, client: FakeClient | None = None) -> None:
        self.clock = Clock()
        self.client = client if client is not None else FakeClient()
        self.last_processed_received_at: float | None = None
        self.controller = Controller(
            self.client,
            state_freshness_seconds=0.2,
            quiescent_linear_speed_mps=0.03,
            quiescent_yaw_rate_rad_s=0.05,
            velocity_command_timeout_seconds=0.25,
            clock=self.clock,
        )

    @property
    def latest(self) -> PublishedNodeState:
        return self.controller.published_state()

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
        observed_at = self.clock() if received_at is None else received_at
        self.controller.receive_observation(
            RobotObservation(
                received_at=observed_at,
                stamp_sec=10,
                stamp_nanosec=20,
                state_machine_code=state_machine_code,
                mode=mode,
                velocity=(vx, 0.0, 0.0),
                yaw_speed=yaw_speed,
            )
        )
        processed_at = self.controller.process_observation(
            after=self.last_processed_received_at,
        )
        if processed_at is not None:
            self.last_processed_received_at = processed_at

    def next_state(self, mode: Go2SportMode, **kwargs: object) -> None:
        self.clock.advance()
        self.state(mode, **kwargs)  # type: ignore[arg-type]

    def posture(self, posture: str) -> bool:
        return self.controller.receive_command(
            PostureCommand(posture=PostureTarget(posture)),
            received_at=self.clock(),
        )

    def velocity(self, vx: float, vy: float = 0.0, vyaw: float = 0.0) -> bool:
        return self.controller.receive_command(
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
                    quiescent_linear_speed_mps=0.03,
                    quiescent_yaw_rate_rad_s=0.05,
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
        self.assertTrue(harness.latest.robot_connected)
        self.assertEqual(
            harness.latest.robot_state.state,
            RobotState.UNSUPPORTED,
        )
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
        harness.controller.enforce_observation_freshness()
        self.assertFalse(harness.latest.robot_connected)
        self.assertEqual(harness.latest.robot_state.state, RobotState.UNKNOWN)
        self.assertEqual(harness.latest.robot_state.reason, UnknownReason.STALE)
        self.assertFalse(harness.latest.accepting_commands)
        self.assertIsNone(harness.latest.requested_velocity)
        self.assertIsNone(harness.latest.robot_state.velocity)
        self.assertIsNone(harness.latest.robot_state.state_machine_code)
        self.assertFalse(harness.velocity(0.5))

        harness.next_state(Go2SportMode.IDLE)
        self.assertTrue(harness.latest.robot_connected)
        self.assertTrue(harness.latest.accepting_commands)
        self.assertEqual(harness.client.calls, [])

    def test_invalid_sample_does_not_refresh_freshness(self) -> None:
        harness = Harness()
        harness.controller.reject_observation("bad startup sample")
        self.assertEqual(
            harness.latest.robot_state.reason,
            UnknownReason.NO_SAMPLE,
        )
        self.assertEqual(harness.latest.last_node_error, "bad startup sample")
        self.assertFalse(harness.controller.has_observation)

        harness.state(Go2SportMode.IDLE)
        harness.clock.advance(0.19)
        harness.controller.reject_observation("bad later sample")
        self.assertTrue(harness.latest.robot_connected)

        harness.clock.advance(0.01)
        harness.controller.enforce_observation_freshness()
        self.assertFalse(harness.latest.robot_connected)

    def test_moving_agile_accepts_and_executes_latest_velocity(self) -> None:
        harness = Harness()
        harness.state(Go2SportMode.IDLE, vx=0.1)
        self.assertEqual(
            harness.latest.robot_state.state,
            RobotState.READY_STAND,
        )
        self.assertTrue(harness.velocity(0.5))
        self.assertTrue(harness.velocity(1.0, 0.2, -0.3))
        self.assertEqual(harness.client.calls, [])

        harness.next_state(Go2SportMode.IDLE, vx=0.2)
        self.assertEqual(harness.client.calls, [("Move", 1.0, 0.2, -0.3)])
        self.assertTrue(harness.latest.accepting_commands)

    def test_command_receipt_alone_never_invokes_the_sdk(self) -> None:
        harness = Harness()
        harness.state(Go2SportMode.IDLE)

        self.assertTrue(harness.velocity(0.5))
        self.assertTrue(harness.posture("down"))

        self.assertEqual(harness.client.calls, [])

    def test_blocking_sdk_does_not_block_or_queue_dds_observations(self) -> None:
        client = BlockingMoveClient()
        harness = Harness(client)
        self.addCleanup(client.release_move.set)
        harness.state(Go2SportMode.IDLE)
        self.assertTrue(harness.velocity(0.5))

        harness.clock.advance()
        blocked_at = harness.clock()
        harness.controller.receive_observation(
            RobotObservation(
                received_at=blocked_at,
                stamp_sec=10,
                stamp_nanosec=20,
                state_machine_code=Go2MotionStateMachine.AGILE,
                mode=Go2SportMode.IDLE,
                velocity=(0.1, 0.0, 0.0),
                yaw_speed=0.0,
            )
        )
        processed: list[float | None] = []
        worker = Thread(
            target=lambda: processed.append(
                harness.controller.process_observation(
                    after=harness.last_processed_received_at,
                )
            )
        )
        worker.start()
        self.assertTrue(client.move_started.wait(timeout=1.0))

        newest_received_at = blocked_at
        for index in range(2, 7):
            harness.clock.advance()
            newest_received_at = harness.clock()
            self.assertTrue(
                harness.controller.receive_observation(
                    RobotObservation(
                        received_at=newest_received_at,
                        stamp_sec=10,
                        stamp_nanosec=20 + index,
                        state_machine_code=Go2MotionStateMachine.AGILE,
                        mode=Go2SportMode.IDLE,
                        velocity=(index / 10, 0.0, 0.0),
                        yaw_speed=0.0,
                    )
                )
            )

        self.assertTrue(worker.is_alive())
        self.assertEqual(
            harness.latest.robot_state.velocity,
            (0.6, 0.0, 0.0),
        )

        client.release_move.set()
        worker.join(timeout=1.0)
        self.assertFalse(worker.is_alive())
        self.assertEqual(processed, [blocked_at])

        processed_at = harness.controller.process_observation(after=blocked_at)
        self.assertEqual(processed_at, newest_received_at)
        self.assertEqual(len(client.calls), 2)

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
                self.assertEqual(
                    harness.latest.robot_state.state,
                    RobotState.UNKNOWN,
                )
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
        self.assertEqual(
            harness.latest.robot_state.state,
            RobotState.UNKNOWN,
        )
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
        self.assertEqual(
            harness.latest.last_sdk_diagnostic.code,  # type: ignore[union-attr]
            -1,
        )

        harness.next_state(Go2SportMode.IDLE, vx=0.2)
        self.assertEqual(harness.client.calls, [("StopMove",)])
        self.assertEqual(
            harness.latest.robot_state.state,
            RobotState.READY_STAND,
        )

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

    def test_velocity_timeout_sends_one_zero_without_closing_input(self) -> None:
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

    def test_shutdown_issues_no_stop_move_from_a_stale_observation(self) -> None:
        harness = Harness()
        harness.state(Go2SportMode.IDLE, received_at=0.9)
        harness.controller.begin_shutdown()
        harness.controller.receive_observation(
            RobotObservation(
                received_at=1.0,
                stamp_sec=10,
                stamp_nanosec=21,
                state_machine_code=Go2MotionStateMachine.AGILE,
                mode=Go2SportMode.IDLE,
                velocity=(0.0, 0.0, 0.0),
                yaw_speed=0.0,
            )
        )
        harness.clock.advance(0.2)

        self.assertEqual(
            harness.controller.process_observation(after=0.9),
            1.0,
        )
        self.assertEqual(harness.client.calls, [])
        self.assertFalse(harness.latest.robot_connected)
        self.assertEqual(
            harness.latest.robot_state.reason,
            UnknownReason.STALE,
        )

    def test_public_state_keeps_sdk_result_separate_from_robot_state(self) -> None:
        harness = Harness()
        harness.client.stop_code = -1
        harness.state(Go2SportMode.IDLE)
        self.assertTrue(harness.posture("down"))
        harness.next_state(Go2SportMode.IDLE)

        self.assertEqual(
            harness.latest.robot_state.state,
            RobotState.UNKNOWN,
        )
        self.assertEqual(
            harness.latest.last_posture_action,
            SdkCommand.STOP_MOVE,
        )
        diagnostic = harness.latest.last_sdk_diagnostic
        self.assertIsNotNone(diagnostic)
        assert diagnostic is not None
        self.assertEqual(diagnostic.code, -1)
        self.assertIsNone(harness.latest.last_node_error)

    def test_public_state_has_stable_schema_without_local_counters_or_times(
        self,
    ) -> None:
        harness = Harness()
        snapshots = [harness.latest]
        harness.state(Go2SportMode.IDLE)
        self.assertTrue(harness.velocity(0.5))
        snapshots.append(harness.latest)
        harness.clock.advance(0.2)
        snapshots.append(harness.latest)

        expected_fields = {
            "lifecycle",
            "robot_connected",
            "robot_state",
            "accepting_commands",
            "requested_posture",
            "requested_velocity",
            "last_posture_action",
            "last_sdk_diagnostic",
            "last_node_error",
        }
        for snapshot in snapshots:
            with self.subTest(lifecycle=snapshot.lifecycle):
                payload = snapshot.model_dump()
                self.assertEqual(set(payload), expected_fields)
                self.assertNotIn("received_at", payload["robot_state"])
                velocity = payload["requested_velocity"]
                if velocity is not None:
                    self.assertNotIn("received_at", velocity)

        old_payload = snapshots[1].model_dump()
        old_payload["robot_state"]["received_at"] = 1.0
        with self.assertRaises(ValidationError):
            PublishedNodeState.model_validate(old_payload)


if __name__ == "__main__":
    unittest.main()

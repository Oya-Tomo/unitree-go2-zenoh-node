from __future__ import annotations

import unittest
from collections.abc import Callable
from dataclasses import dataclass
from threading import Event
from time import monotonic
from typing import Any, Self

from pydantic import ValidationError

from controller import (
    Controller,
    Go2MotionStateMachine,
    Go2SportMode,
    RobotObservation,
)
from node import next_publish_deadline, serve_zenoh, to_observation
from wire import (
    Keyspace,
    PostureCommand,
    PostureTarget,
    PublishedNodeState,
    VelocityCommand,
)


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


class FakeClient:
    def __init__(self) -> None:
        self.calls: list[tuple[object, ...]] = []

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
        return 0

    def StandDown(self) -> int:
        self.calls.append(("StandDown",))
        return 0

    def Move(self, vx: float, vy: float, vyaw: float) -> int:
        self.calls.append(("Move", vx, vy, vyaw))
        return 0


class FakeResource:
    def __enter__(self) -> Self:
        return self

    def __exit__(self, *_exc_info: object) -> None:
        pass


class BlockingPublisher(FakeResource):
    def __init__(self) -> None:
        self.started = Event()
        self.release = Event()
        self.payloads: list[str] = []

    def put(self, payload: str) -> None:
        self.started.set()
        if not self.release.wait(timeout=1.0):
            raise TimeoutError("test did not release Zenoh publication")
        self.payloads.append(payload)


class FakeSession:
    def __init__(self) -> None:
        self.publisher = BlockingPublisher()
        self.subscriber_callbacks: dict[str, Callable[[Any], None]] = {}
        self.query_callback: Callable[[Any], None] | None = None

    def declare_subscriber(
        self,
        key: str,
        callback: Callable[[Any], None],
    ) -> FakeResource:
        self.subscriber_callbacks[key] = callback
        return FakeResource()

    def declare_queryable(
        self,
        _key: str,
        callback: Callable[[Any], None],
        *,
        complete: bool,
    ) -> FakeResource:
        self.query_callback = callback
        return FakeResource()

    def declare_publisher(
        self,
        _key: str,
        *,
        encoding: str,
    ) -> BlockingPublisher:
        return self.publisher


@dataclass(frozen=True)
class FakePayload:
    value: bytes

    def to_bytes(self) -> bytes:
        return self.value


@dataclass(frozen=True)
class FakeSample:
    payload: FakePayload
    key_expr: str = ""


class FakeQuery:
    def __init__(self) -> None:
        self.replies: list[tuple[str, str, str]] = []

    def __enter__(self) -> Self:
        return self

    def __exit__(self, *_exc_info: object) -> None:
        pass

    def reply(self, key: str, payload: str, *, encoding: str) -> None:
        self.replies.append((key, payload, encoding))


class NodeBoundaryTests(unittest.TestCase):
    def test_each_command_key_accepts_only_its_own_payload(self) -> None:
        keyspace = Keyspace("unitree/go2")
        self.assertEqual(keyspace.velocity, "unitree/go2/command/velocity")
        self.assertEqual(keyspace.posture, "unitree/go2/command/posture")
        self.assertEqual(keyspace.state, "unitree/go2/state")

        velocity = VelocityCommand.model_validate_json(
            b'{"vx":0.1,"vy":0.0,"vyaw":-0.2}'
        )
        posture = PostureCommand.model_validate_json(b'{"posture":"down"}')
        self.assertEqual((velocity.vx, velocity.vy, velocity.vyaw), (0.1, 0.0, -0.2))
        self.assertIs(posture.posture, PostureTarget.DOWN)

        rejected = (
            (VelocityCommand, b'{"type":"velocity","vx":0.1,"vy":0.0,"vyaw":0.0}'),
            (VelocityCommand, b'{"posture":"down"}'),
            (VelocityCommand, b'{"vx":"0.1","vy":0.0,"vyaw":0.0}'),
            (PostureCommand, b'{"type":"posture","posture":"down"}'),
            (PostureCommand, b'{"vx":0.1,"vy":0.0,"vyaw":0.0}'),
            (PostureCommand, b'{"posture":"stop"}'),
        )
        for model, payload in rejected:
            with (
                self.subTest(model=model.__name__, payload=payload),
                self.assertRaises(ValidationError),
            ):
                model.model_validate_json(payload)

    def test_state_conversion_uses_the_full_velocity_vector(self) -> None:
        observation = to_observation(FakeState(), received_at=1.5)
        self.assertEqual(observation.velocity, (0.1, 0.2, 0.3))
        self.assertEqual(observation.stamp_sec, 12)
        self.assertEqual(observation.stamp_nanosec, 34)
        self.assertEqual(observation.state_machine_code, 100)

    def test_publish_deadline_skips_missed_periods_without_drift(self) -> None:
        self.assertEqual(
            next_publish_deadline(10.0, now=10.2, period_seconds=1.0),
            11.0,
        )
        self.assertEqual(
            next_publish_deadline(10.0, now=13.2, period_seconds=1.0),
            14.0,
        )

    def test_blocked_publication_does_not_block_commands_control_or_queries(
        self,
    ) -> None:
        client = FakeClient()
        controller = Controller(
            client,
            state_freshness_seconds=10.0,
            quiescent_linear_speed_mps=0.03,
            quiescent_yaw_rate_rad_s=0.05,
            velocity_command_timeout_seconds=10.0,
        )
        first_received_at = monotonic()
        controller.receive_observation(
            RobotObservation(
                received_at=first_received_at,
                stamp_sec=10,
                stamp_nanosec=20,
                state_machine_code=Go2MotionStateMachine.AGILE,
                mode=Go2SportMode.IDLE,
                velocity=(0.0, 0.0, 0.0),
                yaw_speed=0.0,
            )
        )
        self.assertEqual(
            controller.step(after=None, timeout=0.0),
            first_received_at,
        )

        session = FakeSession()
        self.addCleanup(session.publisher.release.set)
        keyspace = Keyspace("unitree/go2")
        with serve_zenoh(
            session,  # type: ignore[arg-type]
            keyspace,
            controller,
            state_publish_frequency_hz=1.0,
        ):
            self.assertTrue(session.publisher.started.wait(timeout=1.0))
            self.assertEqual(
                set(session.subscriber_callbacks),
                {keyspace.velocity, keyspace.posture},
            )
            session.subscriber_callbacks[keyspace.velocity](
                FakeSample(
                    FakePayload(
                        VelocityCommand(
                            vx=0.5,
                            vy=0.0,
                            vyaw=0.0,
                        )
                        .model_dump_json()
                        .encode()
                    )
                )
            )
            self.assertEqual(client.calls, [])

            next_received_at = monotonic()
            controller.receive_observation(
                RobotObservation(
                    received_at=next_received_at,
                    stamp_sec=10,
                    stamp_nanosec=21,
                    state_machine_code=Go2MotionStateMachine.AGILE,
                    mode=Go2SportMode.IDLE,
                    velocity=(0.0, 0.0, 0.0),
                    yaw_speed=0.0,
                )
            )
            self.assertEqual(
                controller.step(after=first_received_at, timeout=0.0),
                next_received_at,
            )
            self.assertEqual(client.calls, [("Move", 0.5, 0.0, 0.0)])

            query_callback = session.query_callback
            assert query_callback is not None
            query = FakeQuery()
            query_callback(query)
            self.assertEqual(len(query.replies), 1)
            state = PublishedNodeState.model_validate_json(query.replies[0][1])
            self.assertTrue(state.robot_connected)

            posture_callback = session.subscriber_callbacks[keyspace.posture]
            with self.assertLogs("node", level="WARNING"):
                posture_callback(
                    FakeSample(FakePayload(b'{"vx":0.0,"vy":0.0,"vyaw":0.0}'))
                )
            self.assertIsNone(controller.published_state().requested_posture)
            posture_callback(FakeSample(FakePayload(b'{"posture":"down"}')))
            self.assertIs(
                controller.published_state().requested_posture,
                PostureTarget.DOWN,
            )

            session.publisher.release.set()


if __name__ == "__main__":
    unittest.main()

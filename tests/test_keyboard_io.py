from __future__ import annotations

import unittest
from dataclasses import dataclass

from examples.keyboard_io import RobotStateCache
from keyspace import RobotKeyspace


@dataclass(frozen=True)
class FakePayload:
    value: bytes

    def to_bytes(self) -> bytes:
        return self.value


@dataclass(frozen=True)
class FakeSample:
    key_expr: object
    payload: FakePayload


class RobotStateCacheTests(unittest.TestCase):
    def test_posture_observation_records_local_receive_time(self) -> None:
        keyspace = RobotKeyspace("unitree/go2")
        cache = RobotStateCache(keyspace, clock=lambda: 12.5)

        cache.update(
            FakeSample(
                key_expr=keyspace.posture,
                payload=FakePayload(b'{"posture":"down"}'),
            )
        )

        observation = cache.posture()
        self.assertIsNotNone(observation)
        assert observation is not None
        self.assertEqual(observation.posture, "down")
        self.assertEqual(observation.received_at, 12.5)

    def test_state_becomes_stale_after_configured_age(self) -> None:
        keyspace = RobotKeyspace("unitree/go2")
        cache = RobotStateCache(keyspace, clock=lambda: 1.0)
        cache.update(
            FakeSample(
                key_expr=keyspace.health,
                payload=FakePayload(b'{"status":"ready"}'),
            )
        )

        observed = cache.snapshot()[keyspace.health]
        self.assertFalse(observed.is_stale(now=3.5, maximum_age=2.5))
        self.assertTrue(observed.is_stale(now=3.6, maximum_age=2.5))

    def test_snapshot_does_not_expose_mutable_cache_values(self) -> None:
        keyspace = RobotKeyspace("unitree/go2")
        cache = RobotStateCache(keyspace, clock=lambda: 1.0)
        cache.update(
            FakeSample(
                key_expr=keyspace.health,
                payload=FakePayload(b'{"status":"ready"}'),
            )
        )

        cache.snapshot()[keyspace.health].value["status"] = "corrupted"

        self.assertEqual(
            cache.snapshot()[keyspace.health].value["status"],
            "ready",
        )


if __name__ == "__main__":
    unittest.main()

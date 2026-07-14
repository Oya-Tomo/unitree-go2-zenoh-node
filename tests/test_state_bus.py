from __future__ import annotations

import json
import unittest
from contextlib import AbstractContextManager
from typing import Any

from models import HealthState, Posture, PostureState, VelocityState
from node import ZenohStateBus


class FakeQueryable(AbstractContextManager["FakeQueryable"]):
    def __enter__(self) -> FakeQueryable:
        return self

    def __exit__(self, *exc_info: object) -> None:
        return None


class FakeSession:
    def __init__(self) -> None:
        self.queryables: dict[str, Any] = {}
        self.puts: list[tuple[str, str, str]] = []

    def declare_queryable(
        self, key: str, callback: Any, *, complete: bool
    ) -> FakeQueryable:
        self.queryables[key] = (callback, complete)
        return FakeQueryable()

    def put(self, key: str, payload: str, *, encoding: str) -> None:
        self.puts.append((key, payload, encoding))


class FakeQuery(AbstractContextManager["FakeQuery"]):
    def __init__(self) -> None:
        self.replies: list[tuple[str, str, str]] = []

    def __enter__(self) -> FakeQuery:
        return self

    def __exit__(self, *exc_info: object) -> None:
        return None

    def reply(self, key: str, payload: str, *, encoding: str) -> None:
        self.replies.append((key, payload, encoding))


class ZenohStateBusTests(unittest.TestCase):
    def test_publishes_and_answers_queries_for_all_state_keys(self) -> None:
        session = FakeSession()
        with ZenohStateBus(session, "unitree/go2") as bus:  # type: ignore[arg-type]
            bus.publish_requested(VelocityState(vx=0.5, vy=0.0, vyaw=0.0))
            bus.publish_applied(VelocityState())
            bus.publish_posture(PostureState(posture=Posture.DOWN))
            bus.publish_health(HealthState())

            expected_keys = {
                "unitree/go2/state/command/requested",
                "unitree/go2/state/command/applied",
                "unitree/go2/state/posture",
                "unitree/go2/state/health",
            }
            self.assertEqual(set(session.queryables), expected_keys)
            self.assertEqual({entry[0] for entry in session.puts}, expected_keys)
            self.assertTrue(
                all(entry[2] == "application/json" for entry in session.puts)
            )

            for key, (callback, complete) in session.queryables.items():
                query = FakeQuery()
                callback(query)
                self.assertTrue(complete)
                self.assertEqual(query.replies[0][0], key)
                self.assertIsInstance(json.loads(query.replies[0][1]), dict)


if __name__ == "__main__":
    unittest.main()

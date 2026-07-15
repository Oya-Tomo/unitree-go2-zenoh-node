from __future__ import annotations

import unittest
from collections.abc import Callable
from dataclasses import dataclass
from threading import Event, Thread

from motion_state import (
    Go2SportMode,
    MotionStateCache,
    MotionStateMonitor,
    MotionStateObservation,
    SportModeStateLike,
    subscribe_sport_mode,
)


@dataclass(frozen=True)
class FakeSportModeState:
    mode: int
    error_code: int


class FakeSubscriber:
    def __init__(
        self,
        *,
        state: FakeSportModeState | None = None,
        init_error: Exception | None = None,
    ) -> None:
        self.state = state
        self.init_error = init_error
        self.initialized = False
        self.closed = False
        self.callback_thread: Thread | None = None

    def Init(self, handler: Callable[[SportModeStateLike], None]) -> None:
        self.initialized = True
        if self.init_error is not None:
            raise self.init_error
        if self.state is not None:
            self.callback_thread = Thread(target=handler, args=(self.state,))
            self.callback_thread.start()

    def Close(self) -> None:
        if self.callback_thread is not None:
            self.callback_thread.join(timeout=0.5)
        self.closed = True


class MotionStateObservationTests(unittest.TestCase):
    def test_known_and_unknown_mode_names(self) -> None:
        known = MotionStateObservation(
            mode=Go2SportMode.LIE_DOWN,
            error_code=0,
            received_at=1.0,
        )
        unknown = MotionStateObservation(mode=255, error_code=0, received_at=1.0)

        self.assertEqual(known.mode_name, "lie_down")
        self.assertEqual(unknown.mode_name, "unknown")

    def test_freshness_rejects_old_and_far_future_samples(self) -> None:
        old = MotionStateObservation(mode=5, error_code=0, received_at=1.0)
        future = MotionStateObservation(mode=5, error_code=0, received_at=3.0)

        self.assertTrue(old.is_fresh(now=1.5, maximum_age=0.5))
        self.assertFalse(old.is_fresh(now=1.51, maximum_age=0.5))
        self.assertFalse(future.is_fresh(now=1.0, maximum_age=0.5))


class MotionStateCacheTests(unittest.TestCase):
    def test_update_records_local_receive_time(self) -> None:
        cache = MotionStateCache(clock=lambda: 12.5)

        cache.update(mode=5, error_code=0)

        self.assertEqual(
            cache.snapshot(),
            MotionStateObservation(mode=5, error_code=0, received_at=12.5),
        )

    def test_wait_for_first_observation_reports_current_readiness(self) -> None:
        cache = MotionStateCache()

        self.assertFalse(cache.wait_for_first_observation(timeout=0.0))
        cache.update(mode=Go2SportMode.LIE_DOWN, error_code=0)
        self.assertTrue(cache.wait_for_first_observation(timeout=0.0))

    def test_update_wakes_first_observation_waiter(self) -> None:
        cache = MotionStateCache()
        waiting = Event()
        result: list[bool] = []

        def wait_for_observation() -> None:
            waiting.set()
            result.append(cache.wait_for_first_observation(timeout=0.5))

        waiter = Thread(target=wait_for_observation)
        waiter.start()
        self.assertTrue(waiting.wait(timeout=0.5))
        cache.update(mode=Go2SportMode.LIE_DOWN, error_code=0)
        waiter.join(timeout=0.5)

        self.assertFalse(waiter.is_alive())
        self.assertEqual(result, [True])


class MotionStateMonitorTests(unittest.TestCase):
    def test_observe_assesses_freshness_and_mode_name(self) -> None:
        cache = MotionStateCache(clock=lambda: 10.0)
        cache.update(mode=Go2SportMode.LIE_DOWN, error_code=0)
        monitor = MotionStateMonitor(cache, maximum_age=0.5)

        fresh = monitor.observe(now=10.5)
        stale = monitor.observe(now=10.51)

        self.assertTrue(fresh.telemetry.fresh)
        self.assertEqual(fresh.telemetry.mode_name, "lie_down")
        self.assertTrue(fresh.confirms_down)
        self.assertFalse(stale.telemetry.fresh)
        self.assertFalse(stale.confirms_down)

    def test_observe_without_a_sample_is_unknown(self) -> None:
        monitor = MotionStateMonitor(MotionStateCache(), maximum_age=0.5)

        motion = monitor.observe(now=1.0)

        self.assertIsNone(motion.telemetry.mode)
        self.assertFalse(motion.telemetry.fresh)
        self.assertFalse(motion.permits_walking)


class SportModeSubscriptionTests(unittest.TestCase):
    def test_callback_arrives_before_context_body_and_subscriber_closes(self) -> None:
        cache = MotionStateCache(clock=lambda: 2.0)
        subscriber = FakeSubscriber(state=FakeSportModeState(mode=5, error_code=0))

        with subscribe_sport_mode(
            "rt/test",
            cache,
            initial_state_timeout_seconds=0.5,
            subscriber_factory=lambda _topic: subscriber,
        ):
            self.assertTrue(subscriber.initialized)
            self.assertIsNotNone(cache.snapshot())
            self.assertFalse(subscriber.closed)

        self.assertTrue(subscriber.closed)

    def test_timeout_warns_and_still_closes_subscriber(self) -> None:
        cache = MotionStateCache()
        subscriber = FakeSubscriber()

        with (
            self.assertLogs("motion_state", level="WARNING"),
            subscribe_sport_mode(
                "rt/test",
                cache,
                initial_state_timeout_seconds=0.001,
                subscriber_factory=lambda _topic: subscriber,
            ),
        ):
            self.assertIsNone(cache.snapshot())

        self.assertTrue(subscriber.closed)

    def test_init_exception_closes_subscriber(self) -> None:
        subscriber = FakeSubscriber(init_error=RuntimeError("init failed"))

        with (
            self.assertRaises(RuntimeError),
            subscribe_sport_mode(
                "rt/test",
                MotionStateCache(),
                initial_state_timeout_seconds=0.5,
                subscriber_factory=lambda _topic: subscriber,
            ),
        ):
            self.fail("subscription body must not run after Init failure")

        self.assertTrue(subscriber.closed)


if __name__ == "__main__":
    unittest.main()

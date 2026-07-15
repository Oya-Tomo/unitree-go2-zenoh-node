from __future__ import annotations

import unittest

from motion_state import (
    Go2SportMode,
    MotionStateCache,
    MotionStateMonitor,
    MotionStateObservation,
)


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


if __name__ == "__main__":
    unittest.main()

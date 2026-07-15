from __future__ import annotations

import unittest
from pathlib import Path

import zenoh
from pydantic import ValidationError

from examples.keyboard import KeyboardConfig, load_keyboard_config
from settings import NodeConfig, load_node_config


class ConfigTests(unittest.TestCase):
    def test_example_node_config_loads(self) -> None:
        config = load_node_config(Path("config/node-config.example.json5"))

        self.assertEqual(config.robot_key, "unitree/go2")
        self.assertEqual(config.safety.command_timeout_seconds, 0.25)
        self.assertEqual(config.safety.motion_state_max_age_seconds, 0.5)
        self.assertEqual(config.safety.balance_confirmation_timeout_seconds, 1.0)
        self.assertEqual(config.dds.sport_mode_state_topic, "rt/sportmodestate")

    def test_example_keyboard_config_loads(self) -> None:
        config = load_keyboard_config(Path("examples/keyboard-config.example.json5"))

        self.assertEqual(config.publish_frequency_hz, 20.0)
        self.assertEqual(config.targets.vx, 0.5)

    def test_keyboard_shared_vx_target_honors_reverse_limit(self) -> None:
        with self.assertRaises(ValidationError) as raised:
            KeyboardConfig.model_validate(
                {
                    "robot_key": "unitree/go2",
                    "publish_frequency_hz": 20.0,
                    "state_stale_after_seconds": 2.5,
                    "targets": {"vx": 2.6, "vy": 0.3, "vyaw": 1.0},
                    "ramp_rates": {"vx": 1.0, "vy": 0.8, "vyaw": 2.0},
                    "posture_requests": {
                        "retry_interval_seconds": 0.5,
                        "timeout_seconds": 10.0,
                    },
                }
            )

        self.assertIn(
            ("targets", "vx"),
            {tuple(error["loc"]) for error in raised.exception.errors()},
        )

    def test_example_zenoh_configs_load(self) -> None:
        for path in (
            "config/zenoh-config.example.json5",
            "examples/keyboard-zenoh-config.example.json5",
        ):
            with self.subTest(path=path):
                config = zenoh.Config.from_file(path)
                self.assertIsInstance(config, zenoh.Config)

    def test_robot_key_must_be_concrete(self) -> None:
        with self.assertRaises(ValidationError):
            NodeConfig.model_validate(
                {
                    "robot_key": "unitree/*",
                    "state_heartbeat_seconds": 1.0,
                    "dds": {
                        "domain_id": 0,
                        "network_interface": "eth0",
                        "rpc_timeout_seconds": 0.2,
                    },
                    "safety": {
                        "command_timeout_seconds": 0.25,
                        "posture_transition_seconds": 3.0,
                        "shutdown_stop_delay_seconds": 1.0,
                    },
                }
            )

    def test_rpc_timeout_is_not_misrepresented_as_watchdog_deadline(self) -> None:
        config = NodeConfig.model_validate(
            {
                "robot_key": "unitree/go2",
                "state_heartbeat_seconds": 1.0,
                "dds": {
                    "domain_id": 0,
                    "network_interface": "eth0",
                    "rpc_timeout_seconds": 0.3,
                },
                "safety": {
                    "command_timeout_seconds": 0.25,
                    "posture_transition_seconds": 3.0,
                    "shutdown_stop_delay_seconds": 1.0,
                },
            }
        )

        self.assertEqual(config.dds.rpc_timeout_seconds, 0.3)

    def test_sport_mode_state_topic_must_be_non_empty(self) -> None:
        with self.assertRaises(ValidationError):
            NodeConfig.model_validate(
                {
                    "robot_key": "unitree/go2",
                    "state_heartbeat_seconds": 1.0,
                    "dds": {
                        "domain_id": 0,
                        "network_interface": "eth0",
                        "rpc_timeout_seconds": 0.3,
                        "sport_mode_state_topic": " ",
                    },
                    "safety": {
                        "command_timeout_seconds": 0.25,
                        "posture_transition_seconds": 3.0,
                        "shutdown_stop_delay_seconds": 1.0,
                    },
                }
            )


if __name__ == "__main__":
    unittest.main()

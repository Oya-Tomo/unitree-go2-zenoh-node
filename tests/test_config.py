from __future__ import annotations

import unittest
from pathlib import Path

from pydantic import ValidationError

from config import KeyboardConfig, NodeConfig, load_keyboard_config, load_node_config

ROOT = Path(__file__).parents[1]


class ConfigTests(unittest.TestCase):
    def test_example_configs_load(self) -> None:
        node = load_node_config(ROOT / "config/node-config.example.json5")
        keyboard = load_keyboard_config(ROOT / "examples/keyboard-config.example.json5")
        self.assertEqual(node.zenoh_key_prefix, keyboard.zenoh_key_prefix)

    def test_removed_gate_and_stability_settings_are_rejected(self) -> None:
        data = load_node_config(ROOT / "config/node-config.example.json5").model_dump()
        data["robot_state"]["stable_sample_count"] = 3
        with self.assertRaises(ValidationError):
            NodeConfig.model_validate(data)

    def test_zenoh_key_prefix_must_be_concrete(self) -> None:
        data = load_node_config(ROOT / "config/node-config.example.json5").model_dump()
        data["zenoh_key_prefix"] = "unitree/*"
        with self.assertRaises(ValidationError):
            NodeConfig.model_validate(data)

    def test_periodic_settings_use_hz(self) -> None:
        node = load_node_config(ROOT / "config/node-config.example.json5")
        self.assertEqual(node.node_state_publish_frequency_hz, 20.0)

        old_node_data = node.model_dump()
        old_node_data.pop("node_state_publish_frequency_hz")
        old_node_data["node_state_publish_interval_seconds"] = 0.05
        with self.assertRaises(ValidationError):
            NodeConfig.model_validate(old_node_data)

        keyboard = load_keyboard_config(ROOT / "examples/keyboard-config.example.json5")
        fractional_loop_data = keyboard.model_dump()
        fractional_loop_data["loop_frequency_hz"] = 20.5
        with self.assertRaises(ValidationError):
            KeyboardConfig.model_validate(fractional_loop_data)


if __name__ == "__main__":
    unittest.main()

from __future__ import annotations

import unittest
from pathlib import Path

from pydantic import ValidationError

from config import NodeConfig, load_keyboard_config, load_node_config

ROOT = Path(__file__).parents[1]


class ConfigTests(unittest.TestCase):
    def test_example_configs_load(self) -> None:
        node = load_node_config(ROOT / "config/node-config.example.json5")
        keyboard = load_keyboard_config(ROOT / "examples/keyboard-config.example.json5")
        self.assertEqual(node.robot_key, keyboard.robot_key)

    def test_removed_gate_and_stability_settings_are_rejected(self) -> None:
        data = load_node_config(ROOT / "config/node-config.example.json5").model_dump()
        data["state"]["stable_sample_count"] = 3
        with self.assertRaises(ValidationError):
            NodeConfig.model_validate(data)

    def test_robot_key_must_be_concrete(self) -> None:
        data = load_node_config(ROOT / "config/node-config.example.json5").model_dump()
        data["robot_key"] = "unitree/*"
        with self.assertRaises(ValidationError):
            NodeConfig.model_validate(data)


if __name__ == "__main__":
    unittest.main()

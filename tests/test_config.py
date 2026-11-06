from __future__ import annotations

import copy
import unittest
from pathlib import Path

from pydantic import ValidationError

from config import KeyboardConfig, NodeConfig, load_config

ROOT = Path(__file__).parents[1]
NODE_EXAMPLE = ROOT / "config/node-config.example.json5"
KEYBOARD_EXAMPLE = ROOT / "examples/keyboard-config.example.json5"


class ConfigTests(unittest.TestCase):
    def test_example_configs_load(self) -> None:
        node = load_config(NodeConfig, NODE_EXAMPLE)
        keyboard = load_config(KeyboardConfig, KEYBOARD_EXAMPLE)
        self.assertEqual(node.zenoh_key_prefix, keyboard.zenoh_key_prefix)

    def test_removed_gate_and_stability_settings_are_rejected(self) -> None:
        data = load_config(NodeConfig, NODE_EXAMPLE).model_dump()
        data["robot_state"]["stable_sample_count"] = 3
        with self.assertRaises(ValidationError):
            NodeConfig.model_validate(data)

    def test_zenoh_key_prefix_must_be_concrete(self) -> None:
        data = load_config(NodeConfig, NODE_EXAMPLE).model_dump()
        data["zenoh_key_prefix"] = "unitree/*"
        with self.assertRaises(ValidationError):
            NodeConfig.model_validate(data)

    def test_periodic_settings_use_hz(self) -> None:
        node = load_config(NodeConfig, NODE_EXAMPLE)
        self.assertEqual(node.node_state_publish_frequency_hz, 20.0)

        old_node_data = node.model_dump()
        old_node_data.pop("node_state_publish_frequency_hz")
        old_node_data["node_state_publish_interval_seconds"] = 0.05
        with self.assertRaises(ValidationError):
            NodeConfig.model_validate(old_node_data)

        keyboard = load_config(KeyboardConfig, KEYBOARD_EXAMPLE)
        fractional_loop_data = keyboard.model_dump()
        fractional_loop_data["loop_frequency_hz"] = 20.5
        with self.assertRaises(ValidationError):
            KeyboardConfig.model_validate(fractional_loop_data)

    def test_values_must_be_finite_and_strictly_typed(self) -> None:
        data = load_config(NodeConfig, NODE_EXAMPLE).model_dump()
        invalid = (
            ("node_state_publish_frequency_hz", float("inf")),
            ("dds.state_freshness_seconds", float("inf")),
            ("dds.state_freshness_seconds", float("nan")),
            ("dds.domain_id", True),
            ("sport_client.rpc_timeout_seconds", "0.2"),
            ("dds.network_interface", "  "),
        )
        for path, value in invalid:
            with self.subTest(path=path, value=value):
                candidate = copy.deepcopy(data)
                *parents, field = path.split(".")
                section = candidate
                for parent in parents:
                    section = section[parent]
                section[field] = value
                with self.assertRaises(ValidationError):
                    NodeConfig.model_validate(candidate)


if __name__ == "__main__":
    unittest.main()

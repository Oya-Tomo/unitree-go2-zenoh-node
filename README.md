# Unitree Go2 Zenoh Node

[日本語](README_ja.md)

A standalone node that receives velocity and posture commands over Zenoh and controls one Unitree Go2 through the high-level SportClient API.

> [!WARNING]
> This software commands physical hardware.
> Support the robot, keep the remote and emergency-stop procedure available, and begin with low speeds.

## Quick start

From a checkout with its submodule initialized:

```console
$ export CYCLONEDDS_HOME="$HOME/Packages/cyclonedds/install"
$ uv sync --all-groups
$ cp config/node-config.example.json5 config/node-config.json5
$ cp config/zenoh-config.example.json5 config/zenoh-config.json5
$ cp examples/keyboard-config.example.json5 examples/keyboard-config.json5
$ cp examples/keyboard-zenoh-config.example.json5 examples/keyboard-zenoh-config.json5
```

Edit the four runtime configurations for this PC and network before starting the node.
The complete procedure, including DDS and Zenoh configuration, is in [Setup](docs/en/setup.md).

Start the robot node, then the keyboard controller:

```console
$ uv run node.py
$ uv run examples/keyboard.py
```

See [Usage](docs/en/usage.md) before sending commands to the robot.

## Documentation

- [Setup](docs/en/setup.md)
- [Usage, controls, and wire contracts](docs/en/usage.md)
- [Observed-State control ADR](docs/adr/0001-observed-state-driven-control-architecture.md)
- [Shared Node State concurrency ADR](docs/adr/0002-shared-node-state-concurrency.md)
- [日本語ドキュメント](README_ja.md)

# Setup

## Requirements

- A Unitree Go2 Edu reachable over a dedicated wired network interface
- Go2 software V1.1.6 or later
- A Linux PC with Python 3.13, Git, and [uv](https://docs.astral.sh/uv/)
- A local CycloneDDS installation compatible with the bundled SDK
- A desktop display for the pygame keyboard controller
- The Go2 remote and a known emergency-stop procedure

The node uses the high-level Unitree SportClient API.
Do not run it while another process is commanding the same high-level service.

## Install

Clone the repository with its SDK submodule and enter the repository root:

```console
$ git clone --recurse-submodules https://github.com/Oya-Tomo/unitree-go2-zenoh-node.git
$ cd unitree-go2-zenoh-node
$ git submodule update --init --recursive
```

Point the CycloneDDS Python build at the local CycloneDDS installation, then install the node and keyboard dependencies:

```console
$ export CYCLONEDDS_HOME="$HOME/Packages/cyclonedds/install"
$ uv sync --all-groups
```

Use the actual install prefix on this PC.
Keep `CYCLONEDDS_HOME` in the shell environment when running commands that need the Unitree SDK.
If dependency installation reports that CycloneDDS cannot be located, verify that the prefix contains the installed CycloneDDS libraries and CMake metadata before retrying `uv sync`.

Default configuration paths are relative to the current working directory.
Run commands from the repository root unless every configuration path is supplied explicitly.

## Prepare the configuration files

Copy the tracked examples to the ignored runtime paths:

```console
$ cp config/node-config.example.json5 config/node-config.json5
$ cp config/zenoh-config.example.json5 config/zenoh-config.json5
$ cp examples/keyboard-config.example.json5 examples/keyboard-config.json5
$ cp examples/keyboard-zenoh-config.example.json5 examples/keyboard-zenoh-config.json5
```

| Runtime file | Purpose |
| --- | --- |
| `config/node-config.json5` | Go2 DDS interface, State freshness, SportClient, and control settings |
| `config/zenoh-config.json5` | Robot node Zenoh mode, listeners, connections, and scouting |
| `examples/keyboard-config.json5` | Keyboard target speeds, ramp rates, display freshness, and `zenoh_key_prefix` |
| `examples/keyboard-zenoh-config.json5` | Keyboard Zenoh connection |

These runtime files are ignored by Git because they contain machine- and network-specific values.
The node and keyboard application configurations are strict JSON5: missing required fields, unknown fields, and incorrectly typed fields fail at startup.
Zenoh configuration is validated by the Zenoh library.

## Select the Go2 DDS interface

Inspect the interfaces on the PC:

```console
$ ip -br link
$ ip -br address
```

Identify the wired interface connected to the Go2 and verify that it is up.
If the interface is named `enp3s0`, useful checks are:

```console
$ ip link show enp3s0
$ ip address show dev enp3s0
$ ip neigh show dev enp3s0
```

Set the interface name—not an IP address—in `config/node-config.json5`:

```json5
dds: {
  domain_id: 0,
  network_interface: "enp3s0",
  sport_mode_state_topic: "rt/sportmodestate",
  first_state_timeout_seconds: 1.0,
  state_freshness_seconds: 0.2,
},
sport_client: {
  rpc_timeout_seconds: 0.2,
}
```

The example interface name is a placeholder.
Do not copy it without checking this PC.
DDS traffic uses this interface independently of the Zenoh connection between the robot node and keyboard.

## Connect the node and keyboard over Zenoh

`zenoh_key_prefix` must be the same concrete Zenoh key prefix in `config/node-config.json5` and `examples/keyboard-config.json5`.
The supplied value is:

```json5
zenoh_key_prefix: "unitree/go2"
```

The two Zenoh files determine how the processes reach each other.
They do not select the DDS interface.

### Both processes on one PC

Use one listener and one client rather than making both processes bind the same TCP port.
A loopback-only robot-node configuration is:

```json5
{
  mode: "peer",
  listen: {
    endpoints: ["tcp/127.0.0.1:7447"],
  },
  connect: {
    endpoints: [],
  },
  scouting: {
    multicast: {
      enabled: false,
    },
  },
}
```

Configure the keyboard as its client:

```json5
{
  mode: "client",
  connect: {
    endpoints: ["tcp/127.0.0.1:7447"],
  },
  scouting: {
    multicast: {
      enabled: false,
    },
  },
}
```

### Processes on different hosts

The robot node can listen on a reachable interface, and the keyboard can connect to that host's address.
For example, the node may listen on `tcp/0.0.0.0:7447`, while the keyboard connects to `tcp/192.168.1.20:7447`.

`0.0.0.0` is a local wildcard bind address; it is not an address to put in a remote `connect.endpoints` list.
Port 7447 must be unused by another listener.
Listening on every interface exposes the endpoint to every reachable network, so restrict the bind address or firewall when appropriate.

The example Zenoh configurations have no authentication or encryption.
Use them only on a trusted network.
Refer to the [Zenoh deployment guide](https://zenoh.io/docs/getting-started/deployment/) when selecting another topology.

## Use configurations from another location

The robot node accepts explicit node and Zenoh paths:

```console
$ uv run node.py \
    --node-config /etc/unitree-go2-zenoh-node/node.json5 \
    --zenoh-config /etc/unitree-go2-zenoh-node/zenoh.json5
```

The keyboard accepts explicit keyboard and Zenoh paths:

```console
$ uv run examples/keyboard.py \
    --keyboard-config /etc/unitree-go2-zenoh-node/keyboard.json5 \
    --zenoh-config /etc/unitree-go2-zenoh-node/keyboard-zenoh.json5
```

Individual configuration values cannot be overridden from the CLI.

## Safety preflight

Before starting either process:

1. Place the Go2 on a clear, level surface and support it during the first posture transition.
2. Keep people, cables, and obstacles outside the robot's motion area.
3. Keep the remote available and confirm the emergency-stop procedure.
4. Start with the conservative target speeds from the example keyboard configuration.
5. Confirm the dedicated DDS interface and the intended `zenoh_key_prefix` in both application configurations.
6. Start the robot node and wait for fresh State before starting the keyboard.

The Zenoh keyboard and process shutdown are not substitutes for an emergency stop.

## Development checks

These checks do not start the robot node or send commands to the Go2:

```console
$ uv run ruff format --check .
$ uv run ruff check .
$ uv run pyright
$ uv run python -m unittest discover -s tests -v
```

Continue with [Usage](usage.md) after setup is complete.

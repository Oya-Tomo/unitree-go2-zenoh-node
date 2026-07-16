# Unitree Go2 Zenoh Node

[日本語](README_ja.md)

A standalone bridge that evaluates Zenoh commands against the latest Unitree
Go2 `SportModeState` before calling `SportClient`.

> [!WARNING]
> This software commands physical hardware. Support the robot, keep the remote
> and emergency-stop procedure available, and begin with low speeds.

## Architecture

```text
DDS SportModeState ---> latest State inbox ---\
                                             +--> State-driven loop --> SDK
Zenoh command -------> latest command buffers /
                              |
                              +--> {robot_key}/state
```

The loop always performs the same sequence:

1. read the latest unprocessed robot State;
2. inspect the latest posture and velocity buffers;
3. evaluate posture first, then velocity; and
4. make at most one SDK call.

Zenoh receipt alone never calls the SDK. The core implementation has only three
files: `node.py` for process and I/O wiring, `controller.py` for State and
control policy, and `config.py` for configuration. See
[ADR-0001](docs/adr/0001-observed-state-driven-control-architecture.md).

## Install

Requirements are Python 3.13, uv, a local CycloneDDS build, and a reachable Go2
Edu running software V1.1.6 or later. The State classifier targets Unitree's
Motion Control Service Interface V2.0.

```bash
git clone --recurse-submodules https://github.com/Oya-Tomo/unitree-go2-zenoh-node.git
cd unitree-go2-zenoh-node
git submodule update --init --recursive

export CYCLONEDDS_HOME="$HOME/Packages/cyclonedds/install"
uv sync --all-groups
```

## Configure

```bash
cp config/node-config.example.json5 config/node-config.json5
cp config/zenoh-config.example.json5 config/zenoh-config.json5
cp examples/keyboard-config.example.json5 examples/keyboard-config.json5
cp examples/keyboard-zenoh-config.example.json5 examples/keyboard-zenoh-config.json5
```

Set `dds.network_interface` to the interface connected to the Go2. The node and
keyboard must use the same concrete `robot_key`. `maximum_age_seconds` limits
how long a State sample remains usable; the velocity thresholds distinguish
measured motion from quiescence.

The example Zenoh files have no authentication or encryption and are intended
for a trusted local network.

## Run

Start the robot node:

```bash
export CYCLONEDDS_HOME="$HOME/Packages/cyclonedds/install"
uv run node.py \
  --node-config config/node-config.json5 \
  --zenoh-config config/zenoh-config.json5
```

Then start the keyboard client:

```bash
uv run --group example python -m examples.keyboard \
  --keyboard-config examples/keyboard-config.json5 \
  --zenoh-config examples/keyboard-zenoh-config.json5
```

## Command contract

Publish JSON to `{robot_key}/command`.

```json
{"type":"velocity","vx":0.5,"vy":0.0,"vyaw":0.2}
```

```json
{"type":"posture","posture":"stand"}
```

```json
{"type":"posture","posture":"down"}
```

Only the latest velocity and posture are retained. A posture request has
priority over velocity. There is no explicit stop command; zero velocity uses
`Move(0, 0, 0)`.

## State-driven behavior

A fresh `SportModeState` sample is used immediately. SDK return values are
diagnostics and never establish posture or motion.

On the V2.0 interface, the DDS field named `error_code` is the current motion
state machine ID. It is published as `state_machine_code`; a nonzero value does
not mean failure. In particular, the real-hardware startup value `100` means
Agile. The controller currently acts only on these state machines:

| State machine ID | Name | Use in this node |
| --- | --- | --- |
| 100 | Agile | quiescent mode 0/1, or mode 3 |
| 1002 | Standing Lock | quiescent mode 0 |
| 1013 | Balance Standing | quiescent mode 1, or mode 3 |
| 1004 / 2006 | Crouch | down when mode 5 is quiescent |

Other state machines are reported but remain non-actionable. A valid unsupported
State makes the node lifecycle `running` while command acceptance stays false.
An inconsistent combination of a supported state machine, coarse mode, and
measured motion is a non-actionable transition.

| Robot State | Stand request | Down request | Velocity |
| --- | --- | --- | --- |
| Quiescent Down | `StandUp` | complete | wait |
| Quiescent idle stand | `BalanceStand` | Stop then `StandDown` | wait |
| Quiescent ready stand | complete | Stop then `StandDown` | `Move` |
| Locomotion | `BalanceStand` | Stop then `StandDown` | `Move` |
| Transition, moving mode 5, unsupported, Unknown | wait | wait | wait |

Before each posture SDK call, the node publishes robot State as `Unknown`.
It remains Unknown during the RPC. Only a valid State received after the RPC
returns can replace it, and command input resumes only when that State is
actionable.

Down is a fixed State-driven workflow:

```text
non-down State -> StopMove once -> newer quiescent State
               -> StandDown -> newer down State -> complete
```

`StopMove()` is not used at startup, for stand, or for zero velocity. A `-1`
result does not cause a retry. If the next State still reports motion, the node
waits without sending either another `StopMove()` or `StandDown()`.

Shutdown follows the same Stop-then-Down workflow. If the robot reports a
quiescent down State, shutdown sends neither command. A moving mode-5 sample is
treated as a transition and cannot complete Stand, Down, or shutdown.

## Published State

The node publishes and answers `get` on `{robot_key}/state`. The snapshot keeps
robot-derived State separate from requested commands and the latest SDK
diagnostic. It reports the V2.0 motion state machine separately from the coarse
sport mode. A command name is never presented as a physical posture.

## Keyboard controls

Shift is the motion deadman and is also required for posture requests.

| Input | Action |
| --- | --- |
| `Shift+W/S` | positive / negative `vx` |
| `Shift+A/D` | positive / negative `vy` |
| `Shift+Q/E` | positive / negative `vyaw` |
| `Shift+R` | publish Stand once |
| `Shift+F` | publish Down once |
| `Space` | publish zero velocity |
| release Shift / lose focus | publish zero velocity |
| `Esc` / close | publish zero velocity and exit |

## Validate

Without starting the robot node:

```bash
uv run ruff format --check .
uv run ruff check .
uv run pyright
uv run python -m unittest discover -s tests -v
```

Hardware acceptance still requires a supported test of down startup, Stand,
slow walking, Down, re-Stand, and shutdown while recording full State and SDK
diagnostics.

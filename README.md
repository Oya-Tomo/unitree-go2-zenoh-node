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
keyboard must use the same concrete `robot_key`. `maximum_age_seconds` is the
DDS connection watchdog: after 0.2 seconds without a valid State, commands are
rejected and buffered commands are cleared. The velocity thresholds distinguish
measured motion from quiescence only for the Down workflow.

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
| 100 | Agile | mode 0/1 is ready stand; mode 3 is locomotion |
| 1001 | Damping | mode 0; posture remains unknown |
| 1002 | Standing Lock | mode 0 is locked stand |
| 1013 | Balance Standing | mode 1 is ready stand; mode 3 is locomotion |
| 1004 / 2006 | Crouch | down when mode 5 is quiescent |

Damping is reported as its own physical class, not as Crouch or Down. The
target hardware reported it after `StandDown()` while physically low with
compliant joints, but Damping alone does not prove posture. It therefore never
completes Down or shutdown. A quiescent Damping State can authorize the
official `RecoveryStand()` operation, which is defined for fallen or crouched
robots and, according to the V2.0 interface, recovers to standing regardless of
whether the robot has fallen.

The coarse mode does not determine command capability by itself. In the target
trace, `BalanceStand()` returned zero and the standing robot continued to
report `100` Agile with mode 0. Agile mode 0 is therefore a State-confirmed
ready stand that can accept `Move`; `1002` Standing Lock with the same mode 0
must first receive `BalanceStand`. No RPC result or internal walking flag is
used to distinguish them.

Command intake has one gate for both command types. It is open while DDS State
is fresh, the node is not waiting after a posture RPC, and shutdown has not
begun. Robot State authorizes the command only when the next State drives the
loop; an illegal command is discarded rather than delayed.

| Robot State | Stand request | Down request | Velocity |
| --- | --- | --- | --- |
| Damping | `RecoveryStand` when quiescent | wait | discard |
| Down | `StandUp` | complete | discard |
| Locked stand | `BalanceStand` | Stop then `StandDown` | discard |
| Ready stand | complete | Stop then `StandDown` | `Move` |
| Locomotion | `BalanceStand` | Stop then `StandDown` | `Move` |
| Unsupported | discard new request | discard new request | discard |
| Unknown | reject | reject | reject |

Before each posture SDK call, the node publishes robot State as `Unknown`.
It remains Unknown during the RPC. Only a valid State received after the RPC
returns can replace it and reopen command intake.

Down is a fixed State-driven workflow:

```text
locked/ready/locomotion State -> StopMove once -> newer quiescent
                              -> StandDown -> newer down State -> complete
```

`StopMove()` is not used at startup, for stand, or for zero velocity. A `-1`
result does not cause a retry. If the next State still reports motion, the node
waits without sending either another `StopMove()` or `StandDown()`.

Shutdown follows the same Stop-then-Down workflow. If the robot reports a
quiescent Crouch/Down State, shutdown sends neither command. Damping and a
moving mode-5 sample cannot complete Down or shutdown.

## Published State

The node publishes and answers `get` on `{robot_key}/state`. The snapshot keeps
robot-derived State separate from requested commands and the latest SDK
diagnostic. It reports the V2.0 motion state machine separately from the coarse
sport mode and exposes one `accepting_commands` value for DDS-backed command
intake. A command name is never presented as a physical posture.

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

# Unitree Go2 Zenoh Node

[日本語](README_ja.md)

A safety-oriented bridge from Zenoh high-level commands to the Unitree Go2
`SportClient`. One process controls one robot through one DDS network interface.

> [!WARNING]
> This software commands physical hardware. Test with the robot supported or
> suspended, keep the wireless remote and emergency-stop procedure available,
> and verify the behavior on your robot/firmware before normal operation.

## Architecture

```text
high-level policy or pygame keyboard node
                  |
                  | Zenoh JSON commands at 20 Hz
                  v
       unitree-go2-zenoh-node
       - Pydantic validation
       - posture state machine
       - 250 ms watchdog
       - observable state/queryables
                  |
                  | Unitree SDK2 / CycloneDDS
                  v
             one Go2 robot
```

The deployment unit is **one node process + one NIC + one robot**. Use a separate
process, configuration, NIC, and unique `robot_key` for another robot. The MVP
does not select multiple robots from one SDK process and does not arbitrate
between command publishers.

## Requirements and installation

- Python 3.13
- [uv](https://docs.astral.sh/uv/)
- CycloneDDS built locally; this project was prepared with
  `~/Packages/cyclonedds/install`
- A Unitree Go2 reachable through the configured network interface

```bash
git clone --recurse-submodules https://github.com/Oya-Tomo/unitree-go2-zenoh-node.git
cd unitree-go2-zenoh-node
git submodule update --init --recursive

export CYCLONEDDS_HOME="$HOME/Packages/cyclonedds/install"
uv sync --all-groups
```

`unitree-sdk2py` is registered in `pyproject.toml` as an editable uv path source
at `third_party/unitree_sdk2_python`. The submodule tracks the
`chore/cdds-py313` branch of `Oya-Tomo/unitree_sdk2_python`.

## Configuration

Runtime values live in JSON5 files. The CLI only selects configuration paths.

```bash
cp config/node-config.example.json5 config/node-config.json5
cp config/zenoh-config.example.json5 config/zenoh-config.json5
cp examples/keyboard-config.example.json5 examples/keyboard-config.json5
cp examples/keyboard-zenoh-config.example.json5 examples/keyboard-zenoh-config.json5
```

Set `dds.network_interface` to the NIC connected to the robot. Set the same
`robot_key` in the robot node and every command publisher. The example key is
`unitree/go2`, but it may be changed to any concrete Zenoh key.

Both Zenoh examples use peer mode, multicast discovery, and the fixed listen
endpoint `tcp/0.0.0.0:7447`. They intentionally provide no TLS, authentication,
authorization, or router configuration. Use them only on a trusted local
network. The example deployment assumes one robot.

## Running

Start the robot-facing node first:

```bash
export CYCLONEDDS_HOME="$HOME/Packages/cyclonedds/install"
uv run node.py \
  --node-config config/node-config.json5 \
  --zenoh-config config/zenoh-config.json5
```

Then start the pygame operator node on the operator PC:

```bash
uv run --group example python -m examples.keyboard \
  --keyboard-config examples/keyboard-config.json5 \
  --zenoh-config examples/keyboard-zenoh-config.json5
```

Only one active velocity-command publisher may control a robot. Do not run the
pygame controller and a high-level policy for the same `robot_key` at the same
time.

## Command contract

Publish JSON to `{robot_key}/command` with encoding `application/json`.

```json
{"type":"velocity","vx":0.5,"vy":0.0,"vyaw":0.2}
```

```json
{"type":"posture","posture":"stand"}
```

```json
{"type":"posture","posture":"down"}
```

`down` calls `StandDown()`—the prone/resting posture—not the SDK's dog-like
`Sit()` action.

### Official velocity envelope

The node validates `Move(vx, vy, vyaw)` against the ranges documented by
[Unitree's Go2 sport service](https://support.unitree.com/home/en/developer/sports_services):

| Field | Range | Unit |
| --- | ---: | --- |
| `vx` | `[-2.5, 3.8]` | m/s |
| `vy` | `[-1.0, 1.0]` | m/s |
| `vyaw` | `[-4.0, 4.0]` | rad/s |

These are validation limits, not recommended operating speeds. The keyboard
example defaults to conservative targets of `0.5 m/s`, `0.3 m/s`, and
`1.0 rad/s`.

Unitree documents `Move` as an unfiltered command maintained for one second.
This node deliberately forwards valid policy output without smoothing or
clipping. Publishers must filter their own output and publish velocity at 20 Hz.

## Safety state machine

The operational policy is to begin with the robot physically down. The process
does not claim that posture without telemetry: on startup it calls `StopMove()`,
publishes posture `unknown`, and ignores velocity until an explicit stand
sequence succeeds.

```text
unknown/down -- stand --> standing_up -- 3 s + BalanceStand --> standing
standing    -- down  --> standing_down -- StandDown ----------> down
```

- Stand: `StopMove()` → `StandUp()` → configured delay (3 s by default) →
  `BalanceStand()` → enable walking.
- Down: disable walking → `StopMove()` → `StandDown()`.
- Velocity is ignored unless posture is `standing` and `walking_enabled` is true.
- The monotonic 250 ms watchdog calls `StopMove()` once after the last forwarded
  velocity becomes stale. A fresh valid velocity command resumes forwarding;
  invalid messages do not refresh the watchdog.
- `dds.rpc_timeout_seconds` must not exceed the watchdog interval; the example
  uses 200 ms so a stalled synchronous SDK call is bounded.
- Graceful `SIGINT`/`SIGTERM` shutdown performs `StopMove()` → 1 s delay →
  `StandDown()` before closing Zenoh resources.
- A hard process, host, or power failure cannot guarantee the down sequence.

## State keys

The node publishes state changes with Zenoh `put` and declares an exact
Queryable for every key, so late clients can issue `get` requests.

| Key | Meaning |
| --- | --- |
| `{robot_key}/state/command/requested` | Last valid velocity received, even when motion was inhibited |
| `{robot_key}/state/command/applied` | Last velocity successfully forwarded to the SDK |
| `{robot_key}/state/posture` | `unknown`, `standing_up`, `standing`, `standing_down`, or `down` |
| `{robot_key}/state/health` | Node status and safety flags |

Velocity state example:

```json
{"vx":0.5,"vy":0.0,"vyaw":0.2,"active":true}
```

Health state example:

```json
{
  "status": "ready",
  "walking_enabled": true,
  "watchdog_triggered": false,
  "last_error": null
}
```

`applied` means that the SDK call returned success. It is not physical velocity
or posture feedback from robot telemetry.

## Pygame controls

Shift is the motion deadman and is also required for posture commands.

| Input | Action |
| --- | --- |
| `Shift+W` / `Shift+S` | Positive / negative `vx` |
| `Shift+A` / `Shift+D` | Positive / negative `vy` |
| `Shift+Q` / `Shift+E` | Positive / negative `vyaw` |
| `Shift+R` | Stand and enter walkable balance mode |
| `Shift+F` | Go down with `StandDown()` |
| `Space` | Publish zero velocity immediately |
| Release `Shift` | Publish zero velocity immediately |
| Window loses focus | Publish zero and require Shift release before re-arming |
| `Esc` or close window | Publish zero velocity and exit |

Multiple motion axes may be held simultaneously. The keyboard node ramps toward
its configured targets, publishes at 20 Hz, and displays requested/applied
velocity, posture, walking enablement, watchdog state, health, and deadman state.
Velocity uses best-effort/drop QoS. Stand/down uses a separate reliable/blocking
publisher on the same command key and retries until the reported target posture
is observed or the configured timeout is shown as an operator error.

## Development checks

These checks require no robot hardware:

```bash
uv run ruff format --check .
uv run ruff check .
uv run pyright
uv run python -m unittest discover -s tests -v
```

Passing them does not replace a controlled on-robot smoke test.

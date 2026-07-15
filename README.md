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
       - best-effort 250 ms watchdog
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
`unitree/go2`, but it may be changed to any concrete Zenoh key. The node
republishes all state every `state_heartbeat_seconds`; the keyboard marks state
stale after `state_stale_after_seconds` without a new heartbeat.

`dds.sport_mode_state_topic` selects the raw DDS `SportModeState` topic. The
default is `rt/sportmodestate`; some firmware layouts use an `lf`-prefixed
topic, so configure the topic actually published by the robot.
`dds.sport_mode_state_startup_timeout_seconds` bounds the initial wait for the
first sample before the startup safety decision. If it expires, startup keeps
the fail-safe behavior and calls `StopMove()` with motion state unavailable.
`safety.motion_state_max_age_seconds` is the maximum trusted local sample age.
`safety.balance_confirmation_timeout_seconds` bounds the wait for a new sample
after `BalanceStand()`. Missing, older, errored, or contradictory telemetry
never bypasses `StopMove()` or enables walking.

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

The operational policy is to begin with the robot physically down. The node
subscribes to Go2 `SportModeState` and recognizes an already-down robot only
when a fresh, error-free sample reports `mode=5` (`lieDown`), following the
[official Unitree mode mapping](https://github.com/unitreerobotics/unitree_ros2).
Otherwise startup calls `StopMove()`, publishes posture `unknown`, and ignores
velocity until an explicit stand sequence succeeds. A failed startup stop is
retried while the node remains non-walkable.

```text
unknown/down -- stand --> standing_up -- 3 s + BalanceStand + observed mode 1 --> standing
standing    -- down  --> standing_down -- StandDown --------------------------> down
```

- Stand: if fresh telemetry already confirms `lieDown`, call `StandUp()`
  directly; otherwise require `StopMove()` first. Then wait for the configured
  delay (3 s by default), call `BalanceStand()`, and wait for a newer, fresh
  `balanceStand` sample before enabling walking. Failure to confirm within
  `balance_confirmation_timeout_seconds` leaves walking disabled and requests
  a stop.
  An explicit stand request may also adopt a fresh `balanceStand` state created
  by the official app without issuing redundant posture calls.
- Down: disable walking → `StopMove()` → `StandDown()`. A failed stop remains a
  pending down transition and is retried; `StandDown()` is never issued before
  stop success is confirmed. A fresh observed `lieDown` completes the safety
  stop without issuing either redundant SDK call.
- An observed `lieDown` also reconciles an external app or remote change: the
  node resets applied velocity to zero, publishes posture `down`, and disables
  walking. During a node-initiated `StandUp()` transition, the preceding down
  sample is not allowed to cancel that same transition. RPC timeout and
  exception results are treated as uncertain: the old mode is accepted again
  only after expected transition progress, an explicit recovery barrier, or
  continuous fresh observations lasting longer than the transition window.
- Inferred posture is never enough to skip a stop. If the node previously
  inferred `down` but telemetry is contradictory, unknown, or stale, a new down
  request follows the conservative `StopMove()` path.
- Pending posture input is a desired state, not a replay queue. Only one target
  is dispatched per control-loop drain, and a pending `down` cannot be replaced
  by a later `stand` burst. A new deliberate `stand` may be sent after that
  `down` has been drained.
- Velocity is ignored unless posture is `standing`, `walking_enabled` is true,
  and fresh error-free motion telemetry reports `balanceStand` or `locomotion`.
  Missing, stale, errored, or non-walking-capable telemetry disables walking and
  requests `StopMove()`; a nonzero stop result remains pending and degraded.
- After the last forwarded velocity becomes stale, the monotonic watchdog calls
  `StopMove()`. A failed stop inhibits new velocity and is retried at a bounded
  interval until confirmed. `watchdog_triggered` refers only to this timeout,
  not to an ordinary SDK failure.
- The watchdog is a best-effort control-loop threshold, not a hard real-time
  deadline. SDK calls are synchronous and block the same thread.
  `dds.rpc_timeout_seconds` bounds each SDK request phase; one reply-bearing call
  may contain more than one such phase and may therefore delay watchdog service.
- Graceful `SIGINT`/`SIGTERM` shutdown exits without redundant motion calls when
  fresh telemetry confirms `lieDown`. Otherwise it attempts `StopMove()`, waits
  1 s, retries once after failure, and calls `StandDown()` only after a
  confirmed stop.
- A hard process, host, or power failure cannot guarantee the down sequence.

### Controlled hardware smoke test

With the robot supported and the remote stop available, record the dashboard's
observed motion mode and last error for this sequence: start the node while
down; `Shift+R`; send a small walk command; `Shift+F`; `Shift+R` again; then
`Shift+F` and terminate the node. The expected down state is `mode=5`, and each
stand must reach fresh `balance_stand` telemetry before `walking_enabled=true`.
If the motion state remains stale, verify `dds.sport_mode_state_topic` before
attempting motion.

## State keys

The node publishes state changes with Zenoh `put`, republishes a periodic state
heartbeat, and declares an exact Queryable for every key. Late clients can issue
`get` requests, and live subscribers can detect a stale node locally.

| Key | Meaning |
| --- | --- |
| `{robot_key}/state/command/requested` | Last valid velocity received, even when motion was inhibited |
| `{robot_key}/state/command/applied` | Current commanded-velocity estimate: last successful `Move`, reset after successful `StopMove` or observed `lieDown` |
| `{robot_key}/state/posture` | Controller posture: `unknown`, `standing_up`, `standing`, `standing_down`, or `down` |
| `{robot_key}/state/motion` | Latest observed Go2 mode, mode name, error code, and local freshness decision |
| `{robot_key}/state/health` | Node status and safety flags |

Velocity state example:

```json
{"vx":0.5,"vy":0.0,"vyaw":0.2}
```

Observed motion example:

```json
{"mode":5,"mode_name":"lie_down","error_code":0,"fresh":true}
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

`applied` is a command estimate derived from successful SDK return codes and is
also reset when telemetry confirms `lieDown`; it is not measured physical
velocity. `posture` is controller state, while `motion` is the separate robot
observation used to reconcile it.

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
| Window loses focus | Publish zero, cancel pending stand retries, and require Shift release before any motion or posture command re-arms |
| `Esc` or close window | Publish zero velocity and exit |

Multiple motion axes may be held simultaneously. The keyboard node ramps toward
its configured targets, publishes at 20 Hz, and displays requested/applied
velocity, controller posture, observed motion, walking enablement, watchdog
state, health, and deadman state.
Velocity uses best-effort/drop QoS. Stand/down uses a separate reliable/blocking
publisher on the same command key and retries until the reported target posture
is observed in a state sample newer than the request, or the configured timeout
is shown as an operator error. Losing focus, releasing Shift, or exiting cancels
pending `stand` retries; an already-requested safety `down` remains eligible for
retry while the keyboard node continues running.

## Development checks

These checks require no robot hardware:

```bash
uv run ruff format --check .
uv run ruff check .
uv run pyright
uv run python -m unittest discover -s tests -v
```

Passing them does not replace a controlled on-robot smoke test.

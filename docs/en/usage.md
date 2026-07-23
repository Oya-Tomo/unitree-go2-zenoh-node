# Usage

Complete [Setup](setup.md) and the physical safety preflight before using this guide.

## Start and stop

Export the CycloneDDS prefix in the shell that starts the robot node:

```console
$ export CYCLONEDDS_HOME="$HOME/Packages/cyclonedds/install"
$ uv run node.py
```

The node waits for a valid `SportModeState` before accepting commands. Start the keyboard in a second terminal only after the node is receiving State:

```console
$ uv run examples/keyboard.py
```

The keyboard sends zero velocity when Shift is released, its window loses focus, the window closes, or the program exits. Stop the keyboard before the robot node.

`Ctrl-C` or `SIGTERM` asks the robot node to run its State-driven Down workflow before closing DDS. Keep the remote ready: normal process shutdown is not an emergency stop and cannot guarantee motion cessation when communication or the SDK is unavailable.

## CLI options

### Robot node

| Option | Default | Description |
| --- | --- | --- |
| `--node-config FILE` | `config/node-config.json5` | Node, DDS, State, and control configuration |
| `--zenoh-config FILE` | `config/zenoh-config.json5` | Robot-node Zenoh configuration |

```console
$ uv run node.py --help
```

### Keyboard

| Option | Default | Description |
| --- | --- | --- |
| `--keyboard-config FILE` | `examples/keyboard-config.json5` | Keyboard targets, rates, and robot key |
| `--zenoh-config FILE` | `examples/keyboard-zenoh-config.json5` | Keyboard Zenoh configuration |

```console
$ uv run examples/keyboard.py --help
```

CLI options select complete files. Individual values cannot be overridden on the command line.

## Robot-node configuration

The complete tracked example is [`config/node-config.example.json5`](../../config/node-config.example.json5). Unknown fields, missing required fields, and incorrectly typed fields are rejected.

### Top-level settings

| Key | Type | Description |
| --- | --- | --- |
| `robot_key` | concrete Zenoh key | Root for this robot's command and State keys; wildcards are rejected |
| `state_heartbeat_seconds` | number greater than 0 | Idle heartbeat interval for public State; a synchronous SDK call can delay publication until the control thread returns |

The keyboard `robot_key` must match exactly.

### `dds` settings

| Key | Type | Description |
| --- | --- | --- |
| `domain_id` | integer, 0 or greater | DDS domain passed to the Unitree channel factory |
| `network_interface` | non-empty string | Dedicated wired interface name connected to the Go2 |
| `rpc_timeout_seconds` | number greater than 0 | Synchronous SportClient RPC timeout |
| `sport_mode_state_topic` | non-empty string | DDS `SportModeState` topic; default contract is `rt/sportmodestate` |

### `state` settings

| Key | Type | Description |
| --- | --- | --- |
| `startup_timeout_seconds` | number greater than 0 | Time allowed for the first valid `SportModeState` |
| `maximum_age_seconds` | number greater than 0 | DDS freshness watchdog deadline used by the control loop to close command ingress |
| `linear_velocity_quiescent_threshold` | number greater than 0 | Maximum absolute measured linear component considered quiescent |
| `yaw_speed_quiescent_threshold` | number greater than 0 | Maximum absolute measured yaw speed considered quiescent |

The quiescent thresholds do not determine whether a standing robot can accept velocity. They are used to confirm Down and to prevent `StandDown()` while measured motion remains above a threshold.

### `control` settings

| Key | Type | Description |
| --- | --- | --- |
| `velocity_deadman_seconds` | number greater than 0 | Maximum age of the latest velocity request; the next applicable State sends one zero `Move` and clears the request |
| `shutdown_timeout_seconds` | number greater than 0 | Maximum time for observed State to complete the shutdown Down workflow |

## Keyboard configuration

The complete tracked example is [`examples/keyboard-config.example.json5`](../../examples/keyboard-config.example.json5).

### General settings

| Key | Type | Description |
| --- | --- | --- |
| `robot_key` | concrete Zenoh key | Must match the robot node |
| `publish_frequency_hz` | number greater than 0 | Keyboard update, velocity publication, and display frequency |
| `state_stale_after_seconds` | number greater than 0 | Dashboard-only timeout for missing public State updates |

`state_stale_after_seconds` affects only the dashboard. It is not the node's DDS watchdog.

### `targets` settings

| Key | Type | Description |
| --- | --- | --- |
| `vx` | positive number, at most 2.5 | Absolute forward/backward keyboard target |
| `vy` | positive number, at most 1.0 | Absolute left/right keyboard target |
| `vyaw` | positive number, at most 4.0 | Absolute yaw-rate keyboard target |

The example targets are intentionally lower than the protocol limits.

### `ramp_rates` settings

| Key | Type | Description |
| --- | --- | --- |
| `vx` | number greater than 0 | Maximum forward/backward command change per second |
| `vy` | number greater than 0 | Maximum lateral command change per second |
| `vyaw` | number greater than 0 | Maximum yaw command change per second |

## Keyboard controls

Shift is the motion deadman and is also required when sending a posture request.

| Input | Action |
| --- | --- |
| `Shift+W` / `Shift+S` | Positive / negative `vx` |
| `Shift+A` / `Shift+D` | Positive / negative `vy` |
| `Shift+Q` / `Shift+E` | Positive / negative `vyaw` |
| `Shift+R` | Publish one Stand request |
| `Shift+F` | Publish one Down request |
| `Space` | Publish zero velocity |
| Release Shift | Publish zero velocity |
| Window loses focus | Publish zero velocity and require Shift release before rearming |
| `Esc` or close window | Publish zero velocity and exit |

The keyboard ramps local velocity toward the configured targets. It publishes the current velocity every update while running. It publishes posture only on the corresponding key-down event and does not retry posture requests.

## Zenoh keys and command contract

For `robot_key: "unitree/go2"`, the node uses:

| Key | Direction | Content |
| --- | --- | --- |
| `unitree/go2/command` | Keyboard/client to node | Velocity or posture JSON |
| `unitree/go2/state` | Node to observers | Canonical Node State JSON; also available through Zenoh `get` |

### Velocity command

```json
{"type":"velocity","vx":0.5,"vy":0.0,"vyaw":0.2}
```

| Field | Range |
| --- | --- |
| `vx` | -2.5 to 3.8 m/s |
| `vy` | -1.0 to 1.0 m/s |
| `vyaw` | -4.0 to 4.0 rad/s |

The keyboard publishes velocity with drop congestion control and best-effort reliability so newer motion input is favored over a backlog.

### Posture command

```json
{"type":"posture","posture":"stand"}
```

```json
{"type":"posture","posture":"down"}
```

The only posture values are `stand` and `down`. The keyboard publishes posture with block congestion control and reliable delivery. There is no separate Stop command; zero velocity is `Move(0, 0, 0)`, while `StopMove()` belongs only to the node's Down workflow.

Only the latest velocity and posture are retained. Posture takes priority. A command that is not legal in the Robot State that processes it is discarded, not delayed for a later State.

## Published State contract

The node publishes a revisioned JSON snapshot on `{robot_key}/state` and answers Zenoh `get` with its latest snapshot.

### Top-level fields

| Field | Description |
| --- | --- |
| `revision` | Node-local increasing publication revision |
| `lifecycle` | `starting`, `running`, `shutting_down`, or `stopped` |
| `connected` | Whether the DDS State freshness watchdog is currently satisfied |
| `robot` | Latest classified robot observation or Unknown State |
| `accepting_commands` | Single ingress gate for both velocity and posture |
| `requested_posture` | Latest buffered or active posture target, otherwise `null` |
| `requested_velocity` | Latest buffered velocity and local receive time, otherwise `null` |
| `posture_action` | Last posture SDK step sent for the active workflow, otherwise `null` |
| `last_sdk` | Latest SDK command diagnostic, otherwise `null` |
| `last_error` | Latest node-level input or shutdown error, otherwise `null` |

`requested_posture`, `posture_action`, and `last_sdk` describe requests and diagnostics. They are not robot posture evidence.

### `robot` fields

| Field | Description |
| --- | --- |
| `state` | `damping`, `down`, `locked_stand`, `ready_stand`, `locomotion`, `unsupported`, or `unknown` |
| `reason` | For Unknown: `no_sample`, `stale`, `invalid_sample`, or `awaiting_state`; otherwise `null` |
| `motion` | `moving`, `quiescent`, or `unknown` |
| `state_machine_code` / `state_machine_name` | Unitree V2.0 motion state machine identity |
| `mode` / `mode_name` | Coarse SportModeState mode |
| `velocity` | Observed three-component linear velocity, or `null` |
| `yaw_speed` | Observed yaw speed, or `null` |
| `received_at` | Node-local monotonic receive time, or `null` |
| `stamp_sec` / `stamp_nanosec` | Robot-provided State timestamp, or `null` |

### SDK diagnostic fields

`last_sdk` contains `command`, optional `velocity`, optional integer `code`, and optional `error`. A nonzero SDK code is reported but does not establish or advance Robot State. In particular, `StopMove()` returning `-1` remains a diagnostic; the next observed State decides what can happen.

## Robot State behavior

| Robot State | Observation | Stand | Down | Velocity |
| --- | --- | --- | --- | --- |
| Damping | state machine 1001, mode 0 | `RecoveryStand` when quiescent | Wait | Discard |
| Down | state machine 1004 or 2006, mode 5, quiescent | `StandUp` | Complete | Discard |
| Locked stand | state machine 1002, mode 0 | `BalanceStand` | Start Down workflow | Discard |
| Ready stand | state machine 100, mode 0/1; or 1013, mode 1 | Complete | Start Down workflow | `Move` |
| Locomotion | state machine 100/1013, mode 3 | `BalanceStand` | Start Down workflow | `Move` |
| Unsupported | Every other fresh combination | Discard new request | Discard new request | Discard |
| Unknown | No usable observed State | Reject | Reject | Reject |

Measured motion never changes a movement-capable Ready stand or Locomotion into another State. It is used only to confirm quiescent Crouch as Down and to wait for quiescence between `StopMove()` and `StandDown()`.

For the architectural decision and invariants, see [ADR-0001](../adr/0001-observed-state-driven-control-architecture.md).

## Timing and execution

These timeouts and frequencies have independent purposes:

| Setting or constant | Example value | Purpose |
| --- | --- | --- |
| DDS publisher frequency | Set by the Go2 | State arrives asynchronously; the node does not poll DDS at a configured rate |
| Control wait maximum | 0.05 s | Maximum sleep while waiting for State and checking lifecycle deadlines; State arrival wakes it immediately |
| `maximum_age_seconds` | 0.2 s | Close command ingress after no valid DDS State; when the control thread is available, detection occurs on its next wake-up, about 0.20–0.25 s with the current wait |
| `rpc_timeout_seconds` | 0.2 s | Maximum synchronous SportClient RPC wait |
| `velocity_deadman_seconds` | 0.25 s | On the next applicable State, expire a buffered velocity request and send one zero `Move` |
| `state_heartbeat_seconds` | 1.0 s | Schedule public State publication even without another public update |
| `state_stale_after_seconds` | 2.5 s | Keyboard dashboard display timeout only |

DDS reception is asynchronous. Its callback converts incoming State and wakes the single synchronous control loop, which retains the newest pending State. The loop classifies one State, checks posture before velocity, and sends at most one SDK call for that State.

SportClient RPCs are synchronous in the control thread. DDS reception continues while an RPC is running, but freshness enforcement, heartbeat publication, and command processing wait for the RPC to return; therefore the 0.2-second watchdog is not a hard scheduling guarantee while that thread is blocked. Before a posture RPC, the node publishes Robot State as Unknown and closes command ingress. Only a valid State received after that RPC returns can replace Unknown and continue the workflow.

## Down and shutdown

A Down request from a supported standing State follows this sequence:

```text
StopMove once -> newer State -> wait until quiescent
              -> StandDown once -> newer observed Down -> complete
```

SDK return values do not skip or complete a step. Damping does not confirm Down. If a new Stand request is accepted while a Down workflow is waiting in Damping, it replaces the Down target and may select `RecoveryStand()` from the next applicable State.

Shutdown closes normal command ingress and uses the same Down workflow. Only observed Down completes it. If the shutdown deadline expires first, the final public State records a node error before the process closes its subscriber.

## Troubleshooting

### `Timed out waiting for SportModeState`

- Confirm the Go2 is on and the dedicated wired interface is up.
- Confirm `dds.network_interface`, `domain_id`, and `sport_mode_state_topic`.
- Confirm CycloneDDS can use the selected interface.

The node exits with a nonzero status if no valid startup State arrives before `startup_timeout_seconds`.

### `connected` is false or `reason` is `stale`

No valid DDS State has arrived within `maximum_age_seconds`. New commands are rejected and buffered commands are cleared. Restore DDS State; do not infer the robot's posture from the last SDK result.

### Robot State is Unknown with `awaiting_state`

This is expected immediately around a posture RPC. Commands remain closed until a valid State received after the RPC returns.

### Robot State is Unsupported

The State sample is fresh, but its motion state machine and coarse mode are not in the supported table. Command ingress can remain open, but commands are discarded when processed in Unsupported. Inspect the raw state machine, mode, motion, and firmware version before changing any classification.

### Dashboard says waiting or stale

Confirm the robot node is running, both applications use the same `robot_key`, and their Zenoh configurations join the same network. Dashboard staleness is separate from DDS freshness shown by `connected`.

### `last_sdk.error` reports an SDK code or exception

Treat it as diagnostic information. Check the next observed Robot State and the physical robot. Do not repeatedly send posture commands based only on the return code.

## Manual real-hardware acceptance

Use low keyboard targets, support the robot, and record the complete public State and SDK diagnostics while checking:

1. Start with the robot physically Down and confirm State reception.
2. Send Stand and wait for an observed Ready stand.
3. Walk slowly in each configured direction and stop.
4. Send Down and verify one `StopMove()`, then `StandDown()` only after a newer quiescent State.
5. Send Stand again, including recovery from Damping if that is what the robot reports.
6. Stop the node and verify shutdown completes only from observed Down.

Stop immediately with the remote if physical behavior and published State do not agree.

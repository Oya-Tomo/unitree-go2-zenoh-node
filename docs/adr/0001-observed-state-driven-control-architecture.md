# ADR-0001: Use One Fresh-State Control Machine

- Status: Accepted
- Date: 2026-07-16
- Issue: #3

> Runtime coordination and periodic publication are superseded by [ADR-0002](0002-shared-node-state-concurrency.md).
> This ADR remains the authority for observed Robot State, command legality, and SDK workflows.

## Context

The Go2 must be controlled from observed robot State, not from SDK return codes or locally inferred posture.
Hardware testing established four facts:

- `StopMove()` may return `-1` without describing the robot's posture.
- the V2.0 DDS field named `error_code` carries a motion state machine ID, so `100` means Agile rather than an error;
- `1001` Damping does not prove that the robot is Down; and
- `100` Agile with coarse mode 0 remains the observable movement-capable State after `BalanceStand()`.

The previous design mixed DDS connection health, robot State classification, command acceptance, measured motion, and posture workflow progress.
A moving sample could therefore close velocity input even while DDS was healthy and the robot remained in Agile.

## Decision

The node will use one State-triggered control machine.
It will keep three independent facts independent:

1. **State freshness**: whether valid DDS State is still being received.
2. **Robot State**: which actions the latest State permits.
3. **Velocity freshness**: whether the latest velocity command is still live.

None of these is inferred from an SDK return code.

## Control loop

```text
DDS callback                    Zenoh callback
    |                                |
    v                                v
latest observation ───────► shared Node State ◄──── latest commands
                                  |
                                  v
                         control worker wakes
                                  |
                                  v
        classify -> posture -> velocity -> at most one SDK call
```

- DDS and Zenoh callbacks never call the SDK.
- One lock-protected Node State keeps the newest observation and commands.
- The control worker keeps its last processed observation time locally.
- Each control step classifies its selected State once.
- Posture is evaluated before velocity.
- One processed State produces at most one SDK call.

There is no executor queue, retry loop, stable-sample window, command ID, generation counter, public revision, or per-State command-acceptance gate.

## State freshness

A valid DDS sample records its local monotonic receive time.
State is fresh while the newest valid sample is less than `dds.state_freshness_seconds` old.
The target timeout is 0.2 seconds.

```text
valid State received within 0.2 s -> connected
no valid State for 0.2 s          -> disconnected
```

Disconnected State is `Unknown`.
The node then clears pending commands and rejects new commands.
A malformed sample does not refresh the deadline.
The next valid sample reconnects the node and drives the loop normally.

This timeout is unrelated to the SDK RPC timeout, velocity-command timeout, Node State publication frequency, or dashboard display timeout.

## Robot State

The classifier reads the V2.0 motion state machine ID and coarse mode.
Measured velocity is retained separately as `moving` or `quiescent`; it does not change a movement-capable State into a transition.

| Robot State | V2.0 observation |
| --- | --- |
| `Damping` | state machine 1001, mode 0 |
| `Down` | state machine 1004 or 2006, mode 5, quiescent |
| `LockedStand` | state machine 1002, mode 0 |
| `ReadyStand` | state machine 100 with mode 0 or 1; or 1013 with mode 1 |
| `Locomotion` | state machine 100 or 1013, mode 3 |
| `Unsupported` | every other fresh combination |
| `Unknown` | disconnected, invalid startup, or waiting after a posture RPC |

Agile remains `ReadyStand` while moving.
The first `Move()` must not require a prior Locomotion sample, because that command can be what produces Locomotion.

Measured motion has only two control uses:

- `Down` is confirmed only from a quiescent Crouch sample; and
- `StandDown()` is sent only after a post-`StopMove()` sample is quiescent.

It does not open or close command input.

The node targets Go2 Edu software V1.1.6 or later:

- [Unitree Motion Control Service Interface V2.0](https://support.unitree.com/home/en/developer/Motion_Services_Interface_V2.0)
- [Unitree ROS 2 SportModeState definition](https://github.com/unitreerobotics/unitree_ros2#1-sportmode-state)

## Command intake

There is one ingress gate.
Commands are accepted only while:

- State is fresh;
- the node is not waiting for State after a posture RPC; and
- the node is not shutting down.

Robot State does not otherwise toggle command intake.
Command legality is decided by the State machine when the command is processed.
A command that is not legal in the current State is discarded rather than deferred until some future State.

A new posture replaces the previous posture target.
A posture workflow already waiting for post-RPC State remains active until it completes or a newer valid posture replaces it.

## Command State machine

| Robot State | Stand | Down | Velocity |
| --- | --- | --- | --- |
| `Damping` | `RecoveryStand` when quiescent; otherwise wait | wait | discard |
| `Down` | `StandUp` | complete | discard |
| `LockedStand` | `BalanceStand` | begin Down workflow | discard |
| `ReadyStand` | complete | begin Down workflow | `Move` |
| `Locomotion` | `BalanceStand` | begin Down workflow | `Move` |
| `Unsupported` | discard new request | discard new request | discard |
| `Unknown` | reject | reject | reject |

`RecoveryStand()` is valid for Damping because the V2.0 contract says it recovers to standing regardless of whether the robot has fallen.
Damping still does not confirm Down.

### Posture RPC boundary

Immediately before `RecoveryStand`, `StandUp`, `BalanceStand`, `StopMove`, or `StandDown`, the node:

1. sets public robot State to `Unknown`;
2. closes command intake;
3. invokes the SDK once; and
4. waits for a valid State received after that RPC returns.

The next qualifying State alone advances the workflow.
The RPC return code and exception are diagnostics only.
The last posture action is retained only to prevent the same action from being resent for an unchanged State.

### Down workflow

```text
LockedStand / ReadyStand / Locomotion
    -> StopMove once
    -> Unknown
    -> newer State
       -> moving: wait
       -> quiescent supported stand: StandDown once
       -> Down: complete
    -> Unknown
    -> newer Down State: complete
```

`StopMove()` is used only here.
A `-1` result does not retry, fail, or advance the workflow.
Damping never completes Down or shutdown.

## Velocity freshness

Velocity-command timeout is independent of State freshness.
While DDS remains fresh:

- a current velocity in `ReadyStand` or `Locomotion` is sent with `Move()`;
- an expired velocity sends one `Move(0, 0, 0)` and is cleared; and
- expiry never changes Robot State or closes command intake.

Velocity received in any other Robot State is discarded and cannot execute later after a stand transition.

## Shutdown

Shutdown closes command intake, clears pending commands, and runs the same Down workflow.
Only an observed `Down` State completes shutdown.
If fresh State does not confirm Down before the shutdown timeout, the node exits with an error.

## Public State

The published `{zenoh_key_prefix}/state` snapshot contains:

- lifecycle and DDS connection freshness;
- raw V2.0 state machine, coarse mode, and measured motion;
- classified Robot State;
- one global `accepting_commands` value;
- active posture target and its last SDK action;
- latest velocity;
- last SDK diagnostic; and
- last node error.

Requested commands, observed State, and SDK diagnostics remain separate.
Periodic publication does not contain a revision counter or local monotonic receive time.
See ADR-0002 for the projection and publisher execution model.

## Files and responsibilities

```text
node.py        DDS/Zenoh adapters, control lifecycle, periodic publisher lifetime
controller.py shared Node State, classifier, State machine, SDK effect boundary
config.py      validated timeouts, thresholds, and endpoint configuration
```

The keyboard under `examples/` publishes only posture and velocity commands.
It does not own robot State or posture workflow.

## Invariants

1. Only fresh DDS State establishes physical State.
2. SDK results never establish or advance physical State.
3. Zenoh receipt alone never calls the SDK.
4. One processed State causes at most one SDK call.
5. Posture has priority over velocity.
6. Measured motion never disables movement-capable State or command intake.
7. Commands received while disconnected or awaiting post-RPC State never run after reconnection.
8. Velocity commands illegal in the current Robot State are discarded.
9. `StopMove()` occurs once per Down workflow and nowhere else.
10. Only a quiescent Crouch observation confirms Down.
11. Damping never confirms Down or shutdown completion.

## Test design

Tests follow the public State machine rather than private synchronization objects:

- table-test the observation-to-State classifier, including moving Agile;
- verify fresh DDS State accepts commands and a 0.2-second gap rejects them;
- table-test Stand, Down, and Velocity actions for every Robot State;
- verify each posture RPC waits for newer post-RPC State;
- verify Stop-then-Down ordering and one-shot behavior;
- verify latest-command replacement and posture priority;
- verify velocity-command timeout sends zero without changing command acceptance; and
- verify shutdown completes only from observed Down.

Tests for per-State ingress flags, command receive-order repair, transition windows, and private buffer interleavings are removed with those mechanisms.

## Consequences

The controller becomes a direct implementation of one small transition table.
DDS connection loss, robot action legality, and velocity expiry can be reasoned about independently.

The design deliberately reacts to one fresh State sample without debounce.
If future hardware evidence requires filtering, that filtering belongs at the DDS observation boundary and must not add another command state machine.

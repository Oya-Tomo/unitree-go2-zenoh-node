# ADR-0001: Drive Every SDK Decision from the Latest Robot State

- Status: Proposed
- Date: 2026-07-16
- Issue: #3

## Context

The previous controller accumulated inferred posture, walking flags, retry
states, dispatch permits, generation counters, and RPC-dependent branches.
Those mechanisms made the local process look authoritative even when the Go2
reported a different physical state.

Real-hardware testing exposed the mismatch. `StopMove()` can return `-1` while
the robot is physically down, and using that result to infer posture or to
retry indefinitely prevents a later stand request. The robot's
`SportModeState` must be the only physical-state authority.

The implementation must also remain small enough that command ordering and all
safety branches can be read directly from the control loop.

## Decision

The node uses one State-triggered control loop:

```text
DDS callback                  Zenoh callback
    |                              |
    v                              v
latest State inbox          latest command buffers
    |                              |
    +----------> control loop <----+
                    |
                    v
      read State -> inspect buffers -> branch -> one SDK call
```

Zenoh callbacks never call the SDK. They only replace one latest velocity and
one latest posture request. The State inbox also keeps only its newest
unprocessed sample. Each processed State can authorize at most one SDK call.

There is no command executor thread, dispatch gate, ingress identifier, source
epoch, node-instance identifier, stable-sample window, or RPC-based state
transition.

## Physical State

A fresh, parseable `SportModeState` sample is classified immediately. No
multi-sample confirmation is required.

The following fields are read from the robot:

- mode;
- error code;
- measured velocity and yaw speed; and
- robot timestamp.

Modes 0, 1, and 3 are classified as idle stand, ready stand, and locomotion.
Mode 5 is down only when measured motion is quiescent; a moving mode-5 sample is
an inconsistent transition and authorizes no command. Modes 2 and 8 are also
transitions. Other modes are unsupported. Motion is derived from measured
velocity thresholds, not from the last `Move()` call.

Missing, malformed, or stale State is `Unknown` and authorizes no SDK call.
SDK return codes and exceptions are diagnostics only.

## Command buffering and priority

The buffers have these semantics:

- velocity replaces the previous velocity;
- posture replaces the previous posture;
- posture is evaluated before velocity;
- a pending posture prevents velocity dispatch; and
- no FIFO command history is retained.

At startup, and after a posture-related SDK call, command input is disabled.
It is enabled again only after a newer valid robot State has been processed.
Commands arriving while disabled are ignored rather than deferred.

A command received after the loop inspected its buffer waits for the next
State. This is the loop's ordering boundary; a later command does not cancel an
SDK call that has already begun.

## Posture workflows

Posture progress is command metadata only. It prevents duplicate SDK calls but
never represents the physical posture.

### Stand

```text
State down       -> StandUp   -> Unknown -> wait for newer State
State idle stand -> BalanceStand -> Unknown -> wait for newer State
State ready      -> request complete
```

After `StandUp()`, a repeated down State does not resend it. After
`BalanceStand()`, the node waits for ready State without retrying from an RPC
result.

### Down

```text
State quiescent down -> request complete, no StopMove
other safe State   -> StopMove once -> Unknown -> wait for newer State
quiescent standing -> StandDown     -> Unknown -> wait for newer State
State down         -> request complete
```

`StopMove()` is used only as the single command immediately preceding a down
transition. It is not used at startup, for stand, for zero velocity, or as a
generic watchdog action. If the State after `StopMove()` still reports motion,
the node waits; it neither sends `StandDown()` nor repeats `StopMove()`.

The `StopMove()` result, including `-1`, does not change this workflow. Only the
next robot State does.

## Velocity

Velocity is dispatched only from ready-stand or locomotion State and only when
no posture request is pending. The newest velocity is used. Zero and deadman
expiry use `Move(0, 0, 0)`; they do not use `StopMove()`.

The deadman is checked when a State is processed. If the latest velocity has
expired, one zero `Move()` is sent and that buffered velocity is cleared.
Complete telemetry loss cannot be made safe from this process; robot-side
safety remains necessary.

## Shutdown

Shutdown disables and clears normal command input, then uses the same down
workflow:

```text
fresh non-down State -> StopMove once -> newer quiescent State
                     -> StandDown -> newer down State -> exit
fresh quiescent down State -> exit without redundant SDK calls
Unknown State        -> wait until shutdown timeout, then exit with an error
```

## Public State

The node publishes one `{robot_key}/state` snapshot containing only:

- lifecycle;
- robot-derived State;
- whether commands are currently accepted;
- latest requested posture and velocity;
- the small posture workflow phase;
- last SDK diagnostic; and
- last error.

Immediately before invoking `StandUp`, `BalanceStand`, `StopMove`, or
`StandDown`, the node publishes robot State as `Unknown`. It remains Unknown
during the synchronous RPC. A sample received before that SDK call began cannot
clear the Unknown state even if it remained queued locally.

## Files and responsibilities

The production implementation is intentionally limited to three Python files:

```text
node.py        process lifecycle, DDS/Zenoh adapters, State-triggered loop
controller.py State classification, latest buffers, policy, SDK serialization
config.py      validated node and keyboard configuration
```

The keyboard UI remains under `examples/`. It publishes only the existing
velocity and posture wire commands. It does not define an explicit stop command.

## Invariants

1. Only robot State establishes physical posture or motion.
2. Zenoh receipt alone never calls the SDK.
3. One processed State produces at most one SDK call.
4. Posture has priority over velocity.
5. At most one latest velocity and one latest posture are retained.
6. A posture SDK call makes physical State `Unknown` immediately.
7. No command is accepted until a newer valid State clears that Unknown state.
8. RPC results never complete or advance a posture workflow.
9. `StopMove()` occurs at most once per down request and nowhere else.
10. `StandDown()` requires newer, quiescent, non-down State after `StopMove()`.
11. Shutdown uses Stop-then-Down and does not stop an already-down robot.

## Test design

Tests cover behavior rather than private synchronization machinery:

- one-sample classification and motion thresholds;
- moving mode 5 remaining non-actionable;
- down-to-stand State sequence;
- moving-to-down sequence with `StopMove() == -1`;
- no repeated StopMove while State still reports motion;
- latest velocity and latest posture replacement;
- posture priority over velocity;
- command rejection during post-SDK Unknown;
- Unknown publication before a blocking posture RPC;
- rejection of pre-command State evidence;
- zero and deadman behavior without StopMove;
- stale State; and
- shutdown from standing and already-down State.

Tests for dispatch permits, source epochs, stable windows, instance retirement,
executor interleavings, and the removed explicit stop wire command are deleted
because those mechanisms are not part of this architecture.

## Consequences

The control path is smaller and its ordering is visible in one loop. A slow SDK
call blocks command decisions, while DDS and Zenoh callbacks continue updating
their latest-value buffers. This is accepted because SDK calls have a configured
transport timeout and no queued SDK command exists behind the call.

Immediate single-sample classification reacts faster but provides no debounce.
If hardware traces later prove that a specific mode field flickers, that
evidence should motivate a narrowly scoped classifier change rather than a
general synchronization framework.

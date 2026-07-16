# ADR-0001: Drive Every SDK Decision from the Latest Robot State

- Status: Accepted
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

A second real-hardware trace exposed a versioned field-semantics error. On Go2
software V1.1.6 and later, the Motion Control Service Interface V2.0 uses the
DDS field named `error_code` to publish the current motion state machine ID.
The observed value `100` means Agile; it is not an error. Treating every
nonzero value as a fault left a confirmed idle State in lifecycle `starting`
and disabled all commands.

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
multi-sample confirmation is required, but standing and down classes require
that sample's measured motion to be quiescent.

The following fields are read from the robot:

- V2.0 motion state machine ID, carried in the SDK field named `error_code`;
- coarse sport mode;
- measured velocity and yaw speed; and
- robot timestamp.

The node targets the V2.0 interface. It assigns physical classes only to the
state machines used by this controller:

| State machine ID | State machine | Coarse mode | Physical class |
| --- | --- | --- | --- |
| 100 | Agile | 0 and quiescent | idle stand |
| 100 | Agile | 1 and quiescent | ready stand |
| 100 | Agile | 3 | locomotion |
| 1002 | Standing Lock | 0 and quiescent | idle stand |
| 1013 | Balance Standing | 1 and quiescent | ready stand |
| 1013 | Balance Standing | 3 | locomotion |
| 1004 or 2006 | Crouch | 5 and quiescent | down |

An inconsistent combination is a transition. A moving standing or Crouch sample
cannot confirm a posture. Other state machines, including damping and special
actions, are unsupported and authorize no command. Motion is derived from
measured velocity thresholds, not from the last `Move()` call.

The source field name `error_code` is retained only at the DDS adapter. The
controller and public State call it `state_machine_code` and preserve its exact
value. SDK RPC return codes such as 3104, 4101, 4201, 4205, and 4206 are a
separate diagnostic channel.

Missing, malformed, or stale State is `Unknown` and authorizes no SDK call.
SDK return codes and exceptions are diagnostics only.

References:

- [Unitree Motion Control Service Interface V2.0](https://support.unitree.com/home/en/developer/Motion_Services_Interface_V2.0)
- [Unitree ROS 2 SportModeState definition](https://github.com/unitreerobotics/unitree_ros2#1-sportmode-state)

The message does not carry an interface-version discriminator. Deployment on a
Go2 Edu running V1.1.6 or later is therefore a precondition rather than
something the node can prove at runtime. An older interface that reports an
actual error whose numeric value collides with an allowed V2.0 state machine ID
could be misclassified. The node fails closed for other values; this numeric
collision remains an accepted deployment risk until Unitree exposes a reliable
runtime discriminator.

## Command buffering and priority

The buffers have these semantics:

- velocity replaces the previous velocity;
- posture replaces the previous posture;
- posture is evaluated before velocity;
- a pending posture prevents velocity dispatch; and
- no FIFO command history is retained.

At startup, and after a posture-related SDK call, command input is disabled.
A newer valid robot State replaces `Unknown`; command input is enabled only
when that State is actionable. Commands arriving while disabled are ignored
rather than deferred. Lifecycle becomes `running` after any fresh, parseable
State; State actionability and runtime command acceptance remain separate
properties.

A command received after the loop inspected its buffer waits for the next
State. This is the loop's ordering boundary; a later command does not cancel an
SDK call that has already begun.

## Posture workflows

Posture progress is command metadata only. It prevents duplicate SDK calls but
never represents the physical posture.

### Stand

```text
State down       -> StandUp   -> Unknown -> wait for newer State
State quiescent idle stand -> BalanceStand -> Unknown -> wait for newer State
State quiescent ready      -> request complete
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
- robot-derived State, including motion state machine and coarse mode;
- whether commands are currently accepted;
- latest requested posture and velocity;
- the small posture workflow phase;
- last SDK diagnostic; and
- last error.

Immediately before invoking `StandUp`, `BalanceStand`, `StopMove`, or
`StandDown`, the node publishes robot State as `Unknown`. It remains Unknown
during the synchronous RPC. The post-command receive-time cutoff is recorded
when the RPC returns, so only a later sample can replace the Unknown state.

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
2. The V2.0 motion state machine ID is never interpreted as an RPC error.
3. Zenoh receipt alone never calls the SDK.
4. One processed State produces at most one SDK call.
5. Posture has priority over velocity.
6. At most one latest velocity and one latest posture are retained.
7. A posture SDK call makes physical State `Unknown` before and during the RPC.
8. Only a valid State received after the RPC replaces that Unknown state, and
   command input resumes only if the replacement State is actionable.
9. RPC results never complete or advance a posture workflow.
10. `StopMove()` occurs at most once per down request and nowhere else.
11. `StandDown()` requires newer, quiescent, non-down State after `StopMove()`.
12. Shutdown uses Stop-then-Down and does not stop an already-down robot.

## Test design

Tests cover behavior rather than private synchronization machinery:

- table-driven state-machine, coarse-mode, and motion classification;
- the real-hardware `100` (Agile), mode-0 startup State;
- V2.0 Standing Lock, Balance Standing, and Crouch classification;
- unsupported state machines remaining non-actionable without holding lifecycle
  in `starting`;
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

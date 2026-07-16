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

A subsequent hardware trace showed the robot entering `1001` Damping after
`StandDown()` returned zero. The robot was physically low and still, and its
joints were compliant with light damping resistance. Damping does not encode a
physical posture, so that observation cannot make every Damping State mean
Down. Treating Damping as completely unsupported was also wrong: it prevented
a newer Stand request from entering the buffer even though Unitree defines
`RecoveryStand()` for recovery from a fallen or crouched robot and explicitly
states that it recovers to standing regardless of whether the robot has fallen.

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
| 1001 | Damping | 0 | damping, with measured motion retained |
| 1002 | Standing Lock | 0 and quiescent | idle stand |
| 1013 | Balance Standing | 1 and quiescent | ready stand |
| 1013 | Balance Standing | 3 | locomotion |
| 1004 or 2006 | Crouch | 5 and quiescent | down |

An inconsistent combination is a transition. A moving standing or Crouch sample
cannot confirm a posture. Damping remains a separate physical class rather than
being relabeled as Crouch or Down; neither its moving nor quiescent form proves
posture. Other state machines, including special actions, are unsupported.
Motion is derived from measured velocity thresholds, not from the last
`Move()` call.

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

The command buffer and active posture workflow have these semantics:

- velocity replaces the previous velocity;
- the latest unprocessed posture replaces the previous unprocessed posture;
- on State processing, that posture becomes the controller's active target;
- an active target persists until completion or replacement by a newer posture;
- posture is evaluated before velocity;
- a pending posture prevents velocity dispatch; and
- no FIFO command history is retained.

At startup, and after a posture-related SDK call, both command types are
disabled. A newer valid robot State replaces `Unknown`, then command-type
eligibility is derived from its physical class:

| Physical State | Posture input | Velocity input |
| --- | --- | --- |
| Damping, Down, idle stand | accept | reject |
| ready stand, locomotion | accept | accept |
| Transition, Unsupported, Unknown | reject | reject |

Commands arriving while their type is disabled are ignored rather than
deferred. Lifecycle becomes `running` after any fresh, parseable State, even
when that State accepts neither command type. Input acceptance means only that
a request may replace its buffer; the posture and velocity policies below still
decide whether the current State authorizes an SDK call.

DDS and Zenoh callbacks timestamp observations and commands with the same local
monotonic clock. A State can reach the DDS inbox just before a command callback
sees the previous acceptance flags. When the control loop applies that State's
acceptance, it atomically discards any now-ineligible command whose receive time
is at or after the State receive time. The active posture workflow is separate
from that unprocessed input slot, so discarding an invalid replacement does not
cancel the active workflow. This preserves callback receive order without
identifiers, restoration logic, or queued command history.

A command received after the loop inspected its buffer waits for the next
State. This is the loop's ordering boundary; a later command does not cancel an
SDK call that has already begun.

## Posture workflows

Posture progress is command metadata only. It prevents duplicate SDK calls but
never represents the physical posture.

### Stand

```text
State quiescent damping -> RecoveryStand -> Unknown -> wait for newer State
State down              -> StandUp       -> Unknown -> wait for newer State
State idle stand        -> BalanceStand  -> Unknown -> wait for newer State
State ready stand       -> request complete
State moving damping    -> wait
```

`RecoveryStand()` is selected from the observed Damping class, not from an
inferred posture; the V2.0 contract makes it applicable regardless of fallen
posture. After a Stand RPC, the same physical class does not resend the same
action. If newer State reports a different supported class, that State selects
its corresponding Stand action. RPC return values select neither path. A newer
Stand request can replace a pending Down request while Damping is observed.

### Down

```text
State down -> request complete, no StopMove
idle stand, ready stand, or locomotion
           -> StopMove once -> Unknown -> wait for newer State
any supported standing class, now quiescent
           -> StandDown     -> Unknown -> wait for newer State
Damping, Transition, Unsupported -> wait
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
fresh idle/ready/locomotion State
                -> StopMove once -> newer quiescent supported standing State
                -> StandDown -> newer down State -> exit
fresh down State -> exit without redundant SDK calls
Damping State    -> wait; it does not prove Down
Unknown State    -> wait until shutdown timeout, then exit with an error
```

## Public State

The node publishes one `{robot_key}/state` snapshot containing only:

- lifecycle;
- robot-derived State, including motion state machine and coarse mode;
- separate posture- and velocity-input acceptance flags;
- latest requested posture and velocity;
- the small posture workflow phase;
- last SDK diagnostic; and
- last error.

Immediately before invoking `RecoveryStand`, `StandUp`, `BalanceStand`,
`StopMove`, or `StandDown`, the node publishes robot State as `Unknown`. It
remains Unknown during the synchronous RPC. The post-command receive-time
cutoff is recorded when the RPC returns, so only a later sample can replace the
Unknown state.

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
6. The command buffer retains at most one latest velocity and one latest
   unprocessed posture; the controller owns at most one active posture target.
7. A posture SDK call makes physical State `Unknown` before and during the RPC.
8. Only a valid State received after the RPC replaces that Unknown state;
   posture and velocity input then resume independently according to that
   State.
9. RPC results never complete or advance a posture workflow.
10. `StopMove()` occurs at most once per down request and nowhere else.
11. `StandDown()` requires newer, quiescent idle/ready/locomotion State after
    `StopMove()`.
12. Damping never confirms Down or shutdown completion.
13. Shutdown uses Stop-then-Down and does not stop an already-down robot.
14. A command received at or after an ineligible State cannot be delayed until
    a later eligible State because the stale-acceptance command is discarded.

## Test design

Tests cover behavior rather than private synchronization machinery:

- table-driven state-machine, coarse-mode, and motion classification;
- the real-hardware `100` (Agile), mode-0 startup State;
- the real-hardware quiescent `1001` (Damping) State after `StandDown()` and a
  subsequent Stand request using `RecoveryStand()`;
- V2.0 Standing Lock, Balance Standing, and Crouch classification;
- unsupported state machines rejecting input without holding lifecycle in
  `starting`;
- moving mode 5 dispatching no SDK command;
- down-to-stand State sequence;
- moving-to-down sequence with `StopMove() == -1`;
- no repeated StopMove while State still reports motion;
- latest velocity and latest posture replacement;
- callback receive-order enforcement at an ineligible State boundary;
- rejected posture replacement preserving the active posture workflow;
- posture priority over velocity;
- command rejection during post-SDK Unknown;
- Unknown publication before a blocking posture RPC;
- rejection of pre-command State evidence;
- zero and deadman behavior without StopMove;
- stale State; and
- shutdown from standing and already-down State; and
- Damping not being inferred as Down during shutdown.

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

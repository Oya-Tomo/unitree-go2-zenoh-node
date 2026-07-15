# ADR-0001: State-driven Go2 control architecture

- Status: Proposed
- Date: 2026-07-15
- Related: [Issue #3](https://github.com/oyatomo/unitree-go2-zenoh-node/issues/3)

## Context

The current controller stores posture, motion telemetry, walking enablement,
pending stop reasons, observation gates, deadlines, and SDK outcomes as partly
independent mutable state. Several paths update posture or action availability
from the command that was sent or from its RPC return code before the Go2 has
reported the resulting State.

Examples include setting an internal posture to `STANDING_UP` or
`STANDING_DOWN` when a command is sent, setting it to `DOWN` after a successful
`StandDown()` RPC, and changing `walking_enabled` from `BalanceStand()` outcome
handling. Startup, watchdog, normal down, and shutdown also implement different
StopMove rules.

Issue #3 added more outcome categories, pending state, and observation gates to
handle `StopMove()` returning SDK code `-1`. That was useful for exposing the
hardware behaviour, but it extends the wrong model: command history and RPC
results still participate in estimating physical state.

This ADR supersedes that implementation direction. The new architecture is not
required to preserve Issue #3's internal classes, branches, or tests.

## Decision

The controller shall be driven by `SportModeState` observations:

1. Only fresh, valid Go2 State samples may establish a non-`Unknown` physical
   state.
2. A discrete state-changing command invalidates the previously confirmed
   state. The state remains `Unknown` until a State sample received after the
   command start is classified.
3. SDK return values are diagnostics only. They never establish posture,
   stopping, State-confirmed command completion, or a policy decision. RPC
   completion may only update command-channel availability.
4. Missing, stale, invalid, or anomalous State disables every new SDK command.
   There is no best-effort StopMove exception in this ADR.
5. One-shot posture/stop requests and streaming velocity setpoints are separate
   concepts with separate lifetimes.
6. There is at most one unbuffered SDK call in flight. A command is never queued
   behind another call for later execution against an old State decision.
7. State classification and command selection each have one canonical, total
   decision table.
8. The node publishes one canonical snapshot. Publication never mutates control
   state.

`Unknown` does not violate rule 1. It is not a new inferred posture; it is the
loss of confirmation. Non-State events may invalidate evidence and therefore
produce `Unknown`, but only State can establish `Down`, `StandingIdle`,
`StandingReady`, `Moving`, or an observed transition.

## Source of physical truth

Unitree's State definition includes a robot timestamp, `error_code`, `mode`,
`progress`, `gait_type`, `position`, `velocity[3]`, `yaw_speed`, and foot data.
The implementation must retain every field used by classification instead of
reducing telemetry to only `mode` and `error_code`.

References:

- [Unitree ROS 2 SportModeState documentation](https://github.com/unitreerobotics/unitree_ros2#1-sportmode-state)
- [Unitree SDK2 Go2 SportClient API](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/go2/sport/sport_client.hpp)
- [Vendored `SportModeState` binding](../../third_party/unitree_sdk2_python/unitree_sdk2py/idl/unitree_go/msg/dds_/_SportModeState_.py)

The local monotonic receive time and receive sequence are authoritative for
freshness and local ordering. The robot timestamp is retained as diagnostic
evidence until real-device traces prove its reset, wrap, and monotonicity
semantics. It is not initially used as the sole observation barrier.

Because `SportModeState` does not provide a sufficient controller identity, the
configured DDS domain, network interface, and topic must resolve to exactly one
target Go2. A source restart or timestamp epoch change clears the observation
window, requests, and dispatch permits and requires a new stable window.

## Canonical State projection

`classify(samples, barrier, now)` is a total function over a small, fixed-size
observation window. It derives orthogonal facts once:

```text
ConfirmedRobotState
  validity: CONFIRMED | UNKNOWN
  unknown_reason: NO_SAMPLE | STALE | INVALID_ORDER | AWAITING_STATE | None
  mode_class: DOWN | IDLE_STAND | READY_STAND | LOCOMOTION | TRANSITION | UNSUPPORTED | UNKNOWN
  motion: QUIESCENT | MOVING | UNKNOWN
  fault_code: int | None
```

This avoids losing facts such as `fault_code != 0` together with observed
non-zero motion. A fault never authorizes a command, but the snapshot still
reports all State-derived facts.

Classification is evaluated in this order:

1. A dispatch barrier, no sample, stale sample, malformed ordering, or an
   invalid sample window makes validity `UNKNOWN` and mode/motion `UNKNOWN`.
2. For confirmed evidence, derive `fault_code` directly from the latest State.
3. Derive motion from mode, velocity, and yaw over the stable sample window.
4. Derive mode class from mode and stability:
   - stable mode 5 plus quiescent motion is `DOWN`;
   - every other mode 5 observation is `TRANSITION`;
   - stable mode 0 is `IDLE_STAND`; unstable mode 0 is `TRANSITION`;
   - stable mode 1 is `READY_STAND`; unstable mode 1 is `TRANSITION`;
   - mode 3 is `LOCOMOTION`;
   - documented transition modes such as 2 and 8 are `TRANSITION`;
   - other modes are `UNSUPPORTED`.
5. Derive one mutually exclusive policy condition in this exact order:

| Priority | Condition | Required facts |
|---:|---|---|
| 1 | `FAULTED` | any non-zero fault code |
| 2 | `UNKNOWN` | validity is unknown |
| 3 | `TRANSITION` | transition mode class, regardless of motion |
| 4 | `UNSUPPORTED` | unsupported mode class, regardless of motion |
| 5 | `DOWN` | down mode class and quiescent motion |
| 6 | `LOCOMOTION_MOVING` | locomotion mode class and moving motion |
| 7 | `LOCOMOTION_QUIESCENT` | locomotion mode class and quiescent motion |
| 8 | `STANDING_MOVING` | idle/ready stand with observed moving motion |
| 9 | `STANDING_IDLE` | idle stand and quiescent motion |
| 10 | `STANDING_READY` | ready stand and quiescent motion |

Transition and down-mode instability never fall through to `LOCOMOTION` or
`STANDING_MOVING`, even if their observed motion is non-zero. They permit no
command.

The exact window length, gap/reset rule, freshness timeout, and velocity/yaw
thresholds must come from real-device traces. Duplicate samples, unacceptable
gaps, and timestamp/source resets clear the window. A new source epoch requires
the configured number of consecutive valid samples.

One classifier owns these rules; action handlers never re-interpret raw modes.
`STANDING_UP` and `STANDING_DOWN` do not exist as physical states. A command by
itself produces `Unknown`; only a reported transition mode produces
`TRANSITION`.

There is no separate capability list. The action-request table and velocity
rule below are the only policy surfaces. This avoids mixing operator requests
with internal SDK command names.

## Requests and targets

### Separate request types

The control owner stores these independently:

```text
ActionRequest
  id
  kind: STAND | DOWN | STOP
  received_at
  deadline

VelocitySetpoint
  vx, vy, vyaw
  received_at
  deadman_deadline
```

An action request is a finite workflow. A velocity setpoint is latest-value
stream data. A velocity event never replaces an action request, and an old
velocity setpoint is never replayed after an action finishes.

The single priority order is:

```text
telemetry unavailable
  > shutdown
  > already-started SDK call
  > Stop/deadman
  > Stand or Down request
  > velocity setpoint
```

Accepting `Stand`, `Down`, or `Stop` clears the velocity setpoint. Shutdown
clears both. Repeating the same action request is idempotent and does not extend
its deadline. Stop preempts a posture request. A later Stand or Down request may
replace the desired posture target, but cannot cancel or overlap an SDK call
already in flight; it is re-evaluated from the next confirmed State. A
dispatching-but-not-started command is cancelled by telemetry loss or shutdown.

### Request targets

| Request | State-confirmed target |
|---|---|
| `STAND` | stable `STANDING_READY` |
| `DOWN` | stable `DOWN` |
| `STOP` | any stable quiescent supported State, including quiescent mode 3 |

### Total action decision table

The table is evaluated only when State is confirmed, the command channel is
ready, and no command target is unresolved.

| Confirmed state | `STAND` | `DOWN` | `STOP` |
|---|---|---|---|
| `DOWN` | `StandUp` | complete request | complete request |
| `STANDING_IDLE` | `BalanceStand` | `StandDown` | complete request |
| `STANDING_READY` | complete request | `StandDown` | complete request |
| `LOCOMOTION_MOVING` / `STANDING_MOVING` | `StopMove` | `StopMove` | `StopMove` |
| `LOCOMOTION_QUIESCENT` | wait for State or request deadline | wait for State or request deadline | complete request |
| `UNKNOWN`, `FAULTED`, transition, unsupported | reject and clear request | reject and clear request | reject and clear request |

After `StopMove` reaches its command target, the still-active `STAND` or `DOWN`
request is re-evaluated against the newly confirmed State. A StopMove RPC result
cannot advance the workflow.

Quiescent mode 3 is explicitly a wait state for Stand and Down. The controller
does not repeat StopMove merely because mode remains 3; it waits for a later
State change or fails the request at its deadline.

### Velocity decision

A non-zero `Move` may be sent only when all of the following are true:

- State is freshly confirmed as `STANDING_READY`, `LOCOMOTION_MOVING`,
  `LOCOMOTION_QUIESCENT`, or `STANDING_MOVING`;
- the velocity setpoint is within its deadman deadline;
- no action request, shutdown, observation barrier, or unresolved command
  target exists;
- the unbuffered executor is idle; and
- the node is the exclusive command controller for the robot.

`StateReceived` is the only event that may trigger a non-zero Move. One
observation sequence authorizes at most one Move, so a setpoint cannot produce a
command stream after State delivery stops. Whether the measured State rate is
sufficient for the required Move refresh rate must be verified on hardware.

The command value is published as `commanded_velocity`, never as measured
velocity. `observed_velocity` is derived only from State.

When a zero setpoint or deadman expiry is processed while fresh State is
`LOCOMOTION_MOVING` or `STANDING_MOVING`, it creates a `STOP` action request.
If State is already quiescent, no SDK command is needed. If State has become
stale, no command is permitted.

## Commands and observation barriers

Each discrete command has a State-only command target:

| SDK command | Command target |
|---|---|
| `StandUp` | post-command stable `STANDING_IDLE` or `STANDING_READY` |
| `BalanceStand` | post-command stable `STANDING_READY` |
| `StopMove` | post-command stable quiescent State |
| `StandDown` | post-command stable `DOWN` |

Command completion and request completion are separate. For example, a
`StandUp` command may complete at `STANDING_IDLE`, while the `STAND` request
remains active and next requires `BalanceStand`.

Transport synchronization and physical target confirmation use separate data:

```text
SdkInvocation                         # every SDK call, including Move
  id
  command
  permit
  started_at

DispatchPermit
  state: ACTIVE | CLAIMED | CANCELLED
  authorization_generation
  evidence_sequence
  fresh_until
  lifecycle_generation

DiscreteCommandAttempt                # never used for Move
  invocation_id
  dispatch_state: DISPATCHING | STARTED
  started_at
  target_deadline
  target_status: PENDING | SATISFIED | EXPIRED

CommandChannelState
  status: READY | RESERVED | BUSY | FAILED
  active_invocation_id
```

The command boundary is unbuffered:

1. `ControlLoop` decides at most one command from the current confirmed State.
2. It creates an `SdkInvocation` with a short-lived permit. A discrete command
   also creates a `DISPATCHING` attempt, which invalidates confirmation before
   handoff. Move creates no discrete attempt and no posture barrier. Handoff
   changes the channel from `READY` to `RESERVED`, preventing a second dispatch.
3. `StateSource`, `ControlLoop`, and the executor share one dispatch gate. State
   ingress high-water updates, processed high-water updates, authorization and
   lifecycle generation changes, cancellation, and claim are ordered by that
   gate; they are not separate unsynchronised reads.
4. Immediately before the SDK call, the executor performs one atomic
   check-and-claim. In that operation it verifies matching ingress/processed
   high-water marks, freshness, generations, channel reservation, and permit
   state, then changes `ACTIVE -> CLAIMED` and records `started_at`.
5. The successful claim is the SDK invocation linearization point. If State
   ingress wins the gate first, claim fails until `ControlLoop` processes it. If
   claim wins first, the later State is post-start evidence.
6. Claim and cancellation are mutually exclusive:
   - `ControlLoop` may change `ACTIVE -> CANCELLED` when authorization is lost;
   - exactly one of claim or cancellation wins;
   - a `CLAIMED` invocation cannot be cancelled.
7. Authorization generation changes only when the same command is no longer
   allowed: State becomes unknown/faulted/transition/unsupported, the relevant
   policy condition changes, request or lifecycle changes, freshness expires,
   or channel ownership fails. An equivalent newer State does not cancel the
   permit, preventing State-rate starvation.
   While a discrete attempt is `DISPATCHING`, public State remains `Unknown`,
   but permit revalidation applies the same classifier to the underlying new
   observations without the dispatch barrier solely to decide whether the
   original command is still authorized. That private revalidation cannot
   publish a non-Unknown State or trigger another command.
8. If `ControlLoop` wins cancellation, it removes the invocation, discrete
   attempt, and dispatch barrier and returns the channel to `READY` in the same
   event. The cancelling State may then be classified, but that event does not
   redispatch; the next eligible event must make a new decision.
9. If the executor observes cancellation or permit expiry before claim, it
   emits `CommandNotStarted`. Handling that event performs the same idempotent
   cleanup and is never dispatch-eligible.
10. If the executor wins claim, it emits `CommandStarted(id, started_at)` and
    the channel changes from `RESERVED` to `BUSY`.
11. For a discrete command, only State received after `started_at` can clear the
   barrier. Stable target State sets target status to `SATISFIED` and may
   complete the action request immediately, independently of RPC return.
12. RPC completion changes only `CommandChannelState` and `RpcDiagnostic`.
    A later command requires a resolved discrete target and a `READY` channel
    and is selected only on a subsequent `StateReceived` event.

If the RPC times out but is still executing, the channel remains busy and no
later command is buffered. A late result may update diagnostics for its
invocation ID but cannot change State, target status, revive a request, or
trigger a command. A physical target can be confirmed while the channel remains
unusable.

The discrete dispatch barrier deliberately becomes `Unknown` slightly before
SDK invocation. If dispatch is cancelled, removing the barrier is safe because
the SDK method never started.

DDS does not carry a causal command ID. A sample generated before the SDK call
may arrive afterward. The local ingress high-water mark, `started_at`, and a
stable run of post-start target samples reduce but cannot mathematically remove
that ambiguity. No new command is dispatched while the command target remains
unresolved. This residual risk and the stable-window rule must be validated
with real transport traces.

Move uses `SdkInvocation`, permit, and channel correlation, but never
`DiscreteCommandAttempt`. It does not establish motion or measured velocity.
State freshness is checked before every Move dispatch, one observation sequence
authorizes at most one Move, and no Move is buffered.

### Total deadline transitions

Every deadline has one terminal transition:

| Deadline | Transition |
|---|---|
| active dispatch permit expires before claim | atomically cancel it, clear any dispatching attempt/barrier, and emit no SDK call |
| action request expires | clear the request and set an operator-visible failure; never retry |
| velocity deadman expires | clear the setpoint; create Stop only if the currently confirmed State permits that decision |
| discrete target expires | set target `EXPIRED`, fail its request, and never infer State or retry |
| process shutdown deadline expires | invalidate active permits, issue no new command, publish final evidence, and terminate |

Target expiry does not lift an observation barrier. If a valid post-start State
has already arrived, that observed State remains publishable. Otherwise State
remains `Unknown` until a later valid observation. A busy channel and late RPC
diagnostics remain associated with the invocation ID independently of the
expired physical target.

The command boundary must not make process shutdown depend on an unbounded
worker join. SDK calls require a bounded transport timeout or an execution
boundary that can be abandoned at the process deadline. This does not turn a
transport timeout into physical-State evidence.

## Single event flow

The control loop is the only owner of mutable domain state:

```text
DDS State -> StateSource ----\
Zenoh input -> ZenohIO -------+--> ordered ControlLoop
timer/signal ----------------/       |
RPC events <- CommandExecutor <------+--> canonical snapshot -> ZenohIO
```

It owns only:

- a fixed-size observation window;
- one optional action request;
- one optional velocity setpoint;
- one optional SDK invocation;
- one optional discrete command attempt;
- command-channel ready/busy/failed status;
- the observation sequence last used for a non-zero Move;
- lifecycle and last RPC diagnostics; and
- a snapshot revision.

It does not store confirmed State, posture, `walking_enabled`, or
observed velocity as independently mutable values. They are derived when the
final snapshot for an event is built.

For every event, processing follows one order:

1. Update only the event's source data: observation window, request/setpoint,
   invocation/attempt metadata, timer, or lifecycle.
2. Derive confirmed State from the final observation evidence and barrier.
3. Reconcile command target and request target from that State without reading
   RPC diagnostics.
4. Determine whether this event may trigger dispatch. `RpcCompleted`,
   `CommandStarted`, and publication events never do. A non-zero Move is
   considered only for a new `StateReceived` sequence.
5. For an eligible event, if State is confirmed, no invocation exists, no
   command target is unresolved, and the channel is `READY`, evaluate the
   canonical decision table.
6. If a command is selected, create its dispatch permit and establish
   `DISPATCHING` before handoff.
7. Re-derive State after all mutations.
8. Build and publish exactly one canonical snapshot.

Freshness is evaluated on every event and immediately before every dispatch.
It is not deferred to `Tick`.

## Missing or invalid State

At startup, the node waits for a valid State window. If the startup deadline
expires, startup fails and the process exits without sending any SDK command.

If State becomes stale, invalid, or anomalous during operation:

- confirmation becomes `Unknown`;
- every new SDK command, including StopMove and StandDown, is forbidden;
- the action request and velocity setpoint are cleared;
- every dispatch permit is invalidated and an attempt that has not started is
  cancelled;
- an already executing RPC cannot be cancelled, but its eventual result is
  diagnostic only; and
- recovery does not resume old work. A new operator request is required after
  State is confirmed again.

Stopping safely after complete telemetry loss cannot be solved by inferring
state from local command history. It requires a Unitree-side watchdog, lease,
or separate safety mechanism and must be decided in a dedicated safety ADR.

## Startup, shutdown, and external control

Startup does not have a StopMove sequence. It only establishes a valid State
window and begins accepting requests.

Shutdown clears normal requests and setpoints. It may issue a final command only
through the same decision table and only while State remains confirmed. If
State is `LOCOMOTION_MOVING` or `STANDING_MOVING`, shutdown requests `STOP`; if
State is quiescent and the configured shutdown policy requests down, it requests
`DOWN`. Unknown State
causes exit without another SDK command. There is no shutdown-specific RPC
success branch, retry loop, or blocking sleep sequence.

An external app's change is handled by the next State sample; no separate
internal posture needs reconciliation. A State that differs from a command
target is published as observed and does not cause automatic retry. The active
attempt waits for its target or expires.

State does not identify which client changed the robot. This ADR therefore
requires exclusive command ownership while the node is active. The official app
may establish a posture before node startup, but it must not concurrently issue
commands. Lease support or another enforceable ownership mechanism remains a
deployment prerequisite.

## Responsibilities and files

Mutable domain state has one owner, `ControlLoop`. I/O objects own only their
external resources; their count is determined by responsibility, not by an
arbitrary class limit.

```text
unitree_go2_zenoh/
  app.py               composition root and lifecycle wiring
  state.py             observations, sample window, total classifier
  policy.py            request targets and total command decision table
  control.py           ordered ControlLoop and command synchronization
  unitree_state.py     DDS subscription and State event conversion
  unitree_command.py   one unbuffered, serialized SportClient call
  zenoh.py             request ingress and canonical snapshot publication
  config.py            validated configuration

examples/
  keyboard.py
  keyboard_dashboard.py

tests/
  test_state.py
  test_policy.py
  test_control_flow.py
  test_unitree_state.py
  test_unitree_command.py
  test_zenoh.py
  test_keyboard.py
  traces/
```

The core pure functions are:

```text
to_observation(message, received_at, sequence) -> RobotObservation
classify(samples, barrier, now) -> ConfirmedRobotState
decide_action(state, request) -> CommandDecision
command_target_satisfied(command, state) -> bool
request_target_satisfied(request, state) -> bool
authorize_dispatch(model, decision, now) -> DispatchPermit | None
build_snapshot(model, now) -> CanonicalState
```

No class is created per mode, request, command, or transition. There is no
generic operation hierarchy, observation-gate object, state setter layer, or
catch-all `domain.py`.

## Canonical publication

One Zenoh key carries the authoritative, revisioned snapshot:

```text
CanonicalState
  revision
  confirmed_robot_state
  state_reason
  supporting_observations
  action_request
  velocity_setpoint
  sdk_invocation
  discrete_command_attempt
  command_channel_state
  rpc_diagnostic
  lifecycle
```

Legacy multi-key State may be emitted temporarily as a deprecated projection of
the same snapshot. It is not atomic and carries no cross-key consistency
guarantee. Consumers requiring consistency must use the canonical key.

The UI may show `discrete_command_attempt=StandUp`, but it must show physical
State as `Unknown` until a post-command observation arrives. It must never
render the command name as a confirmed posture.

## Safety invariants

1. Only observations establish a non-`Unknown` physical state.
2. Discrete command dispatch, missing State, staleness, and invalid ordering can
   only invalidate confirmation; they cannot establish another posture.
3. RPC completion affects only command-channel availability and diagnostics;
   its value never affects confirmed State, command target, request target, or
   policy decisions about physical state.
4. No SDK command is dispatched from `Unknown`, `Faulted`, transition, or
   unsupported State.
5. A discrete command target requires stable post-start State evidence.
6. At most one SDK call exists, with no command buffer behind it.
7. Every SDK invocation revalidates the State-source high-water mark, freshness
   deadline, lifecycle generation, authorization generation, and channel
   readiness.
8. StandDown is dispatched only from confirmed quiescent standing State.
9. Actual velocity is derived only from State.
10. Old requests and setpoints never resume after State loss or shutdown.
11. Publication is a pure projection and occurs after all event mutations.
12. Startup, normal control, deadman, and shutdown use the same classifier and
    decision table.
13. One observation sequence can authorize at most one non-zero Move.
14. A source epoch reset clears the observation window and all old work.

## Test design

Test count is not a target. Every safety invariant and identified hazard must
have automated evidence. Tests that detect distinct failures remain or are
added; tests that only lock old private gates or RPC-based state transitions are
replaced after their safety responsibility has moved.

### Required test groups

| Test group | Required evidence |
|---|---|
| Classification | exhaustive orthogonal State facts and policy-condition order, including transition+motion, stable window, thresholds, missing/stale/error/duplicate/out-of-order input |
| RPC independence | codes `0`, `-1`, other codes, timeout, and exception produce identical physical State and targets |
| Request policy | complete State × `STAND`/`DOWN`/`STOP` table and separate velocity policy |
| Barriers | discrete dispatching is Unknown; Move has no discrete barrier; only post-start State clears a discrete barrier; pre-dispatch samples cannot complete a command |
| Interleavings | State-before-RPC, RPC-before-State, late obsolete RPC, timeout with busy worker, State loss during RPC |
| Priorities | shutdown vs action, action vs velocity, repeated/replaced action request, deadman expiry |
| Publication | one final snapshot per event; command send cannot publish the prior confirmed State afterward |
| Boundaries | full State conversion, atomic permit claim/cancel in every ordering, equivalent-State non-cancellation, unprocessed-ingress blocking, unbuffered executor, event starvation resistance, canonical Zenoh schema |
| Orthogonal State facts | fault+moving, fault+down, transition+motion, and threshold-boundary combinations preserve evidence but permit no unsafe action |

### Required scenario traces

- startup in down, idle stand, and ready stand;
- startup without State, with zero SDK calls;
- down -> StandUp -> Unknown -> State mode 0 -> BalanceStand -> Unknown -> State mode 1;
- moving -> StopMove -> Unknown -> RPC `-1` -> still Unknown -> quiescent State;
- mode 3 moving -> one StopMove -> stable mode 3 quiescent: Stop completes and
  no second StopMove is emitted;
- standing -> StandDown -> Unknown -> RPC `0` -> still Unknown -> State mode 5;
- command target observed before RPC completion and after RPC completion;
- command target observed while the RPC never returns: physical/request targets
  still complete while the command channel remains busy;
- non-target State after a command, followed by timeout without automatic retry;
- State loss and recovery without replaying an old request or velocity setpoint;
- external State change while an attempt is active;
- duplicate/out-of-order State and a late result from an obsolete attempt;
- a State update, fault, staleness, or shutdown invalidating a dispatch permit
  immediately before SDK invocation;
- executor claim winning and ControlLoop cancellation winning the same permit
  race, with no permanent dispatch barrier;
- equivalent high-rate State preserving authorization without starving dispatch;
- a delayed pre-command sample arriving after SDK start without prematurely
  satisfying a stable command target;
- a timestamp/source epoch reset followed by the required new stable window;
- no observation sequence producing more than one non-zero Move;
- Move using invocation/channel correlation without creating a discrete State
  barrier;
- action, target, permit, deadman, and shutdown deadline expiry, including a
  shutdown StopMove whose RPC never returns;
- posture request during a velocity stream; and
- shutdown racing with an operator request.

Each hardware trace fixture records firmware, topic, capture conditions, full
State, local receive time, command start, and RPC diagnostic. It also contains
expected classification, Unknown intervals, expected command effects, command
and request target points, and a hazard/regression identifier. RPC data is
recorded but excluded from the physical-State oracle.

Existing tests that expect RPC success to set posture or walking enablement,
expect `STANDING_UP`/`STANDING_DOWN`, or lock pending-stop reasons and private
gate call sequences are incompatible with this decision. They are replaced only
after the new classification, independence, interleaving, and invariant tests
cover their real safety responsibility. Deadman, schemas, timestamps, transport
serialization, and real regression tests remain necessary and may increase the
suite size.

## Hazards and validation

| Hazard | Design control | Automated evidence |
|---|---|---|
| RPC success treated as completion | State-only targets | RPC independence matrix |
| RPC failure drives unsafe fallback | diagnostic-only result | code/exception equivalence tests |
| old State published after command | dispatch/start barrier and final projection | barrier ordering tests |
| stale decision executes later | expiring permit, unbuffered executor, invocation-time recheck | generation/busy/timeout tests |
| permit cancellation leaves permanent Unknown | atomic claim/cancel and same-event cleanup | all permit race orderings |
| State rate starves command start | authorization generation changes only on material policy change | equivalent-State flood test |
| physical target waits for a hung RPC | State target and channel status are independent | State-before-hung-RPC test |
| StandDown while moving | total decision table | exhaustive request policy |
| quiescent mode 3 repeats StopMove | distinct moving/quiescent locomotion conditions | one-Stop mode-3 trace |
| State loss continues control | no-command Unknown policy | loss/recovery scenarios |
| command and request completion mixed | separate target tables | multi-step Stand/Down traces |
| velocity overwrites posture request | separate storage and priority | request/setpoint race tests |
| overlapping classifier predicates | ordered total classifier | exhaustive classification table |
| transition motion falls through to Move | mode-class priority before motion policy | transition+moving matrix |
| fault hides observed motion/posture | orthogonal State facts | fault+motion combination matrix |
| faulted State also reports motion | no SDK command from faulted State; robot-side safety required | fault+moving policy test |
| external controller conflict | exclusive-owner precondition | deployment check and hardware scenario |
| split public State | one canonical key | publication contract test |

Residual risks include incorrect Go2 telemetry, finite State latency, executor
or host failure, and loss of all communication. This process is not a hard
real-time safety controller. Robot-side safety behaviour requires a separate
decision backed by Unitree documentation and hardware tests.

## Alternatives considered

### Continue patching `RobotController`

Rejected. Retaining mutable posture, walking flags, outcome classes, and gate
objects preserves two sources of physical truth.

### Generic reducer and operation hierarchy

Rejected. A large generic `ControlModel`, effect types, and per-operation phases
add abstraction without adding domain guarantees. A single ordered owner plus
small total functions is sufficient.

### State-only policy without command metadata

Rejected. Minimal command metadata is necessary to reject pre-command samples
and synchronize one in-flight RPC. That metadata never represents physical
state.

### Buffered command executor

Rejected. A queued command may execute after the State used to decide it has
expired. The SDK boundary must accept at most one immediately executable call.

## Atomic migration

1. Capture full hardware traces before selecting classifier thresholds.
2. Complete `state.py`, `policy.py`, replay tests, and interleaving tests offline.
3. If live comparison is needed, run the new classifier as a read-only shadow;
   it must not send SDK commands.
4. Implement the new I/O boundaries and canonical snapshot behind the new
   composition root.
5. Cut the runtime entrypoint over once.
6. In the same cutover change, remove the old controller, observation gates,
   RPC-outcome state transitions, startup/shutdown stop paths, and tests that
   only preserve them.
7. Validate startup, stand, walk, stop, down, re-stand, State loss, and external
   change on the target firmware.

There must never be two production command decision paths. Rollback is a Git
revert, not a runtime dual-controller mode.

## Acceptance criteria

This ADR may become `Accepted` when:

- target-firmware traces establish the total classifier and thresholds;
- mode 0, 1, 3, 5, and transition behaviour is confirmed;
- StopMove, StandUp, BalanceStand, and StandDown are traced for success and
  code `-1` without using RPC results as the State oracle;
- State-loss behaviour and the absence of SDK commands are verified;
- the unbuffered executor, expiring dispatch permits, independent State target
  and channel status, and all event interleavings are tested;
- atomic permit claim/cancel and every deadline terminal transition are tested;
- Move invocation is verified to use no discrete barrier and no more than one
  dispatch per observation;
- the measured State rate supports the one-Move-per-observation rule;
- DDS domain, interface, topic, and source-epoch handling identify one target
  Go2;
- exclusive command ownership is enforced operationally or by lease;
- the canonical Zenoh schema is defined; and
- no old and new production command paths coexist.

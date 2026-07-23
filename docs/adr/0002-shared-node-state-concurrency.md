# ADR-0002: Coordinate Runtime Work Through One Node State

- Status: Accepted
- Date: 2026-07-23
- Issue: #11
- Supersedes: the runtime coordination and publication parts of ADR-0001

## Context

ADR-0001 made fresh DDS `SportModeState` the authority for robot State and SDK actions.
Its implementation nevertheless coupled several independent activities through the control loop:

- DDS samples first entered a separate inbox;
- Zenoh commands lived in a separate command buffer;
- controller transitions published Node State immediately; and
- the public-State heartbeat ran only when the control loop was available.

A synchronous SportClient call could therefore delay freshness enforcement and publication.
The inbox, command buffer, controller fields, and cached public payload also formed multiple stores whose values had to be kept in agreement.

The State-driven safety model remains correct.
Runtime coordination, not the robot action table, needs to change.

## Decision

Use one immutable, lock-protected `NodeState` as the only cross-thread domain state:

```text
Unitree ch_reader callback ── latest observation ──┐
Zenoh callbacks ───────────── latest commands ────┤
main thread ─────────────── lifecycle/control ────► NodeState + Condition
                                                   │
                      ┌────────────────────────────┼──────────────┐
                      ▼                            ▼              ▼
              control decision          periodic publisher   Queryable
                      │                            │
                      ▼                            ▼
                SportClient                   Zenoh put
                      │
                      └──── diagnostic ───────────► NodeState
```

`NodeState` contains:

- the latest valid DDS observation, including local monotonic `received_at`;
- the latest accepted posture and velocity requests and their receive times;
- lifecycle;
- the active posture workflow, its last action, and the post-RPC observation barrier; and
- SDK and node diagnostics.

Robot State classification is derived from the latest observation by one pure classifier.
It is not stored beside the observation, because the two values could otherwise describe different samples while the control worker is catching up.

The aggregate is immutable after construction.
Each update replaces it while holding one `Condition`; readers copy its reference under that same lock and derive values after releasing the lock.
No DDS, Zenoh, logging, serialization, or SportClient call occurs while the lock is held.

## Execution contexts

### DDS observation callback

The Unitree SDK is initialized with `queueLen=1`.
The SDK owns a dedicated `ch_reader` thread and coalesces unread samples to one queue entry.
Its callback:

1. captures local monotonic `received_at`;
2. converts and validates the sample;
3. replaces only the latest observation; and
4. wakes the control worker.

An invalid sample records a diagnostic but does not replace the valid observation or refresh its receive time.
The callback never calls SportClient or Zenoh.

### Zenoh callbacks

Command callbacks decode outside the lock, then atomically check the common freshness and lifecycle gate and replace the applicable latest request.
Command receipt does not wake control and cannot call SportClient.

The Queryable takes a coherent snapshot and serializes and replies after releasing the lock.
It has no cached payload and does not mutate Node State.

### Control worker

The main thread is the single control worker.
Its last processed `observation.received_at` is local loop state, not another shared property.
The worker waits on the Node State condition and processes only an observation with a later receive time.
If DDS outpaces a blocking SDK call, the next step uses the newest observation and does not replay intermediate samples.

For one consumed observation, the worker:

1. checks the common freshness definition;
2. classifies the observation;
3. applies posture priority and selects at most one SDK effect;
4. commits the effect claim and any workflow transition under the lock;
5. calls SportClient after releasing the lock; and
6. records the diagnostic under the lock.

A posture effect closes command intake before the call.
After it returns, its barrier is the local monotonic return time, so only an observation received later can advance the workflow.

### Periodic public-State publisher

`ZenohStateBus` owns one declared Zenoh publisher and one daemon thread for the session lifetime.
The thread publishes immediately and then on absolute monotonic deadlines.
A delayed publication skips missed deadlines instead of emitting a catch-up burst.

Each publication samples Node State briefly, then classifies, serializes, and calls Zenoh outside the lock.
Controller transitions never publish directly.
A slow publication therefore occupies only this publisher thread.

On teardown, the controller first records the final `stopped` lifecycle.
The publisher stop event wakes the thread for one final best-effort publication.
The bus waits for a fixed bounded join before closing its Zenoh entities; an unresponsive external publish is logged rather than blocking process teardown indefinitely.

## Freshness

Every consumer uses this definition:

```text
robot_connected =
    observation exists
    and now < observation.received_at + state_freshness_seconds
```

`received_at` and `now` use the same local monotonic clock.
There is no stored connection flag and no second connection timeout.

When disconnected:

- control selects no SDK effect, including `StopMove`;
- command callbacks reject new commands;
- the control worker clears pending normal commands and workflows;
- shutdown retains only its Down intent and waits for a later observation; and
- publication and queries return `robot_connected: false`, `accepting_commands: false`, Unknown Robot State, and no stale observation values.

The freshness threshold is independent of the SportClient RPC timeout, velocity-command timeout, Node State publication frequency, and keyboard display timeout.

## Public contract

`PublishedNodeState` is a wire projection, not another store.
It exists because internal workflow metadata and local monotonic receive times must not be published.

The contract has no revision or generation.
Repeating an unchanged periodic snapshot is not a domain change, and no counter has useful restart semantics.
It also omits local monotonic `received_at` values.
The DDS source timestamp remains available in `robot_state`.

## Abstraction choices

- `NodeState` owns the one shared domain invariant and synchronization source.
- `Controller` owns that state, its condition, the action table, and the lock-to-effect boundary.
  Splitting these would create another state owner.
- `PublishedNodeState` is the required internal-to-wire projection boundary.
- `ZenohStateBus` already owns Zenoh subscriber/queryable resources and now also owns the publisher resource and its thread lifetime.
- `_SdkInvocation` is a short immutable effect claim that can cross the lock boundary without exposing mutable controller state.

The former `StateInbox`, `CommandBuffer`, cached State payload, `StateSink`, and revision counters are removed.
No class is introduced merely to represent the DDS thread, control thread, publisher event, or freshness check.

## Consequences

- DDS reception, command ingress, control, publication, and queries can continue independently around blocking external calls.
- One freshness definition and one state lock replace several coordinated flags and stores.
- Public State may reflect a newly received observation before the control worker consumes it; both use the same pure classifier, while SDK effects still require control consumption of that observation.
- Periodic publication no longer provides an event history.
  Consumers needing history must record the snapshots they receive.
- Final publication is best effort because a blocked external transport cannot be made reliable without allowing unbounded shutdown.

# ADR-0003: Separate the Wire Contract and Split Command Keys

- Status: Accepted
- Date: 2026-11-06
- Issue: #13
- Supersedes: the single `{zenoh_key_prefix}/command` contract, and the `ZenohStateBus` abstraction choice in ADR-0002

## Context

ADR-0001 and ADR-0002 remain correct about observed Robot State, the action table, the shared `NodeState`, freshness, and publication.
Their implementation, however, carried structure that did not earn its place:

- `config.py` and the keyboard client imported `controller.py`, which owns the SDK-driving `Controller`, only to reach wire models and velocity limits;
- velocity (best-effort, drop) and posture (reliable, block) shared `{zenoh_key_prefix}/command` and were told apart by a payload `type` field;
- one control decision was spread over five single-use private methods, and the node threaded the last consumed observation through three Controller calls per loop;
- `ZenohStateBus`, the keyboard controller, and several helper classes were constructed once only to be run; and
- the same velocity appeared as three internal and wire types with two wire shapes (an object in `requested_velocity`, an array in `last_sdk_diagnostic`).

## Decision

### Module boundary

```text
wire.py        Zenoh contract: keys, JSON encoding, velocity limits,
               commands, published State models, shared enums
controller.py  Go2 SDK enums, observation, classification, NodeState, Controller
node.py        CLI, DDS conversion, Zenoh IO, control loop
config.py      validated settings
examples/      keyboard client
```

`config.py` and `examples/` depend on `wire.py`, never on `controller.py`.
Clients therefore share the contract without importing the control implementation.

### Command keys

| Key | Payload |
| --- | --- |
| `{zenoh_key_prefix}/command/velocity` | `{"vx","vy","vyaw"}` |
| `{zenoh_key_prefix}/command/posture` | `{"posture"}` |
| `{zenoh_key_prefix}/state` | Published Node State (unchanged) |

Zenoh downsampling, QoS overwrite, and access-control rules match key expressions only; the router never inspects the payload.
On one shared key, a rule meant for velocity also applies to posture: for example, downsampling velocity for a wireless link silently drops posture commands, which the keyboard does not retry.
Separate keys let deployments configure each command kind independently, keep each publisher's QoS tied to its key, and let the node validate each key with one model instead of a discriminated union.

A payload is accepted only on its own key; a `type` field or another command's fields are rejected.
This is a breaking change for v0.1.0 clients.

### One velocity representation

`VelocityCommand` is the inbound payload, the buffered request's command, `requested_velocity`, and `last_sdk_diagnostic.velocity`.
The diagnostic velocity is therefore an object rather than an array.

### Abstractions

- `Controller.step(after, timeout)` replaces `wait_for_observation`, `process_observation`, and `enforce_observation_freshness`.
  The control worker still owns the last consumed `received_at` and passes it back as `after`; it is not stored in `NodeState`.
- Classification is one literal `(state machine, mode) -> Robot State` table returning `(RobotState, Motion)`.
  The published robot-State model is built only for publication and queries, not on the control path.
- One selection method plus the pure `posture_action` replace the five single-use private methods.
- One generic `ReceivedCommand` replaces the per-kind request types.
- `serve_zenoh`, a context manager, replaces `ZenohStateBus`.
  It owns both command subscribers, the State queryable, and the publisher thread, with the same bounded teardown and final best-effort publication.
- The keyboard is module functions and local state; it no longer queries State at startup because the node publishes it periodically.

`_SdkInvocation`, `PublishedNodeState`, and the structural protocols for the SDK client and DDS message remain, for the reasons recorded in ADR-0002.

## Consequences

- Behavior of the action table, command gate, freshness, post-RPC barrier, and shutdown is unchanged.
- One edge case changes intentionally: a stop request that arrives together with the first observation now runs the normal Down workflow, as every other stop after startup does, instead of exiting without it.
- v0.1.0 clients must move to the new keys and payloads; the next release is v0.2.0.

## Alternatives rejected

- **Keep one command key:** a discriminator on one channel is a valid design (Unitree's sport API distinguishes commands by `api_id`), but it prevents per-kind Zenoh configuration.
- **Advanced Pub/Sub for late joiners:** marked unstable in zenoh-python 1.9; periodic State publication already reaches late joiners within one period.
- **Liveliness tokens for node presence:** lease expiry is slower than the keyboard's State timeout over periodic publication.
- **Request/response posture commands:** acceptance is already visible through `accepting_commands` and `requested_posture` in published State.

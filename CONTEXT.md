# Domain context

This glossary defines the terms used by the Go2 control node.
Architecture decisions belong in ADRs.

## State freshness

Whether the node has received a valid `SportModeState` within `dds.state_freshness_seconds`, measured with the local monotonic clock.
Freshness is the only condition that establishes DDS connection health and opens command intake.
A malformed sample does not refresh it.

## Robot observation

One valid `SportModeState` sample together with its local monotonic receive time.
Robot observations are the only evidence for physical posture and motion.

## Robot State

One of Damping, Down, LockedStand, ReadyStand, Locomotion, Unsupported, or Unknown, classified from the latest observation's V2.0 motion state machine and coarse mode.
Measured motion is a separate moving/quiescent value used only to confirm Down and to guard `StandDown()`.

The SDK field named `error_code` carries the motion state machine ID.
SDK call return codes are separate diagnostics and never establish Robot State.

## Node State

The one immutable, lock-protected aggregate shared by the DDS callback, Zenoh callbacks, control worker, publisher, Queryable, and lifecycle handling.
It contains the latest observation, accepted commands, control workflow, lifecycle, and diagnostics.

Robot State is derived from the observation instead of being stored beside it.
The control worker's last processed `observation.received_at` is local worker state, not another shared counter or generation.

## Unknown

Robot State used when no fresh valid observation exists or while waiting for a State received after a posture SDK call.
It is not an estimated posture.

## Command intake

One global gate shared by posture and velocity commands.
It is open while DDS State is fresh, the node is not waiting after a posture RPC, and shutdown has not begun.
Robot State determines whether a buffered command is legal when the next State drives the control loop; an illegal command is discarded.

## Velocity request

The newest Zenoh velocity and its local receive time.
It is independent of measured velocity and expires through the velocity-command timeout.

## Posture target

The newest `stand` or `down` request.
It replaces the previous target.
The last posture SDK action prevents duplicate calls while observations drive the workflow; it never represents physical posture.

## SDK diagnostic

The command name, return code, or exception from the latest SDK call.
It is operator information only and cannot advance a posture workflow.

## Published State

The periodic `{zenoh_key_prefix}/state` projection of Node State.
It contains DDS freshness, observed Robot State, accepted commands, the active posture target and action, lifecycle, and diagnostics without mixing their meanings.
It contains neither a revision counter nor process-local monotonic receive times.

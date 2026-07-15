# Domain context

This glossary defines the domain language used by the Go2 control node.
Implementation decisions belong in ADRs, not in this file.

## Robot observation

One `SportModeState` sample received from the Go2, together with its local
receive sequence and monotonic receive time. Robot observations are the only
evidence from which a confirmed physical state may be established.

## Confirmed robot state

The validity, mode class, motion, and fault facts derived once from a fresh,
valid window of robot observations. These facts remain separate so, for
example, a State can report both a fault and non-zero motion. A non-`Unknown`
physical fact can only be established by observations.

## Unknown

An evidence-validity result meaning that the current physical condition cannot
be confirmed. Missing or stale observations and the period after a discrete
state-changing command all produce `Unknown`. It is not an estimated posture.

## Action request

A one-shot operator goal: `Stand`, `Down`, or `Stop`. A request is not a robot
state. It remains active across the minimum sequence of commands needed to
reach its State-confirmed target.

## Velocity setpoint

The latest short-lived operator velocity request, including its receive time
and deadman deadline. It is not an action workflow and is never interpreted as
the measured robot velocity.

## SDK invocation

One SDK call, including its expiring dispatch permit, channel status, and RPC
correlation. Every command, including `Move`, has an SDK invocation.

## Discrete command attempt

A `StandUp`, `BalanceStand`, `StopMove`, or `StandDown` invocation together with
its observation barrier, deadline, and State target. `Move` is not a discrete
command attempt. The attempt describes synchronization, not physical state.

## RPC diagnostic

The return code, exception, and duration of an SDK call. It may report transport
availability but never proves command completion or physical state.

## Command channel state

Whether the SDK command boundary is ready, reserved for an unstarted invocation,
busy, or failed. It controls only
whether another RPC may start and never changes confirmed robot state or a
State-confirmed command target.

## Canonical state

One revisioned snapshot containing the confirmed robot state, supporting
observation, request, setpoint, SDK invocation, discrete attempt, channel state,
and RPC diagnostic without mixing their meanings.

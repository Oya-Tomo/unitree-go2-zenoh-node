# Domain context

This glossary defines the small set of terms used by the Go2 control node.
Implementation decisions belong in ADRs.

## Robot observation

One `SportModeState` sample together with its local monotonic receive time.
Robot observations are the only evidence for physical posture and motion.

## Physical State

The validity, V2.0 motion state machine, coarse mode class, measured motion, and
robot timestamp derived from the latest fresh observation. The SDK field named
`error_code` carries the motion state machine ID; SDK call return codes are a
separate diagnostic channel. SDK calls and their return values never establish
physical State.

## Unknown

The physical State used when no fresh valid observation is available or after a
posture-related SDK call. It is not an estimated posture. Commands are not
accepted until a valid observation received after that call replaces it.

## Command acceptance

Input eligibility is derived from confirmed physical State and command type.
Posture is accepted in Damping, Down, and supported standing or locomotion
States. Velocity is accepted only in ready-stand or locomotion State.
Transition, Unsupported, Unknown, and shutdown accept neither. Acceptance lets
a request replace its latest-value buffer; it does not by itself authorize an
SDK call. Local monotonic receive time preserves the ordering boundary when a
State and command arrive before the control loop applies the State.

## Velocity request

The latest short-lived Zenoh velocity command and its local receive time. It is
not measured velocity. A newer velocity replaces it.

## Posture request

The latest unprocessed Zenoh `stand` or `down` target. The control loop takes it
as the active target on the next State; a newer target replaces that active
workflow on a later State. Posture has priority over velocity.

## Posture phase

The last SDK step sent for the active posture request. It prevents duplicate
calls while the node waits for a newer State, but it never represents the
robot's physical posture.

## SDK diagnostic

The command name, return code, or exception from the latest SDK call. It is
operator information only and cannot advance a posture workflow.

## Published State

One revisioned `{robot_key}/state` snapshot containing physical State, current
requests, posture phase, lifecycle, and the latest SDK diagnostic without
mixing their meanings.

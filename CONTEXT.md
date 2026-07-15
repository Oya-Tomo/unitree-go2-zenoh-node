# Domain context

This glossary defines the small set of terms used by the Go2 control node.
Implementation decisions belong in ADRs.

## Robot observation

One `SportModeState` sample together with its local monotonic receive time.
Robot observations are the only evidence for physical posture and motion.

## Physical State

The validity, mode class, motion, error code, measured velocity, and robot
timestamp derived from the latest fresh observation. SDK calls and their return
values never establish physical State.

## Unknown

The physical State used when no fresh valid observation is available or after a
posture-related SDK call. It is not an estimated posture. Commands are not
accepted until a newer valid observation replaces it.

## Velocity request

The latest short-lived Zenoh velocity command and its local receive time. It is
not measured velocity. A newer velocity replaces it.

## Posture request

The latest Zenoh `stand` or `down` target. A newer posture replaces it, and a
posture request has priority over velocity.

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

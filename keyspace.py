from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RobotKeyspace:
    """Concrete Zenoh keys belonging to one robot control node."""

    robot_key: str

    @property
    def command(self) -> str:
        return f"{self.robot_key}/command"

    @property
    def requested_velocity(self) -> str:
        return f"{self.robot_key}/state/command/requested"

    @property
    def applied_velocity(self) -> str:
        return f"{self.robot_key}/state/command/applied"

    @property
    def posture(self) -> str:
        return f"{self.robot_key}/state/posture"

    @property
    def motion(self) -> str:
        return f"{self.robot_key}/state/motion"

    @property
    def health(self) -> str:
        return f"{self.robot_key}/state/health"

    @property
    def state_selector(self) -> str:
        return f"{self.robot_key}/state/**"

    @property
    def state_keys(self) -> tuple[str, str, str, str, str]:
        return (
            self.requested_velocity,
            self.applied_velocity,
            self.posture,
            self.motion,
            self.health,
        )

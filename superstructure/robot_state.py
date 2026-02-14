from dataclasses import dataclass
from enum import Enum


class RobotState(Enum):
    IDLE = 0
    INTAKING = 1
    PREP_SHOT = 2
    SHOOTING = 3
    CLIMB_AUTO = 4
    CLIMB_MANUAL = 5


@dataclass
class RobotReadiness:
    shooterReady: bool = False
    intakeDeployed: bool = False
    canFeed: bool = False

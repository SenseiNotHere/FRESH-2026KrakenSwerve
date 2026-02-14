from dataclasses import dataclass
from enum import Enum

class RobotState(Enum):
    IDLE = 0
    INTAKING = 1
    PREP_SHOT = 2
    SHOOTING = 3
    CLIMB = 4

@dataclass
class RobotReadiness:
    shooterReady: bool = False
    indexerReady: bool = False
    intakeReady: bool = False
    nearTrench: bool = False
    canFeed: bool = False
    distanceToTrench: float | None = None
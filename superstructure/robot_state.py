from dataclasses import dataclass
from enum import Enum


class RobotState(Enum):
    # TODO: Add more states as needed
    # Teleop states
    IDLE = 0
    INTAKING = 1
    PREP_SHOT = 2
    SHOOTING = 3
    CLIMB_AUTO = 4
    CLIMB_MANUAL = 5

    # Autonomous states
    CLIMBER_DOWN_AUTONOMOUS = 6
    CLIMBER_UP_AUTONOMOUS = 7
    PREP_SHOT_AUTONOMOUS = 8
    INTAKING_AUTONOMOUS = 9
    SHOOTING_AUTONOMOUS = 10

@dataclass
class RobotReadiness:
    shooterReady: bool = False
    intakeDeployed: bool = False
    canFeed: bool = False

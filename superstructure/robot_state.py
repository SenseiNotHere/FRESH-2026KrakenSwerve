from dataclasses import dataclass
from enum import Enum


class RobotState(Enum):

    # General / Neutral States
    IDLE = 0
    PLAYING_SONG = 1
    PLAYING_CHAMPIONSHIP_SONG = 2

    # Intake States (Teleop)
    INTAKING = 3
    INTAKE_DEPLOYED = 4
    INTAKE_RETRACTED = 5

    # Shooter States (Teleop)
    PREP_SHOT = 6
    SHOOTING = 7
    FOLLOWING_HUB_SHOOT = 8   # Follow hub and shoot

    # Climber States (Teleop)
    CLIMB_MANUAL = 9
    AIRBREAK_ENGAGED_UP = 10
    AIRBREAK_ENGAGED_DOWN = 11

    # Elevator States (Teleop)
    ELEVATOR_RISING = 12
    ELEVATOR_LOWERING = 13

    # Drivetrain States (Teleop)
    APPROACHING_OUTPOST = 14
    APPROACHING_TOWER = 15

    # Autonomous States
    CLIMBER_DOWN_AUTONOMOUS = 20
    CLIMBER_UP_AUTONOMOUS = 21
    PREP_SHOT_AUTONOMOUS = 22
    INTAKING_AUTONOMOUS = 23
    SHOOTING_AUTONOMOUS = 24


@dataclass
class RobotReadiness:

    # General readiness
    shooterReady: bool = False
    intakeDeployed: bool = False
    canFeed: bool = False
    elevatorAtTarget: bool = False

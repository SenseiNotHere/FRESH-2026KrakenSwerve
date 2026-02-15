from dataclasses import dataclass
from enum import Enum


class RobotState(Enum):

    # General / Neutral States
    IDLE = 0
    PLAYING_SONG = 1

    # Intake States (Teleop)
    INTAKING = 2
    INTAKE_DEPLOYED = 3
    INTAKE_RETRACTED = 4

    # Shooter States (Teleop)
    PREP_SHOT = 5
    SHOOTING = 6
    FOLLOWING_HUB_SHOOT = 7   # Follow hub and shoot

    # Climber States (Teleop)
    CLIMB_MANUAL = 8
    AIRBREAK_ENGAGED_UP = 9
    AIRBREAK_ENGAGED_DOWN = 10

    # Elevator States (Teleop)
    ELEVATOR_RISING = 11
    ELEVATOR_LOWERING = 12

    # Drivetrain States (Teleop)
    APPROACHING_OUTPOST = 13
    APPROACHING_TOWER = 14

    # Autonomous States
    CLIMBER_DOWN_AUTONOMOUS = 20
    CLIMBER_UP_AUTONOMOUS = 21
    PREP_SHOT_AUTONOMOUS = 22
    INTAKING_AUTONOMOUS = 23
    SHOOTING_AUTONOMOUS = 24


@dataclass
class RobotReadiness:
    shooterReady: bool = False
    intakeDeployed: bool = False
    canFeed: bool = False
    elevatorAtTarget: bool = False
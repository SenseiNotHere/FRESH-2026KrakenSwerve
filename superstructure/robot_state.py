from dataclasses import dataclass
from enum import Enum


class RobotState(Enum):

    # General / Neutral States
    IDLE = 0

    # Intake States (Teleop)
    INTAKING = 1
    INTAKE_DEPLOYED = 2
    INTAKE_RETRACTED = 3

    # Shooter States (Teleop)
    PREP_SHOT = 4
    SHOOTING = 5
    FOLLOWING_HUB_SHOOT = 6   # Follow hub and shoot

    # Climber States (Teleop)
    CLIMB_MANUAL = 7
    AIRBREAK_ENGAGED_UP = 8
    AIRBREAK_ENGAGED_DOWN = 9

    # Elevator States (Teleop)
    ELEVATOR_RISING = 10
    ELEVATOR_LOWERING = 11

    # Drivetrain States (Teleop)
    APPROACHING_OUTPOST = 12
    APPROACHING_TOWER = 13

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
from typing import Optional

from commands2 import FunctionalCommand

from subsystems.shooter.shootersubsystem import Shooter
from subsystems.shooter.shot_calculator import ShotCalculator
from subsystems.intake.intakesubsystem import Intake
from subsystems.climber.climbersubsystem import Climber
from subsystems.intake.intakesubsystem import IntakeConstants
from subsystems.shooter.indexersubsystem import Indexer
from subsystems.vision.limelightcamera import LimelightCamera

from .robot_state import RobotState, RobotReadiness

from constants.constants import ShooterConstants, IndexerConstants, ClimberConstants


class Superstructure:
    def __init__(
        self,
        drivetrain,
        shooter: Shooter | None = None,
        indexer: Indexer | None = None,
        shotCalculator: ShotCalculator | None = None,
        intake: Intake | None = None,
        climber: Climber | None = None,
        vision: LimelightCamera | None = None
    ):
        # Subsystems
        self.drivetrain = drivetrain
        self.shooter = shooter
        self.indexer = indexer
        self.shotCalculator = shotCalculator
        self.intake = intake
        self.climber = climber
        self.limelight = vision

        # State
        self.robot_state = RobotState.IDLE
        self.robot_readiness = RobotReadiness()

    # Main update loop

    def update(self):

        self._update_readiness()

        if self.robot_state == RobotState.IDLE:
            self._handle_idle()

        elif self.robot_state == RobotState.PREP_SHOT:
            self._handle_prep_shot()

        elif self.robot_state == RobotState.SHOOTING:
            self._handle_shooting()

    def _update_readiness(self):

        if ShooterConstants.kShooterEnabled:
            self.robot_readiness.shooterReady = self.shooter.atSpeed(tolerance_rpm=150)

        if IndexerConstants.kIndexerEnabled:
            self.robot_readiness.indexerReady = self.indexer.isReady()

        # canFeed depends on state + shooter speed
        self.robot_readiness.canFeed = (
                self.robot_state == RobotState.SHOOTING
                and self.robot_readiness.shooterReady
        )

    # State transitions (Public API)

    def createStateCommand(self, state: RobotState):
        return FunctionalCommand(
            onInit=lambda: self.setState(state),
            onExecute=lambda: None,
            onEnd=lambda interrupted: self.setState(RobotState.IDLE),
            isFinished=lambda: False
        )

    def setState(self, newState: RobotState):
        self.robot_state = newState

    # State handers

    def _handle_idle(self):
        self._stop_shooter_system()

    def _handle_prep_shot(self):
        if not ShooterConstants.kShooterEnabled or not self.shooter:
            return

        self._apply_shooter_speed()

        # Never feed in prep
        self._stop_feeder()

    def _handle_shooting(self):
        """
        Spin up shooter and feed when ready.
        """
        if not ShooterConstants.kShooterEnabled or not self.shooter:
            return

        self._apply_shooter_speed()

        if (
            self.robot_readiness.shooterReady
            and self.indexer
            and IndexerConstants.kIndexerEnabled
        ):
            self.indexer.enable()
        else:
            self._stop_feeder()


    # Helpers

    def _stop_feeder(self):
        if IndexerConstants.kIndexerEnabled:
            self.indexer.disable()

    def _stop_shooter_system(self):
        if ShooterConstants.kShooterEnabled:
            self.shooter.disable()
        self._stop_feeder()

    def _apply_shooter_speed(self):

        if not self.shooter:
            return

        if ShooterConstants.kShotCalculatorEnabled and self.shotCalculator:
            # Use distance-based RPS
            target_rps = self.shotCalculator.getTargetSpeedRPS()
            self.shooter.setTargetRPS(target_rps)
        else:
            # Use SmartDashboard chooser percent
            percent = self.shooter.getDashboardPercent()
            self.shooter.setPercent(percent)

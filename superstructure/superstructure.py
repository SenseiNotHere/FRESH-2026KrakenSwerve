from commands2 import FunctionalCommand
from wpilib import SmartDashboard

from subsystems.shooter.shootersubsystem import Shooter
from subsystems.shooter.shot_calculator import ShotCalculator
from subsystems.intake.intakesubsystem import Intake
from subsystems.climber.climbersubsystem import Climber
from subsystems.shooter.indexersubsystem import Indexer
from subsystems.vision.limelightcamera import LimelightCamera

from .robot_state import RobotState, RobotReadiness

from constants.constants import ClimberConstants


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
        self.drivetrain = drivetrain

        self.shooter = shooter
        self.indexer = indexer
        self.shotCalculator = shotCalculator
        self.intake = intake
        self.climber = climber
        self.vision = vision

        # Subsystem availability (safe for build season)
        self.hasShooter = shooter is not None
        self.hasIndexer = indexer is not None
        self.hasShotCalc = shotCalculator is not None
        self.hasIntake = intake is not None
        self.hasClimber = climber is not None
        self.hasVision = vision is not None

        self.robot_state = RobotState.IDLE
        self.robot_readiness = RobotReadiness()

    # Update Loop (Call from robotPeriodic)

    def update(self):

        SmartDashboard.putString(
            "Superstructure/State",
            self.robot_state.name
        )

        self._update_readiness()

        if self.robot_state == RobotState.IDLE:
            self._handle_idle()

        elif self.robot_state in [
            RobotState.INTAKING,
            RobotState.INTAKING_AUTONOMOUS
        ]:
            self._handle_intaking()

        elif self.robot_state in [
            RobotState.PREP_SHOT,
            RobotState.PREP_SHOT_AUTONOMOUS
        ]:
            self._handle_prep_shot()

        elif self.robot_state in [
            RobotState.SHOOTING,
            RobotState.SHOOTING_AUTONOMOUS
        ]:
            self._handle_shooting()

        elif self.robot_state == RobotState.CLIMB_AUTO:
            self._handle_climb_auto()

        elif self.robot_state == RobotState.CLIMB_MANUAL:
            self._handle_climb_manual()

    # Readiness

    def _update_readiness(self):

        if self.hasShooter:
            self.robot_readiness.shooterReady = (
                self.shooter.atSpeed(tolerance_rpm=150)
            )
        else:
            self.robot_readiness.shooterReady = False

        if self.hasIntake:
            self.robot_readiness.intakeDeployed = (
                self.intake.isDeployed()
            )
        else:
            self.robot_readiness.intakeDeployed = False

        self.robot_readiness.canFeed = (
            self.robot_state == RobotState.SHOOTING
            and self.robot_readiness.shooterReady
            and self.hasIndexer
        )

    # Public State API

    def createStateCommand(self, state: RobotState):
        return FunctionalCommand(
            onInit=lambda: self.setState(state),
            onExecute=lambda: None,
            onEnd=lambda interrupted: self.setState(RobotState.IDLE),
            isFinished=lambda: False
        )

    def setState(self, newState: RobotState):

        # No-op if same state
        if newState == self.robot_state:
            return

        # Safety: once climbing, only allow IDLE transition
        if self.robot_state in [
            RobotState.CLIMB_AUTO,
            RobotState.CLIMB_MANUAL
        ] and newState != RobotState.IDLE:
            return

        oldState = self.robot_state
        self.robot_state = newState

        # Log state transition
        print(f"[Superstructure] {oldState.name} -> {newState.name}")

        SmartDashboard.putString(
            "Superstructure/State",
            newState.name
        )

        # One-time actions on state entry
        if newState == RobotState.CLIMB_AUTO and self.hasClimber:
            self.climber.releaseAirbrake()
            self.climber.setPosition(
                ClimberConstants.kClimbHeight
            )

    # State Handlers

    def _handle_idle(self):

        self._stop_shooter()
        self._stop_indexer()
        self._stow_intake()

    # Start intaking on entry, but keep handling to manage shooter and indexer states
    def _handle_intaking(self):

        self._stop_shooter()

        if self.hasIntake:
            self.intake.startIntaking()

        if self.hasIndexer:
            self.indexer.feed()

    # Start prep on entry, but keep handling in case we need to adjust for vision
    def _handle_prep_shot(self):

        if not self.hasShooter:
            return

        # Spin up shooter
        if self.hasShotCalc:
            target_rps = self.shotCalculator.getTargetSpeedRPS()
            self.shooter.setTargetRPS(target_rps)
        else:
            self.shooter.useDashboardPercent()

        # Hold note while spinning
        if self.hasIndexer:
            self.indexer.hold()

    # Start shooting on entry, but keep handling to manage feeding and adjust shooter speed if needed
    def _handle_shooting(self):

        if not self.hasShooter:
            return

        # Keep spinning
        if self.hasShotCalc:
            target_rps = self.shotCalculator.getTargetSpeedRPS()
            self.shooter.setTargetRPS(target_rps)
        else:
            self.shooter.useDashboardPercent()

        # Feed only if ready
        if (
            self.hasIndexer
            and self.robot_readiness.shooterReady
        ):
            self.indexer.feed()
        elif self.hasIndexer:
            self.indexer.hold()

    # Start climb on entry, but keep handling to manage airbrake
    def _handle_climb_auto(self):

        self._stop_shooter()
        self._stop_indexer()
        self._stow_intake()

        if not self.hasClimber:
            return

        if self.climber.atTarget():
            self.climber.engageAirbrake()
        else:
            self.climber.releaseAirbrake()

    # Start manual climb on entry, but keep handling to manage subsystems and airbrake
    def _handle_climb_manual(self):

        self._stop_shooter()
        self._stop_indexer()
        self._stow_intake()

        # Manual joystick adjustment handled by command

    # Helper Methods

    def _stop_shooter(self):
        if self.hasShooter:
            self.shooter.stop()

    def _stop_indexer(self):
        if self.hasIndexer:
            self.indexer.stop()

    def _stow_intake(self):
        if self.hasIntake:
            self.intake.stow()

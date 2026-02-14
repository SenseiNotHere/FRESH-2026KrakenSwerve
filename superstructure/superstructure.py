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

        elif self.robot_state in [
            RobotState.ELEVATOR_RISING,
            RobotState.ELEVATOR_LOWERING,
            RobotState.AIRBREAK_ENGAGED_UP,
            RobotState.AIRBREAK_ENGAGED_DOWN
        ]:
            self._handle_elevator_states()
        
        elif self.robot_state == RobotState.CLIMB_MANUAL:
            self._handle_climb_manual()
        
        elif self.robot_state in [
            RobotState.INTAKE_DEPLOYED,
            RobotState.INTAKE_RETRACTED
        ]:
            self._handle_intake_position()

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

        if self.hasClimber:
            self.robot_readiness.elevatorAtTarget = (
                self.climber.atTarget()
            )
        else:
            self.robot_readiness.elevatorAtTarget = False

        self.robot_readiness.canFeed = (
            self.robot_state in [
                RobotState.SHOOTING,
                RobotState.SHOOTING_AUTONOMOUS
            ]
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

        oldState = self.robot_state
        self.robot_state = newState

        # Log state transition
        print(f"[Superstructure] {oldState.name} -> {newState.name}")

        SmartDashboard.putString(
            "Superstructure/State",
            newState.name
        )
        
    # State Handlers

    # Handles idle state by ensuring all subsystems are stopped.
    def _handle_idle(self):

        self._stop_shooter()
        self._stop_indexer()

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
            self.indexer.stop()
        
        if self.robot_readiness.shooterReady:
            self.setState(RobotState.SHOOTING)

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
            self.indexer.stop()

    # Start elevator movement on entry, but keep handling to manage subsystems and airbrake
    def _handle_elevator_states(self):

        self._stop_shooter()
        self._stop_indexer()
        self._stow_intake()

        if not self.hasClimber:
            return

        state = self.robot_state

        # Elevator Rising
        if state == RobotState.ELEVATOR_RISING:
            self.climber.releaseAirbrake()
            self.climber.setPosition(ClimberConstants.kClimbHeight)

            if self.robot_readiness.elevatorAtTarget:
                self.climber.engageAirbrake()
                self.setState(RobotState.AIRBREAK_ENGAGED_UP)

        # Elevator Lowering
        elif state == RobotState.ELEVATOR_LOWERING:
            self.climber.releaseAirbrake()
            self.climber.setPosition(ClimberConstants.kMinPosition)

            if self.robot_readiness.elevatorAtTarget:
                self.setState(RobotState.IDLE)

        # Brake engaged up
        elif state == RobotState.AIRBREAK_ENGAGED_UP:
            self.climber.engageAirbrake()

        # Brake engaged down (debug only)
        elif state == RobotState.AIRBREAK_ENGAGED_DOWN:
            self.climber.engageAirbrake()


    def _handle_climb_manual(self):

        # Disable other mechanisms
        self._stop_shooter()
        self._stop_indexer()
        self._stow_intake()

        # DO NOT auto engage brake here.
        # ManualClimb command controls brake + movement.
    
    # Starts intake deployment/retraction on entry, but keeps handling to manage shooter/indexer states and ensure we return to IDLE after action is done
    def _handle_intake_position(self):

        if not self.hasIntake:
            return

        if self.robot_state == RobotState.INTAKE_DEPLOYED:
            self.intake.deploy()

        elif self.robot_state == RobotState.INTAKE_RETRACTED:
            self.intake.stow()

        # After executing, go back to IDLE
        self.setState(RobotState.IDLE)


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

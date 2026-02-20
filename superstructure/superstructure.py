from commands2 import FunctionalCommand
from commands2.button import CommandGenericHID
from wpilib import SmartDashboard, Timer, XboxController
from wpilib.interfaces import GenericHID

from subsystems.shooter.shootersubsystem import Shooter
from subsystems.shooter.shot_calculator import ShotCalculator
from subsystems.intake.intakesubsystem import Intake
from subsystems.climber.climbersubsystem import Climber
from subsystems.shooter.indexersubsystem import Indexer
from subsystems.shooter.agitadorsubsystem import Agitator
from subsystems.vision.limelightcamera import LimelightCamera
from subsystems.drive.drivesubsystem import DriveSubsystem
from subsystems.orchestra.orchestrasubsystem import OrchestraSubsystem

from .robot_state import RobotState, RobotReadiness

from constants.constants import *
from constants.field_constants import *


class Superstructure:

    def __init__(
        self,
        drivetrain: DriveSubsystem | None = None,
        shooter: Shooter | None = None,
        indexer: Indexer | None = None,
        agitator: Agitator | None = None,
        shotCalculator: ShotCalculator | None = None,
        intake: Intake | None = None,
        climber: Climber | None = None,
        vision: LimelightCamera | None = None,
        orchestra: OrchestraSubsystem | None = None,
        driverController: CommandGenericHID | None = None,
        operatorController: CommandGenericHID | None = None
    ):
        self.drivetrain = drivetrain
        self.shooter = shooter
        self.indexer = indexer
        self.agitator = agitator
        self.shotCalculator = shotCalculator
        self.intake = intake
        self.climber = climber
        self.vision = vision
        self.orchestra = orchestra
        self.driverController = driverController
        self.operatorController = operatorController

        # Subsystems
        self.hasShooter = shooter is not None
        self.hasIndexer = indexer is not None
        self.hasAgitator = agitator is not None
        self.hasShotCalc = shotCalculator is not None
        self.hasIntake = intake is not None
        self.hasClimber = climber is not None
        self.hasVision = vision is not None
        self.hasOrchestra = orchestra is not None

        self.robot_state = RobotState.IDLE
        self.robot_readiness = RobotReadiness()

        # Shooter readiness debounce
        self._shooter_ready_since = None

        # Elevator readiness
        self._prev_elevator_at_target = False

        # Rumble controller
        self._rumble_end_time = None

    # Update Loop

    def update(self):
        """
        Superstructure update loop.
        Should be called periodically to update subsystem states.
        """

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

        elif self.robot_state == RobotState.PLAYING_SONG:
            self._handle_playing_song()

        elif self.robot_state == RobotState.PLAYING_CHAMPIONSHIP_SONG:
            self._handle_playing_championship_song()


        # Ensure we stop music if we leave the song state
        if self.robot_state != RobotState.PLAYING_SONG:
            self.orchestra.stop()

        if getattr(self, "_rumble_end_time", None):
            if Timer.getFPGATimestamp() >= self._rumble_end_time:
                self._rumble_controller(
                    self.driverController,
                    XboxController.RumbleType.kBothRumble,
                    0.0
                )
                self._rumble_end_time = None

    # Readiness

    def _update_readiness(self):

        # Shooter readiness
        if self.hasShooter:
            self.robot_readiness.shooterReady = self.shooter.atSpeed(tolerance_rpm=100)
        else:
            self.robot_readiness.shooterReady = False

        # Intake readiness
        if self.hasIntake:
            self.robot_readiness.intakeDeployed = self.intake.isDeployed()
        else:
            self.robot_readiness.intakeDeployed = False

        # Climber readiness
        if self.hasClimber:
            self.robot_readiness.elevatorAtHighTarget = self.climber.atHighTarget()
            self.robot_readiness.elevatorAtLowTarget = self.climber.atLowTarget()
            self.robot_readiness.elevatorAtClimbTarget = self.climber.atClimbTarget()

        # Feeding rule
        self.robot_readiness.canFeed = (
                self.robot_state in [
            RobotState.SHOOTING,
            RobotState.SHOOTING_AUTONOMOUS
        ]
                and self.robot_readiness.shooterReady
                and self.hasIndexer
        )

        # Rumble Controller - Readiness
        current = self.robot_readiness.elevatorAtHighTarget or self.robot_readiness.elevatorAtClimbTarget

        if current and not self._prev_elevator_at_target:
            self._rumble_controller(
                self.driverController,
                XboxController.RumbleType.kRightRumble,
                0.5
            )
            self._rumble_end_time = Timer.getFPGATimestamp() + 0.2

        self._prev_elevator_at_target = current

    # Public State API

    def createStateCommand(self, state: RobotState):
        return FunctionalCommand(
            onInit=lambda: self.setState(state),
            onExecute=lambda: None,
            onEnd=lambda interrupted: self.setState(RobotState.IDLE),
            isFinished=lambda: False
        )

    def setState(self, newState: RobotState):

        if newState == self.robot_state:
            return

        oldState = self.robot_state
        self.robot_state = newState

        # Reset shooter-ready debounce when entering/exiting PREP
        if newState in [
            RobotState.PREP_SHOT,
            RobotState.PREP_SHOT_AUTONOMOUS
        ]:
            self._shooter_ready_since = None
        else:
            self._shooter_ready_since = None

        # Rumble controller - States
        if newState in [
            RobotState.PREP_SHOT,
            RobotState.SHOOTING,
            RobotState.INTAKING,
            RobotState.ELEVATOR_RISING
        ]:
            self._rumble_controller(
                self.driverController,
                XboxController.RumbleType.kBothRumble,
                0.3
            )
            self._rumble_end_time = Timer.getFPGATimestamp() + 0.15

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
        self._stop_intake()
        self._stop_agitator()
        self._stop_orchestra()
        
    # Start intaking on entry
    def _handle_intaking(self):

        self._stop_shooter()

        if self.hasIntake:
            self.intake.startIntaking()

    # Start prep shot on entry
    def _handle_prep_shot(self):

        if not self.hasShooter:
            return

        # Spin up shooter
        if self.hasShotCalc:
            target_rps = self.shotCalculator.getTargetSpeedRPS()
            self.shooter.setTargetRPS(target_rps)
        else:
            self.shooter.useDashboardPercent()

        if self.hasIndexer:
            self.indexer.stop()

        # Debounced transition to SHOOTING
        now = Timer.getFPGATimestamp()

        if self.robot_readiness.shooterReady:
            if self._shooter_ready_since is None:
                self._shooter_ready_since = now
            elif (now - self._shooter_ready_since) >= 0.12:
                self.setState(RobotState.SHOOTING)
        else:
            self._shooter_ready_since = None

    # Start shooting on entry
    def _handle_shooting(self):

        if not self.hasShooter:
            return

        # Keep spinning
        if self.hasShotCalc:
            target_rps = self.shotCalculator.getTargetSpeedRPS()
            self.shooter.setTargetRPS(target_rps)
        else:
            self.shooter.useDashboardPercent()

        # Feed ONLY when ready
        if self.robot_readiness.canFeed:
            self.indexer.feed()
            if self.hasAgitator:
                self.agitator.feed()
        else:
            if self.hasIndexer:
                self.indexer.stop()
            if self.hasAgitator:
                self.agitator.stop()

    # Start elevator movement on entry
    def _handle_elevator_states(self):
        self._stop_shooter()
        self._stop_indexer()
        self._stop_intake()

        if not self.hasClimber:
            return

        state = self.robot_state

        # Max / Rise Height
        if state == RobotState.ELEVATOR_RISING:
            self.climber.releaseAirbrake()
            self.climber.setPosition(ClimberConstants.kRisenHeight)

        # Mid / Lower Height
        elif state == RobotState.ELEVATOR_LOWERING:
            self.climber.releaseAirbrake()
            self.climber.setPosition(ClimberConstants.kClimbedHeight)

        # Minimum Height
        elif state == RobotState.ELEVATOR_MINIMUM:
            self.climber.releaseAirbrake()
            self.climber.setPosition(ClimberConstants.kMinPosition)

        # Airbrake engaged (up)
        elif state == RobotState.AIRBREAK_ENGAGED_UP:
            self.climber.engageAirbrake()

        # Airbrake engaged (down / debug)
        elif state == RobotState.AIRBREAK_ENGAGED_DOWN:
            self.climber.engageAirbrake()

    # Start manual climb on entry
    def _handle_climb_manual(self):

        # Disable other mechanisms
        self._stop_shooter()
        self._stop_indexer()
        self._stop_intake()

        # DO NOT auto engage brake here.
        # ManualClimb command controls brake + movement.

    # Starts intake deployment/retraction on entry
    def _handle_intake_position(self):

        if not self.hasIntake:
            return

        if self.robot_state == RobotState.INTAKE_DEPLOYED:
            self.intake.deploy()

        elif self.robot_state == RobotState.INTAKE_RETRACTED:
            self.intake.stow()

        # After executing, go back to IDLE
        self.setState(RobotState.IDLE)

    # Start song on entry
    def _handle_playing_song(self):

        self.orchestra.play_selected_song()

    # Start championship song on entry
    def _handle_playing_championship_song(self):
        self.orchestra.play_championship_song()

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
            
    def _stop_intake(self):
        if self.hasIntake:
            self.intake.stop()
    
    def _stop_orchestra(self):
        self.orchestra.stop()
        
    def _stop_agitator(self):
        if self.hasAgitator:
            self.agitator.stop()

    @staticmethod
    def _rumble_controller(
            controller,
            rumble_type: XboxController.RumbleType,
            rumble_value: float
    ):
        if controller is None:
            return

        controller.getHID().setRumble(
            rumble_type,
            rumble_value
        )
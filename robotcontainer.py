from __future__ import annotations

import typing
import commands2

from commands2 import InstantCommand
from commands2.button import CommandGenericHID

from pathplannerlib.auto import NamedCommands

from wpilib import (
    XboxController,
    SmartDashboard, SendableChooser
)

from wpimath.geometry import (
    Rotation2d,
    Translation3d,
)

from subsystems.drive.drivesubsystem import DriveSubsystem, BadSimPhysics, AutoBuilder
from subsystems.vision.limelightcamera import LimelightCamera
from subsystems.vision.limelight_localizer import LimelightLocalizer
from subsystems.shooter.shootersubsystem import Shooter
from subsystems.shooter.indexersubsystem import Indexer
from subsystems.climber.climbersubsystem import Climber
from subsystems.intake.intakesubsystem import Intake
from subsystems.pneumatics.pneumaticssubsystem import Pneumatics
from subsystems.shooter.shot_calculator import ShotCalculator
from subsystems.orchestra.orchestrasubsystem import OrchestraSubsystem
from superstructure.superstructure import Superstructure
from superstructure.robot_state import RobotState
from commands.drive.holonomic_drive import HolonomicDrive

from commands.climber.climber_commands import ManualClimb

from buttonbindings import ButtonBindings

from constants.constants import (
    OIConstants,
    ShooterConstants,
    IndexerConstants,
    ClimberConstants,
    IntakeConstants,
    PneumaticsConstants
)


class RobotContainer:
    """
    The container for the robot. Subsystems are initialized here,
    button bindings are set up, and auto chooser is sent to the dashboard.
    """

    def __init__(self, robot):

        # Drive Subsystem

        self.robotDrive = DriveSubsystem()

        if commands2.TimedCommandRobot.isSimulation():
            self.robotDrive.simPhysics = BadSimPhysics(self.robotDrive, robot)

        # Shot Calculator

        self.shotCalculator = ShotCalculator(
            drivetrain=self.robotDrive
        )

        # Auto Chooser

        self.autoChooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("Auto Chooser", self.autoChooser)

        # Test Chooser
        self.testChooser = SendableChooser()
        SmartDashboard.putData("Test Chooser", self.testChooser)

        # Controllers

        self.driverController = CommandGenericHID(
            OIConstants.kDriverControllerPort
        )
        self.operatorController = CommandGenericHID(
            OIConstants.kOperatorControllerPort
        )

        # Vision / Localization

        self.localizer = LimelightLocalizer(
            drivetrain=self.robotDrive,
            flipIfRed=True,
        )

        self.limelight = LimelightCamera("limelight-front")
        self.limelight.setPiPMode(1)
        self.limelightBack = LimelightCamera("limelight-back")

        self.localizer.addCamera(
            camera=self.limelight,
            cameraPoseOnRobot=Translation3d(0.0, 0.0, 0.0),
            cameraHeadingOnRobot=Rotation2d.fromDegrees(0),
            minPercentFrame=0.07,
            maxRotationSpeed=720,
        )

        self.localizer.addCamera(
            camera=self.limelightBack,
            cameraPoseOnRobot=Translation3d(0.0, 0.0, 0.0),
            cameraHeadingOnRobot=Rotation2d.fromDegrees(180),
            minPercentFrame=0.07,
            maxRotationSpeed=720,
        )

        # Default Drive Command

        self.robotDrive.setDefaultCommand(
            HolonomicDrive(
                self.robotDrive,
                forwardSpeed=lambda: -self.driverController.getRawAxis(
                    XboxController.Axis.kLeftY
                ),
                leftSpeed=lambda: -self.driverController.getRawAxis(
                    XboxController.Axis.kLeftX
                ),
                rotationSpeed=lambda: self.driverController.getRawAxis(
                    XboxController.Axis.kRightX
                ),
                deadband=OIConstants.kDriveDeadband,
                fieldRelative=True,
                rateLimit=False,
                square=True,
            )
        )

        # Pneumatics

        self.pneumatics = Pneumatics(
            moduleID=PneumaticsConstants.kPCMID,
            moduleType=PneumaticsConstants.kModuleType
        )

        # Subsystems

        self.shooter = Shooter(
            motorCANID=ShooterConstants.kShooterMotorID,
            motorInverted=True,
        )

        self.indexer = Indexer(
            motorCANID=IndexerConstants.kIndexerMotorID,
            motorInverted=True,
        )

        self.intake = Intake(
            motorCANID=IntakeConstants.kIntakeMotorCANID,
            motorInverted=False,
            solenoidModuleID=IntakeConstants.kSolenoidModuleID,
            pneumaticsModuleType=IntakeConstants.kPneumaticsModuleType,
            forwardChannel=IntakeConstants.kSolenoidForwardChannel,
            reverseChannel=IntakeConstants.kSolenoidReverseChannel
        )

        self.climber = Climber(
            motorCANID=ClimberConstants.kMotorID,
            motorInverted=ClimberConstants.kMotorInverted,
            solenoidCANID=ClimberConstants.kPCMID,
            pneumaticsModuleType=ClimberConstants.kPneumaticsModuleType,
            forwardChannel=ClimberConstants.kForwardChannel,
            reverseChannel=ClimberConstants.kReverseChannel,
            canCoderCANID=ClimberConstants.kCanCoderCANID,
            canCoderInverted=False
        )

        self.orchestra = OrchestraSubsystem(
            self.robotDrive,
            self.climber,
            self.shooter,
        )
 
        # Superstructure (MUST BE LAST)

        self.superstructure = Superstructure(
            drivetrain=self.robotDrive,
            shooter=self.shooter,
            indexer=self.indexer,
            shotCalculator=self.shotCalculator,
            intake=self.intake,
            climber=self.climber,
            vision=self.limelight,
            orchestra=self.orchestra,
            driverController=self.driverController,
            operatorController=self.operatorController
        )

        # Button Bindings

        manual = ManualClimb(
            self.superstructure,
            lambda: self.operatorController.getRawAxis(
                XboxController.Axis.kLeftY
            )
        )

        # Down (positive values)
        self.operatorController.axisGreaterThan(
            XboxController.Axis.kLeftY, 0.1
        ).whileTrue(manual)

        # Up (negative values)
        self.operatorController.axisLessThan(
            XboxController.Axis.kLeftY, -0.1
        ).whileTrue(manual)


        self.buttonBindings = ButtonBindings(self)
        self.buttonBindings.configureButtonBindings()

        # PathPlanner Lib command register
        NamedCommands.registerCommand('Deploy Intake', self.superstructure.createStateCommand(RobotState.INTAKE_DEPLOYED))
        NamedCommands.registerCommand('Start Intaking', self.superstructure.createStateCommand(RobotState.INTAKING))
        NamedCommands.registerCommand('PREP_SHOT State', self.superstructure.createStateCommand(RobotState.PREP_SHOT))
        NamedCommands.registerCommand('IDLE State', self.superstructure.createStateCommand(RobotState.IDLE))
        NamedCommands.registerCommand('Elevator to Max', self.superstructure.createStateCommand(RobotState.ELEVATOR_RISING))
        NamedCommands.registerCommand('Elevator to Climbed', self.superstructure.createStateCommand(RobotState.ELEVATOR_LOWERING))

    # Autonomous

    def getAutonomousCommand(self) -> commands2.Command:
        command = self.autoChooser.getSelected()

        if command is None:
            print("WARNING: No autonomous routines selected!")
            return InstantCommand()

        print(f"Running autonomous routine: {command.getName()}")
        return command

    # Test Mode
    def getTestCommand(self) -> typing.Optional[commands2.Command]:
        self.testChooser.setDefaultOption("None", None)
        return self.testChooser.getSelected()
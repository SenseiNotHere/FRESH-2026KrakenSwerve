from __future__ import annotations

import typing
import commands2

from commands2 import InstantCommand
from commands2.button import CommandGenericHID

from wpilib import (
    XboxController,
    SmartDashboard,
    SendableChooser
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
from commands.drive.holonomic_drive import HolonomicDrive
from buttonbindings import ButtonBindings

from commands.climber.climber_commands import ManualClimb

from constants.constants import (
    OIConstants,
    ShooterConstants,
    IndexerConstants,
    ClimberConstants,
    IntakeConstants,
    PneumaticsConstants
)
import tests


class RobotContainer:
    """
    The container for the robot. Subsystems are initialized here,
    button bindings are set up, and auto chooser is sent to the dashboard.
    """

    def __init__(self, robot):
        # Subsystems
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

        # Song Chooser
        self.songChooser = SendableChooser()
        self.songChooser.setDefaultOption(
            "Bloodline - Ariana Grande",
            "/home/lvuser/py/deploy/files/Bloodline.chrp",
        )
        self.songChooser.addOption(
            "Yes And? - Ariana Grande",
            "/home/lvuser/py/deploy/files/Yesand.chrp",
        )
        self.songChooser.addOption(
            "Lavender Town",
            "/home/lvuser/py/deploy/files/LavenderTown.chrp",
        )
        self.songChooser.addOption(
            "Espresso - Sabrina Carpenter",
            "/home/lvuser/py/deploy/files/Espresso.chrp",
        )
        self.songChooser.addOption(
            "Needy - Ariana Grande",
            "/home/lvuser/py/deploy/files/Needy.chrp",
        )
        SmartDashboard.putData("Song Selection", self.songChooser)

        # Controllers
        self.driverController = CommandGenericHID(
            OIConstants.kDriverControllerPort
        )
        self.operatorController = CommandGenericHID(1)

        # Vision / Localization
        self.localizer = LimelightLocalizer(
            drivetrain=self.robotDrive,
            flipIfRed=True,
        )

        self.limelight = LimelightCamera("limelight-front")
        self.limelightBack = LimelightCamera("limelight-back")
        self.limelightBack.setPiPMode(1)

        self.localizer.addCamera(
            camera=self.limelight,
            cameraPoseOnRobot=Translation3d(0.0, 0.0, 0.0),
            cameraHeadingOnRobot=Rotation2d.fromDegrees(0),
            minPercentFrame=0.07,
            maxRotationSpeed=720,
        )

        self.localizer.addCamera(
            camera=self.limelightBack,
            cameraPoseOnRobot=Translation3d(-18.0, 0.0, 0.0),
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

        self.pneumatics = Pneumatics(
            moduleID=PneumaticsConstants.kPCMID,
            moduleType=PneumaticsConstants.kModuleType
        )

        # Shooter / Indexer
        if ShooterConstants.kShooterEnabled:
            self.shooter = Shooter(
                motorCANID=ShooterConstants.kShooterMotorID,
                motorInverted=False,
            )

        if IndexerConstants.kIndexerEnabled:
            self.indexer = Indexer(
                motorCANID=IndexerConstants.kIndexerMotorID,
                motorInverted=False,
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

        self.climber.setDefaultCommand(
            ManualClimb(
                self.climber,
                joystick=lambda: -self.operatorController.getRawAxis(
                    XboxController.Axis.kRightY
                )
            )
        )

        if IntakeConstants.kIntakeEnabled:
            self.intake = Intake(
                motorCANID=IntakeConstants.kIntakeMotorCANID,
                motorInverted=False,
                solenoidModuleID=IntakeConstants.kSolenoidModuleID,
                pneumaticsModuleType=IntakeConstants.kPneumaticsModuleType,
                forwardChannel=IntakeConstants.kSolenoidForwardChannel,
                reverseChannel=IntakeConstants.kSolenoidReverseChannel
            )

        # Orchestra

        self.orchestra = OrchestraSubsystem(driveSubsystem=self.robotDrive)

        # Superstructure - MUST BE LAST TO INITIALIZE!
 #       self.superstructure = Superstructure(
 #           drivetrain=self.robotDrive,
 #           shooter=self.shooter,
 #           shotCalculator=self.shotCalculator,
 #           indexer=self.indexer,
 #           climber=self.climber,
 #           vision=self.limelight
 #       )

        # Button Bindings
        self.buttonBindings = ButtonBindings(self)
        self.buttonBindings.configureButtonBindings()

    def disablePIDSubsystems(self):
        """Disables all PID subsystems."""
        pass

    def getAutonomousCommand(self) -> commands2.Command:
        command = self.autoChooser.getSelected()

        if command is None:
            print("WARNING: No autonomous routines selected!")
            return InstantCommand()

        print(f"Running autonomous routine: {command.getName()}")
        return command

    def getTestCommand(self) -> typing.Optional[commands2.Command]:
        self.testChooser.setDefaultOption("None", None)

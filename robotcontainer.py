from __future__ import annotations

import math
import typing

import wpilib
import wpimath
import commands2

from commands2 import cmd, InstantCommand, RunCommand
from commands2.button import CommandGenericHID

from wpilib import (
    XboxController,
    PS4Controller,
    SmartDashboard,
    DriverStation,
    DutyCycle,
)

from wpimath.controller import (
    PIDController,
    ProfiledPIDControllerRadians,
    HolonomicDriveController,
)

from wpimath.geometry import (
    Pose2d,
    Rotation2d,
    Translation2d,
    Translation3d,
)

from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveModuleState,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
)

from subsystems.drivesubsystem import DriveSubsystem, BadSimPhysics, AutoBuilder
from subsystems.phoenixswervemodule import PhoenixSwerveModule
from subsystems.limelightcamera import LimelightCamera
from subsystems.limelight_localizer import LimelightLocalizer
from subsystems.shootersubsystem import Shooter
from subsystems.indexersubsystem import Indexer
from subsystems.shotcalculator import ShotCalculator
from subsystems.orchestrasubsystem import OrchestraSubsystem

from commands.holonomicDrive import HolonomicDrive
from buttonbindings import ButtonBindings

from constants import *
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
        self.testChooser = wpilib.SendableChooser()

        # Song Chooser
        self.songChooser = wpilib.SendableChooser()
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

        # Orchestra

        self.orchestra = OrchestraSubsystem(driveSubsystem=self.robotDrive)

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
        self.testChooser.addOption(
            "Drivetrain",
            InstantCommand(
                tests.drivetrainTest.testDrive(self.robotDrive)
            ),
        )

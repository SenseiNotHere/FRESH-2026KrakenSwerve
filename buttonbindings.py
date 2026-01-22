from wpilib import XboxController, PS4Controller
from wpimath.geometry import Pose2d, Rotation2d
from commands2 import cmd, InstantCommand, RunCommand
from commands2.button import CommandGenericHID
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.limelightcamera import LimelightCamera
from subsystems.orchestrasubsystem import OrchestraSubsystem
from constants import *
from fieldConstants import AprilTags

from commands.reset_XY import ResetXY, ResetSwerveFront
from commands.followObject import FollowObject
from commands.limelightComands import SetCameraPipeline
from commands.drive_torwards_object import SwerveTowardsObject
from commands.point_torwards_location import PointTowardsLocation

class ButtonBindings:
    def __init__(self, robot_container):
        """Initialize ButtonBindings with access to the robot container
        """
        self.robotContainer = robot_container
        self.robotDrive = robot_container.robotDrive
        self.driverController = robot_container.driverController
        self.limelight = robot_container.limelight
        if ShooterConstants.kShooterEnabled:
            self.shooter = robot_container.shooter
        if IndexerConstants.kIndexerEnabled:
            self.indexer = robot_container.indexer

        self.orchestra = robot_container.orchestra


    def configureButtonBindings(self):
        """Configure button bindings for the robot."""

        # Driver Controls
        # Reset XY Position
        povUpDriverButton = self.driverController.pov(0)
        povUpDriverButton.onTrue(
            ResetXY(
                x=0.0,
                y=0.0,
                headingDegrees=0.0,
                drivetrain=self.robotDrive
            )
        )

        # Reset Swerve Front
        povDownDriverButton = self.driverController.pov(180)
        povDownDriverButton.onTrue(ResetSwerveFront(self.robotDrive))

        # X-Break
        povLeftDriverButton = self.driverController.pov(270)
        povLeftDriverButton.whileTrue(InstantCommand(self.robotDrive.setX, self.robotDrive))

        # Play selected song
        bButton = self.driverController.button(XboxController.Button.kB)
#        bButton.onTrue(InstantCommand(lambda: self.orchestra.loadSound(self.robotContainer.songChooser.getSelected())))
#        bButton.onTrue(InstantCommand(lambda: self.orchestra.playSound()))

        # Stop song
#        aButton = self.driverController.button(XboxController.Button.kA)
#        aButton.onTrue(InstantCommand(lambda: self.robotDrive.stopSound()))

        # Point torwards currently looking tag
        aButton = self.driverController.button(XboxController.Button.kA)
        aButton.whileTrue(
            PointTowardsLocation(
                drivetrain=self.robotDrive,
                location=lambda: AprilTags.APRIL_TAG_POSITIONS.get(
                    self.limelight.getAprilTagID()
                ),
                locationIfRed=lambda: AprilTags.APRIL_TAG_POSITIONS.get(
                    self.limelight.getRedAprilTagID()
                )
            )
        )

        # Shooter + Indexer
        xButton = self.driverController.button(XboxController.Button.kX)

        # Shooter only
        if ShooterConstants.kShooterEnabled and not IndexerConstants.kIndexerEnabled:
            # Shooter
            xButton.whenTrue(InstantCommand(lambda: self.robotContainer.shooter.enable()))
            xButton.whenFalse(InstantCommand(lambda: self.robotContainer.shooter.disable()))

        # Shooter + Indexer
        elif ShooterConstants.kShooterEnabled and IndexerConstants.kIndexerEnabled:
            # Shooter
            xButton.whenTrue(InstantCommand(lambda: self.shooter.enable()))
            xButton.whenFalse(InstantCommand(lambda: self.shooter.disable()))

            # Indexer
            xButton.whileTrue(InstantCommand(lambda: self.indexer.enable() if self.shooter.atSpeed() else print("Not ready to index yet")))
            xButton.whenFalse(InstantCommand(lambda: self.indexer.disable()))

            # elif statement end

        # Swerve to Object
        yButton = self.driverController.button(XboxController.Button.kY)
        yButton.whileTrue(
            SwerveTowardsObject(
                self.robotDrive,
                speed=lambda: -1.0,
                camera=self.limelight,
                cameraLocationOnRobot=Pose2d(-0.4, 0, Rotation2d.fromDegrees(180))
            ))
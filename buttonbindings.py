from wpilib import XboxController, PS4Controller, SmartDashboard
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
#        bButton = self.driverController.button(XboxController.Button.kB)
#        bButton.onTrue(InstantCommand(lambda: self.orchestra.loadSound(self.robotContainer.songChooser.getSelected())))
#        bButton.onTrue(InstantCommand(lambda: self.orchestra.playSound()))

        # Stop song
#        aButton = self.driverController.button(XboxController.Button.kA)
#        aButton.onTrue(InstantCommand(lambda: self.robotDrive.stopSound()))

        # Climber
        if ClimberConstants.kClimberEnabled:
            bButton = self.driverController.button(XboxController.Button.kB)
            bButton.whenTrue(InstantCommand(lambda: self.robotContainer.climber.toggle()))

        # Point torwards currently looking tag
        aButton = self.driverController.button(XboxController.Button.kA)
        aButton.whileTrue(
            PointTowardsLocation(
                drivetrain=self.robotDrive,
                location=lambda: self._log_and_get_april_tag_position(self.limelight.getAprilTagID(), "getAprilTagID"),
                locationIfRed=lambda: self._log_and_get_april_tag_position(self.limelight.getRedAprilTagID(), "getRedAprilTagID")
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

        # Swerve to Object using left trigger button of the joystick
        driveToGamepiece = SwerveTowardsObject(
            drivetrain=self.robotDrive,
            speed=lambda: -self.driverController.getRawAxis(XboxController.Axis.kLeftTrigger),  # speed controlled by "left trigger" stick of the joystick
            camera=self.limelight,
            cameraLocationOnRobot=Pose2d(-0.4, 0, Rotation2d.fromDegrees(180)),
            # ^^ camera located at the rear middle looking straight back
        )

        # setup a condition for when to run that command
        self.driverController.axisGreaterThan(
            XboxController.Axis.kLeftTrigger, threshold=0.05
        ).whileTrue(driveToGamepiece)

    def _log_and_get_april_tag_position(self, tag_id_callable, tag_id_name):
        tag_id = tag_id_callable()
        SmartDashboard.putString(
            f"command/c{self.__class__.__name__}/{tag_id_name}",
            f"Tag ID: {tag_id}"
        )
        position = AprilTags.APRIL_TAG_POSITIONS.get(tag_id)
        SmartDashboard.putString(
            f"command/c{self.__class__.__name__}/{tag_id_name}_position",
            f"Position: {position}"
        )
        return position

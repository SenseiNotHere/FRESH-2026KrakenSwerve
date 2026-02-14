from wpilib import XboxController, SmartDashboard
from wpimath.geometry import Pose2d, Rotation2d
from commands2 import InstantCommand

from superstructure.robot_state import RobotState

from constants.constants import *
from constants.field_constants import AprilTags

from commands.drive.reset_xy import ResetXY, ResetSwerveFront
from commands.auto.drive_torwards_object import SwerveTowardsObject
from commands.drive.point_torwards_location import PointTowardsLocation
from commands.climber.climber_commands import ToggleClimber
from commands.vision.limelight_comands import SetCameraPipeline
from commands.climber.climber_commands import ToggleClimber
from commands.shooter.shoot_index_commands import ShootWithIndexer
from commands.shooter.shooter_commands import RunShooter


class ButtonBindings:
    def __init__(self, robot_container):
        """Initialize ButtonBindings with access to the robot container."""
        self.robotContainer = robot_container

        # Core subsystems
        self.robotDrive = robot_container.robotDrive
#       self.superstructure = robot_container.superstructure
        self.driverController = robot_container.driverController
        self.operatorController = robot_container.operatorController
        self.limelight = robot_container.limelight
        self.orchestra = robot_container.orchestra

        # Optional subsystems
        self.shooter = getattr(robot_container, "shooter", None)
        self.indexer = getattr(robot_container, "indexer", None)
        self.climber = getattr(robot_container, "climber", None)

    # Main Binding Configuration

    def configureButtonBindings(self):
        self._configureDriverBindings()
        self._configureOperatorBindings()

    # Driver Controls

    def _configureDriverBindings(self):

        # Reset Controls

        self.driverController.pov(0).onTrue(
            ResetXY(
                x=0.0,
                y=0.0,
                headingDegrees=0.0,
                drivetrain=self.robotDrive
            )
        )

        self.driverController.pov(180).onTrue(
            ResetSwerveFront(self.robotDrive)
        )

        self.driverController.pov(270).whileTrue(
            InstantCommand(self.robotDrive.setX, self.robotDrive)
        )

        # Climber

        if ClimberConstants.kClimberEnabled:
            self.driverController.button(
                XboxController.Button.kB
            ).onTrue(ToggleClimber(self.robotContainer.climber))

        # Point Toward AprilTag

        self.driverController.button(
            XboxController.Button.kA
        ).whileTrue(
            PointTowardsLocation(
                drivetrain=self.robotDrive,
                location=lambda: self._get_apriltag_position(
                    self.limelight.getAprilTagID,
                    "getAprilTagID"
                ),
                locationIfRed=lambda: self._get_apriltag_position(
                    self.limelight.getRedAprilTagID,
                    "getRedAprilTagID"
                )
            )
        )

        xButton = self.driverController.button(XboxController.Button.kX)

        # Shooter only
        if ShooterConstants.kShooterEnabled and not IndexerConstants.kIndexerEnabled:
            xButton.whileTrue(RunShooter(self.shooter))

        # Shooter + Indexer
        elif ShooterConstants.kShooterEnabled and IndexerConstants.kIndexerEnabled:
            xButton.whileTrue(
                ShootWithIndexer(
                    self.shooter,
                    self.indexer,
                    tolerance_rpm=100.0  # adjust as needed
                )
            )

#        xButton.whileTrue(
#           self.superstructure.createStateCommand(RobotState.SHOOTING)
#       )

        # Swerve Toward Gamepiece

        driveToGamepiece = SwerveTowardsObject(
            drivetrain=self.robotDrive,
            speed=lambda: -self.driverController.getRawAxis(
                XboxController.Axis.kLeftTrigger
            ),
            camera=self.limelight,
            cameraLocationOnRobot=Pose2d(
                -0.4,
                0,
                Rotation2d.fromDegrees(180)
            ),
        )

        self.driverController.axisGreaterThan(
            XboxController.Axis.kLeftTrigger,
            threshold=0.05
        ).whileTrue(driveToGamepiece)

    # Operator Controls

    def _configureOperatorBindings(self):

        operatorX = self.operatorController.button(XboxController.Button.kX)
        operatorX.whileTrue(InstantCommand(lambda: ToggleClimber(self.climber)))

        operatorA = self.operatorController.button(
            XboxController.Button.kA
        )

        operatorA.whileTrue(
            SetCameraPipeline(
                camera=self.limelight,
                pipelineIndex=1
            ).andThen(
                SwerveTowardsObject(
                    camera=self.limelight,
                    drivetrain=self.robotDrive,
                    speed=lambda: 0.5
                ).withTimeout(5)
            ).andThen(
                ToggleClimber(self.robotContainer.climber)
            )
        )

    # Helpers

    def _get_apriltag_position(self, tag_id_callable, tag_name):
        tag_id = tag_id_callable()

        SmartDashboard.putString(
            f"command/c{self.__class__.__name__}/{tag_name}",
            f"Tag ID: {tag_id}"
        )

        position = AprilTags.APRIL_TAG_POSITIONS.get(tag_id)

        SmartDashboard.putString(
            f"command/c{self.__class__.__name__}/{tag_name}_position",
            f"Position: {position}"
        )

        return position

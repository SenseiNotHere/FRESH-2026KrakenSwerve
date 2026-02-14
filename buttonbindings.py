from wpilib import XboxController, SmartDashboard
from wpimath.geometry import Pose2d, Rotation2d
from commands2 import InstantCommand

from superstructure.robot_state import RobotState

from constants.constants import *
from constants.field_constants import AprilTags

from commands.drive.reset_xy import ResetXY, ResetSwerveFront
from commands.auto.drive_torwards_object import SwerveTowardsObject
from commands.drive.point_torwards_location import PointTowardsLocation
from commands.vision.limelight_comands import SetCameraPipeline


class ButtonBindings:
    def __init__(self, robot_container):
        """Initialize ButtonBindings with access to the robot container."""
        self.robotContainer = robot_container

        # Core subsystems
        self.robotDrive = robot_container.robotDrive
        self.superstructure = robot_container.superstructure
        self.driverController = robot_container.driverController
        self.operatorController = robot_container.operatorController
        self.limelight = robot_container.limelight
        self.orchestra = robot_container.orchestra

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

        # Auto Shoot (X Button)

        self.driverController.button(
            XboxController.Button.kX
        ).whileTrue(
            self.superstructure.createStateCommand(
                RobotState.SHOOTING
            )
        )

        # Prep Shot (A Button)

        self.driverController.button(
            XboxController.Button.kA
        ).whileTrue(
            self.superstructure.createStateCommand(
                RobotState.PREP_SHOT
            )
        )

        # Intake (B Button)

        self.driverController.button(
            XboxController.Button.kB
        ).whileTrue(
            self.superstructure.createStateCommand(
                RobotState.INTAKING
            )
        )

        # Climb Auto (Y Button)

        self.driverController.button(
            XboxController.Button.kY
        ).onTrue(
            InstantCommand(
                lambda: self.superstructure.setState(
                    RobotState.CLIMB_AUTO
                )
            )
        )

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

        # Manual Climb (Left Stick Press example)

        self.operatorController.button(
            XboxController.Button.kLeftStick
        ).whileTrue(
            InstantCommand(
                lambda: self.superstructure.setState(
                    RobotState.CLIMB_MANUAL
                )
            )
        )

        # Vision Assisted Climb Sequence Example

        self.operatorController.button(
            XboxController.Button.kA
        ).whileTrue(
            SetCameraPipeline(
                camera=self.limelight,
                pipelineIndex=1
            ).andThen(
                SwerveTowardsObject(
                    camera=self.limelight,
                    drivetrain=self.robotDrive,
                    speed=lambda: 0.5
                ).withTimeout(5)
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

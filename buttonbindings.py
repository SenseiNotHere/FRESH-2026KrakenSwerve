from wpilib import XboxController, SmartDashboard
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from commands2 import InstantCommand

from commands.climber.climber_commands import HoldAirbrake, ManualClimb, ToggleClimbAuto
from commands.intake.intake_commands import DeployAndRunIntake, ReverseIntake
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

        # Climb Auto (Left Bumper)
        self.driverController.button(
            XboxController.Button.kLeftBumper
        ).onTrue(
            ToggleClimbAuto(self.robotContainer.superstructure)
        )

        # Engage Airbrakes (Right Bumper)
        self.driverController.button(
            XboxController.Button.kRightBumper
        ).whileTrue(
                HoldAirbrake(self.robotContainer.climber)
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

        # create a command for keeping the robot nose pointed 45 degrees (for traversing the hump on a swerve drive)
        keepNoseAt45Degrees = PointTowardsLocation(
            drivetrain=self.robotDrive,
            location=Translation2d(x=999999, y=999999),  # if we want 50 degrees or 40 degrees, change the ratio of x/y
            locationIfRed=Translation2d(x=-999999, y=-999999),
        )
        self.driverController.button(XboxController.Button.kRightBumper).whileTrue(keepNoseAt45Degrees)
        # ^^ set up a condition for when to do this: do it when the joystick right trigger is pressed by more than 50%

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
        return position    # Operator Controls

    def _configureOperatorBindings(self):

        # Deploy and Run Intake (Right Bumper)
        self.operatorController.button(
            XboxController.Button.kRightBumper
        ).whileTrue(
            DeployAndRunIntake(self.robotContainer.superstructure)
        )

        # Run intake in reverse (Left Bumper)
        self.operatorController.button(
            XboxController.Button.kLeftBumper
        ).whileTrue(
            ReverseIntake(self.robotContainer.superstructure)
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

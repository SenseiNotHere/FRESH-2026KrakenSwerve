from commands2 import Command
from pathplannerlib.path import PathConstraints

from subsystems.drive.drivesubsystem import AutoBuilder, DriveSubsystem
from constants.field_constants import Tower

class ApproachBlueTowerBottom(Command):
    def __init__(self, drivetrain: DriveSubsystem, targetPose=None):
        super().__init__()
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)

        self.constraints = PathConstraints(
            3.0,  # max velocity
            3.0,  # max acceleration
            540.0, # max angular velocity (deg/s)
            720.0  # max angular acceleration (deg/s²)
        )

        self.targetPose = targetPose or Tower.BLUE_TOWER_CLIMB_BOTTOM
        self.pathCommand = None

    def initialize(self):
        self.pathCommand = AutoBuilder.pathfindToPose(
            self.targetPose,
            self.constraints
        )
        self.pathCommand.initialize()

    def execute(self):
        self.pathCommand.execute()

    def end(self, interrupted: bool):
        self.pathCommand.end(interrupted)
        self.drivetrain.stop()

    def isFinished(self):
        return self.pathCommand.isFinished()

class ApproachBlueTowerTop(ApproachBlueTowerBottom):
    def __init__(self, drivetrain: DriveSubsystem):
        super().__init__(drivetrain, targetPose=Tower.BLUE_TOWER_CLIMB_TOP)

class ApproachRedTowerBottom(ApproachBlueTowerBottom):
    def __init__(self, drivetrain: DriveSubsystem):
        super().__init__(drivetrain, targetPose=Tower.RED_TOWER_CLIMB_BOTTOM)

class ApproachRedTowerTop(ApproachBlueTowerBottom):
    def __init__(self, drivetrain: DriveSubsystem):
        super().__init__(drivetrain, targetPose=Tower.RED_TOWER_CLIMB_TOP)
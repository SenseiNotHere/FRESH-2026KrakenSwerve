from commands2 import Subsystem
from wpimath.geometry import Pose2d, Pose3d

from constants.field_constants import Hub
from constants.constants import ShooterConstants


class ShotCalculator(Subsystem):
    """
    Computes distance-based shooter speed and yaw.
    Does NOT control shooter. Pure calculation subsystem.

    Credits to FRC Team 868 - TechHOUNDS
    """

    def __init__(self, drivetrain):
        super().__init__()

        self.drivetrain = drivetrain

        self.target_location: Pose3d = Hub.CENTER

        # Computed outputs
        self._target_distance: float = 0.0
        self._target_speed_rps: float = 0.0
        self._effective_target_pose: Pose3d = Hub.CENTER
        self._effective_yaw: float = 0.0

    # Periodic

    def periodic(self):

        drivetrain_pose: Pose2d = self.drivetrain.getPose()

        # 2D Distance
        self._target_distance = (
            drivetrain_pose.translation()
            .distance(self.target_location.toPose2d().translation())
        )

        # Distance -> Speed Lookup
        lookup = ShooterConstants.DISTANCE_TO_RPS
        self._target_speed_rps = lookup.get(self._target_distance)

        # Effective target (future SOTM logic goes here)
        self._effective_target_pose = self.target_location

        relative_pose = (
            self.target_location
            .toPose2d()
            .relativeTo(drivetrain_pose)
        )

        self._effective_yaw = relative_pose.rotation().radians()

    # Public API

    def getTargetDistance(self) -> float:
        return self._target_distance

    def getTargetSpeedRPS(self) -> float:
        return self._target_speed_rps

    def getEffectiveTargetPose(self) -> Pose3d:
        return self._effective_target_pose

    def getEffectiveYaw(self) -> float:
        return self._effective_yaw

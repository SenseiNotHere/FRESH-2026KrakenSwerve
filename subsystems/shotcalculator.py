from commands2 import Subsystem
from wpimath.geometry import Pose2d, Pose3d
from wpimath.kinematics import ChassisSpeeds

from fieldConstants import Hub
from constants import ShooterConstants


class ShotCalculator(Subsystem):
    """
    Computes distance-based shooter speed and (optionally) effective target
    for shot-on-the-fly. No hood assumed.
    """

    def __init__(self, drivetrain):
        super().__init__()

        self.drivetrain = drivetrain

        # Target (static hub center)
        self.target_location: Pose3d = Hub.CENTER

        # Outputs
        self.target_distance: float = 0.0
        self.target_speed_rps: float = 0.0

        # Optional SOTM outputs
        self.current_effective_target_pose: Pose3d = Hub.CENTER
        self.current_effective_yaw: float = 0.0

    # Periodic

    def periodic(self):
        """
        Recompute target distance and shooter speed every loop.
        """

        drivetrain_pose: Pose2d = self.drivetrain.getPose()

        # Distance from robot to hub (2D)
        self.target_distance = (
            drivetrain_pose.translation()
            .distance(self.target_location.toPose2d().translation())
        )

        # Distance -> shooter speed (RPS)
        self.target_speed_rps = ShooterConstants.DISTANCE_TO_RPS.get(
            self.target_distance
        )

        # For now, effective target == real target
        self.current_effective_target_pose = self.target_location

        # Optional yaw calc
        relative_pose = self.target_location.toPose2d().relativeTo(drivetrain_pose)
        self.current_effective_yaw = relative_pose.rotation().radians()

    # Public API

    def getTargetDistance(self) -> float:
        return self.target_distance

    def getTargetSpeedRPS(self) -> float:
        return self.target_speed_rps

    def getCurrentEffectiveTargetPose(self) -> Pose3d:
        return self.current_effective_target_pose

    def getCurrentEffectiveYaw(self) -> float:
        return self.current_effective_yaw

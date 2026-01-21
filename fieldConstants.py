from wpimath.geometry import Pose2d, Pose3d, Rotation3d
from wpimath.units import inchesToMeters

# Field dimensions (meters)

FIELD_LENGTH = inchesToMeters(651.22)
FIELD_WIDTH = inchesToMeters(317.69)


# Alliance-based pose flipping

def rotateBluePoseIfNecessary(original: Pose2d) -> Pose2d:
    """
    Rotates a blue-alliance pose to red alliance if needed.
    You must implement shouldFlipValueToRed() yourself.
    """
    if shouldFlipValueToRed():
        return rotatePoseAcrossField(original, FIELD_LENGTH, FIELD_WIDTH)
    return original


def shouldFlipValueToRed() -> bool:
    """
    Replace this with DriverStation alliance logic.
    """
    from wpilib import DriverStation

    alliance = DriverStation.getAlliance()
    return alliance == DriverStation.Alliance.kRed


def rotatePoseAcrossField(
    pose: Pose2d, field_length: float, field_width: float
) -> Pose2d:
    """
    Mirrors a pose across the field (blue → red).
    """
    return Pose2d(
        field_length - pose.x,
        field_width - pose.y,
        pose.rotation().rotateBy(
            Rotation3d(0.0, 0.0, 3.141592653589793).toRotation2d()
        ),
    )


# Ball constants (simple container)

class BallConstants:
    def __init__(
        self,
        mass_kg: float,
        radius_m: float,
        drag_coeff: float,
        lift_coeff: float,
        magnus_coeff: float,
        spin_decay: float,
        gravity: float,
        max_iter: int,
    ):
        self.mass_kg = mass_kg
        self.radius_m = radius_m
        self.drag_coeff = drag_coeff
        self.lift_coeff = lift_coeff
        self.magnus_coeff = magnus_coeff
        self.spin_decay = spin_decay
        self.gravity = gravity
        self.max_iter = max_iter


BALL_CONSTANTS = BallConstants(
    mass_kg=0.210,                      # 210 grams
    radius_m=inchesToMeters(3.0),       # 3 inch radius
    drag_coeff=1.2,
    lift_coeff=0.30,
    magnus_coeff=1.2,
    spin_decay=0.35,
    gravity=9.81,
    max_iter=20,
)

# Hub locations

class Hub:
    CENTER = Pose3d(
        inchesToMeters(182.11),
        inchesToMeters(158.84),
        inchesToMeters(72.0),
        Rotation3d(),
    )

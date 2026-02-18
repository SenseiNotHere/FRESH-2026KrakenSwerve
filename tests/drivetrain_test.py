from commands.drive.direct.aim_to_direction import AimToDirection
from commands.drive.direct.holonomic_drive import HolonomicDrive

def testDrive(drivetrain):
    AimToDirection(
        degrees=0, drivetrain=drivetrain
    ).andThen(
    HolonomicDrive(
        drivetrain=drivetrain,
        forwardSpeed=0.5,
        leftSpeed=0.5,
        rotationSpeed=0.5
    ).withTimeout(10)).andThen(
    HolonomicDrive(
        drivetrain=drivetrain,
        forwardSpeed=-0.5,
        leftSpeed=-0.5,
        rotationSpeed=-0.5
    ).withTimeout(1)).andThen(
        AimToDirection(
            degrees=180,
            drivetrain=drivetrain
        )
    ).andThen(
        drivetrain.stop
    )
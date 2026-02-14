import math

from wpilib import PneumaticsModuleType
from wpimath import units
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfileRadians

from phoenix6.hardware import talon_fx
from phoenix6.signals import NeutralModeValue

from pathplannerlib.config import RobotConfig, ModuleConfig, DCMotor

from utils.interpolatingMap import InterpolatingMap


class KrakenX60:
    kFreeSpeedRpm = 5800
    kMaxSpeedMetersPerSecond = 2.8

class DrivingConstants:

    # Physical Limits
    kMaxMetersPerSecond = 3.0
    kMaxAngularSpeed = math.tau

    # Slew Rate Limiting
    kMagnitudeSlewRate = 9.8
    kRotationalSlewRate = 12.0

    # Robot Geometry
    kTrackWidth = units.inchesToMeters(26.5)
    kWheelBase = units.inchesToMeters(26.5)

    kModulePositions = [
        Translation2d(+kWheelBase / 2, +kTrackWidth / 2),
        Translation2d(+kWheelBase / 2, -kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, +kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
    ]

    kDriveKinematics = SwerveDrive4Kinematics(*kModulePositions)

    # CAN IDs – Drive Motors
    kFrontLeftDriving = 7
    kFrontRightDriving = 5
    kBackLeftDriving = 1
    kBackRightDriving = 3

    # CAN IDs – Turning Motors
    kFrontLeftTurning = 8
    kFrontRightTurning = 6
    kBackLeftTurning = 2
    kBackRightTurning = 4

    # CAN IDs – Absolute Encoders
    kFrontLeftTurningEncoder = 4
    kFrontRightTurningEncoder = 3
    kBackLeftTurningEncoder = 1
    kBackRightTurningEncoder = 2

    # Gyro
    kGyroReversed = -1

    # Lock Deadbands
    kLockVxDeadband = 0.05
    kLockVyDeadband = 0.05
    kLockOmegaDeadband = 0.10

class ModuleConstants:

    # Gear Ratios / Mechanics
    kDrivingMotorPinionTeeth = 14
    kDrivingMotorReduction = 6.12
    kTurningMotorReduction = 287 / 11.0

    kWheelDiameterMeters = 0.0965
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi

    # Derived Values
    kDrivingMotorFreeSpeedRps = KrakenX60.kFreeSpeedRpm / 60.0
    kDriveWheelFreeSpeedRps = (
        kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters
    ) / kDrivingMotorReduction

    # Motor Inversions
    kTurningEncoderInverted = False
    kTurningMotorInverted = False

    kFrontLeftDriveMotorInverted = True
    kFrontRightDriveMotorInverted = False
    kBackLeftDriveMotorInverted = True
    kBackRightDriveMotorInverted = False

    # Absolute Encoder Offsets
    kFrontLeftTurningEncoderOffset = 0.264892578125
    kFrontRightTurningEncoderOffset = 0.22265625
    kBackLeftTurningEncoderOffset = -0.080078125
    kBackRightTurningEncoderOffset = 0.09521484375

    # Turning Encoder Conversion
    kTurningEncoderPositionFactor = math.tau
    kTurningEncoderVelocityFactor = math.tau / 60.0
    kTurningEncoderPositionPIDMinInput = 0.0
    kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor

    # Drive PID + FF
    kDrivingP = 0.3
    kDrivingI = 0.0
    kDrivingD = 0.0
    kDrivingS = 0.0
    kDrivingV = 0.124
    kDrivingFF = 1 / kDriveWheelFreeSpeedRps

    kDrivingMinOutput = -1.0
    kDrivingMaxOutput = 1.0

    # Turning PID + FF
    kTurningP = 10.0
    kTurningI = 0.0
    kTurningD = 0.1
    kTurningS = 0.1
    kTurningV = 2.49
    kTurningA = 0.0

    kTurningDeadbandRot = 0.002
    kTurningMinOutput = -1.0
    kTurningMaxOutput = 1.0

    # Drift / Sync
    kFusedAngleRefreshSeconds = 0.02
    kTurningKalmanGain = 0.05
    kTurningDriftDegrees = 10.0
    kTurningSyncMaxVelocity = 0.5
    kDrivingSyncMaxVelocity = 0.2

    # Neutral Modes
    kDrivingMotorIdleMode = talon_fx.signals.NeutralModeValue(NeutralModeValue.BRAKE)
    kTurningMotorIdleMode = talon_fx.signals.NeutralModeValue(NeutralModeValue.COAST)

    # Current Limits
    kDrivingMotorCurrentLimit = 70
    kDrivingMotorStatorCurrentLimit = 120
    kTurningMotorCurrentLimit = 40
    kTurningStatorCurrentLimit = 60

    # Misc
    kDrivingMinSpeedMetersPerSecond = 0.01
    kSteerDriveCouplingRatio = 3.857142857142857
    kSteerKs = 0.1
    kSteerHoldDeadband = math.radians(0.25) * (
        1.0 / ((2 * math.pi) / kTurningMotorReduction)
    )

class OIConstants:
    # Driver controller
    kDriverControllerPort = 0
    kDriveDeadband = 0.05

    # Operator controller
    kOperatorControllerPort = 1

class AutoConstants:

    moduleConfig = ModuleConfig(
        maxDriveVelocityMPS=DrivingConstants.kMaxMetersPerSecond,
        driveMotor=DCMotor.krakenX60().withReduction(ModuleConstants.kDrivingMotorReduction),
        driveCurrentLimit=ModuleConstants.kDrivingMotorCurrentLimit,
        numMotors=4,
        wheelRadiusMeters=ModuleConstants.kWheelDiameterMeters / 2,
        wheelCOF=1.0
    )

    config = RobotConfig(
        massKG=60.0,
        MOI=8.0,
        moduleConfig=moduleConfig,
        moduleOffsets=DrivingConstants.kModulePositions
    )

    config.maxModuleSpeed = DrivingConstants.kMaxMetersPerSecond
    config.driveBaseRadius = 0.45
    config.maxCentripetalAcceleration = 3.0

    kUseSqrtControl = True

    kMaxMetersPerSecond = 1.2
    kMaxAccelerationMetersPerSecondSquared = 3.5

    kMaxAngularSpeedRadiansPerSecond = 5.0
    kMaxAngularSpeedRadiansPerSecondSquared = 25.0

    kPXController = 5.0
    kPYController = 5.0
    kPThetaController = 6.0

    kIXController = 0.0
    kIYController = 0.0
    kIThetaController = 0.0

    kDXController = 0.0
    kDYController = 0.0
    kDThetaController = 0.0

    kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(
        kMaxAngularSpeedRadiansPerSecond,
        kMaxAngularSpeedRadiansPerSecondSquared
    )

class ShooterConstants:

    kShooterEnabled = True
    kShooterMotorID = 9

    kMinRPM = 600
    kMaxRPM = 6000

    kFF = 0.12
    kP = 0.2
    kD = 0.0

    kShooterSupplyLimit = 40
    kShooterStatorLimit = 140

    DISTANCE_TO_RPS = InterpolatingMap()
    DISTANCE_TO_RPS.insert(1.5, 42.0)
    DISTANCE_TO_RPS.insert(2.0, 48.0)
    DISTANCE_TO_RPS.insert(2.5, 54.0)
    DISTANCE_TO_RPS.insert(3.0, 60.0)
    DISTANCE_TO_RPS.insert(3.5, 66.0)

class PneumaticsConstants:
    kPCMID = 0
    kModuleType = PneumaticsModuleType.CTREPCM

class IntakeConstants:

    kIntakeEnabled = False
    kIntakeMotorCANID = 1

    kIntakeRPS = 20.0

    kFF = 0.12
    kP = 0.2
    kD = 0.0

    kPneumaticsModuleType = PneumaticsModuleType.CTREPCM
    kSolenoidModuleID = 0
    kSolenoidForwardChannel = 1
    kSolenoidReverseChannel = 0

class IndexerConstants:

    kIndexerEnabled = True
    kIndexerMotorID = 2

    kMaxRPS = 40.0
    kFeedRPS = 18.0

    kP = 0.0
    kD = 0.0
    kFF = 0.12

class ClimberConstants:

    kClimberEnabled = True

    kMotorID = 10
    kMotorInverted = False
    kCanCoderCANID = 5

    kPneumaticsModuleType = PneumaticsModuleType.CTREPCM
    kPCMID = 0
    kForwardChannel = 3
    kReverseChannel = 4

    kMaxPosition = -2.0
    kMinPosition = -5.8
    kClimbHeight = 10.0
    kHeightTolerance = 0.01
    kPositionDeadband = 0.01
    kVelocityDeadband = 1.0
    kStallCurrent = 55.0
    kStallTime = 0.3
    kManualSpeed = 1.0  # rotations per second

    kP = 0.2
    kI = 0.0
    kD = 0.0
    kFF = 0.0

    kStatorCurrentLimit = 60
    kSupplyCurrentLimit = 40

    kManualRPS = 0.5

class TrenchAssistConstants:

    kTrenchAssistEnabled = False
    kTriggerDistance = 0.9

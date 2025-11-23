import math
from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.configs import TalonFXConfiguration, FeedbackConfigs
from phoenix6.signals import NeutralModeValue, InvertedValue, FeedbackSensorSourceValue
from phoenix6.controls import VelocityVoltage, PositionVoltage
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from constants import ModuleConstants

class PhoenixSwerveModule:
    def __init__(
            self,
            drivingCANId: int,
            turningCANId: int,
            feedbackDevice: int,
            chassisAngularOffset: float,
            turnMotorInverted: bool = True,
    ) -> None:
        """
        :param drivingCANId: The CAN ID for the driving motor.
        :param turningCANId: The CAN ID for the turning motor.
        :param chassisAngularOffset: Offset angle for this module.
        :param turnMotorInverted: Whether the turning motor is inverted or not.
        """

        self.chassisAngularOffset = chassisAngularOffset
        self.desiredState = SwerveModuleState(0.0, Rotation2d())

        #Initialize the TalonFX Controllers
        self.drivingMotor = TalonFX(drivingCANId)
        self.turningMotor = TalonFX(turningCANId)

        # Initialize feedback devices
        self.feedbackDevice = CANcoder(feedbackDevice)

        #Initialize Driving Motors
        drivingConfig = TalonFXConfiguration()
        drivingConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        # Set appropriate PID values for velocity control
        drivingConfig.slot0.k_p = 0.3
        drivingConfig.slot0.k_i = 0.0
        drivingConfig.slot0.k_d = 0.0
        self.drivingMotor.configurator.apply(drivingConfig)

        #Initialize Turning Motors
        turningConfig = TalonFXConfiguration()
        turningConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        #Set appropriate PID values for position control
        turningConfig.slot0.k_p = 8.0
        turningConfig.slot0.k_i = 0.0
        turningConfig.slot0.k_d = 0.05
        #Use InvertedValue enum instead of bool
        turningConfig.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE if turnMotorInverted else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        turningConfig.feedback.feedback_sensor_source = FeedbackSensorSourceValue.FUSED_CANCODER
        turningConfig.feedback.feedback_device = feedbackDevice
        self.turningMotor.configurator.apply(turningConfig)

        #Set up velocity and position requests for the motors
        self.velocity_request = VelocityVoltage(0).with_slot(0)
        self.position_request = PositionVoltage(0).with_slot(0)

        #Reset encoders to starting position
        self.resetEncoders()

    def getState(self) -> SwerveModuleState:
        """Returns the current state of the module."""
        velocity = self.drivingMotor.get_velocity().value
        angle = self.getTurningPosition() - self.chassisAngularOffset

        return SwerveModuleState(velocity, Rotation2d(angle))

    def getPosition(self) -> SwerveModulePosition:
        """Returns the current position of the module."""
        distance = self.drivingMotor.get_position().value
        angle = self.getTurningPosition() - self.chassisAngularOffset

        return SwerveModulePosition(distance, Rotation2d(angle))

    def getTurningPosition(self) -> float:
        """Gets the turning motor position in radians."""
        # Convert rotations to radians (2π radians per rotation)
        return self.turningMotor.get_position().value * 2 * math.pi

    def getCancoderPosition(self):
        return 2 * math.pi * self.feedbackDevice.get_position().value

    def setDesiredState(self, desiredState: SwerveModuleState) -> None:
        """Sets the desired state for the module."""

        # Apply chassis angular offset to the desired state
        correctedDesiredState = SwerveModuleState(
            desiredState.speed,
            desiredState.angle + Rotation2d(self.chassisAngularOffset)
        )

        # Get the current angle for optimization
        cancoder_pos = self.getCancoderPosition()

        if cancoder_pos is None:
            current_angle = Rotation2d(0)
        else:
            current_angle = Rotation2d(cancoder_pos * 2 * math.pi)

        optimized = SwerveModuleState(correctedDesiredState.speed, current_angle)

        # Convert optimized angle to rotations
        angle_in_rotations = optimized.angle.radians() / (2 * math.pi)

        # Send commands to motors
        self.drivingMotor.set_control(self.velocity_request.with_velocity(
            optimized.speed * ModuleConstants.kDrivingMotorReduction / ModuleConstants.kWheelCircumferenceMeters))

        self.turningMotor.set_control(
            self.position_request.with_position(angle_in_rotations * ModuleConstants.kTurningMotorReduction))

        self.desiredState = desiredState

    def stop(self):
        """Stops the module."""
        self.drivingMotor.set_control(self.velocity_request.with_velocity(0))
        current_position = self.turningMotor.get_position().value
        self.turningMotor.set_control(self.position_request.with_position(current_position))

        if self.desiredState.speed != 0:
            self.desiredState = SwerveModuleState(speed=0, angle=self.desiredState.angle)

    def resetEncoders(self) -> None:
        """Zeroes the Absolute Encoders."""
        self.drivingMotor.set_position(0)
        absolute_rotations = self.feedbackDevice.get_position().value
        self.turningMotor.set_position(absolute_rotations * ModuleConstants.kTurningMotorReduction)

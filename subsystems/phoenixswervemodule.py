import math
from phoenix6.hardware import TalonFX,CANcoder
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration
from phoenix6.signals import NeutralModeValue, InvertedValue
from phoenix6.controls import VelocityVoltage, PositionVoltage
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from constants import ModuleConstants

class PhoenixSwerveModule:
    def __init__(
            self,
            drivingCANId: int,
            turningCANId: int,
            turnMotorInverted: bool,
            canCoderCANId: int,
            canCoderInverted: bool,
            chassisAngularOffset: float
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
        self.canCoder = CANcoder(canCoderCANId)

        # Configure CANCoder
        canCoderConfig = CANcoderConfiguration()
        canCoderConfig.sensor_direction = InvertedValue.CLOCKWISE_POSITIVE if canCoderInverted else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.canCoder.configurator.apply(canCoderConfig)

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

        self.turningMotor.configurator.apply(turningConfig)

        #Set up velocity and position requests for the motors
        self.velocity_request = VelocityVoltage(0).with_slot(0)
        self.position_request = PositionVoltage(0).with_slot(0)

        #Reset encoders to starting position
        self.resetEncoders()

    def getState(self) -> SwerveModuleState:
        """Returns the current state of the module."""
        velocity = self.drivingMotor.get_velocity().value
        angle = self._getTurningPosition() - self.chassisAngularOffset

        return SwerveModuleState(velocity, Rotation2d(angle))

    def getPosition(self) -> SwerveModulePosition:
        """Returns the current position of the module."""
        distance = self.drivingMotor.get_position().value
        angle = self._getTurningPosition() - self.chassisAngularOffset

        return SwerveModulePosition(distance, Rotation2d(angle))

    def _syncTurningEncoder(self) -> None:
        """Synchronizes the turning motor's internal encoder with the CANCoder."""
        # Get absolute position from CANCoder (in rotations)
        absolute_position = self.canCoder.get_absolute_position().value
        # Set the turning motor's position to match
        self.turningMotor.set_position(absolute_position)

    def _getTurningPosition(self) -> float:
        """Gets the turning motor position in radians using CANCoder for absolute reference."""
        # Use CANCoder for absolute position
        position_rotations = self.canCoder.get_absolute_position().value
        return position_rotations * 2 * math.pi

    def setDesiredState(self, desiredState: SwerveModuleState) -> None:
        """Sets the desired state for the module."""
        # Debug current state
        current_angle_rad = self._getTurningPosition()
        current_angle_deg = math.degrees(current_angle_rad)
        cancoder_pos = self.canCoder.get_absolute_position().value
        motor_pos = self.turningMotor.get_position().value
        
        print(f"[{self.drivingMotor.device_id}] DESIRED: Speed={desiredState.speed:.3f}, Angle={desiredState.angle.degrees():.1f}°")
        print(f"[{self.drivingMotor.device_id}] CURRENT: CANCoder={cancoder_pos:.3f}rot, Motor={motor_pos:.3f}rot, Angle={current_angle_deg:.1f}°")
        
        if abs(desiredState.speed) < ModuleConstants.kDrivingMinSpeedMetersPerSecond:
            # If speed is too low, don't move, save power
            inXBrake = abs(abs(desiredState.angle.degrees()) - 45) < 0.01
            if not inXBrake:
                print(f"[{self.drivingMotor.device_id}] STOPPING - Speed too low")
                self.stop()
                return

        # Apply chassis angular offset to the desired state
        correctedDesiredState = SwerveModuleState(
            desiredState.speed,
            desiredState.angle + Rotation2d(self.chassisAngularOffset)
        )

        # Get the current angle for optimization
        current_angle = Rotation2d(self._getTurningPosition())

        # Manual optimization instead of using SwerveModuleState.optimize()
        angle_diff = current_angle.radians() - correctedDesiredState.angle.radians()

        # Normalize the angle difference to -π to π
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        delta = correctedDesiredState.angle - current_angle

        if abs(delta.degrees()) > 90.0:
            optimized_speed = -correctedDesiredState.speed
            optimized_angle = correctedDesiredState.angle + Rotation2d(math.pi)
            print(f"[{self.drivingMotor.device_id}] FLIPPED - Speed: {optimized_speed:.3f}, Angle: {optimized_angle.degrees():.1f}°, Delta: {delta.degrees():.1f}°")
        else:
            optimized_speed = correctedDesiredState.speed
            optimized_angle = correctedDesiredState.angle
            print(f"[{self.drivingMotor.device_id}] NORMAL - Speed: {optimized_speed:.3f}, Angle: {optimized_angle.degrees():.1f}°, Delta: {delta.degrees():.1f}°")

        # Convert optimized angle to rotations
        angle_in_rotations = optimized_angle.radians() / (2 * math.pi)

        #Send commands to motors
        self.drivingMotor.set_control(self.velocity_request.with_velocity(optimized_speed * ModuleConstants.kDrivingMotorReduction / ModuleConstants.kWheelCircumferenceMeters))
        #print(f"Speed {optimized_speed} {optimized_speed * ModuleConstants.kDrivingMotorReduction / ModuleConstants.kWheelCircumferenceMeters}")
        self.turningMotor.set_control(self.position_request.with_position(angle_in_rotations * ModuleConstants.kTurningMotorReduction))
        #print(f"Target angle:{angle_in_rotations} {desiredState.angle} {Rotation2d(self.chassisAngularOffset)}")

        self.desiredState = desiredState

    def stop(self):
        """Stops the module."""
        self.drivingMotor.set_control(self.velocity_request.with_velocity(0))
        current_position = self.turningMotor.get_position().value
        self.turningMotor.set_control(self.position_request.with_position(current_position))

        if self.desiredState.speed != 0:
            self.desiredState = SwerveModuleState(speed=0, angle=self.desiredState.angle)

    def resetEncoders(self) -> None:
        """Zeroes the driving encoder."""
        self.drivingMotor.set_position(0)
        
        # Debug info before sync
        cancoder_pos = self.canCoder.get_absolute_position().value
        motor_pos_before = self.turningMotor.get_position().value
        print(f"[{self.drivingMotor.device_id}] RESET - CANCoder: {cancoder_pos:.3f} rot, Motor before: {motor_pos_before:.3f} rot")
        
        self._syncTurningEncoder()
        
        # Debug info after sync
        motor_pos_after = self.turningMotor.get_position().value
        angle_radians = self._getTurningPosition()
        print(f"[{self.drivingMotor.device_id}] RESET - Motor after: {motor_pos_after:.3f} rot, Angle: {math.degrees(angle_radians):.1f}°")

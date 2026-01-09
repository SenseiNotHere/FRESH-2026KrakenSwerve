import math

import wpilib
from commands2 import Subsystem
from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration, CurrentLimitsConfigs
from phoenix6.signals import NeutralModeValue, InvertedValue, SensorDirectionValue, FeedbackSensorSourceValue
from phoenix6.controls import VelocityVoltage, PositionVoltage, MotionMagicVoltage
from phoenix6.orchestra import Orchestra
from wpilib import Timer, DriverStation, SmartDashboard
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition

from constants import ModuleConstants


class PhoenixSwerveModule(Subsystem):
    def __init__(
        self,
        drivingCANId: int,
        turningCANId: int,
        turnMotorInverted: bool,
        driveMotorInverted: bool,
        canCoderCANId: int,
        canCoderInverted: bool,
        canCoderOffset: float,
        chassisAngularOffset: float,
        modulePlace: str,
    ) -> None:
        super().__init__()

        self.chassisAngularOffset = chassisAngularOffset
        self.desiredState = SwerveModuleState(0.0, Rotation2d())
        self.modulePlace = modulePlace

        # driving motorRot -> wheelMeters
        self.driveMotorRotToMeters = (
            ModuleConstants.kWheelCircumferenceMeters
            / ModuleConstants.kDrivingMotorReduction
        )
        self.driveMotorRpsToMps = self.driveMotorRotToMeters

        # steering motorRot -> radians
        self.steerMotorRotToRad = (2 * math.pi) / ModuleConstants.kTurningMotorReduction
        self.radToSteerMotorRot = 1.0 / self.steerMotorRotToRad

        # Hardware
        self.drivingMotor = TalonFX(drivingCANId)
        self.turningMotor = TalonFX(turningCANId)
        self.canCoder = CANcoder(canCoderCANId)
        self.canCoderOffset = canCoderOffset

        # CANcoder config
        canCoderConfig = CANcoderConfiguration()
        canCoderConfig.magnet_sensor.magnet_offset = self.canCoderOffset
        canCoderConfig.magnet_sensor.sensor_direction = (
            SensorDirectionValue.CLOCKWISE_POSITIVE
            if canCoderInverted
            else SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        )
        self.canCoder.configurator.apply(canCoderConfig)

        # Drive motor config
        drivingConfig = TalonFXConfiguration()
        drivingConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        drivingConfig.motor_output.inverted = (
            InvertedValue.CLOCKWISE_POSITIVE
            if driveMotorInverted
            else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        drivingConfig.slot0.k_p = ModuleConstants.kDrivingP
        drivingConfig.slot0.k_i = ModuleConstants.kDrivingI
        drivingConfig.slot0.k_d = ModuleConstants.kDrivingD
        drivingConfig.feedback.sensor_source = FeedbackSensorSourceValue.ROTOR_SENSOR
        self.drivingMotor.configurator.apply(drivingConfig)

        self.drivingMotor.get_velocity().set_update_frequency(100)
        self.drivingMotor.get_position().set_update_frequency(50)

        # Turn motor config
        turningConfig = TalonFXConfiguration()
        turningConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        turningConfig.motor_output.inverted = (
            InvertedValue.CLOCKWISE_POSITIVE
            if turnMotorInverted
            else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        turningConfig.slot0.k_p = ModuleConstants.kTurningP
        turningConfig.slot0.k_i = ModuleConstants.kTurningI
        turningConfig.slot0.k_d = ModuleConstants.kTurningD
        turningConfig.feedback.sensor_source = FeedbackSensorSourceValue.FUSED_CANCODER
        turningConfig.feedback.feedback_remote_sensor_id = canCoderCANId
        turningConfig.feedback.rotor_to_sensor_ratio = ModuleConstants.kTurningMotorReduction
        turningConfig.closed_loop_general.continuous_wrap = True
        turningConfig.motion_magic.motion_magic_cruise_velocity = ModuleConstants.kMotionMagicCruiseVelocity
        turningConfig.motion_magic.motion_magic_acceleration = ModuleConstants.kMotionMagicAcceleration
        self.turningMotor.configurator.apply(turningConfig)

        self.turningMotor.get_position().set_update_frequency(100)  # Hz
        self.turningMotor.get_velocity().set_update_frequency(100)

        # Current limits
        # Driving Current Limits
        drivingCurrentLimits = CurrentLimitsConfigs()
        drivingCurrentLimits.supply_current_limit = ModuleConstants.kDrivingMotorCurrentLimit
        drivingCurrentLimits.stator_current_limit = ModuleConstants.kDrivingMotorStatorCurrentLimit
        drivingCurrentLimits.supply_current_limit_enable = True
        drivingCurrentLimits.stator_current_limit_enable = True
        self.drivingMotor.configurator.apply(drivingCurrentLimits)

        # Turning Current Limits
        turningCurrentLimits = CurrentLimitsConfigs()
        turningCurrentLimits.supply_current_limit = ModuleConstants.kTurningMotorCurrentLimit
        turningCurrentLimits.stator_current_limit = ModuleConstants.kTurningStatorCurrentLimit
        turningCurrentLimits.supply_current_limit_enable = True
        turningCurrentLimits.stator_current_limit_enable = True
        self.turningMotor.configurator.apply(turningCurrentLimits)

        # Control requests
        self.velocity_request = VelocityVoltage(0).with_slot(0).with_feed_forward(0.0)
        self.turning_request = MotionMagicVoltage(0).with_slot(0).with_feed_forward(0.0)

        # Kalman timing
        self.nextSyncTime = 0.0

        # Initial alignment
        self.resetEncoders()

        # Orchestra stuff
        self.orchestra = Orchestra([self.drivingMotor, self.turningMotor])

    # Encoders

    def resetEncoders(self) -> None:
        # Zero drive distance
        self.drivingMotor.set_position(0)

        # One-time absolute alignment for steering
        abs_rot = self.canCoder.get_absolute_position().value  # 0–1 rotations
        self.turningMotor.set_position(
            abs_rot * ModuleConstants.kTurningMotorReduction
        )

        abs_sig = self.canCoder.get_absolute_position()
        abs_sig.refresh()
        abs_rot = abs_sig.value  # 0..1 rotations

        print(f"[{self.modulePlace}] abs_rot={abs_rot:.6f} offset={self.canCoderOffset:.6f}")

        abs_sig = self.canCoder.get_absolute_position()
        abs_sig.refresh()

        if not abs_sig.status.is_ok():
            print(f"[{self.modulePlace}] CANcoder not ready yet")

    # Periodic

    def periodic(self):
        # Only correct when module is basically not moving
        if abs(self.desiredState.speed) < 0.05:
            abs_rot = self.canCoder.get_absolute_position().value  # 0–1 rotations
            motor_rot = self.turningMotor.get_position().value

            target_rot = abs_rot * ModuleConstants.kTurningMotorReduction

            # wrap target near current position
            diff = motor_rot - target_rot
            wraps = round(diff / ModuleConstants.kTurningMotorReduction)
            target_rot += wraps * ModuleConstants.kTurningMotorReduction

            error = motor_rot - target_rot

            # only nudge if error is meaningful (ex: > ~2 degrees)
            if abs(error) > (2 / 360) * ModuleConstants.kTurningMotorReduction:
                self.turningMotor.set_position(target_rot)

    # State / Odometry

    def getTurningPosition(self) -> float:
        """
        :return: The current turning position of the swerve module in radians.
        """
        motor_rot = self.turningMotor.get_position().value
        return motor_rot * self.steerMotorRotToRad

    def getState(self) -> SwerveModuleState:
        """
        :return: The current state of the swerve module.
        """
        motor_rps = self.drivingMotor.get_velocity().value
        wheel_mps = motor_rps * self.driveMotorRpsToMps
        angle = self.getTurningPosition() - self.chassisAngularOffset
        return SwerveModuleState(wheel_mps, Rotation2d(angle))

    def getPosition(self) -> SwerveModulePosition:
        """
        :return: The current position of the swerve module.
        """
        motor_rot = self.drivingMotor.get_position().value
        wheel_meters = motor_rot * self.driveMotorRotToMeters
        angle = self.getTurningPosition() - self.chassisAngularOffset
        return SwerveModulePosition(wheel_meters, Rotation2d(angle))

    # Optimize

    def _optimizeState(self, desired: SwerveModuleState) -> SwerveModuleState:
        current = Rotation2d(self.getTurningPosition())
        target = desired.angle

        # signed smallest-angle difference, in radians (-pi .. pi)
        delta = target.radians() - current.radians()
        delta = math.atan2(math.sin(delta), math.cos(delta))

        if abs(delta) > math.pi / 2:
            return SwerveModuleState(
                -desired.speed,
                Rotation2d(target.radians() + math.pi)
            )

        return desired

    # Control

    def setDesiredState(self, desiredState: SwerveModuleState) -> None:
        # Apply chassis angular offset once
        desired_module = SwerveModuleState(
            desiredState.speed,
            desiredState.angle + Rotation2d(self.chassisAngularOffset),
        )

        # Optimize for speed flip ONLY (no angle wrapping)
        optimized = self._optimizeState(desired_module)

        # ---------------- DRIVE ----------------
        motor_rps = optimized.speed / self.driveMotorRotToMeters
        self.drivingMotor.set_control(
            self.velocity_request.with_velocity(motor_rps)
        )

        # ---------------- TURN ----------------
        target_rot = optimized.angle.radians() * self.radToSteerMotorRot

        self.turningMotor.set_control(
            self.turning_request.with_position(target_rot)
        )

        self.desiredState = desiredState

    # Extras

    def getTemperature(self):
        """
        :return: Returns the temperatures of the module's motors.
        """
        drivingTemp = self.drivingMotor.get_device_temp().value
        turningTemp = self.turningMotor.get_device_temp().value
        return drivingTemp, turningTemp

    def getSupplyCurrent(self):
        """
        :return: Returns the current draw of the module's motors.
        """
        drivingCurrent = self.drivingMotor.get_supply_current()
        turningCurrent = self.turningMotor.get_supply_current()
        return drivingCurrent, turningCurrent

    # Orchestra

    def loadMusic(self, path: str):
        """
        Loads a music file into the orchestra.

        :path: (str): The file path to the music file to be loaded.
        """
        self.orchestra.load_music(path)

    def playMusic(self):
        """
        Plays music using the configured orchestra.
        """
        self.orchestra.play()

    def stopMusic(self):
        """
        Stops the music playback by halting the orchestra.
        """
        self.orchestra.stop()
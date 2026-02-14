from typing import Optional

from commands2 import Subsystem
from wpilib import DoubleSolenoid, PneumaticsModuleType, SmartDashboard, Timer

from phoenix6.controls import PositionVoltage
from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.configs import (
    TalonFXConfiguration,
    CANcoderConfiguration,
    Slot0Configs,
    CurrentLimitsConfigs
)
from phoenix6.signals import (
    NeutralModeValue,
    InvertedValue,
    FeedbackSensorSourceValue,
    SensorDirectionValue
)
from phoenix6.signals import (
    ReverseLimitSourceValue,
    ReverseLimitValue,
    ReverseLimitTypeValue
)

from constants.constants import ClimberConstants


class Climber(Subsystem):

    def __init__(
        self,
        motorCANID: int,
        motorInverted: bool,
        canCoderCANID: int,
        canCoderInverted: bool,
        solenoidCANID: int,
        pneumaticsModuleType: PneumaticsModuleType,
        forwardChannel: int,
        reverseChannel: int,
        canCoderOffset: Optional[float] = None
    ):
        super().__init__()

        # Hardware

        self.motor = TalonFX(motorCANID)
        self.canCoder = CANcoder(canCoderCANID)

        self.airbrake = DoubleSolenoid(
            module=solenoidCANID,
            moduleType=pneumaticsModuleType,
            forwardChannel=forwardChannel,
            reverseChannel=reverseChannel
        )

        # CANcoder Config

        canCoderConfig = CANcoderConfiguration()

        if canCoderOffset is not None:
            canCoderConfig.magnet_sensor.magnet_offset = canCoderOffset

        canCoderConfig.magnet_sensor.sensor_direction = (
            SensorDirectionValue.CLOCKWISE_POSITIVE
            if canCoderInverted
            else SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        )

        self.canCoder.configurator.apply(canCoderConfig)

        # Motor Config

        motorConfig = TalonFXConfiguration()

        motorConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        motorConfig.motor_output.inverted = (
            InvertedValue.CLOCKWISE_POSITIVE
            if motorInverted
            else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )

        motorConfig.feedback.feedback_sensor_source = FeedbackSensorSourceValue.FUSED_CANCODER
        motorConfig.feedback.feedback_remote_sensor_id = canCoderCANID
        motorConfig.hardware_limit_switch.reverse_limit_enable = True
        motorConfig.hardware_limit_switch.reverse_limit_source = ReverseLimitSourceValue.LIMIT_SWITCH_PIN
        motorConfig.hardware_limit_switch.reverse_limit_autoset_position_enable = True
        motorConfig.hardware_limit_switch.reverse_limit_autoset_position_value = ClimberConstants.kMinPosition
        motorConfig.hardware_limit_switch.reverse_limit_type = ReverseLimitTypeValue.NORMALLY_OPEN
        self.motor.configurator.apply(motorConfig)

        # PID / Feedforward

        slotConfig = Slot0Configs()
        (
            slotConfig
            .with_k_p(ClimberConstants.kP)
            .with_k_i(ClimberConstants.kI)
            .with_k_d(ClimberConstants.kD)
            .with_k_v(ClimberConstants.kFF)
        )

        self.motor.configurator.apply(slotConfig)

        # Current Limits

        currentLimits = CurrentLimitsConfigs()
        (
            currentLimits
            .with_supply_current_limit(ClimberConstants.kSupplyCurrentLimit)
            .with_supply_current_limit_enable(True)
            .with_stator_current_limit(ClimberConstants.kStatorCurrentLimit)
            .with_stator_current_limit_enable(True)
        )

        self.motor.configurator.apply(currentLimits)

        # Control Request

        self.positionRequest = PositionVoltage(0).with_slot(0)

        # Sync motor position to absolute
        self.motor.set_position(self.getAbsolutePosition())

        # State Variables

        self.targetPosition: float = self.getRelativePosition()
        self.commandedActive: bool = False

        self.jamTimer: float = 0.0
        self.lastTime = Timer.getFPGATimestamp()

        self.airbrakeEngaged: bool = False
        self.airbrake.set(DoubleSolenoid.Value.kReverse)

        # Set climber to minimum position
#        self.setPosition(ClimberConstants.kMinPosition)

    # Periodic (Safety / Jam Detection)

    def periodic(self):

        now = Timer.getFPGATimestamp()
        dt = now - self.lastTime
        self.lastTime = now

        velocity = self.motor.get_velocity().value
        current = self.motor.get_stator_current().value
        position = self.getRelativePosition()

        positionError = abs(self.targetPosition - position)

        tryingToMove = (
            self.commandedActive and
            positionError > ClimberConstants.kPositionDeadband
        )

        moving = abs(velocity) > ClimberConstants.kVelocityDeadband

        # Stall Detection
        if tryingToMove and not moving and current > ClimberConstants.kStallCurrent:
            self.jamTimer += dt
        else:
            self.jamTimer = 0.0

        if self.jamTimer > ClimberConstants.kStallTime:
            self.stop()
            SmartDashboard.putBoolean("Climber/Jammed", True)
        else:
            SmartDashboard.putBoolean("Climber/Jammed", False)

        SmartDashboard.putNumber("Climber/Position", position)
        SmartDashboard.putBoolean("Climber/Airbrake", self.airbrakeEngaged)

    # Positioning

    def setPosition(self, pos: float):

        pos = max(
            ClimberConstants.kMinPosition,
            min(pos, ClimberConstants.kMaxPosition)
        )

        self.targetPosition = pos
        self.commandedActive = True

        self.motor.set_control(
            self.positionRequest.with_position(pos)
        )

    def stop(self):
        self.commandedActive = False
        self.motor.set_control(
            self.positionRequest.with_position(self.getRelativePosition())
        )

    def toggle(self):

        if self.getRelativePosition() < ClimberConstants.kClimbHeight:
            self.setPosition(ClimberConstants.kClimbHeight)
        else:
            self.setPosition(ClimberConstants.kMinPosition)

    def manualAdjust(self, joystickValue: float):

        # Deadband
        if abs(joystickValue) < 0.1:
            return

        # Scale speed (very important)
        # This controls how fast it moves per second
        speed = joystickValue * ClimberConstants.kManualSpeed  # rotations per second

        # Increment target
        newTarget = self.targetPosition + speed * 0.02  # 20ms loop

        # Clamp
        newTarget = max(
            ClimberConstants.kMinPosition,
            min(newTarget, ClimberConstants.kMaxPosition)
        )

        self.setPosition(newTarget)

    # Airbrake

    def engageAirbrake(self):
        self.airbrake.set(DoubleSolenoid.Value.kForward)
        self.airbrakeEngaged = True

    def releaseAirbrake(self):
        self.airbrake.set(DoubleSolenoid.Value.kReverse)
        self.airbrakeEngaged = False

    def toggleAirbrake(self):
        if self.airbrakeEngaged:
            self.releaseAirbrake()
        else:
            self.engageAirbrake()

    # Sensors

    def getRelativePosition(self) -> float:
        return self.motor.get_position().value

    def getAbsolutePosition(self) -> float:
        return self.canCoder.get_absolute_position().value

from __future__ import annotations

from commands2 import Subsystem
from phoenix6.configs import (
    TalonFXConfiguration,
    CANcoderConfiguration,
    Slot0Configs
)
from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.controls import PositionVoltage
from phoenix6.signals import (
    NeutralModeValue,
    InvertedValue,
    SensorDirectionValue,
    FeedbackSensorSourceValue,
    ForwardLimitSourceValue,
)

from constants import ClimberConstants


class Climber(Subsystem):
    def __init__(
        self,
        motorCANID: int,
        motorInverted: bool,
        canCoderCANID: int,
        canCoderInverted: bool,
        magnetOffset: float,
    ):
        """
        Initialize the climber subsystem.

        :motorCANID: CAN ID of the Talon FX controlling the climber motor.
        :motorInverted: Whether the motor output should be inverted.
        :canCoderCANID: CAN ID of the CANcoder used for absolute position.
        :canCoderInverted: Whether the CANcoder direction should be inverted.
        :magnetOffset: Magnet offset for the CANcoder (in rotations).
        """
        super().__init__()

        # Hardware
        self.motor = TalonFX(motorCANID)
        self.canCoder = CANcoder(canCoderCANID)

        # CANCoder Config
        canCoderConfig = CANcoderConfiguration()
        canCoderConfig.magnet_sensor.magnet_offset = magnetOffset
        canCoderConfig.magnet_sensor.sensor_direction = (
            SensorDirectionValue.CLOCKWISE_POSITIVE
            if canCoderInverted
            else SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        )
        self.canCoder.configurator.apply(canCoderConfig)

        # Motor Config
        climberMotorConfig = TalonFXConfiguration()
        climberMotorConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        climberMotorConfig.motor_output.inverted = (
            InvertedValue.CLOCKWISE_POSITIVE
            if motorInverted
            else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )

        # Use fused CANcoder for closed-loop feedback
        climberMotorConfig.feedback.feedback_sensor_source = (
            FeedbackSensorSourceValue.FUSED_CANCODER
        )
        climberMotorConfig.feedback.feedback_remote_sensor_id = canCoderCANID

        # Hardware forward limit switch (top hard stop)
        climberMotorConfig.hardware_limit_switch.forward_limit_source = (
            ForwardLimitSourceValue.LIMIT_SWITCH_PIN
        )
        climberMotorConfig.hardware_limit_switch.forward_limit_enable = True

        self.motor.configurator.apply(climberMotorConfig)

        # Slot 0 Config
        climberSlotConfig = Slot0Configs()
        (
            climberSlotConfig
            .with_k_p(ClimberConstants.kP)
            .with_k_d(ClimberConstants.kD)
        )
        self.motor.configurator.apply(climberSlotConfig)

        # Control Request
        self.position_request = PositionVoltage(0.0).with_slot(0)

        # Sync motor to absolute encoder
        self.resetPosition()

        # Hold current position on boot to prevent sudden motion
        self.motor.set_control(
            self.position_request.with_position(self.getMotorPosition())
        )

    def periodic(self) -> None:
        pass

    # Position Helpers

    def getAbsolutePosition(self) -> float:
        """
        Get the absolute position of the climber from the absolute encoder.

        :returns: Absolute position in rotations.
        """
        return self.canCoder.get_position().value

    def getMotorPosition(self) -> float:
        """
        Get the internal motor position of the Talon FX.

        :returns: Motor position in rotations.
        """
        return self.motor.get_position().value

    # Control

    def setPosition(self, position: float) -> None:
        """
        Command the climber to a target position.

        The target is clamped between the configured minimum and maximum
        positions to enforce software soft limits.

        :position: Desired climber position in rotations.
        """
        targetPos = max(
            ClimberConstants.kMinPosition,
            min(position, ClimberConstants.kMaxPosition),
        )

        self.motor.set_control(
            self.position_request.with_position(targetPos)
        )

    # Zeroing

    def resetPosition(self) -> None:
        """
        Synchronize the Talon FX internal position to the absolute CANcoder position.

        This is typically called on boot to recover the correct position after
        power loss or brownouts.
        """
        absPosition = self.getAbsolutePosition()
        self.motor.set_position(absPosition)

    # Extras

    def getMotor(self):
        """
        :yields: The Talon FX controlling the climber motor.
        """
        yield self.motor
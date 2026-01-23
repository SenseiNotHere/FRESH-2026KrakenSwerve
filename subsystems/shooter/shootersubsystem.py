from commands2 import Subsystem

from phoenix6.hardware import TalonFX
from phoenix6.controls import VelocityVoltage
from phoenix6.configs import (
    TalonFXConfiguration,
    Slot0Configs,
    CurrentLimitsConfigs,
)
from phoenix6.signals import NeutralModeValue, InvertedValue

from wpilib import SmartDashboard, SendableChooser

from constants.constants import ShooterConstants


class Shooter(Subsystem):
    """
    Closed-loop velocity shooter using Phoenix 6.
    Timing is managed by commands (withTimeout), not the subsystem.
    """

    def __init__(self, motorCANID: int, motorInverted: bool):
        super().__init__()

        # Motor Init
        self.motor = TalonFX(motorCANID)

        motorConfig = TalonFXConfiguration()
        motorConfig.motor_output.neutral_mode = NeutralModeValue.COAST
        motorConfig.motor_output.inverted = (
            InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            if motorInverted
            else InvertedValue.CLOCKWISE_POSITIVE
        )
        self.motor.configurator.apply(motorConfig)

        # Control Gains
        slot0 = Slot0Configs()
        (
            slot0
            .with_k_p(ShooterConstants.kP)
            .with_k_d(ShooterConstants.kD)
            .with_k_v(ShooterConstants.kFF)
        )
        self.motor.configurator.apply(slot0)

        # Current Limits
        currentLimits = CurrentLimitsConfigs()
        (
            currentLimits
            .with_supply_current_limit(
                ShooterConstants.kShooterSupplyLimit
            )
            .with_stator_current_limit(
                ShooterConstants.kShooterStatorLimit
            )
            .with_supply_current_limit_enable(True)
            .with_stator_current_limit_enable(True)
        )
        self.motor.configurator.apply(currentLimits)

        # Control Request
        self.velocityRequest = VelocityVoltage(0.0).with_slot(0)

        # State
        self.enabled: bool = False
        self.outputPercent: float = 0.0

        # Dashboard
        self.speedChooser = SendableChooser()
        self.speedChooser.addOption("Off", 0.0)
        self.speedChooser.addOption("10%", 0.1)
        self.speedChooser.addOption("20%", 0.2)
        self.speedChooser.addOption("30%", 0.3)
        self.speedChooser.addOption("40%", 0.4)
        self.speedChooser.addOption("50%", 0.5)
        self.speedChooser.addOption("60%", 0.6)
        self.speedChooser.addOption("70%", 0.7)
        self.speedChooser.addOption("80%", 0.8)
        self.speedChooser.addOption("90%", 0.9)
        self.speedChooser.setDefaultOption("100%", 1.0)

        SmartDashboard.putData("Shooter Speed", self.speedChooser)
        SmartDashboard.putBoolean("Shooter Use Auto Speed", False)

        # Constants
        self.kMaxRPM = ShooterConstants.kMaxRPM

    # Periodic

    def periodic(self):
        """
        Applies shooter velocity control.
        Chooser is ignored when auto-speed is enabled.
        """

        if self.enabled:
            if SmartDashboard.getBoolean(
                "Shooter Use Auto Speed", False
            ):
                pass  # outputPercent set externally
            else:
                self.outputPercent = self.speedChooser.getSelected()
        else:
            self.outputPercent = 0.0

        # Convert percent -> RPM -> RPS
        target_rpm = self.outputPercent * self.kMaxRPM
        target_rps = target_rpm / 60.0

        self.motor.set_control(
            self.velocityRequest.with_velocity(target_rps)
        )

        # Telemetry
        SmartDashboard.putNumber(
            "Shooter Target RPM", target_rpm
        )
        SmartDashboard.putNumber(
            "Shooter Velocity (RPS)",
            self.motor.get_velocity().value
        )

    # Public API

    def enable(self):
        """Enable shooter output."""
        self.enabled = True

    def disable(self):
        """Disable shooter output and stop motor."""
        self.enabled = False
        self.outputPercent = 0.0
        self.motor.set_control(
            self.velocityRequest.with_velocity(0.0)
        )

    def setPercent(self, percent: float):
        """
        Manual percent control (teleop/testing).
        """
        self.enabled = True
        self.outputPercent = max(min(percent, 1.0), 0.0)

    def setTargetRPS(self, target_rps: float):
        """
        Sets shooter velocity directly in RPS.
        Used by ShotCalculator / vision.
        """
        self.enabled = True

        target_rpm = target_rps * 60.0
        self.outputPercent = max(
            min(target_rpm / self.kMaxRPM, 1.0),
            0.0
        )

    def atSpeed(self, tolerance_rpm: float) -> bool:
        """
        Returns True if shooter is within tolerance of target speed.
        """
        target_rpm = self.outputPercent * self.kMaxRPM
        current_rpm = self.motor.get_velocity().value * 60.0
        return current_rpm >= (target_rpm - tolerance_rpm)

    def getMotors(self):
        """
        :yields: The Talon FX controlling the climber motor.
        """
        yield self.motor
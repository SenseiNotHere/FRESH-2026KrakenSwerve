import time

from commands2 import Subsystem
from phoenix6.configs import TalonFXConfiguration
from phoenix6.hardware import TalonFX
from phoenix6.controls import DutyCycleOut
from phoenix6.signals import NeutralModeValue, InvertedValue
from wpilib import SmartDashboard, SendableChooser


class Shooter(Subsystem):
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

        # Percent Output Control
        self.percentRequest = DutyCycleOut(0.0)

        # State
        self.enabled = False
        self.outputPercent = 0.0

        # Run-for-time state
        self.runForTimeActive = False
        self.runForTimeEndTime = 0.0

        # Dashboard Chooser
        self.speedChooser = SendableChooser()
        self.speedChooser.setDefaultOption("Off", 0.0)
        self.speedChooser.addOption("10%", 0.1)
        self.speedChooser.addOption("20%", 0.2)
        self.speedChooser.addOption("30%", 0.3)
        self.speedChooser.addOption("40%", 0.4)
        self.speedChooser.addOption("50%", 0.5)
        self.speedChooser.addOption("60%", 0.6)
        self.speedChooser.addOption("70%", 0.7)
        self.speedChooser.addOption("80%", 0.8)
        self.speedChooser.addOption("90%", 0.9)
        self.speedChooser.addOption("100%", 1.0)

        SmartDashboard.putData("Shooter Speed", self.speedChooser)

    # Periodic

    def periodic(self):
        # Handle timed run
        if self.runForTimeActive and time.time() >= self.runForTimeEndTime:
            self.runForTimeActive = False
            self.outputPercent = 0.0

        # Normal control
        if self.enabled and not self.runForTimeActive:
            self.outputPercent = self.speedChooser.getSelected()
        elif not self.enabled and not self.runForTimeActive:
            self.outputPercent = 0.0

        # Apply output
        self.motor.set_control(
            self.percentRequest.with_output(self.outputPercent)
        )

        # Telemetry
        SmartDashboard.putNumber(
            "Shooter Output %",
            self.outputPercent * 100.0
        )
        SmartDashboard.putNumber(
            "Shooter Velocity (RPS)",
            self.motor.get_velocity().value
        )

    # Public API

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False
        self.outputPercent = 0.0
        self.motor.set_control(self.percentRequest.with_output(0.0))

    def setPercent(self, percent: float):
        self.outputPercent = max(min(percent, 1.0), -1.0)

    def runForTime(self, percent: float, seconds: float):
        self.outputPercent = max(min(percent, 1.0), -1.0)
        self.runForTimeActive = True
        self.runForTimeEndTime = time.time() + seconds

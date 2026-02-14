from commands2 import Subsystem
from rev import (
    SparkMax,
    SparkMaxConfig,
    SparkLowLevel,
    SparkBase,
    ResetMode,
    PersistMode
)
from wpilib import SmartDashboard, SendableChooser

from constants.constants import IndexerConstants


class Indexer(Subsystem):
    def __init__(
        self,
        motorCANID: int,
        motorInverted: bool
    ):
        super().__init__()

        # Motor Init
        self.motor = SparkMax(motorCANID, SparkLowLevel.MotorType.kBrushless)

        # Motor config
        config = SparkMaxConfig()
        config.setIdleMode(SparkMaxConfig.IdleMode.kCoast)
        config.inverted(motorInverted)

        # PID config
        config.closedLoop.P(IndexerConstants.kP)
        config.closedLoop.I(0.0)
        config.closedLoop.D(IndexerConstants.kD)
        config.closedLoop.velocityFF(IndexerConstants.kFF)
        config.closedLoop.outputRange(-1.0, 1.0)

        self.motor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        )

        self.pidController = self.motor.getClosedLoopController()

        # State
        self._enabled = False
        self._targetRPM: float | None = None
        self._lastTargetRPM: float | None = None

        # Speed chooser
        self.speedChooser = SendableChooser()
        self.speedChooser.setDefaultOption("100%", 1.0)
        self.speedChooser.addOption("90%", 0.9)
        self.speedChooser.addOption("80%", 0.8)
        self.speedChooser.addOption("70%", 0.7)
        self.speedChooser.addOption("60%", 0.6)
        self.speedChooser.addOption("50%", 0.5)
        self.speedChooser.addOption("40%", 0.4)
        self.speedChooser.addOption("30%", 0.3)
        self.speedChooser.addOption("20%", 0.2)
        self.speedChooser.addOption("10%", 0.1)
        self.speedChooser.addOption("0%", 0.0)

        SmartDashboard.putData("Indexer Speed", self.speedChooser)

    # Periodic
    def periodic(self) -> None:
        if not self._enabled:
            self._targetRPM = None
            self._lastTargetRPM = None
            self.motor.set(0.0)
            return

        if self._targetRPM is not None:
            if self._targetRPM != self._lastTargetRPM:
                self.pidController.setReference(self._targetRPM, SparkBase.ControlType.kVelocity)
                self._lastTargetRPM = self._targetRPM

        SmartDashboard.putBoolean("Indexer Enabled", self._enabled)
        SmartDashboard.putNumber(
            "Indexer Target RPM",
            self._targetRPM if self._targetRPM is not None else 0.0
        )

    # API
    def enable(self) -> None:
        self._enabled = True
        scale = self.speedChooser.getSelected()
        self._targetRPM = IndexerConstants.kFeedRPS * 60.0 * scale

    def stop(self) -> None:
        self._enabled = False
        self._targetRPM = None
        self.motor.set(0.0)

    def feedPercent(self, percent: float) -> None:
        self._enabled = False
        self._targetRPM = None
        self.motor.set(percent)

    def getMotors(self):
        """
        :yields: The SparkMax controlling the indexer motor.
        """
        yield self.motor

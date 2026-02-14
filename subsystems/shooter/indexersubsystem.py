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

        # Motor Setup

        self.motor = SparkMax(
            motorCANID,
            SparkLowLevel.MotorType.kBrushless
        )

        config = SparkMaxConfig()
        config.setIdleMode(SparkMaxConfig.IdleMode.kCoast)
        config.inverted(motorInverted)

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

        self.pid = self.motor.getClosedLoopController()

        # State

        self._targetRPM: float | None = None
        self._lastTargetRPM: float | None = None

        # Speed chooser
        self.speedChooser = SendableChooser()
        self.speedChooser.setDefaultOption("100%", 1.0)
        self.speedChooser.addOption("75%", 0.75)
        self.speedChooser.addOption("50%", 0.5)
        self.speedChooser.addOption("25%", 0.25)
        self.speedChooser.addOption("0%", 0.0)

        SmartDashboard.putData("Indexer Speed", self.speedChooser)

    # Periodic

    def periodic(self):

        if self._targetRPM is None:
            self.motor.set(0.0)
            return

        if self._targetRPM != self._lastTargetRPM:
            self.pid.setReference(
                self._targetRPM,
                SparkBase.ControlType.kVelocity
            )
            self._lastTargetRPM = self._targetRPM

        SmartDashboard.putNumber(
            "Indexer/Target RPM",
            self._targetRPM if self._targetRPM else 0.0
        )

    # High-Level API (Superstructure Calls These)

    def feed(self):
        scale = self.speedChooser.getSelected()
        self._targetRPM = IndexerConstants.kFeedRPS * 60.0 * scale

    def reverse(self):
        scale = self.speedChooser.getSelected()
        self._targetRPM = -IndexerConstants.kFeedRPS * 60.0 * scale

    def hold(self):
        # Very low speed just to prevent rollback
        self._targetRPM = IndexerConstants.kHoldRPS * 60.0

    def stop(self):
        self._targetRPM = None
        self.motor.set(0.0)

    # Optional Helpers

    def isRunning(self) -> bool:
        return self._targetRPM is not None

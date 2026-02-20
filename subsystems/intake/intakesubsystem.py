from typing import Optional

from commands2 import Subsystem
from rev import (
    SparkMax,
    SparkMaxConfig,
    SparkLowLevel,
    ResetMode,
    PersistMode
)
from wpilib import (
    DoubleSolenoid,
    PneumaticsModuleType,
    SmartDashboard,
    SendableChooser
)

from constants.constants import IntakeConstants


class Intake(Subsystem):

    def __init__(
        self,
        motorCANID: int,
        motorInverted: bool,
        solenoidModuleID: int,
        pneumaticsModuleType: PneumaticsModuleType,
        forwardChannel: int,
        reverseChannel: int
    ):
        super().__init__()

        # Motor

        self.motor = SparkMax(
            motorCANID,
            SparkLowLevel.MotorType.kBrushed
        )

        config = SparkMaxConfig()
        config.setIdleMode(SparkMaxConfig.IdleMode.kCoast)
        config.inverted(motorInverted)

        # No closed-loop / PID configuration; we will command raw percentages.
        self.motor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        )

        # Pneumatics

        self.deploySolenoid = DoubleSolenoid(
            module=solenoidModuleID,
            moduleType=pneumaticsModuleType,
            forwardChannel=forwardChannel,
            reverseChannel=reverseChannel
        )

        self._deployed = False
        self._setDeployed(False)  # Start safe

        # State (raw percent output -1.0..1.0)

        self._targetPercent: Optional[float] = None

        # Speed chooser
        self.speedChooser = SendableChooser()
        self.speedChooser.setDefaultOption("50%", 0.5)
        self.speedChooser.addOption("75%", 0.75)
        self.speedChooser.addOption("25%", 0.25)
        self.speedChooser.addOption("100%", 1.0)
        self.speedChooser.addOption("0%", 0.0)

        SmartDashboard.putData("Intake Speed", self.speedChooser)

    # Periodic

    def periodic(self):

        if self._targetPercent is None:
            self.motor.set(0.0)
        else:
            # Raw open-loop percent output
            self.motor.set(self._targetPercent)

        SmartDashboard.putBoolean("Intake/Deployed", self._deployed)
        SmartDashboard.putNumber(
            "Intake/Target Percent",
            self._targetPercent if self._targetPercent is not None else 0.0
        )

    # High-Level API (Superstructure Calls These)

    def startIntaking(self):
        scale = self.speedChooser.getSelected()
        self._targetPercent = float(scale)

    def reverse(self):
        scale = self.speedChooser.getSelected()
        self._targetPercent = -float(scale)

    def stop(self):
        self._targetPercent = None
        self.motor.set(0.0)

    def deploy(self):
        self._setDeployed(True)

    def stow(self):
        self.stop()
        self._setDeployed(False)

    # Internal Deploy Control

    def _setDeployed(self, deployed: bool):
        if deployed:
            self.deploySolenoid.set(DoubleSolenoid.Value.kForward)
        else:
            self.deploySolenoid.set(DoubleSolenoid.Value.kReverse)

        self._deployed = deployed

    # State Queries

    def isDeployed(self) -> bool:
        return self._deployed

from commands2 import Subsystem
from wpilib import SmartDashboard, SendableChooser

from rev import (
    SparkMax,
    SparkMaxConfig,
    SparkLowLevel,
    SparkBase,
    ResetMode,
    PersistMode
)

from constants.constants import AgitatorConstants

class Agitator(Subsystem):
    def __init__(
            self,
            motorCANID: int,
            motorInverted: bool,
    ):
        super().__init__()
        # Motor init
        self.motor = SparkMax(motorCANID, SparkLowLevel.MotorType.kBrushless)

        motorConfig = SparkMaxConfig()

        motorConfig.setIdleMode(SparkMaxConfig.IdleMode.kCoast)
        motorConfig.inverted(motorInverted)

        self.motor.configure(
            motorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        )

        self.speedChooser = SendableChooser()
        self.speedChooser.setDefaultOption("25%", 0.25)
        self.speedChooser.addOption("5%", 0.05)
        self.speedChooser.addOption("50%", 0.5)
        self.speedChooser.addOption("75%", 0.75)
        self.speedChooser.addOption("100%", 1.0)
        SmartDashboard.putData('Agitator Chooser', self.speedChooser)

    def periodic(self):
        SmartDashboard.putNumber("Agitator Speed", self.motor.get())
        SmartDashboard.putBoolean("Agitator Running", self.isRunning())

    def feed(self) -> None:
        self.motor.set(self.speedChooser.getSelected())

    def reverse(self) -> None:
        self.motor.set(-self.speedChooser.getSelected())

    def stop(self) -> None:
        self.motor.set(0)

    # Optional helper

    def isRunning(self) -> bool:
        return abs(self.motor.get()) > 0.01

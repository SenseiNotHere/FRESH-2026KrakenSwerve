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
            leadMotorCANID: int,
            followMotorCANID: int,
            leadMotorInverted: bool,
            followMotorInverted: bool
    ):
        super().__init__()
        # Motor init
        self.leadMotor = SparkMax(leadMotorCANID, SparkLowLevel.MotorType.kBrushed)
        self.followMotor = SparkMax(followMotorCANID, SparkLowLevel.MotorType.kBrushed)

        leadConfig = SparkMaxConfig()

        leadConfig.setIdleMode(SparkMaxConfig.IdleMode.kCoast)
        leadConfig.inverted(leadMotorInverted)

        leadConfig.closedLoop.P(AgitatorConstants.kP)
        leadConfig.closedLoop.I(AgitatorConstants.kI)
        leadConfig.closedLoop.D(AgitatorConstants.kD)

        self.leadMotor.configure(
            leadConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        )

        followConfig = SparkMaxConfig()
        followConfig.follow(leadMotorCANID, followMotorInverted)
        self.followMotor.configure(
            followConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        )

        self.speedChooser = SendableChooser()
        self.speedChooser.setDefaultOption("25%", 0.25)
        self.speedChooser.addOption("5%", 0.05)
        self.speedChooser.addOption("50%", 0.5)
        self.speedChooser.addOption("75%", 0.75)
        self.speedChooser.addOption("100%", 1.0)
        SmartDashboard.putData(self.speedChooser)

    def periodic(self):
        SmartDashboard.putNumber("Agitator Speed", self.leadMotor.get())
        SmartDashboard.putBoolean("Agitator Running", self.isRunning())

    def feed(self) -> None:
        self.leadMotor.set(self.speedChooser.getSelected())

    def reverse(self) -> None:
        self.leadMotor.set(-self.speedChooser.getSelected())

    def stop(self) -> None:
        self.leadMotor.set(0)

    # Optional helper

    def isRunning(self) -> bool:
        return abs(self.leadMotor.get()) > 0.01

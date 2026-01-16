from commands2 import Subsystem
from phoenix6.configs import TalonFXConfiguration
from phoenix6.hardware import TalonFX
from phoenix6.controls import VelocityVoltage
from phoenix6.signals import NeutralModeValue, InvertedValue
from wpilib import SmartDashboard, SendableChooser

class Indexer(Subsystem):
    def __init__(
            self,
            motorCANID: int,
    ):
        super().__init__()

        self.motor = TalonFX(motorCANID)

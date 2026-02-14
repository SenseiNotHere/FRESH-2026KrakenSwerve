from commands2 import Subsystem
from wpilib import (
    Compressor,
    PneumaticsModuleType,
    SmartDashboard
)


class Pneumatics(Subsystem):
    def __init__(self, moduleID: int, moduleType: PneumaticsModuleType):
        super().__init__()

        self.compressor = Compressor(moduleID, moduleType)

        # Enable closed loop control (REV PH or PCM)
        self.compressor.enableDigital()

    def enable(self):
        self.compressor.enableDigital()

    def disable(self):
        self.compressor.disable()

    def isEnabled(self) -> bool:
        return self.compressor.enabled()

    def periodic(self):
        SmartDashboard.putBoolean(
            "Pneumatics/Compressor Enabled",
            self.compressor.isEnabled()
        )

from commands2 import Subsystem
from wpilib import Compressor, DoubleSolenoid, PneumaticsModuleType


class ClimberSubsystem(Subsystem):
    def __init__(
        self,
        pcmCANID: int,
        forwardChannel: int,
        reverseChannel: int,
    ):
        super().__init__()

        # Compressor on CTRE PCM
        self.compressor = Compressor(
            pcmCANID,
            PneumaticsModuleType.CTREPCM
        )
        self.compressor.enableDigital()

        # Double solenoid on the PCM
        self.solenoid = DoubleSolenoid(
            pcmCANID,
            PneumaticsModuleType.CTREPCM,
            forwardChannel,
            reverseChannel
        )

    def extend(self):
        self.solenoid.set(DoubleSolenoid.Value.kForward)

    def retract(self):
        self.solenoid.set(DoubleSolenoid.Value.kReverse)

    def toggle(self):
        if self.solenoid.get() == DoubleSolenoid.Value.kForward:
            self.retract()
        else:
            self.extend()

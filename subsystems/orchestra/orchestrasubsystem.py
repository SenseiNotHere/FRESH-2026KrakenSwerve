from typing import Optional

from commands2 import Subsystem
from phoenix6.orchestra import Orchestra

class OrchestraSubsystem(Subsystem):
    def __init__(
        self,
        driveSubsystem,
        climberSubsystem: Optional[Subsystem] = None,
        indexerSubsystem: Optional[Subsystem] = None,
        intakeSubsystem: Optional[Subsystem] = None,
        shooterSubsystem: Optional[Subsystem] = None,
    ):
        super().__init__()

        self.orchestra = Orchestra()

        # Register all motors ONCE
        for subsystem in (
            driveSubsystem,
            climberSubsystem,
            indexerSubsystem,
            intakeSubsystem,
            shooterSubsystem,
        ):

            if subsystem is None:
                continue

            for motor in subsystem.getMotors():
                self.orchestra.add_instrument(motor)

    # Orchestra Controls

    def loadSound(self, path: str):
        """
        Loads a .chrp music file into the orchestra.
        """
        self.orchestra.load_music(path)

    def playSound(self):
        """
        Starts playing the loaded song.
        """
        self.orchestra.play()

    def stopSound(self):
        """
        Stops music playback.
        """
        self.orchestra.stop()

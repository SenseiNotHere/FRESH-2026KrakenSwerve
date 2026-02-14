from commands2 import Command
from subsystems.climber.climbersubsystem import Climber

class ToggleClimber(Command):
    def __init__(self, climber: Climber):
        super().__init__()
        self.climber = climber
        self.addRequirements(self.climber)

    def initialize(self):
        self.climber.toggle()

    def isFinished(self) -> bool:
        return True

class ManualClimb(Command):

    def __init__(self, climber, joystick):
        super().__init__()
        self.climber = climber
        self.ySupplier = joystick
        self.addRequirements(climber)

    def execute(self):
        value = -self.ySupplier()
        self.climber.manualAdjust(value)

    def isFinished(self):
        return False

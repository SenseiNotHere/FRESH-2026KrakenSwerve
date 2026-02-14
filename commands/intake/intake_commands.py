from commands2 import Command
from superstructure.superstructure import Superstructure
from superstructure.robot_state import RobotState


class RunIntake(Command):

    def __init__(self, superstructure: Superstructure):
        super().__init__()
        self.superstructure = superstructure

    def initialize(self):
        self.superstructure.setState(RobotState.INTAKING)

    def end(self, interrupted: bool):
        self.superstructure.setState(RobotState.IDLE)

    def isFinished(self) -> bool:
        return False

class ReverseIntake(Command):

    def __init__(self, superstructure: Superstructure):
        super().__init__()
        self.superstructure = superstructure

        if superstructure.hasIntake:
            self.addRequirements(superstructure.intake)
        if superstructure.hasIndexer:
            self.addRequirements(superstructure.indexer)

    def initialize(self):
        self.superstructure.setState(RobotState.IDLE)

    def execute(self):
        if self.superstructure.hasIntake:
            self.superstructure.intake.reverse()

        if self.superstructure.hasIndexer:
            self.superstructure.indexer.reverse()

    def end(self, interrupted: bool):
        self.superstructure.setState(RobotState.IDLE)

    def isFinished(self):
        return False

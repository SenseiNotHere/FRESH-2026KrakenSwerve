from commands2 import Command
from superstructure.superstructure import Superstructure
from superstructure.robot_state import RobotState


class PrepShot(Command):

    def __init__(self, superstructure: Superstructure):
        super().__init__()
        self.superstructure = superstructure

    def initialize(self):
        self.superstructure.setState(RobotState.PREP_SHOT)

    def end(self, interrupted):
        self.superstructure.setState(RobotState.IDLE)

    def isFinished(self):
        return False

class Shoot(Command):

    def __init__(self, superstructure: Superstructure):
        super().__init__()
        self.superstructure = superstructure

    def initialize(self):
        self.superstructure.setState(RobotState.SHOOTING)

    def end(self, interrupted):
        self.superstructure.setState(RobotState.IDLE)

    def isFinished(self):
        return False

class ManualIndexer(Command):

    def __init__(self, superstructure: Superstructure):
        super().__init__()
        self.superstructure = superstructure

        if superstructure.hasIndexer:
            self.addRequirements(superstructure.indexer)

    def initialize(self):
        if self.superstructure.hasIndexer:
            self.superstructure.indexer.feed()

    def end(self, interrupted):
        if self.superstructure.hasIndexer:
            self.superstructure.indexer.stop()

    def isFinished(self):
        return False

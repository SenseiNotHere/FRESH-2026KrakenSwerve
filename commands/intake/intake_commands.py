from commands2 import Command
from superstructure.superstructure import Superstructure
from superstructure.robot_state import RobotState


class RunIntake(Command):

    def __init__(self, superstructure: Superstructure):
        super().__init__()
        self.superstructure = superstructure

        self.addRequirements(superstructure.intake)

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

        self.addRequirements(superstructure.intake)

    def initialize(self):
        self.superstructure.setState(RobotState.IDLE)

    def execute(self):
        if self.superstructure.hasIntake:
            self.superstructure.intake.reverse()

    def end(self, interrupted: bool):
        self.superstructure.setState(RobotState.IDLE)

    def isFinished(self):
        return False

class DeployAndRunIntake(Command):

    def __init__(self, superstructure: Superstructure):
        super().__init__()
        self.superstructure = superstructure

        self.addRequirements(superstructure.intake)

    def initialize(self):

        if not self.superstructure.hasIntake:
            return

        # Deploy if not already deployed
        if not self.superstructure.intake.isDeployed():
            self.superstructure.intake.deploy()

        # Switch to INTAKING state
        self.superstructure.setState(RobotState.INTAKING)

    def end(self, interrupted: bool):
        self.superstructure.setState(RobotState.IDLE)

    def isFinished(self):
        return False
    
class DeployRetractIntake(Command):

    def __init__(self, superstructure: Superstructure):
        super().__init__()
        self.superstructure = superstructure

        self.addRequirements(superstructure.intake)

    def initialize(self):
        if not self.superstructure.hasIntake:
            return

        # Deploy if not already deployed
        if not self.superstructure.intake.isDeployed():
            self.superstructure.intake.deploy()
        else:
            self.superstructure.intake.stow()

    def end(self, interrupted: bool):
        self.superstructure.setState(RobotState.IDLE)

    def isFinished(self):
        return True
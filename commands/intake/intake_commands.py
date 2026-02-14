from commands2 import Command


class RunIntake(Command):

    def __init__(self, intake):
        super().__init__()
        self.intake = intake
        self.addRequirements(intake)

    def initialize(self):
        self.intake.intake()

    def end(self, interrupted: bool):
        self.intake.stop()

    def isFinished(self) -> bool:
        return False

class ReverseIntake(Command):

    def __init__(self, intake):
        super().__init__()
        self.intake = intake
        self.addRequirements(intake)

    def initialize(self):
        self.intake.reverse()

    def end(self, interrupted: bool):
        self.intake.stop()

    def isFinished(self):
        return False

class ToggleIntakeDeploy(Command):

    def __init__(self, intake):
        super().__init__()
        self.intake = intake
        self.addRequirements(intake)

    def initialize(self):
        self.intake.toggleDeploy()

    def isFinished(self):
        return True

class DeployAndRunIntake(Command):

    def __init__(self, intake):
        super().__init__()
        self.intake = intake
        self.addRequirements(intake)

    def initialize(self):
        self.intake.deploy()
        self.intake.intake()

    def end(self, interrupted):
        self.intake.stop()

    def isFinished(self):
        return False

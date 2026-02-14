from commands2 import Command
from superstructure.superstructure import Superstructure
from superstructure.robot_state import RobotState


class ToggleClimbAuto(Command):

    def __init__(self, superstructure: Superstructure):
        super().__init__()
        self.superstructure = superstructure

    def initialize(self):
        # Toggle between IDLE and CLIMB_AUTO
        if self.superstructure.robot_state == RobotState.CLIMB_AUTO:
            self.superstructure.setState(RobotState.IDLE)
        else:
            self.superstructure.setState(RobotState.CLIMB_AUTO)

    def isFinished(self) -> bool:
        return True

class ManualClimb(Command):

    def __init__(self, superstructure: Superstructure, joystick):
        super().__init__()
        self.superstructure = superstructure
        self.ySupplier = joystick

        # Only require climber subsystem
        if superstructure.hasClimber:
            self.addRequirements(superstructure.climber)

    def initialize(self):
        self.superstructure.setState(RobotState.CLIMB_MANUAL)

    def execute(self):

        if not self.superstructure.hasClimber:
            return

        value = -self.ySupplier()

        self.superstructure.climber.manualAdjust(value)

    def end(self, interrupted: bool):
        self.superstructure.setState(RobotState.IDLE)

    def isFinished(self):
        return False
from commands2 import Command
from superstructure.superstructure import Superstructure
from superstructure.robot_state import RobotState


class ToggleClimbAuto(Command):

    def __init__(self, superstructure: Superstructure):
        super().__init__()
        self.superstructure = superstructure

    def initialize(self):

        state = self.superstructure.robot_state

        # If already up and locked go down
        if state == RobotState.AIRBREAK_ENGAGED_UP:
            self.superstructure.setState(RobotState.ELEVATOR_LOWERING)

        # Otherwise go up
        else:
            self.superstructure.setState(RobotState.ELEVATOR_RISING)

    def isFinished(self) -> bool:
        return True


class ManualClimb(Command):

    def __init__(self, superstructure: Superstructure, joystickSupplier):
        super().__init__()
        self.superstructure = superstructure
        self.joystickSupplier = joystickSupplier

        if superstructure.hasClimber:
            self.addRequirements(superstructure.climber)

    def initialize(self):
        self.superstructure.setState(RobotState.CLIMB_MANUAL)

    def execute(self):

        if not self.superstructure.hasClimber:
            return

        value = -self.joystickSupplier()

        # If joystick active ensure brake released
        if abs(value) > 0.1:
            self.superstructure.climber.releaseAirbrake()
            self.superstructure.climber.manualVelocity(value)

    def end(self, interrupted: bool):

        if self.superstructure.hasClimber:
            self.superstructure.climber.stop()

        self.superstructure.setState(RobotState.IDLE)

    def isFinished(self):
        return False

class HoldAirbrake(Command):

    def __init__(self, climber):
        super().__init__()
        self.climber = climber
        self.addRequirements(climber)

    def initialize(self):
        self.climber.engageAirbrake()

    def end(self, interrupted: bool):
        self.climber.releaseAirbrake()

    def isFinished(self):
        return False
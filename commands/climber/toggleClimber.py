from commands2 import InstantCommand

class ToggleClimber(InstantCommand):
    def __init__(self, climber):
        super().__init__(climber.toggle, climber)

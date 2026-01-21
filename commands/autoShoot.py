from commands2 import Command
from wpilib import SmartDashboard


class AutoShoot(Command):
    def __init__(self, shooter, indexer, shotCalculator, tolerance_rpm=150):
        super().__init__()

        self.shooter = shooter
        self.indexer = indexer
        self.shotCalculator = shotCalculator
        self.tolerance_rpm = tolerance_rpm

        self.addRequirements(shooter, indexer)

    def initialize(self):
        SmartDashboard.putBoolean("Shooter Use Auto Speed", True)
        self.shooter.enable()

    def execute(self):
        target_rps = self.shotCalculator.target_speed_rps
        self.shooter.setTargetRPS(target_rps)

        if self.shooter.atSpeed(self.tolerance_rpm):
            self.indexer.enable()
        else:
            self.indexer.disable()

    def end(self, interrupted: bool):
        SmartDashboard.putBoolean("Shooter Use Auto Speed", False)
        self.indexer.disable()
        self.shooter.disable()

    def isFinished(self) -> bool:
        return False

from commands2 import Command
from subsystems.shooter.shootersubsystem import Shooter
from subsystems.shooter.indexersubsystem import Indexer


class ShootWithIndexer(Command):
    def __init__(self, shooter: Shooter, indexer: Indexer, tolerance_rpm: float):
        super().__init__()
        self.shooter = shooter
        self.indexer = indexer
        self.tolerance = tolerance_rpm

        self.addRequirements(shooter, indexer)

    def initialize(self):
        self.shooter.enable()

    def execute(self):
        if self.shooter.atSpeed(self.tolerance):
            self.indexer.enable()
        else:
            self.indexer.stop()

    def end(self, interrupted: bool):
        self.shooter.disable()
        self.indexer.stop()

    def isFinished(self) -> bool:
        return False

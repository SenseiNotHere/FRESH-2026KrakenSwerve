from commands2 import Command
from subsystems.shooter.indexersubsystem import Indexer


class RunIndexer(Command):
    def __init__(self, indexer: Indexer):
        super().__init__()
        self.indexer = indexer
        self.addRequirements(indexer)

    def initialize(self):
        self.indexer.enable()

    def end(self, interrupted: bool):
        self.indexer.stop()

    def isFinished(self) -> bool:
        return False

class RunIndexerPercent(Command):
    def __init__(self, indexer: Indexer, percent: float):
        super().__init__()
        self.indexer = indexer
        self.percent = percent
        self.addRequirements(indexer)

    def initialize(self):
        self.indexer.feedPercent(self.percent)

    def end(self, interrupted: bool):
        self.indexer.stop()

    def isFinished(self) -> bool:
        return False

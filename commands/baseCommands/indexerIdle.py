from commands2 import Command

from subsystems.indexer import Indexer


class IndexerIdle(Command):
    def __init__(self, indexer: Indexer):
        super().__init__()
        self.indexer = indexer
        self.addRequirements(indexer)

    def initialize(self):
        self.indexer.setSetpoint(0.1)

    def isFinished(self) -> bool:
        return True

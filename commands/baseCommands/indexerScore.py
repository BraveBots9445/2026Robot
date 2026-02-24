from commands2 import Command

from subsystems.indexer import Indexer


class IndexerScore(Command):
    def __init__(self, indexer: Indexer):
        super().__init__()
        self.indexer = indexer
        self.addRequirements(indexer)

    def initialize(self):
        self.indexer.setSetpointShooting()

    def isFinished(self) -> bool:
        return True

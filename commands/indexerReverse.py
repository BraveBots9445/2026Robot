from commands2 import Command

from subsystems.indexer import Indexer


class IndexerReverse(Command):
    def __init__(self, indexer: Indexer):
        self._indexer = indexer
        self.addRequirements(indexer)

    def initialize(self) -> None:
        self._indexer.setSetpoint(-1.0)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self._indexer.setSetpoint(0.0)

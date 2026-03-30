from wpilib import Timer

from wpimath.units import seconds

from commands.indexerReverse import IndexerReverse
from subsystems.indexer import Indexer


class IndexerDejam(IndexerReverse):
    def __init__(self, indexer: Indexer, timeout: seconds | None = None):
        super().__init__(indexer)
        self.timer = Timer()
        self.timeout = timeout

    def initialize(self) -> None:
        self.timer.reset()
        self.timer.start()
        super().initialize()

    def isFinished(self) -> bool:
        return self.timeout is not None and self.timer.hasElapsed(self.timeout)

    def end(self, interrupted: bool) -> None:
        self.timer.stop()
        super().end(interrupted)

from commands2 import Command

from wpilib import Timer
from wpimath.units import seconds

from subsystems.indexer import Indexer


class IndexerDejam(Command):
    def __init__(self, indexer: Indexer, time: seconds = 0.5):
        super().__init__()
        self.indexer = indexer
        self.addRequirements(indexer)
        self.timer = Timer()
        self.time = time

    def initialize(self):
        self.timer.restart()
        self.indexer.setSetpoint(-0.1)

    def isFinished(self) -> bool:
        return self.timer.hasElapsed(self.time)

    def end(self, interrupted: bool):
        self.indexer.setSetpoint(0.0)

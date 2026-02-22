from commands2 import Command
from subsystems.indexer import Indexer

class IndexerIndex (Command):
    def __init__(self, indexer: Indexer):
        super().__init__()
        self.addRequirements(indexer)
        self.indexer = indexer 

    def initialize(self):
        pass

    def execute(self):
        self.indexer.setMotorSpeed(.5)

    def end(self, interrupted: bool):
        self.indexer.setMotorSpeed(0)

    def isFinished(self) -> bool:
        return False


"""
class IndexerIndex(Command):
    self.addRequirements(indexer)

    def initialize(self):
        pass

    def execute(self):
        self.indexer.setMotorSpeed(.5)

    def end(self, interrupted: bool):
        self.indexer.setMotorSpeed(0)
        pass
"""

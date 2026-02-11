"""
Copy this file and write your own commands based on it. This file should not be imported anywhere
"""

from commands2 import Command
from subsystems.indexer import Indexer


class IndexerIndex(Command):
    def __init__(self, indexer: Indexer):
        super().__init__()
        self.indexer =  indexer
        # Use addRequirements() here to declare subsystem dependencies.
        # e.g. self.addRequirements(subsystem)
        self.addRequirements(indexer)

    def initialize(self):
        pass

    def execute(self):
        self.indexer.setMotorSpeed(1)
        

    def end(self, interrupted: bool):
        self.indexer.setMotorSpeed(0)
        pass

    def isFinished(self) -> bool:
        return False

"""
Copy this file and write your own commands based on it. This file should not be imported anywhere
"""

from commands2 import Command


class CommandTemplate(Command):
    def __init__(self):
        super().__init__()
        # Use addRequirements() here to declare subsystem dependencies.
        # e.g. self.addRequirements(subsystem)

    def initialize(self):
        pass

    def execute(self):
        pass

    def end(self, interrupted: bool):
        pass

    def isFinished(self) -> bool:
        return False

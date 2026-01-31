"""
Copy this file and write your own commands based on it. This file should not be imported anywhere
"""

from commands2 import Command
from subsystems.intake import Intake

class IntakeRoationSpin(Command):
    def __init__(self, intake: Intake):
        super().__init__()
        self.addRequirements(intake)
        self.intake = intake

    def initialize(self):
        pass

    def execute(self):
        self.intake.set_angle(.6)
        pass

    def end(self, interrupted: bool):
        self.intake.set_angle(0)
        pass

    def isFinished(self) -> bool:
        return False

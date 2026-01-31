"""
Copy this file and write your own commands based on it. This file should not be imported anywhere
"""

from commands2 import Command
from subsystems.intake import Intake

class IntakeSetsetpoint(Command):
    def __init__(self, intake: Intake, setpoint):
        super().__init__()
        self.addRequirements(Intake)
        self.setpoint = setpoint
        self.intake = intake
        # Use addRequirements() here to declare subsystem dependencies.
        # e.g. self.addRequirements(subsystem)

    def initialize(self):
        pass

    def execute(self):
        self.intake.setsetpoint(self.setpoint)
        pass

    def end(self, interrupted: bool):
        pass

    def isFinished(self) -> bool:
        return False

"""
Copy this file and write your own commands based on it. This file should not be imported anywhere
"""

from commands2 import Command
from subsystems.climber import Climber

class ClimberClimb(Command):
    def __init__(self, climber: Climber):
        super().__init__()
        self.climber = climber
        self.addRequirements(climber)
        # Use addRequirements() here to declare subsystem dependencies.
        # e.g. self.addRequirements(subsystem)

    def initialize(self):
        pass

    def execute(self,number):
        self.climber.setVelocity(number)

    def end(self, interrupted: bool):
        self.climber.setVelocity(0)
        pass

    def isFinished(self) -> bool:
        return False

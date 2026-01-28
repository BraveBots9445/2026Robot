"""
Copy this file and write your own commands based on it. This file should not be imported anywhere
"""

from commands2 import Command
from subsystems.shooter import Shooter


class ShooterShoot(Command):
    def __init__(self, shooter: Shooter):
        super().__init__()
        self.shooter = shooter
        self.addRequirements(shooter)
        # Use addRequirements() here to declare subsystem dependencies.
        # e.g. self.addRequirements(subsystem)

    def initialize(self):
        pass

    def execute(self):
        self.shooter.setVelocity(0.65)

    def end(self, interrupted: bool):
        pass

    def isFinished(self) -> bool:
        return False

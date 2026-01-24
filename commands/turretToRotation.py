"""
Copy this file and write your own commands based on it. This file should not be imported anywhere
"""
from typing import Callable
from wpimath.units import degrees
from commands2 import Command
from subsystems.turret import Turret

class TurretToRotation(Command):
    def __init__(self,
                 turret: Turret,
                 position: degrees):
        super().__init__()
        self.turret = turret
        self.position = position

        self.addRequirements(self.turret)
        # Use addRequirements() here to declare subsystem dependencies.
        # e.g. self.addRequirements(subsystem)

    def initialize(self):
        self.turret.setPosition( self.position )

    def execute(self):
        pass
        # currentPosition = self.turret.getPosition()
        # newPosition = self.turret.getDesiredPosition() + ( self.rotate() )
        # self.turret.setPosition( newPosition )

    def end(self, interrupted: bool):
        pass

    def isFinished(self) -> bool:
        return self.turret.atPosition()

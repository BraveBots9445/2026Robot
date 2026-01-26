"""
Copy this file and write your own commands based on it. This file should not be imported anywhere
"""
from typing import Callable
from commands2 import Command
from subsystems.turret import Turret

class TurretByStick(Command):
    def __init__(self,
                 turret: Turret,
                 axis: Callable[[], float] = lambda: 0.0):
        super().__init__()
        self.turret = turret
        self.rotate = axis

        self.addRequirements(self.turret)

    def initialize(self):
        pass

    def execute(self):
        newPosition = self.turret.getDesiredPosition() + ( self.rotate() )
        self.turret.setPosition( newPosition )

    def end(self, interrupted: bool):
        pass

    def isFinished(self) -> bool:
        return False

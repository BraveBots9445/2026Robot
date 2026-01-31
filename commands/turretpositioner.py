#turret position thing

from commands2 import Command
import wpilib
from subsystems.turret import Turret

class TurretPositionCommand(Command):
    def __init__(self, turret: Turret, position: float) -> None:
        super().__init__()
        self.turret = turret
        self.position = position
        self.addRequirements(turret)

    def initialize(self) -> None:
        self.turret.set_angle(self.position)

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return self.turret.atsetpoint()
    
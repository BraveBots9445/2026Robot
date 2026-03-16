from typing import Callable

from commands2 import Command

from wpimath.geometry import Rotation2d

from subsystems import Turret


class TurretManual(Command):
    _velocityFactor: float = 1

    def __init__(self, turret: Turret, moveVel: Callable[[], float]) -> None:
        self.turret = turret
        self.moveVel = moveVel

        self.addRequirements(turret)

    def execute(self):
        self.turret.bumpManualOffset(self.moveVel() * self._velocityFactor)

    def isFinished(self) -> bool:
        return False

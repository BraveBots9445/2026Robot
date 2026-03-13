from typing import Callable

from commands2 import Command

from wpimath.geometry import Rotation2d

from subsystems import Turret


class TurretManual(Command):
    def __init__(self, turret: Turret, moveVel: Callable[[], float]) -> None:
        self.turret = turret
        self.moveVel = moveVel

        self.addRequirements(turret)

    def execute(self):
        self.turret.bumpManualOffset(self.moveVel() * 0.02)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        self.turret._manualSetpointOffset = Rotation2d()

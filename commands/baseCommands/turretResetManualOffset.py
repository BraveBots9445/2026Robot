from commands2 import Command

from wpimath.geometry import Rotation2d

from subsystems import Turret


class TurretResetManualOffset(Command):
    def __init__(self, turret: Turret) -> None:
        self.turret = turret

        self.addRequirements(turret)

    def initialize(self):
        self.turret.resetManualOffset()

    def isFinished(self) -> bool:
        return True

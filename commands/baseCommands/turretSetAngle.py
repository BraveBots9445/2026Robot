from commands2 import Command

from wpimath.units import degrees

from subsystems import Turret


class TurretSetAngle(Command):
    def __init__(self, turret: Turret, angle: degrees):
        super().__init__()
        self.turret = turret
        self.angle = angle
        self.addRequirements(turret)

    def initialize(self):
        self.turret.setSetpointDegrees(self.angle)

    def isFinished(self) -> bool:
        return True

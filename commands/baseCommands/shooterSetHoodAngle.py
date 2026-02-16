from commands2 import Command

from subsystems import Shooter

from wpimath.units import degrees


class ShooterSetHoodAngle(Command):
    def __init__(self, shooter: Shooter, angle: degrees):
        super().__init__()
        self.shooter = shooter
        self.angle = angle
        self.addRequirements(shooter)

    def initialize(self):
        self.shooter.setHoodAngleSetpointDegrees(self.angle)

    def isFinished(self) -> bool:
        return True

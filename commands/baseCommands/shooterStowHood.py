from commands2 import Command

from subsystems import Shooter


class ShooterStowHood(Command):
    def __init__(self, shooter: Shooter):
        super().__init__()
        self.shooter = shooter
        self.addRequirements(shooter)

    def initialize(self):
        self.shooter.setHoodAngleSetpoint(self.shooter.maxHoodAngle, ignoreManual=True)

    def isFinished(self) -> bool:
        return False

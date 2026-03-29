from commands2 import Command

from subsystems.shooter import Shooter


class ShooterStow(Command):
    def __init__(self, shooter: Shooter):
        self.shooter = shooter

        self.setName("ShooterStow")
        self.addRequirements(self.shooter)

    def initialize(self):
        self.shooter.setForceNotReady(True)
        # self.shooter.setFlywheelSetpoint(0)
        self.shooter.setHoodAngleSetpoint(self.shooter.minHoodAngle)

    def end(self, interrupted: bool):
        self.shooter.setForceNotReady(False)

    def isFinished(self) -> bool:
        return False

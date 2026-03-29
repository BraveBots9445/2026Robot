from commands2 import Command

from subsystems.shooter import Shooter


class ShooterStop(Command):
    def __init__(self, shooter: Shooter):
        self.shooter = shooter

        self.setName("ShooterStop")
        self.addRequirements(self.shooter)

    def initialize(self):
        self.shooter.setFlywheelSetpoint(0)

    def isFinished(self) -> bool:
        return False

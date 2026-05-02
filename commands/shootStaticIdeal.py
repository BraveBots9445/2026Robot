from commands2 import Command

from subsystems.shooter import Shooter


class ShooterStaticIdeal(Command):
    def __init__(self, shooter: Shooter) -> None:
        self.shooter = shooter
        self.addRequirements(shooter)

    def initialize(self):
        self.shooter.setFlywheelSetpoint(2500)
        self.shooter.setHoodAngleSetpointDegrees(72.9)

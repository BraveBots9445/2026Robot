from commands2 import Command

from subsystems.shooter import Shooter


class ShootPassIdeal(Command):
    def __init__(self, shooter: Shooter) -> None:
        self.shooter = shooter
        self.addRequirements(shooter)

    def initialize(self):
        self.shooter.setFlywheelSetpoint(3000)
        self.shooter.setHoodAngleSetpointDegrees(70)

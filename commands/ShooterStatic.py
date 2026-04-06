from commands2 import Command

from subsystems.shooter import Shooter


class ShooterStatic(Command):
    def __init__(self, shooter: Shooter, willFinish: bool = False):
        self.shooter = shooter

        self.willFinish = willFinish

        self.setName("ShooterStatic")
        self.addRequirements(self.shooter)

    def initialize(self):
        self.shooter.setFlywheelSetpoint(3000)
        self.shooter.setHoodAngleSetpointDegrees(67)

    def isFinished(self) -> bool:
        return self.willFinish

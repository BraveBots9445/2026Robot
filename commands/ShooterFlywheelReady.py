from commands2 import Command

from subsystems.shooter import Shooter


class ShooterFlywheelReady(Command):
    def __init__(self, shooter: Shooter, speedRPM: float = 3000):
        self.shooter = shooter
        self.speedRPM = speedRPM

        self.setName("ShooterFlywheelReady")
        self.addRequirements(self.shooter)

    def initialize(self):
        self.shooter.setFlywheelSetpoint(self.speedRPM)

    def isFinished(self) -> bool:
        return False

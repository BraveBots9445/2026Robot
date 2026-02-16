from commands2 import Command

from wpimath.units import revolutions_per_minute, degrees

from subsystems import Shooter


class ShooterSetState(Command):
    def __init__(
        self,
        shooter: Shooter,
        flywheelVelocity: revolutions_per_minute,
        hoodAngle: degrees,
    ):
        super().__init__()
        self.shooter = shooter
        self.flywheelVelocity = flywheelVelocity
        self.hoodAngle = hoodAngle
        self.addRequirements(shooter)

    def initialize(self):
        self.shooter.setFlywheelSetpoint(self.flywheelVelocity)
        self.shooter.setHoodAngleSetpointDegrees(self.hoodAngle)

    def isFinished(self) -> bool:
        return True

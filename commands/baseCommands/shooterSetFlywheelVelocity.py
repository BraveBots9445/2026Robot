from commands2 import Command

from subsystems import Shooter

from wpimath.units import revolutions_per_minute


class ShooterSetFlywheelVelocity(Command):
    def __init__(self, shooter: Shooter, velocity: revolutions_per_minute):
        super().__init__()
        self.shooter = shooter
        self.velocity = velocity
        self.addRequirements(shooter)

    def initialize(self):
        self.shooter.setFlywheelSetpoint(self.velocity)

    def isFinished(self) -> bool:
        return True

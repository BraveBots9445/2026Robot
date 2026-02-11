from commands2 import Command
from subsystems.shooter import Shooter


class ShooterShoot(Command):
    def __init__(self, shooter: Shooter, speed: float):
        super().__init__()
        self.shooter = shooter
        self.addRequirements(shooter)
        self.speed = speed
        # Use addRequirements() here to declare subsystem dependencies.
        # e.g. self.addRequirements(subsystem)

    def initialize(self):
        pass

    def execute(self):
        self.shooter.setVelocity(self.speed)

    def end(self, interrupted: bool):
        self.shooter.setVelocity(0)


    def isFinished(self) -> bool:
        return False

from commands2 import Command

from subsystems import Shooter, Turret


class ShootStaticTowerRight(Command):
    """
    This is supposed to be a dumb command that does no logic and just uses static setpoints to shoot into the hub when on the right post of the tower
    """

    def __init__(
        self,
        shooter: Shooter,
        turret: Turret,
    ):
        self.shooter = shooter
        self.turret = turret
        self.addRequirements(shooter, turret)

    def execute(self):
        self.shooter.setFlywheelSetpoint(4000)
        self.shooter.setHoodAngleSetpointDegrees(45)
        self.turret.setSetpointDegrees(12)

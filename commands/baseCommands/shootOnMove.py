from commands2 import Command

from subsystems import Shooter, Turret, ShootOnMoveCalculator

from tools.rebuilt import Rebuilt, RebuiltPositions


# TODO: Implement
class ShootOnMove(Command):
    def __init__(
        self,
        shooter: Shooter,
        turret: Turret,
        shootOnMoveCalculator: ShootOnMoveCalculator,
    ):
        super().__init__()
        self.shooter = shooter
        self.turret = turret
        self.shootOnMoveCalculator = shootOnMoveCalculator
        self.target = Rebuilt.getPosition(RebuiltPositions.Hub)
        self.addRequirements(shooter, turret)

    def initialize(self):
        self.target = Rebuilt.getPosition(RebuiltPositions.Hub)

    def execute(self):
        setpoints = self.shootOnMoveCalculator.getSetpoints(self.target)
        if setpoints:
            self.shooter.setFlywheelSetpoint(setpoints.flywheelRpm)
            self.shooter.setHoodAngleSetpoint(setpoints.hoodAngle)
            self.turret.setSetpoint(setpoints.turretAngle)

    def isFinished(self) -> bool:
        return False

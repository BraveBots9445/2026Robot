from commands2 import Command

from subsystems import Turret, ShootOnMoveCalculator

from tools.rebuilt import Rebuilt, RebuiltPositions


class TurretShootOnMove(Command):
    def __init__(
        self,
        turret: Turret,
        shootOnMoveCalculator: ShootOnMoveCalculator,
    ):
        super().__init__()
        self.shootOnMoveCalculator = shootOnMoveCalculator
        self.turret = turret
        self.target = Rebuilt.getPosition(RebuiltPositions.Hub)
        self.addRequirements(self.turret)

    def initialize(self):
        self.target = Rebuilt.getPosition(RebuiltPositions.Hub)

    def execute(self):
        setpoints = self.shootOnMoveCalculator.getSetpoints(self.target)
        self.turret.setSetpoint(setpoints.turretAngle)

    def isFinished(self) -> bool:
        return False

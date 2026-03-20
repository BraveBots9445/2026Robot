from commands2 import Command

from wpimath.geometry import Pose3d, Rotation3d
from wpimath.units import inchesToMeters

from subsystems import Shooter, ZoneManager, ShootOnMoveCalculator
from tools.rebuilt import Rebuilt, RebuiltPositions


class ShooterPass(Command):
    def __init__(
        self,
        zoneManager: ZoneManager,
        shootOnMoveCalculator: ShootOnMoveCalculator,
        shooter: Shooter,
    ):
        """
        A command to pass the ball to a teammate.

        :param shooter: The Shooter Subsytem
        :type shooter: Shooter
        """
        super().__init__()
        self.shooter = shooter
        self.zoneManager = zoneManager
        self.shootOnMoveCalculator = shootOnMoveCalculator

        self.addRequirements(shooter)

    def execute(self):
        target = Pose3d()
        if self.zoneManager.getOnLeftBool():
            target = Rebuilt.getPosition(RebuiltPositions.PassLeft)
        else:
            target = Rebuilt.getPosition(RebuiltPositions.PassRight)

        setpoints = self.shootOnMoveCalculator.getSetpoints(target, passing=True)
        self.shooter.setFlywheelSetpoint(setpoints.flywheelRpm)
        self.shooter.setHoodAngleSetpoint(setpoints.hoodAngle)

    def isFinished(self) -> bool:
        return self.zoneManager.getInAllianceZoneBool()

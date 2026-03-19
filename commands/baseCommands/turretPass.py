from commands2 import Command

from wpimath.geometry import Pose3d, Rotation3d
from wpimath.units import inchesToMeters

from subsystems import Turret, ZoneManager, ShootOnMoveCalculator
from tools.rebuilt import Rebuilt


class TurretPass(Command):
    def __init__(
        self,
        zoneManager: ZoneManager,
        shootOnMoveCalculator: ShootOnMoveCalculator,
        turret: Turret,
    ):
        """
        A command to pass the ball to a teammate.

        :param shooter: The Shooter Subsytem
        :type shooter: Shooter
        """
        super().__init__()
        self.turret = turret
        self.zoneManager = zoneManager
        self.shootOnMoveCalculator = shootOnMoveCalculator

        self.addRequirements(turret)

    def execute(self):
        target = Pose3d()
        if self.zoneManager.getOnLeftBool():
            target = Rebuilt.getPosition(
                Pose3d(
                    inchesToMeters(32),
                    inchesToMeters(270),
                    inchesToMeters(20),
                    Rotation3d(),
                )
            )
        else:
            target = Rebuilt.getPosition(
                Pose3d(
                    inchesToMeters(48),
                    inchesToMeters(48),
                    inchesToMeters(20),
                    Rotation3d(),
                )
            )

        setpoints = self.shootOnMoveCalculator.getSetpoints(target, passing=True)
        self.turret.setSetpoint(setpoints.turretAngle)

    def isFinished(self) -> bool:
        return self.zoneManager.getInAllianceZoneBool()

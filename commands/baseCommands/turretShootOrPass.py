from commands2 import SelectCommand, RepeatCommand

from commands.baseCommands.turretShootOnMove import TurretShootOnMove
from commands.baseCommands.turretPass import TurretPass

from subsystems import Turret, ShootOnMoveCalculator, ZoneManager


class TurretShootOrPass(RepeatCommand):
    def __init__(
        self,
        turret: Turret,
        shootOnMoveCalculator: ShootOnMoveCalculator,
        zoneManager: ZoneManager,
    ):
        super().__init__(
            SelectCommand(
                {
                    True: TurretShootOnMove(turret, shootOnMoveCalculator),
                    False: TurretPass(zoneManager, shootOnMoveCalculator, turret),
                },
                zoneManager.getInAllianceZoneBool,
            )
        )

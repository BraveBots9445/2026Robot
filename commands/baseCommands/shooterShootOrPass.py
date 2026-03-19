from commands2 import SelectCommand, RepeatCommand

from commands.baseCommands.shooterShootOnMove import ShooterShootOnMove
from commands.baseCommands.shooterPass import ShooterPass

from subsystems import Shooter, ShootOnMoveCalculator, ZoneManager


class ShooterShootOrPass(RepeatCommand):
    def __init__(
        self,
        shooter: Shooter,
        shootOnMoveCalculator: ShootOnMoveCalculator,
        zoneManager: ZoneManager,
    ):
        super().__init__(
            SelectCommand(
                {
                    True: ShooterShootOnMove(shooter, shootOnMoveCalculator),
                    False: ShooterPass(zoneManager, shootOnMoveCalculator, shooter),
                },
                zoneManager.getInAllianceZoneBool,
            )
        )

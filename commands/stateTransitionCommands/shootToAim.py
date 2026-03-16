from commands2 import ParallelCommandGroup, SequentialCommandGroup

from commands.baseCommands.kickerDeploy import KickerDeploy
from commands.baseCommands.indexerIdle import IndexerIdle
from commands.baseCommands.woahvalIdle import WoahvalIdle
from commands.baseCommands.shooterShootOnMove import ShootOnMove
from subsystems import Kicker, Indexer, Woahval, Shooter, Turret, ShootOnMoveCalculator


class ShootToAim(SequentialCommandGroup):
    def __init__(
        self,
        kicker: Kicker,
        indexer: Indexer,
        woahval: Woahval,
        shooter: Shooter,
        turret: Turret,
        shootOnMoveCalculator: ShootOnMoveCalculator,
    ):
        super().__init__(
            ParallelCommandGroup(
                KickerDeploy(kicker),
                IndexerIdle(indexer),
                WoahvalIdle(woahval),
            ),
            ShootOnMove(shooter, turret, shootOnMoveCalculator),
        )

from commands2 import ParallelCommandGroup

from commands.baseCommands.kickerStow import KickerStow
from commands.baseCommands.indexerScore import IndexerScore
from commands.baseCommands.woahvalScore import WoahvalScore
from commands.baseCommands.shootOnMove import ShootOnMove
from subsystems import Kicker, Indexer, Woahval, Shooter, Turret, ShootOnMoveCalculator


class AimToShoot(ParallelCommandGroup):
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
            KickerStow(kicker),
            IndexerScore(indexer),
            WoahvalScore(woahval),
            ShootOnMove(shooter, turret, shootOnMoveCalculator),
        )

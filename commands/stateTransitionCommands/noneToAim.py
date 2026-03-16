from commands2 import SequentialCommandGroup, ParallelCommandGroup, WaitCommand

from commands.baseCommands.shooterShootOnMove import ShootOnMove
from commands.baseCommands.kickerDeploy import KickerDeploy
from commands.baseCommands.woahvalScore import WoahvalScore
from commands.baseCommands.indexerScore import IndexerScore

from subsystems import Shooter, Turret, ShootOnMoveCalculator, Kicker, Indexer, Woahval


class NoneToAim(SequentialCommandGroup):
    def __init__(
        self,
        shooter: Shooter,
        turret: Turret,
        kicker: Kicker,
        indexer: Indexer,
        woahval: Woahval,
        shootOnMoveCalculator: ShootOnMoveCalculator,
    ):
        super().__init__(
            ParallelCommandGroup(
                KickerDeploy(kicker),
                SequentialCommandGroup(
                    IndexerScore(indexer),
                    WaitCommand(0.25),
                    WoahvalScore(woahval),
                ),
            ),
            ShootOnMove(shooter, turret, shootOnMoveCalculator),
        )

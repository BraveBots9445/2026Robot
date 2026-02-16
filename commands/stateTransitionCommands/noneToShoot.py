from commands2 import SequentialCommandGroup

from commands.stateTransitionCommands.noneToAim import NoneToAim
from commands.stateTransitionCommands.aimToShoot import AimToShoot
from subsystems import Shooter, Turret, Kicker, Indexer, Woahval, ShootOnMoveCalculator


class NoneToShoot(SequentialCommandGroup):
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
            NoneToAim(shooter, turret, kicker, indexer, woahval, shootOnMoveCalculator),
            AimToShoot(
                kicker, indexer, woahval, shooter, turret, shootOnMoveCalculator
            ),
        )

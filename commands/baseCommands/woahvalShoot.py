from commands2 import SequentialCommandGroup

from subsystems import Woahval

from commands.baseCommands.woahvalDejam import WoahvalDejam
from commands.baseCommands.woahvalScore import WoahvalScore


class WoahvalShoot(SequentialCommandGroup):
    def __init__(self, woahval: Woahval):
        super().__init__(
            WoahvalDejam(woahval, 0.25),
            WoahvalScore(woahval),
        )

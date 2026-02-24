from commands2 import ParallelCommandGroup

from subsystems import Woahval, Indexer

from commands.baseCommands.woahvalStop import WoahvalStop
from commands.baseCommands.indexerStop import IndexerStop


class AimToNone(ParallelCommandGroup):
    def __init__(self, woahval: Woahval, indexer: Indexer):
        super().__init__(WoahvalStop(woahval), IndexerStop(indexer))

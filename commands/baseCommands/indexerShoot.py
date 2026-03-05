from commands2 import SequentialCommandGroup

from subsystems import Indexer

from commands.baseCommands.indexerDejam import IndexerDejam
from commands.baseCommands.indexerScore import IndexerScore


class IndexerShoot(SequentialCommandGroup):
    def __init__(self, indexer: Indexer):
        super().__init__(
            IndexerDejam(indexer, 0.25),
            IndexerScore(indexer),
        )

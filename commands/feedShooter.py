from commands2 import ParallelCommandGroup, SequentialCommandGroup, WaitCommand

from subsystems.indexer import Indexer
from subsystems.hopper import HopperFloor

from commands import IndexerDejam, IndexerForward, HopperFeed


class FeedShooter(ParallelCommandGroup):
    def __init__(self, indexer: Indexer, hopper: HopperFloor):
        super().__init__(
            IndexerForward(indexer),
            HopperFeed(hopper),
        )

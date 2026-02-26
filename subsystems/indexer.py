from .openLoopWheel import OpenLoopWheel

from .BraveLogger import IndexerData


class Indexer(OpenLoopWheel):
    """
    The indexer subsystem.
    This is the "woahval" that feeds fuel into the shooter
    """

    def __init__(self) -> None:
        super().__init__(
            motorId=23,
            name="Indexer",
            inverted=False,
            rampTime=0,
            shootingDutyCycle=1.0,
            idleDutyCycle=0.0,
        )
        self._data = IndexerData(0.0, 0.0, 0.0)

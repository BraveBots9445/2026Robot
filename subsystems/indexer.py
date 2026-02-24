from .openLoopWheel import OpenLoopWheel, OpenWheelData


class IndexerData(OpenWheelData):
    # this is just a wrapper with a different name for readability
    pass


class Indexer(OpenLoopWheel):
    """
    The indexer subsystem.
    This is the "woahval" that feeds fuel into the shooter
    """

    def __init__(self) -> None:
        super().__init__(
            id=23,
            name="Indexer",
            inverted=False,
            rampTime=0,
            shootingDutyCycle=0.0,
            idleDutyCycle=0.0,
        )
        self._data = IndexerData(0.0, 0.0, 0.0)

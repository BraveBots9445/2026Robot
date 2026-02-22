from subsystems.openLoopWheel import OpenLoopWheel, OpenWheelData


class WoahvalData(OpenWheelData):
    # this is just a wrapper with a different name for readability
    pass


class Woahval(OpenLoopWheel):
    """
    The woahval subsystem.
    This is the "woahval" that feeds fuel into the indexer
    """

    def __init__(self) -> None:
        super().__init__(
            id=30,
            name="Woahval",
            shootingDutyCycle=0.5,
            idleDutyCycle=0.1,
        )
        self._data = WoahvalData(0.0, 0.0, 0.0)

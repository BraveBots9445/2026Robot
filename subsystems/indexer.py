from .openLoopWheel import OpenLoopWheel


class Indexer(OpenLoopWheel):
    """
    The indexer subsystem.
    This is the "woahval" that feeds fuel into the shooter
    """

    def __init__(self) -> None:
        super().__init__(
            id=31,
            name="Indexer",
            rampTime=0,
            shootingDutyCycle=0.75,
            idleDutyCycle=0.0,
        )

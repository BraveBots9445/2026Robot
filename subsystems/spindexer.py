from .openLoopWheel import OpenLoopWheel


class Spindexer(OpenLoopWheel):
    """
    The spindexer subsystem.
    This is the "woahval" that feeds fuel into the indexer
    """

    def __init__(self) -> None:
        super().__init__(
            id=30,
            name="Spindexer",
            shootingDutyCycle=0.5,
            idleDutyCycle=0.1,
        )

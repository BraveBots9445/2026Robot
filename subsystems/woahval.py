from subsystems.openLoopWheel import OpenLoopWheel


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

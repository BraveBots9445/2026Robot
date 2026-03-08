from wpilib import SmartDashboard

from subsystems.openLoopWheel import OpenLoopWheel

from .BraveLogger import WoahvalData


class Woahval(OpenLoopWheel):
    """
    The woahval subsystem.
    This is the "woahval" that feeds fuel into the indexer
    """

    def __init__(self) -> None:
        super().__init__(
            motorId=22,
            inverted=False,
            name="Woahval",
            shootingDutyCycle=1.0,
            # shootingDutyCycle=0.0,
            idleDutyCycle=0.1,
            rampTime=0,
        )
        self._data = WoahvalData(0.0, 0.0, 0.0)

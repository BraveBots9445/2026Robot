from commands2 import Subsystem

from phoenix6.hardware import CANdle
from phoenix6.configs import CANdleConfiguration


class LED(Subsystem):
    _candleCanId: int = 0
    _canbus: str = ""

    def __init__(self):
        super().__init__()

        self._candle = CANdle(self._candleCanId, self._canbus)

        candleConfig = CANdleConfiguration()
        self._candle.configurator.apply(candleConfig)

    def periodic(self):
        pass

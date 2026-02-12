from dataclasses import dataclass

from wpilib import CAN, CANData, Notifier

from wpimath.units import seconds


@dataclass
class ToFData:
    distance_m: int
    """
    1000 times the distance in meters
    u24
    """

    dis_status: int
    """
    Status of the distance measurement
    0 	Measuring distance is valid
    1 	Standard deviation is more than 15mm
    2 	Signal strength is lower than 1Mcps
    4 	Phase exceeds boundary
    5 	HW or VCSEL has fault
    7 	Phase is not matched
    8 	Internal algorithm underflow
    14 	Measuring distance is invalid 
    u8
    """

    signal_strength: int
    """
    u16
    """


class WaveshareTof:
    """
    A class for interfacing with the Waveshare ToF sensor over CAN.
    Data is stored periodically as a ToFData object, which can be accessed with the getData() method.
    https://www.waveshare.com/tof-laser-range-sensor.htm
    """

    MANUFACTURER_ID = 0x200

    _data: ToFData | None = None

    _notifier: Notifier

    def __init__(self, canId: int):
        if canId < 0 or canId > 63:
            raise ValueError("CAN ID must be between 0 and 63")
        self.can = CAN(self.MANUFACTURER_ID + canId)
        self._notifier = Notifier(self._updateMeasurements)
        self._notifier.startPeriodic(seconds(0.01))  # 100 Hz update rate

    def _updateMeasurements(self) -> None:
        """
        A private method to get the latest data from the ToF sensor
        The notifier in the class should be calling this periodically, not an external user
        """
        buf = CANData()
        self.can.readPacketLatest(1, buf)
        data = buf.data

        distance_m = int.from_bytes(data[0:3], byteorder="big", signed=False)
        dis_status = data[3]
        signal_strength = int.from_bytes(data[4:6], byteorder="big", signed=False)
        # the rest of the bytes are reserved
        self._data = ToFData(
            distance_m=distance_m,
            dis_status=dis_status,
            signal_strength=signal_strength,
        )

    def getData(self) -> ToFData | None:
        """
        Gets the most recent ToF data. Data will be periodically updated by the class itself

        :return: The most recent ToF data or None if no data is available
        :rtype: ToFData | None
        """
        return self._data

    def stopMeasurements(self) -> None:
        """
        Stops the periodic measurement updates
        """
        self._notifier.stop()

    def startMeasurements(self, interval: seconds = 0.01) -> None:
        """
        Sets the measurement interval for the ToF sensor

        :param interval: The interval in seconds
        :type interval: seconds
        """
        if interval <= 0:
            interval = 0.01
        self._notifier.stop()
        self._notifier.startPeriodic(interval)

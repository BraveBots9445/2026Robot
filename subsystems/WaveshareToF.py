from dataclasses import dataclass

from wpilib import CAN, CANData, Notifier, RobotController, SmartDashboard
from wpiutil.wpistruct import make_wpistruct

from wpimath.units import seconds, microseconds, meters


@make_wpistruct()
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

    time: float
    """
    The FPGA time (microseconds) at which the measurement was taken
    """


class WaveshareTof:
    """
    A class for interfacing with the Waveshare ToF sensor over CAN.
    Data is stored periodically as a ToFData object, which can be accessed with the getData() method.
    https://www.waveshare.com/tof-laser-range-sensor.htm
    """

    MANUFACTURER_ID = 0x200

    _data: ToFData | None = None

    _canID: int

    _notifier: Notifier

    def __init__(self, canId: int):
        if canId < 0 or canId > 63:
            raise ValueError("CAN ID must be between 0 and 63")
        self._canID = canId
        self.can = CAN(canId)
        # self._notifier = Notifier(self._updateMeasurements)
        # self._notifier.startPeriodic(0.01)  # 100 Hz update rate

    def _updateMeasurements(self) -> None:
        """
        A private method to get the latest data from the ToF sensor
        The notifier in the class should be calling this periodically, not an external user
        """
        buf = CANData()
        # for id in range(0, 0b1111111111):
        if not self.can.readPacketLatest(0x400, buf):
            SmartDashboard.putBoolean("Failed", True)
            return
        else:
            SmartDashboard.putBoolean("Failed", False)
        data = buf.data
        # vals = [int(x) for x in data]
        #     if not all(x == 0 for x in vals):
        #         print(id)
        #         print(vals)
        #         break
        # return

        distance_m = int.from_bytes(data[0:3], byteorder="big", signed=False)
        dis_status = data[3]
        signal_strength = int.from_bytes(data[4:6], byteorder="big", signed=False)
        # the rest of the bytes are reserved
        self._data = ToFData(
            distance_m=distance_m,
            dis_status=dis_status,
            signal_strength=signal_strength,
            time=RobotController.getFPGATime(),
        )

    def getData(self, timeout: microseconds = float("inf")) -> ToFData | None:
        """
        Gets the most recent ToF data. Data will be periodically updated by the class itself

        :param timeout: The maximum time to wait for data in microseconds. If no data is available after this time, None will be returned. Default is infinite.
        :type timeout: microseconds
        :return: The most recent ToF data or None if no data is available
        :rtype: ToFData | None
        """
        currTime = RobotController.getFPGATime()
        if self._data is None or (currTime - self._data.time) > timeout:
            return None
        return self._data

    def getDistance(self, timeout: microseconds = float("inf")) -> meters | None:
        """
        Gets the most recent distance measurement in meters.

        :param timeout: The maximum time to wait for data in microseconds. If no data is available after this time, None will be returned. Default is infinite.
        :type timeout: microseconds
        :return: The most recent distance measurement in meters or None if no data is available
        :rtype: meters | None
        """
        data = self.getData(timeout)
        if data is None:
            return None
        return data.distance_m

    def getSignalStrength(self, timeout: microseconds = float("inf")) -> int | None:
        """
        Gets the most recent signal strength measurement.

        :param timeout: The maximum time to wait for data in microseconds. If no data is available after this time, None will be returned. Default is infinite.
        :type timeout: microseconds
        :return: The most recent signal strength measurement or None if no data is available
        :rtype: int | None
        """
        data = self.getData(timeout)
        if data is None:
            return None
        return data.signal_strength

    def getSignalStatus(self, timeout: microseconds = float("inf")) -> int | None:
        """
        Gets the most recent signal status measurement.

        :param timeout: The maximum time to wait for data in microseconds. If no data is available after this time, None will be returned. Default is infinite.
        :type timeout: microseconds
        :return: The most recent signal status measurement or None if no data is available
        :rtype: int | None
        """
        data = self.getData(timeout)
        if data is None:
            return None
        return data.dis_status

    def getRecentMeasurementTime(self) -> microseconds | None:
        """
        Gets the FPGA time at which the most recent measurement was taken.

        :return: The FPGA time of the most recent measurement or None if no data is available
        :rtype: microseconds | None
        """
        data = self.getData()
        if data is None:
            return None
        return data.time

    def getTimeSinceRecentMeasurement(self) -> microseconds | None:
        """
        Gets the time since the most recent measurement was taken.

        :return: The time since the most recent measurement in microseconds or None if no data is available
        :rtype: microseconds | None
        """
        lastTime = self.getRecentMeasurementTime()
        if lastTime is None:
            return None
        return RobotController.getFPGATime() - lastTime

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

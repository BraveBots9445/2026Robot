from copy import deepcopy

from dataclasses import dataclass

from threading import Lock

from commands2 import Subsystem, Command

from ntcore import NetworkTableInstance, NetworkTable, StructPublisher

from wpilib import Mechanism2d, MechanismLigament2d, SmartDashboard

from wpimath.units import degrees, inches

from wpiutil.wpistruct import make_wpistruct

from wpilib import Servo

from .BraveLogger import BraveLogger, KickerData


class Kicker(Subsystem):
    """
    The kicker in the ramp that can stop fuel from entering the shooter
    """

    _servo: Servo

    _servoPort: int = 10

    _setpoint: inches = 0

    _gearRatio: float = 1.0
    """
    Measured as the rotations of the servo required to move the kicker 1 inch
    """

    _data: KickerData

    _lock: Lock

    _mech: MechanismLigament2d

    def __init__(self):
        self._servo = Servo(self._servoPort)

        self._nettable = NetworkTableInstance.getDefault().getTable("000Kicker")
        self._dataPub = self._nettable.getStructTopic("Data", KickerData).publish()

        self._data = KickerData(angleDeg=0, positionIn=0, deployed=False)

        mech = Mechanism2d(100, 100)

        self._mech = mech.getRoot("Kicker", 50, 50).appendLigament("KickerArm", 0, 0)

        self._lock = Lock()
        # SmartDashboard.putData("Kicker", self)
        # SmartDashboard.putData("Kicker Mech", mech)

    def periodic(self) -> None:
        with self._lock:
            self._data.angleDeg = self._servo.getAngle()
            self._data.positionIn = self._setpoint
            self._data.deployed = self._setpoint > 0.5
            BraveLogger.pushSubsystemData(deepcopy(self._data))

        self._servo.set(self._getInchesToRotations(self._setpoint))

        # self._mech.setLength(self._data.positionIn * 10 + 5)

        self._dataPub.set(self._data)

    def getPosition(self) -> inches:
        """
        Get the current position of the kicker in inches.
        """
        return self._getRotationsToInches(self._servo.get())

    def setPosition(self, position: inches) -> None:
        """
        Set the desired position of the kicker in inches.
        """
        self._setpoint = position

    def deploy(self) -> None:
        """
        Deploy the kicker to the default position.
        """
        self.setPosition(2.0)

    def stow(self) -> None:
        """
        Stow the kicker to the default position.
        """
        self.setPosition(0.0)

    def _getRotationsToInches(self, rotations: float) -> inches:
        return rotations * self._gearRatio

    def _getInchesToRotations(self, inches: float) -> float:
        return inches / self._gearRatio

    def getData(self) -> KickerData:
        """
        Get the current data of the kicker.
        """
        with self._lock:
            return self._data

from dataclasses import dataclass

from threading import Lock

from commands2 import Subsystem

from commands2.command import Command
from ntcore import NetworkTableInstance, NetworkTable, StructPublisher

from wpilib import Servo, SmartDashboard, Mechanism2d, MechanismLigament2d

from wpimath.units import degrees
from wpimath.geometry import Rotation2d

from wpiutil.wpistruct import make_wpistruct

from .BraveLogger import BraveLogger, PassiveHooksData


class PassiveHooks(Subsystem):
    _servoLeft: Servo

    _servoRight: Servo

    _nettable: NetworkTable

    _mech: MechanismLigament2d

    _data: PassiveHooksData

    _lock: Lock

    _setpoint: Rotation2d = Rotation2d.fromDegrees(90)

    def __init__(self) -> None:
        self._servoLeft = Servo(8)
        self._servoRight = Servo(9)

        self._nettable = NetworkTableInstance.getDefault().getTable("PassiveHooks")

        mech = Mechanism2d(100, 100)
        self._mech = mech.getRoot("PassiveHooks", 50, 50).appendLigament("Hook", 40, 90)

        self._data = PassiveHooksData(90, False)

        self._dataPub = self._nettable.getStructTopic(
            "Data", PassiveHooksData
        ).publish()

        self._lock = Lock()

        # SmartDashboard.putData("PassiveHooks/Mech", mech)
        # SmartDashboard.putData("PassiveHooks/ServoLeft", self._servoLeft)
        # SmartDashboard.putData("PassiveHooks/ServoRight", self._servoRight)
        # SmartDashboard.putData("PassiveHooks/Subsystem", self)

    def periodic(self) -> None:
        # with self._lock:
        self._data.setpointDegrees = self._setpoint.degrees()
        # self._data.setpoint = self._setpoint
        self._data.deployed = self._setpoint.degrees() < 85
        BraveLogger.pushSubsystemData(self._data)

        self._servoLeft.setAngle(self._setpoint.degrees())
        self._servoRight.setAngle(self._setpoint.degrees())

        # self._mech.setAngle(self._setpoint.degrees())

    def setSetpoint(self, setpoint: Rotation2d) -> None:
        self._setpoint = setpoint

    def setSetpointDegrees(self, setpointDegrees: degrees) -> None:
        self._setpoint = Rotation2d.fromDegrees(setpointDegrees)

    def deploy(self) -> None:
        self.setSetpointDegrees(0)

    def retract(self) -> None:
        self.setSetpointDegrees(90)

    def getSetpoint(self) -> Rotation2d:
        return self._setpoint

    def getData(self) -> PassiveHooksData:
        with self._lock:
            return self._data

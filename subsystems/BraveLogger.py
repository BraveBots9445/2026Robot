import threading
import time
from dataclasses import dataclass
from typing import Any

from ntcore import NetworkTableInstance, NetworkTable, StructPublisher

from wpimath.geometry import Rotation2d

from wpiutil.wpistruct import make_wpistruct

from wpimath.units import (
    degrees,
    inches,
    amperes,
)
from wpimath.geometry import Rotation2d

from phoenix6.units import rotations_per_second, rotation
from phoenix6.status_signal import StatusSignal


@make_wpistruct
@dataclass
class ClimberData:
    motorCurrent: amperes
    motorOutputPercent: float
    motorPositionRaw: rotation
    motorVelocityRaw: rotations_per_second
    state: int


@make_wpistruct
@dataclass
class OpenWheelData:
    dutyCycle: float
    velocity: rotations_per_second
    current: amperes


class IndexerData(OpenWheelData):
    # this is just a wrapper with a different name for readability
    pass


class WoahvalData(OpenWheelData):
    # this is just a wrapper with a different name for readability
    pass


@make_wpistruct
@dataclass
class IntakeData:
    # pivotPosition: Rotation2d
    # pivotSetpoint: Rotation2d
    pivotPositionDegrees: degrees
    pivotSetpointDegrees: degrees
    pivotCurrent: float
    pivotDutyCycle: float
    pivotClosedLoopSlot: int
    pivotVelocity: float
    rollerSetpoint: float
    rollerDutyCycle: float
    rollerCurrent: float
    rollerVelocity: float


@make_wpistruct
@dataclass
class KickerData:
    angleDeg: degrees
    positionIn: inches
    deployed: bool


@make_wpistruct
@dataclass
class PassiveHooksData:
    setpointDegrees: degrees
    # setpoint: Rotation2d
    deployed: bool


@make_wpistruct
@dataclass
class ShooterData:
    actualFlywheelSpeedRpm: float
    desiredFlywheelSpeedRpm: float
    actualHoodAngleDegrees: degrees
    desiredHoodAngleDegrees: degrees
    # actualHoodAngle: Rotation2d
    # desiredHoodAngle: Rotation2d
    motorCurrent: float
    motorDutyCycle: float
    hoodMotorCurrent: float


@make_wpistruct
@dataclass
class TurretData:
    # _rotation: Rotation2d
    _rotationDegrees: degrees
    # _rotationSetpoint: Rotation2d
    _rotationSetpointDegrees: degrees
    _motorCurrent: amperes
    _motorDutyCycle: float


@make_wpistruct
@dataclass
class TimerData:
    autoWinner: int
    """
    None = 0
    US = 1 
    Them = 2
    """
    currentShift: int
    shiftTime: float
    matchTimeUp: float
    matchTimeDown: float
    timeLeftInShift: float
    ourActivePeriod: bool
    rawOurActivePeriod: bool


@make_wpistruct
@dataclass
class BraveData:
    turretData: TurretData
    climberData: ClimberData
    indexerData: IndexerData
    woahvalData: WoahvalData
    passiveHooksData: PassiveHooksData
    intakeData: IntakeData
    shooterData: ShooterData
    timerData: TimerData


class BraveLogger:
    _nettable: NetworkTable

    _dataPub: StructPublisher

    _data: BraveData

    _statusSignals: list[StatusSignal] = []
    _statusSignalIndex: int = 0
    _statusSignalBatchSize: int = 12

    def __init__(
        self,
    ) -> None:
        BraveLogger._nettable = NetworkTableInstance.getDefault().getTable(
            "000BraveLogger"
        )
        BraveLogger._dataPub = self._nettable.getStructTopic(
            "Data", BraveData
        ).publish()
        BraveLogger._data = BraveData(
            TurretData(0, 0, 0, 0),
            ClimberData(0, 0, 0, 0, 0),
            IndexerData(0, 0, False),
            WoahvalData(0, 0, False),
            PassiveHooksData(0, False),
            IntakeData(0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
            ShooterData(0, 0, 0, 0, 0, 0, 0),
            TimerData(0, 0, 0, 0, 0, 0, False, False),
        )

        def _log_loop():
            while True:
                BraveLogger.log()
                time.sleep(0.10)

        def _refresh_loop():
            while True:
                BraveLogger.refreshStatusSignals()
                time.sleep(0.04)

        threading.Thread(target=_log_loop, daemon=True, name="BraveLogger-log").start()
        threading.Thread(
            target=_refresh_loop, daemon=True, name="BraveLogger-refresh"
        ).start()

    @staticmethod
    def log() -> None:
        BraveLogger._dataPub.set(BraveLogger._data)

    @staticmethod
    def refreshStatusSignals() -> None:
        """
        Refreshes the status signals for the BraveLogger subsystem.
        This method is called periodically to update the status signals.
        """
        signals = BraveLogger._statusSignals
        total = len(signals)
        if total == 0:
            return

        start = BraveLogger._statusSignalIndex
        end = min(start + BraveLogger._statusSignalBatchSize, total)
        batch = signals[start:end]
        if batch:
            StatusSignal.refresh_all(batch)  # type: ignore

        BraveLogger._statusSignalIndex = 0 if end >= total else end

    @staticmethod
    def pushSubsystemData(data: Any) -> None:
        """
        Pushes the given data to the network table for the subsystem.

        :param data: The data to push to the network table.
        :type data: Any wpistruct type (e.g., TurretData, ClimberData, etc.)
        """
        if isinstance(data, TurretData):
            BraveLogger._data.turretData = data
        elif isinstance(data, ClimberData):
            BraveLogger._data.climberData = data
        elif isinstance(data, IndexerData):
            BraveLogger._data.indexerData = data
        elif isinstance(data, WoahvalData):
            BraveLogger._data.woahvalData = data
        elif isinstance(data, PassiveHooksData):
            BraveLogger._data.passiveHooksData = data
        elif isinstance(data, IntakeData):
            BraveLogger._data.intakeData = data
        elif isinstance(data, ShooterData):
            BraveLogger._data.shooterData = data
        elif isinstance(data, TimerData):
            BraveLogger._data.timerData = data

    @staticmethod
    def registerStatusSignal(signal: StatusSignal | list[StatusSignal]) -> None:
        """
        Registers a status signal to be refreshed periodically.

        :param signal: The status signal to register.
        :type signal: StatusSignal
        """
        if isinstance(signal, list):
            BraveLogger._statusSignals.extend(signal)
        else:
            BraveLogger._statusSignals.append(signal)

from dataclasses import dataclass
from typing import Callable

from ntcore import NetworkTableInstance, NetworkTable, StructPublisher

from wpilib import Notifier
from wpimath.geometry import Rotation2d

from wpiutil.wpistruct import make_wpistruct

from subsystems import (
    Intake,
    Shooter,
    Climber,
    Turret,
    TurretData,
    ClimberData,
    IndexerData,
    WoahvalData,
    PassiveHooksData,
)


@make_wpistruct
@dataclass
class BraveData:
    turretData: TurretData
    climberData: ClimberData
    indexerData: IndexerData
    woahvalData: WoahvalData
    passiveHooksData: PassiveHooksData


class BraveLogger:
    _nettable: NetworkTable

    _dataPub: StructPublisher

    _data: BraveData

    def __init__(
        self,
        getTurretData: Callable[[], TurretData],
        getClimberData: Callable[[], ClimberData],
        getIndexerData: Callable[[], IndexerData],
        getWoahvalData: Callable[[], WoahvalData],
        getPassiveHooksData: Callable[[], PassiveHooksData],
    ) -> None:
        self._nettable = NetworkTableInstance.getDefault().getTable("000BraveLogger")
        self._dataPub = self._nettable.getStructTopic("Data", BraveData).publish()
        self._data = BraveData(
            TurretData(Rotation2d(), 0, Rotation2d(), 0, 0, 0),
            ClimberData(0, 0, 0, Rotation2d(), 0, 0, 0, 0, 0, 0),
            IndexerData(0, 0, False),
            WoahvalData(0, 0, False),
            PassiveHooksData(0, Rotation2d(), False),
        )

        self._getTurretData = getTurretData
        self._getClimberData = getClimberData
        self._getIndexerData = getIndexerData
        self._getWoahvalData = getWoahvalData
        self._getPassiveHooksData = getPassiveHooksData

        self._notifier = Notifier(self._publishData)
        self._notifier.startPeriodic(0.02)

    def _publishData(self) -> None:
        self._data = BraveData(
            self._getTurretData(),
            self._getClimberData(),
            self._getIndexerData(),
            self._getWoahvalData(),
            self._getPassiveHooksData(),
        )
        self._dataPub.set(self._data)
